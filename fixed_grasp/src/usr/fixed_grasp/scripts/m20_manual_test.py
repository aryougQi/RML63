#!/usr/bin/env python3
"""One manual place on the real arm, without a dog; dedicated test topics only."""
import argparse
from copy import deepcopy
import fcntl
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import uuid

import rospy
from std_msgs.msg import String
from m20_ledger import canonical
from m20_protocol import number, parse_json

PREFIX = '/m20_cooperation_test'


class ManualPlace:
    """One explicit user command; only protocol heartbeats repeat automatically."""
    def __init__(self, clock):
        self.clock = clock; self.lock = threading.RLock()
        self.boot = 'manual-test-'+uuid.uuid4().hex
        self.run_id = 'manual-run-'+uuid.uuid4().hex
        self.tx = self.run_id+':place'
        self.request_id = 'manual-request-'+uuid.uuid4().hex
        self.seq = 0; self.arm = None; self.arm_boot = None
        self.request = None; self.result = None; self.error = None; self.grasp = None

    def observe_status(self, status):
        with self.lock:
            if status.get('manual_test') is not True:
                self.error = '服务未处于手动测试模式，请使用 --m20-test 启动'
                return
            if not number(status.get('sample_stamp')) or not 0 <= self.clock()-status['sample_stamp'] <= .5:
                return
            if self.arm_boot is not None and status.get('boot_id') != self.arm_boot:
                self.error = '机械臂服务重启，本次结果需要人工核实'
                return
            if self.arm is not None and status.get('boot_id') == self.arm.get('boot_id') and status.get('seq', -1) <= self.arm.get('seq', -1):
                return
            self.arm = dict(status)

    def ready(self):
        with self.lock:
            if self.error: raise RuntimeError(self.error)
            status = self.arm
            if not status or not 0 <= self.clock()-status['sample_stamp'] <= .5: return False
            if status.get('next_grasp') is None:
                raise RuntimeError('测试顺序已完成两块；补料后停止服务并重置测试计数')
            return bool(status.get('healthy') and status.get('ready') and status.get('active_request_id') is None)

    def arm_test(self):
        with self.lock:
            if not self.ready(): raise RuntimeError('机械臂尚未就绪：需要空载、停止并位于 Home')
            self.arm_boot = self.arm['boot_id']; self.grasp = self.arm['next_grasp']

    def heartbeat(self):
        with self.lock:
            self.seq += 1
            common = dict(schema_version=1, boot_id=self.boot, seq=self.seq, sample_stamp=self.clock())
            robot = dict(common, healthy=True, stationary=True, posture='crouched', basic_age_s=0., motion_age_s=0.)
            task = dict(common, state='ARM_EXECUTING', run_id=self.run_id, transaction_id=self.tx)
            return robot, task

    def authorized(self):
        with self.lock:
            if not self.ready(): return False
            ref = self.arm.get('manual_robot_state_ref')
            return bool(self.arm.get('manual_authorized_transaction_id') == self.tx and
                        ref and ref.get('boot_id') == self.boot)

    def build_request(self):
        with self.lock:
            if self.request is not None: raise RuntimeError('本进程已经发送过 place，不会再次执行')
            if not self.authorized(): raise RuntimeError('测试心跳尚未被机械臂服务确认')
            now = self.clock()
            self.request = dict(schema_version=1, request_id=self.request_id, source='task', run_id=self.run_id,
                mission_id='manual-test', waypoint_id='manual-place-'+str(self.grasp), transaction_id=self.tx,
                operation='execute', task_type='place', issued_at=now, expires_at=now+10.,
                robot_state_ref=deepcopy(self.arm['manual_robot_state_ref']), payload={})
            return deepcopy(self.request)

    def feedback(self, data):
        with self.lock:
            if (not self.request or data.get('operation') != 'execute' or data.get('request_id') != self.request_id or
                    data.get('transaction_id') != self.tx or data.get('run_id') != self.run_id):
                return False
            if data.get('status') in ('succeeded', 'failed', 'cancelled'):
                self.result = dict(data)
            return True

    def stowed(self):
        with self.lock:
            return bool(self.arm and 0 <= self.clock()-self.arm['sample_stamp'] <= .5 and
                        self.arm.get('transaction_id') == self.tx and self.arm.get('active_request_id') is None and
                        self.arm.get('arm_safe_to_move') is True)

    def cancel_request(self):
        now = self.clock()
        return dict(schema_version=1, request_id='manual-cancel-'+uuid.uuid4().hex, source='manual',
                    run_id=self.run_id, transaction_id=self.tx, target_request_id=self.request_id,
                    operation='cancel', issued_at=now, expires_at=now+10.)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['place'])
    parser.add_argument('--execute', action='store_true', help='实际驱动机械臂；确认没有连接机械狗、Place承托面已准备好')
    parser.add_argument('--timeout', type=float, default=90., help='等待完整搬运的最长秒数，超时请求取消')
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error('这是实际机械臂测试，请显式添加 --execute')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout 必须为有限正数')
    with open(Path(tempfile.gettempdir())/'fixed_grasp_m20_manual_test.lock', 'a') as lease:
        try: fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('已有手动测试客户端运行，请先结束它')
        rospy.init_node('m20_manual_test', anonymous=True, disable_signals=True)
        test = ManualPlace(lambda: rospy.Time.now().to_sec())
        request_pub = rospy.Publisher(PREFIX+'/arm/request', String, queue_size=10, latch=False)
        robot_pub = rospy.Publisher(PREFIX+'/robot/status', String, queue_size=10, latch=False)
        task_pub = rospy.Publisher(PREFIX+'/task/status', String, queue_size=10, latch=False)
        def status_callback(message):
            try: test.observe_status(parse_json(message.data))
            except (ValueError, TypeError): pass
        def feedback_callback(message):
            try: data = parse_json(message.data)
            except (ValueError, TypeError): return
            if test.feedback(data):
                print('%s grasp=%s %s: %s' % (data.get('status'), data.get('grasp'), data.get('stage', ''), data.get('message', '')), flush=True)
        subscribers = [rospy.Subscriber(PREFIX+'/arm/status', String, status_callback, queue_size=100),
                       rospy.Subscriber(PREFIX+'/arm/feedback', String, feedback_callback, queue_size=100)]
        def wait(condition, timeout, description, check_error=True):
            deadline = time.monotonic()+timeout
            while not condition():
                if check_error and test.error: raise RuntimeError(test.error)
                if rospy.is_shutdown(): raise RuntimeError('ROS 已关闭')
                if time.monotonic() >= deadline: raise RuntimeError('等待超时：'+description)
                time.sleep(.02)
            if check_error and test.error: raise RuntimeError(test.error)
        timer = None
        try:
            wait(test.ready, 10., '--m20-test 服务、空载 Home 和真实机械臂反馈')
            test.arm_test()
            print('手动真机测试：本次放第 %s 块。transaction_id=%s' % (test.grasp, test.tx), flush=True)
            def heartbeat(_event):
                robot, task = test.heartbeat()
                robot_pub.publish(String(data=canonical(robot))); task_pub.publish(String(data=canonical(task)))
            timer = rospy.Timer(rospy.Duration(.05), heartbeat)
            wait(lambda: test.authorized() and request_pub.get_num_connections() > 0, 5., '测试心跳确认')
            request_pub.publish(String(data=canonical(test.build_request())))
            wait(lambda: test.result is not None, args.timeout, 'place 完整搬运结果')
            if test.result['status'] != 'succeeded' or not test.result.get('task_success'):
                raise RuntimeError('测试失败：'+test.result.get('message', 'unknown result'))
            wait(test.stowed, 30., '实际空载收臂确认')
            print('第 %s 块完成并已回 Home。下一块：%s' % (test.grasp, test.arm.get('next_grasp')), flush=True)
            return 0
        finally:
            if test.request is not None and test.result is None and not rospy.is_shutdown():
                request_pub.publish(String(data=canonical(test.cancel_request())))
                print('已请求取消，等待机械臂停止反馈……', flush=True)
                try: wait(lambda: test.result is not None, 10., '取消后的原任务结果', check_error=False)
                except RuntimeError as error: print(str(error)+'；请检查机械臂状态。', flush=True)
            if timer: timer.shutdown()
            for subscriber in subscribers: subscriber.unregister()
            request_pub.unregister(); robot_pub.unregister(); task_pub.unregister()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print(str(error), flush=True)
        raise SystemExit(1)
