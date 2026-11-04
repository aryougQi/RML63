"""M20 ROS-JSON protocol and watchdog, independent of ROS transport."""
import json
import math
import threading
import uuid

from core import Cancelled
from m20_ledger import digest


class ProtocolError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def required_text(data, key):
    if not isinstance(data.get(key), str) or not data[key].strip():
        raise ProtocolError('INVALID_REQUEST', 'nonempty '+key+' required')


def parse_json(raw):
    def reject_constant(_value):
        raise ValueError('nonfinite JSON')
    value = json.loads(raw, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError('JSON object required')
    return value


def robot_ok(data):
    if data.get('healthy') is not True or data.get('stationary') is not True or data.get('posture') != 'crouched':
        return False
    # M20's healthy/stationary values are its measured aggregation, not ACKs.
    # Check raw ages as well whenever its aggregate packet exposes them.
    for key, limit in (('basic_age_s', 1.5), ('motion_age_s', .5)):
        if key in data and (not number(data[key]) or not 0 <= data[key] <= limit):
            return False
    return True


class StateStream:
    def __init__(self):
        self.boot = None
        self.seen_boots = set()
        self.seq = -1
        self.stamp = None
        self.epoch = 0
        self.current = None
        self.history = {}

    def invalidate(self):
        self.epoch += 1
        self.current = None

    def update(self, data, now, predicate=lambda data: True):
        try:
            if data.get('schema_version') != 1 or isinstance(data.get('schema_version'), bool):
                raise ValueError('schema')
            required_text(data, 'boot_id')
            if not integer(data.get('seq')) or not number(data.get('sample_stamp')):
                raise ValueError('header')
            if not 0 <= now-data['sample_stamp'] <= .5:
                raise ValueError('stale/future')
            boot, seq, stamp = data['boot_id'], data['seq'], data['sample_stamp']
            if boot != self.boot:
                if boot in self.seen_boots:
                    raise ValueError('retired boot')
                self.invalidate()
                self.boot = boot; self.seen_boots.add(boot); self.seq = -1; self.stamp = None
                self.history.clear()
            if seq <= self.seq or (self.stamp is not None and stamp <= self.stamp):
                raise ValueError('replayed/out-of-order status')
            if self.stamp is not None and now-self.stamp > .5:
                self.invalidate()
            self.seq = seq; self.stamp = stamp; self.current = dict(data)
            if predicate(data):
                self.history[seq] = self.epoch
                if len(self.history) > 4096:
                    self.history.pop(next(iter(self.history)))
            else:
                self.epoch += 1
            return True
        except (ValueError, TypeError, AttributeError):
            self.invalidate()
            return False

    def fresh(self, now):
        return self.current is not None and 0 <= now-self.current['sample_stamp'] <= .5


class Coordinator:
    def __init__(self, ledger, clock, reference_wait=.3):
        self.ledger = ledger; self.clock = clock; self.reference_wait = reference_wait
        self.lock = threading.RLock()
        self.robot = StateStream(); self.task = StateStream()
        self.local = dict(healthy=False, ready=False, arm_safe_to_move=False)
        self.boot_id = uuid.uuid4().hex; self.seq = 0
        self.active = None; self.pending = None; self.outbox = []; self.closed = False
        latest = ledger.latest()
        self.last_tx = latest['tx'] if latest else None

    def feedback(self, request, status, success, code, message, record=None):
        result = {key: request.get(key) for key in
                  ('request_id', 'run_id', 'mission_id', 'waypoint_id', 'transaction_id', 'operation')}
        result.update(schema_version=1, status=status, task_success=bool(success), arm_safe_to_move=False,
                      error_code=code, message=message)
        if request.get('operation') == 'execute':
            result['task_type'] = request.get('task_type')
        if 'target_request_id' in request:
            result['target_request_id'] = request['target_request_id']
        if record:
            original = json.loads(record['request'])
            for key in ('mission_id', 'waypoint_id'):
                if result.get(key) is None: result[key] = original[key]
            result.update(grasp=record['block'], transaction_status=record['status'], stage=record['stage'])
            if record['status'] == 'unknown': result['result_state'] = 'unknown'
            if request['operation'] == 'query':
                result['arm_safe_to_move'] = bool(not self.active and record['tx'] == self.last_tx and
                                                  self.local.get('arm_safe_to_move'))
        self.outbox.append(result)

    def replay(self, request, record):
        if request['operation'] == 'query':
            status = 'succeeded'
        else:
            status = 'failed' if record['status'] == 'unknown' else record['status']
        self.feedback(request, status, record['task_success'], record['error_code'], record['message'], record)

    def drain(self):
        with self.lock:
            messages, self.outbox = self.outbox, []
            return messages

    def ingest(self, channel, data):
        with self.lock:
            stream = self.robot if channel == 'robot' else self.task
            stream.update(data, self.clock(), robot_ok if channel == 'robot' else lambda _: True)
            self._watchdog()

    @staticmethod
    def signature(request):
        return digest({key: request[key] for key in ('run_id', 'mission_id', 'waypoint_id', 'task_type', 'payload')})

    def _validate(self, request):
        if request.get('schema_version') != 1 or isinstance(request.get('schema_version'), bool):
            raise ProtocolError('INVALID_SCHEMA', 'schema_version=1 required')
        for key in ('request_id', 'run_id', 'transaction_id', 'operation'):
            required_text(request, key)
        if request['operation'] not in ('execute', 'cancel', 'query'):
            raise ProtocolError('UNSUPPORTED_OPERATION', 'execute/cancel/query supported')
        if not number(request.get('issued_at')) or not number(request.get('expires_at')) or request['issued_at'] > request['expires_at']:
            raise ProtocolError('INVALID_REQUEST', 'valid issued_at/expires_at required')
        digest(request)  # Also rejects nonfinite/unsupported JSON payload values.
        if request['operation'] == 'execute':
            for key in ('mission_id', 'waypoint_id', 'source', 'task_type'):
                required_text(request, key)
            if request['source'] != 'task' or request['task_type'] != 'place':
                raise ProtocolError('UNSUPPORTED_TASK', 'only source=task, task_type=place is enabled')
            ref = request.get('robot_state_ref')
            if not isinstance(ref, dict) or not isinstance(request.get('payload'), dict):
                raise ProtocolError('INVALID_REQUEST', 'robot_state_ref and payload objects required')
            required_text(ref, 'boot_id')
            if not integer(ref.get('seq')):
                raise ProtocolError('INVALID_REQUEST', 'robot_state_ref.seq must be nonnegative integer')
        else:
            required_text(request, 'target_request_id')

    def _authorization(self, request, wait_allowed=False):
        now = self.clock(); ref = request['robot_state_ref']
        if not self.robot.fresh(now):
            raise ProtocolError('WAIT_REFERENCE' if wait_allowed else 'ROBOT_HEARTBEAT_LOST', 'fresh M20 robot/status required')
        if ref['boot_id'] != self.robot.boot:
            future = ref['boot_id'] not in self.robot.seen_boots
            raise ProtocolError('WAIT_REFERENCE' if wait_allowed and future else 'ROBOT_SESSION_CHANGED', 'M20 robot boot_id mismatch')
        if self.robot.seq < ref['seq']:
            raise ProtocolError('WAIT_REFERENCE' if wait_allowed else 'REFERENCE_NOT_OBSERVED', 'referenced robot sample has not arrived')
        if not robot_ok(self.robot.current):
            raise ProtocolError('ROBOT_NOT_CROUCHED', 'M20 must be healthy, stationary and actually crouched')
        if self.robot.history.get(ref['seq']) != self.robot.epoch:
            raise ProtocolError('REFERENCE_INVALIDATED', 'reference was not observed or robot state changed since that sample')
        phase = None if not self.task.fresh(now) else self.task.current.get('state', self.task.current.get('phase'))
        if phase != 'ARM_EXECUTING' or any(self.task.current.get(k) != request[k] for k in ('run_id', 'transaction_id')):
            raise ProtocolError('TASK_AUTHORIZATION_LOST', 'fresh ARM_EXECUTING authorization for this run/transaction required')
        if 'state' in self.task.current and 'phase' in self.task.current and self.task.current['state'] != self.task.current['phase']:
            raise ProtocolError('TASK_AUTHORIZATION_LOST', 'conflicting task phase fields')
        if self.task.current.get('healthy', True) is not True:
            raise ProtocolError('TASK_AUTHORIZATION_LOST', 'task authorization is unhealthy')

    def _admit(self, request):
        now = self.clock()
        if not request['issued_at'] <= now <= request['expires_at']:
            raise ProtocolError('REQUEST_EXPIRED', 'request is expired or issued in the future')
        if self.closed or self.active is not None:
            raise ProtocolError('BUSY', 'arm already has an active transaction')
        if self.ledger.cursor >= 2:
            raise ProtocolError('SEQUENCE_COMPLETE', 'blocks 1 and 2 completed; replenish then explicitly reset sequence')
        if not self.local.get('healthy') or not self.local.get('ready'):
            raise ProtocolError('ARM_NOT_READY', 'arm must be healthy, idle and verified at Home with an empty gripper')
        self._authorization(request, wait_allowed=True)
        record = self.ledger.accept(request, self.signature(request))
        self.active = dict(record, robot_boot=self.robot.boot, robot_epoch=self.robot.epoch,
                           task_boot=self.task.boot, abort=None, cancel_sent=False)
        self.last_tx = record['tx']
        self.feedback(request, 'accepted', False, 'OK', record['message'], record)
        return record

    def handle(self, request):
        """Return ('execute'|'cancel', record) for transport to act on after releasing the lock."""
        with self.lock:
            if self.closed: return None
            try:
                self._validate(request)
                prior = self.ledger.request(request['request_id'])
                if prior and prior['signature'] != digest(request):
                    raise ProtocolError('REQUEST_ID_CONFLICT', 'request_id reused with different contents')
                record = self.ledger.transaction(request['transaction_id'])
                if request['operation'] == 'execute':
                    if record:
                        if record['signature'] != self.signature(request):
                            raise ProtocolError('TRANSACTION_CONFLICT', 'transaction_id reused with different task parameters')
                        self.ledger.remember_request(request)
                        self.replay(request, record)
                        return None
                    if self.pending:
                        first = self.pending['request']
                        if request['transaction_id'] != first['transaction_id']:
                            raise ProtocolError('BUSY', 'waiting for another request reference')
                        if self.signature(request) != self.signature(first):
                            raise ProtocolError('TRANSACTION_CONFLICT', 'pending transaction parameters differ')
                        if request['request_id'] != first['request_id']:
                            raise ProtocolError('REFERENCE_PENDING', 'original transaction is waiting for robot_state_ref')
                        return None
                    try:
                        return 'execute', self._admit(request)
                    except ProtocolError as error:
                        if error.code == 'WAIT_REFERENCE':
                            self.pending = dict(request=dict(request), deadline=min(request['expires_at'], self.clock()+self.reference_wait))
                            return None
                        raise
                if not record:
                    raise ProtocolError('UNKNOWN_TRANSACTION', 'transaction was never accepted')
                original = json.loads(record['request'])
                if request['run_id'] != original['run_id'] or any(
                        key in request and request[key] != original[key] for key in ('mission_id', 'waypoint_id')):
                    raise ProtocolError('TRANSACTION_CONFLICT', 'transaction ownership fields differ')
                target = self.ledger.request(request['target_request_id'])
                if not target or target['tx'] != record['tx'] or target['operation'] != 'execute':
                    raise ProtocolError('TARGET_MISMATCH', 'target_request_id does not identify this execute transaction')
                if not prior and not request['issued_at'] <= self.clock() <= request['expires_at']:
                    raise ProtocolError('REQUEST_EXPIRED', 'request is expired or issued in the future')
                self.ledger.remember_request(request)
                if request['operation'] == 'query':
                    self.replay(request, record)
                    return None
                if prior:
                    self.feedback(request, 'succeeded', False, 'OK', 'Cancel request already processed', record)
                    return None
                if self.active and self.active['tx'] == record['tx']:
                    self.active['abort'] = self.active['abort'] or ('CANCEL_REQUESTED', 'cancel requested by M20/operator')
                    self.feedback(request, 'succeeded', False, 'OK', 'Cancel processed; original task and arm safety remain separate', record)
                    return 'cancel', record
                self.feedback(request, 'succeeded', False, 'ALREADY_TERMINAL', 'No active matching job; no motion/release command sent', record)
            except (ProtocolError, ValueError, TypeError, KeyError) as error:
                self.feedback(request, 'failed', False, getattr(error, 'code', 'INVALID_REQUEST'), str(error))
        return None

    def _watchdog(self):
        if not self.active or self.active['abort']:
            return
        request = json.loads(self.active['request'])
        try:
            self._authorization(request)
            if self.task.boot != self.active['task_boot'] or self.robot.epoch != self.active['robot_epoch']:
                raise ProtocolError('AUTHORIZATION_CHANGED', 'task/robot session or robot safety continuity changed')
            if not self.local.get('healthy'):
                raise ProtocolError('ARM_UNHEALTHY', 'arm feedback or hardware health was lost')
        except ProtocolError as error:
            self.active['abort'] = (error.code, str(error))

    def guard(self, tx):
        with self.lock:
            self._watchdog()
            if self.closed or not self.active or self.active['tx'] != tx:
                raise Cancelled('M20 transaction no longer active')
            if self.active['abort']:
                raise Cancelled('M20 '+self.active['abort'][0]+': '+self.active['abort'][1])

    def tick(self, local):
        with self.lock:
            if self.closed: return None
            self.local = dict(local)
            self._watchdog()
            if self.active and self.active['abort'] and not self.active['cancel_sent']:
                self.active['cancel_sent'] = True
                return 'cancel', dict(self.active)
            if self.pending:
                pending = self.pending
                self.pending = None
                if self.clock() > pending['deadline']:
                    self.feedback(pending['request'], 'failed', False, 'REFERENCE_TIMEOUT', 'robot_state_ref wait expired')
                    return None
                try:
                    return 'execute', self._admit(pending['request'])
                except ProtocolError as error:
                    if error.code == 'WAIT_REFERENCE' and self.clock() < pending['deadline']:
                        self.pending = pending
                    else:
                        self.feedback(pending['request'], 'failed', False,
                                      'REFERENCE_TIMEOUT' if error.code == 'WAIT_REFERENCE' else error.code, str(error))
        return None

    def progress(self, tx, stage, detail):
        with self.lock:
            if self.closed or not self.active or self.active['tx'] != tx: return
            self.ledger.progress(tx, stage)
            record = self.ledger.transaction(tx)
            self.feedback(json.loads(record['request']), 'running', False, 'OK', detail or stage, record)

    def finish(self, tx, success, cancelled, reason, stage):
        with self.lock:
            if self.closed or not self.active or self.active['tx'] != tx: return
            self._watchdog()
            abort = self.active['abort']
            success = bool(success and not abort)
            status = 'succeeded' if success else 'cancelled' if cancelled and (not abort or abort[0] == 'CANCEL_REQUESTED') else 'failed'
            code = 'OK' if success else abort[0] if abort else 'CANCELLED' if cancelled else 'ARM_TASK_FAILED'
            message = reason if not abort else abort[1]+'; '+reason
            self.ledger.finish(tx, status, success, code, message, stage)
            record = self.ledger.transaction(tx)
            self.feedback(json.loads(record['request']), status, success, code, message, record)
            self.active = None

    def status(self):
        with self.lock:
            self.seq += 1
            idle = self.active is None and self.pending is None and not self.closed
            cursor = self.cursor_at_close if self.closed else self.ledger.cursor
            return dict(schema_version=1, boot_id=self.boot_id, seq=self.seq, sample_stamp=self.clock(),
                healthy=bool(self.local.get('healthy') and not self.closed),
                ready=bool(idle and self.local.get('ready') and cursor < 2),
                transaction_id=self.last_tx, active_request_id=(json.loads(self.active['request'])['request_id'] if self.active else None),
                arm_safe_to_move=bool(idle and self.local.get('arm_safe_to_move')),
                next_grasp=(1, 2)[cursor] if cursor < 2 else None)

    def shutdown(self):
        with self.lock:
            self.closed = True
            self.cursor_at_close = self.ledger.cursor
            if self.active:
                self.ledger.finish(self.active['tx'], 'unknown', False, 'RESULT_UNKNOWN', 'Arm bridge shut down during an accepted task')
