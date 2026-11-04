"""M20 protocol fault paths and durable place order, without ROS or hardware."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from core import Cancelled
from m20_ledger import Ledger
from m20_protocol import Coordinator, parse_json


class M20ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'transactions.sqlite3'
        self.ledger = Ledger(self.path)
        self.addCleanup(lambda: self.ledger.close())
        self.now = 100.
        self.engine = Coordinator(self.ledger, lambda: self.now)
        self.local = dict(healthy=True, ready=True, arm_safe_to_move=True)
        self.engine.tick(self.local)
        self.robot_seq = self.task_seq = 0
        self.authorize('tx-1')

    def robot(self, **changes):
        self.robot_seq += 1
        data = dict(schema_version=1, boot_id='dog-boot', seq=self.robot_seq, sample_stamp=self.now,
                    healthy=True, stationary=True, posture='crouched', basic_age_s=.2, motion_age_s=.05)
        data.update(changes)
        self.engine.ingest('robot', data)
        return data

    def task(self, tx='tx-1', **changes):
        self.task_seq += 1
        data = dict(schema_version=1, boot_id='task-boot', seq=self.task_seq, sample_stamp=self.now,
                    state='ARM_EXECUTING', run_id='run-1', transaction_id=tx)
        data.update(changes)
        self.engine.ingest('task', data)
        return data

    def authorize(self, tx):
        self.robot(); self.task(tx)

    def request(self, tx='tx-1', request_id='req-1', **changes):
        data = dict(schema_version=1, request_id=request_id, source='task', run_id='run-1', mission_id='mission-1',
                    waypoint_id='point-1', transaction_id=tx, operation='execute', task_type='place',
                    issued_at=self.now, expires_at=self.now+10.,
                    robot_state_ref=dict(boot_id='dog-boot', seq=self.robot_seq), payload={})
        data.update(changes)
        return data

    def terminal(self, success=True, cancelled=False):
        self.engine.finish('tx-1', success, cancelled, 'completed' if success else 'stopped', 'DONE' if success else 'Lift')

    def last_feedback(self):
        return self.engine.drain()[-1]

    def test_two_distinct_place_transactions_select_1_then_2_without_automatic_second_dispatch(self):
        command = self.engine.handle(self.request())
        self.assertEqual(command[0], 'execute'); self.assertEqual(command[1]['block'], 1)
        self.assertEqual(self.ledger.cursor, 0)
        self.assertIsNone(self.engine.tick(self.local))
        self.terminal()
        self.assertEqual(self.ledger.cursor, 1)
        self.assertIsNone(self.engine.active)
        self.now += .1; self.authorize('tx-2')
        command = self.engine.handle(self.request('tx-2', 'req-2'))
        self.assertEqual(command[1]['block'], 2)
        self.engine.finish('tx-2', True, False, 'completed', 'DONE')
        self.assertEqual(self.ledger.cursor, 2)
        self.assertFalse(self.engine.status()['ready'])
        self.now += .1; self.authorize('tx-3')
        self.assertIsNone(self.engine.handle(self.request('tx-3', 'req-3')))
        self.assertEqual(self.last_feedback()['error_code'], 'SEQUENCE_COMPLETE')

    def test_duplicate_completed_transaction_replays_result_even_after_acceptance_expiry(self):
        request = self.request(); self.engine.handle(request); self.terminal()
        self.now += 20.
        for repeated in (request, dict(request, request_id='alias-1')):
            self.assertIsNone(self.engine.handle(repeated))
            result = self.last_feedback()
            self.assertEqual(result['status'], 'succeeded'); self.assertTrue(result['task_success'])
            self.assertEqual(result['request_id'], repeated['request_id'])
            self.assertEqual(self.ledger.cursor, 1)

    def test_conflicting_payload_or_reused_request_id_never_dispatches(self):
        request = self.request(); self.engine.handle(request)
        self.assertIsNone(self.engine.handle(dict(request, request_id='req-new', payload={'object_id': 'different'})))
        self.assertEqual(self.last_feedback()['error_code'], 'TRANSACTION_CONFLICT')
        self.assertIsNone(self.engine.handle(dict(request, transaction_id='other-tx')))
        self.assertEqual(self.last_feedback()['error_code'], 'REQUEST_ID_CONFLICT')

    def test_grasp_wrong_schema_expired_future_and_unready_requests_do_not_consume_order(self):
        for update, code in (({'task_type': 'grasp'}, 'UNSUPPORTED_TASK'), ({'schema_version': 2}, 'INVALID_SCHEMA'),
                             ({'expires_at': 99., 'issued_at': 90.}, 'REQUEST_EXPIRED'),
                             ({'issued_at': 101.}, 'REQUEST_EXPIRED'), ({'payload': []}, 'INVALID_REQUEST')):
            with self.subTest(update=update):
                self.assertIsNone(self.engine.handle(self.request(**update)))
                self.assertEqual(self.last_feedback()['error_code'], code)
                self.assertEqual(self.ledger.cursor, 0)
        self.engine.tick(dict(self.local, ready=False))
        self.assertIsNone(self.engine.handle(self.request()))
        self.assertEqual(self.last_feedback()['error_code'], 'ARM_NOT_READY')

    def test_reference_can_arrive_later_but_wait_cannot_extend_request_or_short_deadline(self):
        request = self.request(robot_state_ref=dict(boot_id='dog-boot', seq=2))
        self.assertIsNone(self.engine.handle(request)); self.assertIsNotNone(self.engine.pending)
        self.now += .1; self.robot()
        command = self.engine.tick(self.local)
        self.assertEqual(command[1]['block'], 1)
        self.terminal(False)
        # A new request waits for the next not-yet-observed reference.
        self.now += .1; self.authorize('tx-2')
        request = self.request('tx-2', 'req-2', robot_state_ref=dict(boot_id='dog-boot', seq=self.robot_seq+1))
        self.engine.handle(request)
        self.now += .4; self.robot(); self.task('tx-2')
        self.assertIsNone(self.engine.tick(self.local))
        self.assertEqual(self.last_feedback()['error_code'], 'REFERENCE_TIMEOUT')

    def test_failed_or_cancelled_job_does_not_advance_to_block_2(self):
        self.engine.handle(self.request()); self.terminal(False, True)
        self.assertEqual(self.ledger.cursor, 0)
        self.now += .1; self.authorize('tx-2')
        command = self.engine.handle(self.request('tx-2', 'req-2'))
        self.assertEqual(command[1]['block'], 1)

    def test_robot_posture_change_then_recovery_does_not_revalidate_old_reference_or_running_job(self):
        old = self.request()
        self.now += .05; self.robot(posture='standing')
        self.now += .05; self.robot()
        self.assertIsNone(self.engine.handle(old))
        self.assertEqual(self.last_feedback()['error_code'], 'REFERENCE_INVALIDATED')
        self.engine.handle(self.request())
        self.now += .05; self.robot(stationary=False)
        self.now += .05; self.robot()
        with self.assertRaises(Cancelled): self.engine.guard('tx-1')
        self.terminal(True)
        self.assertFalse(self.last_feedback()['task_success'])
        self.assertEqual(self.ledger.cursor, 0)

    def test_lost_robot_or_task_heartbeat_and_boot_change_stop_an_active_job(self):
        for cause in ('robot', 'task', 'robot_boot', 'task_boot', 'task_phase', 'local_health'):
            with self.subTest(cause=cause):
                # End any prior subcase without consuming the placement cursor.
                if self.engine.active: self.terminal(False)
                self.now += 1.; self.authorize('tx-'+cause)
                request = self.request('tx-'+cause, 'req-'+cause)
                command = self.engine.handle(request)
                self.assertIsNotNone(command)
                if cause == 'robot': self.now += .51
                elif cause == 'task':
                    self.now += .51; self.robot()
                elif cause == 'robot_boot':
                    self.now += .01; self.robot(boot_id='new-dog-'+cause)
                elif cause == 'task_boot':
                    self.now += .01; self.task(request['transaction_id'], boot_id='new-task-'+cause)
                elif cause == 'task_phase':
                    self.now += .01; self.task(request['transaction_id'], state='PAUSED')
                else: self.engine.tick(dict(self.local, healthy=False))
                with self.assertRaises(Cancelled): self.engine.guard(request['transaction_id'])
                self.engine.finish(request['transaction_id'], False, True, 'stopped', 'Lift')
                self.engine.tick(self.local)
                # New boot IDs remain current; do not replay a retired status boot.
                self.engine.robot.seen_boots.clear(); self.engine.robot.boot = None
                self.engine.task.seen_boots.clear(); self.engine.task.boot = None

    def test_replayed_status_cannot_refresh_safety_and_raw_age_can_invalidate_health(self):
        self.engine.handle(self.request())
        repeated = dict(self.engine.robot.current)
        self.now += .1; self.engine.ingest('robot', repeated)
        with self.assertRaises(Cancelled): self.engine.guard('tx-1')
        self.terminal(False)
        self.now += .1; self.authorize('tx-2'); self.now += .01; self.robot(basic_age_s=2.)
        self.assertIsNone(self.engine.handle(self.request('tx-2', 'req-2')))
        self.assertEqual(self.last_feedback()['error_code'], 'ROBOT_NOT_CROUCHED')

    def test_cancel_ownership_ack_does_not_claim_job_success_or_stowed_arm(self):
        request = self.request(); self.engine.handle(request)
        cancel = dict(schema_version=1, request_id='cancel-1', run_id='run-1', transaction_id='tx-1',
                      operation='cancel', target_request_id='req-1', issued_at=100., expires_at=110.)
        self.assertEqual(self.engine.handle(cancel)[0], 'cancel')
        ack = self.last_feedback()
        self.assertEqual(ack['status'], 'succeeded'); self.assertFalse(ack['task_success'])
        self.assertFalse(ack['arm_safe_to_move']); self.assertEqual(ack['target_request_id'], 'req-1')
        self.assertIsNone(self.engine.handle(cancel))
        with self.assertRaises(Cancelled): self.engine.guard('tx-1')
        self.engine.finish('tx-1', False, True, 'cancelled', 'Lift')
        self.assertEqual(self.last_feedback()['status'], 'cancelled')
        self.assertEqual(self.ledger.cursor, 0)

    def test_query_returns_durable_result_and_matched_identity_without_dispatch(self):
        self.engine.handle(self.request()); self.terminal()
        query = dict(schema_version=1, request_id='query-1', run_id='run-1', transaction_id='tx-1',
                     operation='query', target_request_id='req-1', issued_at=100., expires_at=110.)
        self.assertIsNone(self.engine.handle(query))
        response = self.last_feedback()
        self.assertEqual(response['operation'], 'query'); self.assertTrue(response['task_success'])
        self.assertEqual(response['mission_id'], 'mission-1'); self.assertEqual(response['grasp'], 1)
        bad = dict(query, request_id='query-2', target_request_id='unrelated')
        self.assertIsNone(self.engine.handle(bad)); self.assertEqual(self.last_feedback()['error_code'], 'TARGET_MISMATCH')

    def test_task_success_and_actual_stowage_are_separate_and_last_transaction_is_retained(self):
        self.engine.handle(self.request()); self.engine.tick(dict(self.local, ready=False, arm_safe_to_move=False))
        self.terminal()
        self.assertTrue(self.last_feedback()['task_success'])
        status = self.engine.status()
        self.assertFalse(status['arm_safe_to_move']); self.assertIsNone(status['active_request_id'])
        self.assertEqual(status['transaction_id'], 'tx-1')
        self.now += .1; self.engine.tick(self.local)
        next_status = self.engine.status()
        self.assertTrue(next_status['arm_safe_to_move']); self.assertEqual(next_status['transaction_id'], 'tx-1')
        self.assertGreater(next_status['seq'], status['seq'])
        self.assertGreater(next_status['sample_stamp'], status['sample_stamp'])
        self.assertEqual(next_status['boot_id'], status['boot_id'])

    def test_crash_recovers_unknown_and_never_replays_an_accepted_transaction(self):
        request = self.request(); self.engine.handle(request)
        self.ledger.close()
        self.ledger = Ledger(self.path)
        restarted = Coordinator(self.ledger, lambda: self.now)
        self.assertIsNone(restarted.handle(request))
        response = restarted.drain()[-1]
        self.assertEqual(response['status'], 'failed'); self.assertEqual(response['result_state'], 'unknown')
        self.assertEqual(response['error_code'], 'RESULT_UNKNOWN'); self.assertEqual(self.ledger.cursor, 0)
        self.assertNotEqual(restarted.boot_id, self.engine.boot_id)
        self.assertFalse(restarted.status()['arm_safe_to_move'])

    def test_success_order_survives_restart_and_reset_keeps_old_deduplication(self):
        request = self.request(); self.engine.handle(request); self.terminal()
        self.ledger.close(); self.ledger = Ledger(self.path)
        self.assertEqual(self.ledger.cursor, 1)
        self.ledger.reset_sequence()
        restarted = Coordinator(self.ledger, lambda: self.now)
        self.assertIsNone(restarted.handle(request))
        self.assertTrue(restarted.drain()[-1]['task_success']); self.assertEqual(self.ledger.cursor, 0)

    def test_manual_reconciliation_advances_only_after_explicit_confirmed_completion(self):
        self.engine.handle(self.request()); self.terminal(False)
        self.ledger.confirm_completed('tx-1', 'operator inspected completed placement and Home')
        self.assertEqual(self.ledger.cursor, 1)
        self.assertEqual(self.ledger.transaction('tx-1')['error_code'], 'OPERATOR_CONFIRMED')
        with self.assertRaises(ValueError): self.ledger.confirm_completed('tx-1', 'repeat')

    def test_old_unknown_result_cannot_advance_a_replenished_new_cycle(self):
        self.engine.handle(self.request()); self.terminal(False)
        self.ledger.reset_sequence()
        with self.assertRaises(ValueError):
            self.ledger.confirm_completed('tx-1', 'old cycle was completed')
        self.assertEqual(self.ledger.cursor, 0)

    def test_second_bridge_cannot_open_same_live_ledger(self):
        with self.assertRaises(RuntimeError): Ledger(self.path)

    def test_nonfinite_or_nonobject_json_is_rejected(self):
        for raw in ('[]', 'null', '{"payload":NaN}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError): parse_json(raw)


if __name__ == '__main__': unittest.main()
