"""External program must continue only after the correlated full-transfer result."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import ros_grasp_example as example


class ExternalClientTests(unittest.TestCase):
    def result(self, **changes):
        fields = dict(success=True, planning_success=True, execution_success=True,
                      grasp_verified=True, stage='DONE', reason='completed')
        fields.update(changes)
        return SimpleNamespace(**fields)

    def call(self, client, grasp=1):
        with patch.object(example.actionlib, 'SimpleActionClient', return_value=client) as factory, \
                patch.object(example.rospy, 'is_shutdown', return_value=False):
            result = example.execute_grasp(grasp)
        return factory, result

    def client(self, result=None, state=3):
        client = Mock()
        client.wait_for_result.return_value = True
        client.get_state.return_value = state
        client.get_result.return_value = result if result is not None else self.result()
        return client

    def test_sends_correct_block_and_requires_full_execution(self):
        for grasp, endpoint in ((1, '/fixed_grasp/task'), (2, '/fixed_grasp/task_2')):
            with self.subTest(grasp=grasp):
                client = self.client()
                factory, result = self.call(client, grasp)
                self.assertEqual(factory.call_args.args[0], endpoint)
                goal = client.send_goal.call_args.args[0]
                self.assertEqual(goal.mode, 1); self.assertTrue(goal.full_transfer)
                self.assertTrue(result.execution_success)

    def test_failed_rejected_cancelled_or_incomplete_result_does_not_return_success(self):
        for state, result in ((4, self.result()), (5, self.result()), (2, self.result()),
                              (3, self.result(execution_success=False)), (3, self.result(success=False)),
                              (3, self.result(grasp_verified=False)), (3, self.result(stage='Lift'))):
            with self.subTest(state=state, result=result):
                with self.assertRaises(RuntimeError): self.call(self.client(result, state))

    def test_interruption_cancels_own_goal(self):
        client = self.client()
        client.wait_for_result.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt): self.call(client)
        client.cancel_goal.assert_called_once()

    def test_unavailable_server_does_not_send_goal(self):
        client = self.client(); client.wait_for_server.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'unavailable'): self.call(client)
        client.send_goal.assert_not_called()


if __name__ == '__main__':
    unittest.main()
