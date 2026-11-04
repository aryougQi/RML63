import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from core import Runner, Cancelled


class Backend:
    def __init__(self): self.calls = []; self.fail = None; self.on = lambda n: None
    def call(self, n):
        self.calls.append(n)
        self.on(n)
        if n == self.fail: raise RuntimeError(n)
    def preflight(self, execute, full): self.call('preflight')
    def plan(self, names, check):
        self.call('plan'); check()
        self.planned_names = list(names)
        return dict.fromkeys(names)
    def move(self, name, plan): self.call(name)
    def require_open(self): self.call('check_open')
    def prepare_gripper(self): self.call('prepare_gripper')
    def close(self): self.call('close')
    def attach(self): self.call('attach')
    def verify_hold(self): self.call('verify_hold')
    def verify_support(self): self.call('support')
    def open(self): self.call('open')
    def detach(self): self.call('detach')
    def stop(self): self.call('stop')


class Tests(unittest.TestCase):
    def test_plan_never_moves(self):
        b = Backend(); r = Runner(b); r.run(0, True)
        self.assertEqual(b.calls, ['preflight', 'plan'])
        self.assertTrue(r.planned); self.assertFalse(r.grasp_verified); self.assertFalse(r.executed)
        self.assertEqual(b.planned_names[-2:], ['Retreat', 'ReturnHome'])

    def test_lift_holds(self):
        b = Backend(); r = Runner(b); r.run(1)
        self.assertTrue(r.holding); self.assertTrue(r.grasp_verified)
        self.assertNotIn('open', b.calls)
        self.assertLess(b.calls.index('attach'), b.calls.index('Lift'))

    def test_transfer_order(self):
        b = Backend(); r = Runner(b); r.run(1, True)
        self.assertEqual(b.calls[-7:], ['Preplace', 'Place', 'support', 'open', 'detach', 'Retreat', 'ReturnHome'])
        self.assertFalse(r.holding)

    def test_release_or_retreat_failure_prevents_return_home(self):
        for stage in ('open', 'detach', 'Retreat'):
            with self.subTest(stage=stage):
                b = Backend(); b.fail = stage; r = Runner(b)
                with self.assertRaises(RuntimeError): r.run(1, True)
                self.assertNotIn('ReturnHome', b.calls)
                self.assertFalse(r.executed)

    def test_cancel_after_retreat_prevents_return_home(self):
        b = Backend(); r = Runner(b)
        b.on = lambda name: r.cancel.set() if name == 'Retreat' else None
        with self.assertRaises(Cancelled): r.run(1, True)
        self.assertNotIn('ReturnHome', b.calls)

    def test_return_home_failure_is_not_reported_as_success(self):
        b = Backend(); b.fail = 'ReturnHome'; r = Runner(b)
        with self.assertRaises(RuntimeError): r.run(1, True)
        self.assertEqual(r.stage, 'ReturnHome')
        self.assertFalse(r.executed)
        self.assertFalse(r.holding)

    def test_prepare_before_approach(self):
        b = Backend(); Runner(b).run(1)
        self.assertLess(b.calls.index('Home'), b.calls.index('prepare_gripper'))
        self.assertLess(b.calls.index('prepare_gripper'), b.calls.index('Pregrasp'))
        self.assertLess(b.calls.index('Grasp'), b.calls.index('close'))

    def test_prepare_failure_stops_approach(self):
        b = Backend(); b.fail = 'prepare_gripper'
        with self.assertRaises(RuntimeError): Runner(b).run(1)
        self.assertNotIn('Pregrasp', b.calls)
        self.assertNotIn('close', b.calls)

    def test_failures_never_release_or_continue(self):
        for stage in ['Home', 'Pregrasp', 'Grasp', 'close', 'attach', 'Lift', 'verify_hold', 'Preplace', 'Place', 'support']:
            b = Backend(); b.fail = stage; r = Runner(b)
            with self.assertRaises(RuntimeError): r.run(1, True)
            self.assertEqual(b.calls[-1], stage)
            self.assertNotIn('open', b.calls)
            self.assertFalse(r.executed)

    def test_cancel_every_boundary(self):
        for stage in ['preflight', 'plan', 'check_open', 'Home', 'Pregrasp', 'Grasp', 'close', 'Lift', 'verify_hold', 'Place', 'support']:
            b = Backend(); r = Runner(b)
            b.on = lambda n: r.cancel.set() if n == stage else None
            with self.assertRaises(Cancelled): r.run(1, True)
            self.assertNotIn('open', b.calls)

    def test_bad_mode(self):
        b = Backend()
        with self.assertRaises(ValueError): Runner(b).run(2)
        self.assertEqual(b.calls, [])

    def test_planning_failure_has_no_execution(self):
        b = Backend(); b.fail = 'plan'; r = Runner(b)
        with self.assertRaises(RuntimeError): r.run(1)
        self.assertFalse(r.planned); self.assertFalse(r.executed)
        self.assertNotIn('Home', b.calls)


if __name__ == '__main__': unittest.main()
