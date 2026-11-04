"""ROS-independent task sequencing. Unknown holding state never triggers release."""
import threading


class Cancelled(RuntimeError):
    pass


class Runner:
    def __init__(self, backend, feedback=lambda *args: None, guard=None):
        self.b = backend
        self.feedback = feedback
        self.guard = guard
        self.cancel = threading.Event()
        self.stage = 'IDLE'
        self.holding = False
        self.planned = False
        self.executed = False
        self.grasp_verified = False

    def check(self):
        if self.cancel.is_set():
            raise Cancelled('task cancelled; no automatic release or recovery')
        if self.guard is not None:
            self.guard()

    def phase(self, name, detail=''):
        self.check()
        self.stage = name
        self.feedback(name, detail, self.holding)

    def run_home(self, mode, confirm_empty=False):
        if mode not in (0, 1):
            raise ValueError('unknown mode')
        execute = mode == 1
        options = {'home_only': True}
        if confirm_empty:
            options['confirm_empty'] = True
        self.phase('PREFLIGHT')
        self.b.preflight(execute, False, **options)
        self.phase('PLANNING')
        plans = self.b.plan(['Home'], self.check)
        self.planned = True
        self.check()
        if not execute:
            self.phase('PLANNED', 'Home planning only')
            return
        self.b.preflight(True, False, **options)
        self.phase('CHECK_EMPTY', 'operator confirmed empty' if confirm_empty else 'verify idle gripper')
        self.b.require_home_empty(confirm_empty)
        self.phase('Home')
        self.b.move('Home', plans['Home'])
        self.check()
        self.executed = True
        self.phase('DONE', 'returned Home')

    def run(self, mode, full=False):
        if mode not in (0, 1):
            raise ValueError('unknown mode')
        execute = mode == 1
        self.phase('PREFLIGHT')
        self.b.preflight(execute, full)
        names = ['Home', 'Pregrasp', 'Grasp', 'Lift']
        if full:
            names += ['Preplace', 'Place', 'Retreat', 'ReturnHome']
        self.phase('PLANNING')
        plans = self.b.plan(names, self.check)
        self.planned = True
        self.check()
        if not execute:
            self.phase('PLANNED', 'planning only; no physical grasp verified')
            return
        self.b.preflight(True, full)
        self.phase('CHECK_OPEN')
        self.b.require_open()  # Never auto-open an unknown/loaded gripper.
        for name in names:
            if name == 'Pregrasp':
                self.phase('PREPARE_GRIPPER', 'set approach opening and verify feedback')
                self.b.prepare_gripper()
            self.phase(name)
            self.b.move(name, plans[name])
            self.check()
            if name == 'Grasp':
                self.phase('CLOSE')
                self.holding = True  # Unknown after command until feedback verifies.
                self.b.close()
                self.check()
                self.b.attach()
            elif name == 'Lift':
                self.b.verify_hold()
                self.grasp_verified = True
            elif name == 'Place':
                self.phase('VERIFY_SUPPORT', 'validate configured release conditions at Place')
                self.b.verify_support()
                self.phase('OPEN')
                self.b.open()
                self.check()
                self.b.detach()
                self.holding = False
        self.executed = True
        self.phase('DONE', 'holding basket at Lift' if not full else 'placed, retreated and returned Home')
