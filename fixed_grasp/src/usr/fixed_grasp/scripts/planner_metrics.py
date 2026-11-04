"""Monotonic, buffered planning statistics. No logging or ROS operations here."""
from collections import Counter,defaultdict
from contextlib import contextmanager
from functools import wraps
import time
import numpy as np


class PlanningTimeout(RuntimeError): pass
class IKFailure(RuntimeError): pass
class CollisionFailure(RuntimeError): pass
class JointConstraintFailure(RuntimeError): pass
class NoFeasiblePlan(RuntimeError): pass


def failure_kind(error):
    if isinstance(error,NoFeasiblePlan):return 'no_feasible_solution'
    if isinstance(error,PlanningTimeout) or 'time budget exceeded' in str(error):return 'budget_timeout'
    if isinstance(error,CollisionFailure) or 'collision' in str(error) or 'clearance' in str(error):return 'collision'
    if isinstance(error,IKFailure) or 'IK did not converge' in str(error):return 'ik_unreachable'
    if isinstance(error,JointConstraintFailure) or 'joint travel' in str(error) or 'branch jump' in str(error):return 'joint_constraint'
    return 'planning_failure'


class PlannerMetrics:
    def __init__(self):
        self.samples=defaultdict(list);self.counts=Counter();self.paths=[];self.failures=Counter()
        self.started=time.monotonic();self.first_solution_sec=None
        self.error_values=defaultdict(list)
    def begin_task(self): self.started=time.monotonic()
    @contextmanager
    def measure(self,name):
        start=time.monotonic()
        try:yield
        finally:self.samples[name].append(time.monotonic()-start)
    def solution(self):
        if self.first_solution_sec is None:
            self.first_solution_sec=time.monotonic()-self.started
            self.search_first_solution_sec=time.monotonic()-getattr(self,'search_started',self.started)
    def snapshot(self,outcome):
        stats={name:dict(count=len(values),total_sec=float(sum(values)),
            mean_sec=float(np.mean(values)),p95_sec=float(np.percentile(values,95)))
            for name,values in self.samples.items() if values}
        return dict(clock='time.monotonic',outcome=outcome,counts=dict(self.counts),timings=stats,
            paths=self.paths,failures=dict(self.failures),
            first_complete_solution_sec=self.first_solution_sec,
            search_first_complete_solution_sec=getattr(self,'search_first_solution_sec',None),
            final_return_sec=time.monotonic()-self.started,
            errors={name:dict(max=float(max(v)),p95=float(np.percentile(v,95))) for name,v in self.error_values.items() if v},
            timing_note='nested timings overlap; final_return excludes visual acquisition; not vision FPS')


def profiled(name):
    def decorate(function):
        @wraps(function)
        def wrapper(self,*args,**kwargs):
            metrics=getattr(self,'metrics',None)
            if not isinstance(metrics,PlannerMetrics):return function(self,*args,**kwargs)
            before=metrics.counts.copy();start=time.monotonic();result=None;error=None
            stage=getattr(metrics,'current_stage','unspecified')
            metrics.counts[name+'_calls']+=1
            try:
                result=function(self,*args,**kwargs);return result
            except Exception as exc:
                error=exc;metrics.counts[name+'_failures']+=1
                if name=='ik' and isinstance(exc,IKFailure):metrics.counts['ik_numeric_failures']+=1
                raise
            finally:
                elapsed=time.monotonic()-start;metrics.samples[name].append(elapsed)
                if name in ('cartesian_fine','cartesian_coarse'):
                    a=args[1] if len(args)>1 else kwargs.get('start_tcp')
                    b=args[2] if len(args)>2 else kwargs.get('end_tcp')
                    metrics.samples['path_stage_'+stage].append(elapsed)
                    metrics.paths.append(dict(kind=name,stage=stage,duration_sec=elapsed,
                        requested_distance_m=float(np.linalg.norm(np.asarray(b)[:3,3]-np.asarray(a)[:3,3])) if a is not None and b is not None else None,
                        ik_calls=metrics.counts['ik_calls']-before['ik_calls'],
                        ik_failures=metrics.counts['ik_numeric_failures']-before['ik_numeric_failures'],
                        ik_query_failures=metrics.counts['ik_failures']-before['ik_failures'],
                        waypoint_count=len(result.joint_trajectory.points) if name=='cartesian_fine' and result is not None else metrics.counts['cartesian_generated_points']-before['cartesian_generated_points'],
                        success=error is None,failure=None if error is None else failure_kind(error)))
        return wrapper
    return decorate
