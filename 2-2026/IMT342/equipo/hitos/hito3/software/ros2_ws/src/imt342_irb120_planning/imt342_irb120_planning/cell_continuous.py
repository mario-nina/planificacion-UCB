"""Continuous flange-line reference for the optional RAPID MoveL revision.

Dense IK knots are analysis data, NOT RAPID waypoints. A single MoveL is
requested per straight pass. The reference uses a scalar LSPB in Cartesian
progress and a differentiable joint-space approximation of the IK branch.
"""
from dataclasses import dataclass, replace
from itertools import accumulate
from math import ceil, dist, sqrt
from typing import ClassVar

from imt342_irb120_kinematics.analysis import pose_residual
from . import Segment, Sample, Sequence, _sample_times, AUTO_REFERENCE_VELOCITIES, DEFAULT_ACCELERATION
from .cell_task import plan_cell_task, MIN_SCALED_SIGMA
from .task import TaskInputError


@dataclass(frozen=True)
class LinearLeg:
    nodes: tuple
    slopes: tuple
    duration: float
    peak_v: tuple
    peak_a: tuple
    profile: ClassVar[str] = 'lspb'
    blend_fraction: ClassVar[float] = .25
    linear_cartesian: ClassVar[bool] = True

    @property
    def start(self): return self.nodes[0]

    @property
    def goal(self): return self.nodes[-1]

    def sample(self, t):
        scalar = Segment((0.,)*6, (1.,)*6, self.duration, self.blend_fraction).sample(t)
        s, ds, dds = scalar.positions[0], scalar.velocities[0], scalar.accelerations[0]
        n = len(self.nodes)-1
        i = min(int(s*n), n-1)
        u = min(1., max(0., s*n-i))
        q0,q1 = self.nodes[i:i+2]
        m0,m1 = self.slopes[i:i+2]
        q,v,a = [],[],[]
        for p,r,m,k in zip(q0,q1,m0,m1):
            b=m/n; c=3*(r-p)-2*b-k/n; d=2*(p-r)+b+k/n
            q.append(p+u*(b+u*(c+u*d)))
            first=n*(b+u*(2*c+3*u*d)); second=n*n*(2*c+6*u*d)
            v.append(first*ds); a.append(second*ds*ds+first*dds)
        if t == 0.: q=list(self.start)
        if t == self.duration: q=list(self.goal)
        return Sample(t,tuple(q),tuple(v),tuple(a))

    def collision_chords(self):
        n=len(self.nodes)-1
        for p,r,m,k in zip(self.nodes,self.nodes[1:],self.slopes,self.slopes[1:]):
            margin=max(max(abs(a+b/(3*n)-(2*a+c)/3),abs(c-d/(3*n)-(a+2*c)/3))
                       for a,c,b,d in zip(p,r,m,k))
            yield p,r,margin

    def samples(self, period=.02):
        return tuple(self.sample(t) for t in _sample_times(self.duration,period))


def linear_leg(points, cell, start, goal, wall, velocity):
    model=points.flange
    a,b=model.fk(start),model.fk(goal)
    xyz=lambda m: tuple(row[3] for row in m[:3])
    length=dist(xyz(a),xyz(b))
    if length <= 1e-6 or pose_residual(a,b)['orientation_error_rad'] > 1e-6:
        raise TaskInputError('MoveL requiere extremos diferentes y orientación fija de brida')
    n=max(20,ceil(length/.005))
    nodes=[tuple(start)]
    expected=tuple(goal)
    def desired(s):
        return tuple(tuple(row[:3])+(row[3]+s*(b[i][3]-row[3]),) for i,row in enumerate(a[:3]))+(a[3],)
    for i in range(1,n):
        candidates=model.ik_pieper(desired(i/n),seed=nodes[-1])
        candidate=min((c for c in candidates if not c.singular), key=lambda c:dist(c.q,nodes[-1]),default=None)
        if candidate is None or max(abs(x-y) for x,y in zip(candidate.q,nodes[-1])) > .10:
            raise TaskInputError('MoveL: rama IK discontinua')
        nodes.append(tuple(candidate.q))
    if max(abs(x-y) for x,y in zip(expected,nodes[-1])) > .10:
        raise TaskInputError('MoveL: rama final no coincide con la previsualización')
    nodes.append(expected)
    slopes=[tuple(n*(b-a) for a,b in zip(nodes[0],nodes[1]))]
    slopes.extend(tuple(n*(b-a)/2 for a,b in zip(nodes[i-1],nodes[i+1])) for i in range(1,n))
    slopes.append(tuple(n*(b-a) for a,b in zip(nodes[-2],nodes[-1])))
    max_d=[0.]*6; max_dd=[0.]*6
    configs=0
    for i,(p,r) in enumerate(zip(nodes,nodes[1:])):
        # Cubic Bezier convex-hull bounds: deviation from the linear chord,
        # joint limits and first/second derivatives with respect to progress.
        margin=0.
        for j,(a0,a1,m0,m1) in enumerate(zip(p,r,slopes[i],slopes[i+1])):
            c0=a0; c1=a0+m0/(3*n); c2=a1-m1/(3*n); c3=a1
            lo,hi=model.limits[j]
            if min(c0,c1,c2,c3)<lo or max(c0,c1,c2,c3)>hi:
                raise TaskInputError('MoveL: envolvente de interpolación fuera de límite')
            margin=max(margin,abs(c1-(2*a0+a1)/3),abs(c2-(a0+2*a1)/3))
            d0,d1,d2=(3*n*(c1-c0),3*n*(c2-c1),3*n*(c3-c2))
            max_d[j]=max(max_d[j],abs(d0),abs(d1),abs(d2))
            max_dd[j]=max(max_dd[j],abs(2*n*(d1-d0)),abs(2*n*(d2-d1)))
        checked=cell.check_segment(p,r,tool_offset=points.tool_offset,
                                   joint_margin_rad=cell.joint_margin_rad+margin)
        configs+=checked.configurations_checked
        if not checked.ok: raise TaskInputError('MoveL: '+checked.reason)
    f=.25; sv=1/(1-f); sa=sv/f
    v=tuple(d*sv for d in max_d)
    acc=tuple(dd*sv*sv+d*sa for dd,d in zip(max_dd,max_d))
    # Cartesian speed ceiling is a reference, not a controller speed override.
    t=max(.1,length*sv/.10,*(x/y for x,y in zip(v,velocity)),
          *(sqrt(x/y) for x,y in zip(acc,DEFAULT_ACCELERATION)))
    if t>30: raise TaskInputError('MoveL: duración superior a 30 s')
    leg=LinearLeg(tuple(nodes),tuple(slopes),t,tuple(x/t for x in v),tuple(x/t**2 for x in acc))
    # Evaluate the spline against the intended line, including midpoint of
    # every IK interval. This is a model check, not an ABB servo guarantee.
    dev=[]; clear=[]; sigma=[]; path=[]; ideal=[]; rotations=[]
    # Evaluate position at uniform progress by direct cubic formula via inverse LSPB.
    for i in range(2*n+1):
        s=i/(2*n); cut=.5*sa*f*f
        u=sqrt(2*s/sa) if s<cut else 1-sqrt(2*(1-s)/sa) if s>1-cut else f+(s-cut)/sv
        q=leg.sample(min(t,max(0.,u*t))).positions
        actual=model.fk(q); residual=pose_residual(desired(s),actual)
        ideal.append(xyz(desired(s))); rotations.append(residual['orientation_error_rad'])
        dev.append(residual['position_error_m']); clear.append(cell.wall_distance(wall,xyz(actual)))
        sigma.append(model.singular_values(q)[-1]); path.append(xyz(actual))
        if residual['orientation_error_rad']>.01:
            raise TaskInputError('MoveL: orientación fuera de cota')
    if max(dev)>.001 or min(sigma)<MIN_SCALED_SIGMA or any(abs(x-.1)>.01 for x in clear):
        raise TaskInputError('MoveL: referencia lineal, separación o singularidad fuera de cota')
    return leg,dict(deviation=max(dev),clearance_min=min(clear),clearance_max=max(clear),
                    sigma=min(sigma),configurations=configs,path=path,ideal=ideal,orientation=max(rotations))


def plan_continuous_wall(points,home,task,velocity,cell,**kwargs):
    if task.get('kind')!='cell_wall' or task.get('point','flange')!='flange':
        raise TaskInputError('MoveL continuo disponible para recorrido longitudinal de brida')
    if task.get('profile') not in (None,'lspb'):
        raise TaskInputError('El recorrido lineal continuo usa referencia LSPB cartesiana')
    old,e=plan_cell_task(points,home,dict(task,profile='lspb'),velocity,cell,**kwargs)
    count=len(old.order); seg=old.trajectory.segments[:count]
    last_out=max(i for i,name in enumerate(old.order) if name.startswith('AVANCE_'))
    last_return=max(i for i,name in enumerate(old.order) if name.startswith('REGRESO_'))
    forward,metrics=linear_leg(points,cell,seg[1].goal,seg[last_out].goal,e['wall'],velocity)
    # Same exact path in reverse, with reversed derivative signs.
    reverse=replace(forward,nodes=forward.nodes[::-1],slopes=tuple(tuple(-v for v in row) for row in forward.slopes[::-1]))
    indices=(0,1,last_out,last_return,count-2,count-1)
    if reverse.goal!=seg[last_return].goal: raise TaskInputError('Retorno lineal no coincide con INICIO')
    legs=(seg[0],seg[1],forward,reverse,seg[-2],seg[-1])
    seq=legs*old.cycles
    trajectory=Sequence(seq,tuple(accumulate(s.duration for s in seq)))
    order=('APROX','INICIO','IDA_CONTINUA','VUELTA_CONTINUA','SALIDA','HOME')
    records=[]; checks=[]
    for name,i,leg in zip(order,indices,legs):
        row=dict(old.waypoints[i],name=name,first_duration_s=leg.duration,first_lspb_s=leg.duration)
        records.append(row)
        checks.append(dict(old.leg_checks[i],waypoint=name,duration_s=leg.duration,lspb_s=leg.duration,
                           max_delta_rad=max(abs(a-b) for a,b in zip(leg.start,leg.goal)),
                           min_sigma_scaled=metrics["sigma"] if getattr(leg,"linear_cartesian",False) else old.leg_checks[i]["min_sigma_scaled"]))
    result=replace(old,trajectory=trajectory,order=order,waypoints=tuple(records),leg_checks=tuple(checks))
    e.update(execution_style='continuous_movel',waypoint_names=order,command_segments=6,
             total_command_segments=6*old.cycles,reference_xyz_m=tuple(metrics['ideal'])+tuple(metrics['ideal'][-2::-1]),
             planned_xyz_m=tuple(tuple(row[3] for row in points.flange.fk(s.positions)[:3]) for s in trajectory.samples(.05)),
             max_tcp_deviation_m=metrics['deviation'],wall_clearance_min_m=metrics['clearance_min'],
             wall_clearance_max_m=metrics['clearance_max'],min_sigma_scaled=metrics['sigma'],
             max_tcp_orientation_error_rad=metrics['orientation'],
             sampled_configurations=len(metrics['path']),collision_configurations=e['collision_configurations']+metrics['configurations'])
    return result,e
