"""Equilateral 100 mm flange triangle, using the existing joint order protocol."""
from dataclasses import replace
from math import ceil, dist, sqrt

from . import plan, _joint_vector, planner_for_profile
from .task import TaskInputError, plan_task
from .cell_task import (_ik_routes, _Destination, _check_wall_collision, _xyz,
                        _distance_to_segment, MIN_SCALED_SIGMA)
from imt342_irb120_kinematics.analysis import pose_residual

SIDE_M = .100
MAX_DEVIATION_M = .006


def plan_air_triangle(points, start, task, velocity, cell, *, segment_planner=plan, max_duration=None):
    if not isinstance(task, dict) or task.get('kind') != 'air_triangle':
        raise TaskInputError('Tipo de figura no reconocido')
    if task.get('point', 'flange') != 'flange':
        raise TaskInputError('El triángulo usa la brida como referencia')
    cycles = task.get('cycles')
    if type(cycles) is not int or not 1 <= cycles <= 8:
        raise TaskInputError('El triángulo requiere de 1 a 8 ciclos')
    if any(key in task for key in ('waypoints', 'order', 'duration_s')):
        raise TaskInputError('Los vértices del triángulo se calculan automáticamente')
    if task.get('profile') is not None:
        planner_for_profile(task['profile'])
    home = _joint_vector(start, 'HOME')
    report = cell.check_pose(home, tool_offset=points.tool_offset)
    if not report.ok:
        raise TaskInputError('HOME: ' + report.reason, 'HOME')
    model = points.flange
    # All coordinates are in cell_frame. Search a small set of work areas,
    # not an arbitrary offset from an unknown initial orientation.
    h = sqrt(3.) * SIDE_M / 2
    last_error = 'sin rama IK factible'
    for cx, cy in ((.18, 0.), (.12, -.12), (.12, .12), (0., -.18), (0., .18)):
        for height in (.25, .20, .30):
            z = cell.wall_top_z + height
            a = (cx - SIDE_M/2, cy - h/3, z)
            b = (cx + SIDE_M/2, cy - h/3, z)
            c = (cx, cy + 2*h/3, z)
            (xmin, xmax), (ymin, ymax) = cell.inner_bounds
            if not all(xmin+.05 <= x <= xmax-.05 and ymin+.05 <= y <= ymax-.05 for x,y,_ in (a,b,c)):
                continue
            approach = (a[0], a[1], a[2]+.08)
            for divisions in (1,):
                knots = [(approach,None), (a,None)]
                labels = ['APROX','A']
                for label, first, last in (('B',a,b),('C',b,c),('CIERRE_A',c,a)):
                    for i in range(1,divisions+1):
                        knots.append((tuple(p+(q-p)*i/divisions for p,q in zip(first,last)),None))
                        labels.append(label if i==divisions else f'{label}_{i}')
                for route in _ik_routes(model, cell, knots, home):
                    dest = [_Destination(name, sol.q, pose, sol.branch) for name,(sol,pose) in zip(labels,route)]
                    if max(abs(x-y) for x,y in zip(dest[1].q,dest[-1].q)) > 1e-7:
                        continue
                    dest[-1] = replace(dest[1], name='CIERRE_A')
                    dest.extend((replace(dest[0],name='SALIDA'), _Destination('HOME',home,model.fk(home),None)))
                    try:
                        checked = _check_wall_collision(points,cell,home,dest)
                        names = tuple(d.name for d in dest)
                        p = plan_task(points,home,dict(kind='custom',cycles=cycles,order=list(names),
                            profile=task.get('profile'),waypoints={d.name:{'q_rad':list(d.q)} for d in dest}),
                            velocity,segment_planner=segment_planner)
                        if max_duration is not None and any(s.duration>max_duration for s in p.trajectory.segments):
                            raise TaskInputError('Duración de tramo fuera del protocolo')
                        deviations=[]; rotations=[]; sigmas=[]; trace=[]
                        # Only trace the three sides; approach and return are separate motions.
                        for i in range(2,len(dest)-2):
                            s=p.trajectory.segments[i]
                            first,last=_xyz(dest[i-1].pose),_xyz(dest[i].pose)
                            n=max(16,ceil(max(abs(x-y) for x,y in zip(s.start,s.goal))/.01))
                            previous=0.
                            for k in range(n+1):
                                q=tuple(x+(y-x)*k/n for x,y in zip(s.start,s.goal))
                                actual=model.fk(q); xyz=_xyz(actual)
                                deviation,progress=_distance_to_segment(xyz,first,last)
                                rot=pose_residual(dest[i].pose,actual)['orientation_error_rad']
                                sigma=model.singular_values(q)[-1]
                                if progress<previous-.002 or deviation>MAX_DEVIATION_M or rot>.14 or sigma<MIN_SCALED_SIGMA:
                                    raise TaskInputError('La aproximación articular de la figura supera sus cotas')
                                previous=progress
                                deviations.append(deviation); rotations.append(rot); sigmas.append(sigma); trace.append(xyz)
                        records=tuple(dict(record,point='flange',branch=d.branch,requested_matrix=d.pose,
                                           calculated_matrix=model.fk(d.q)) for record,d in zip(p.waypoints,dest))
                        vertices=tuple(cell.to_base(v) for v in (a,b,c,a))
                        result=replace(p,kind='air_triangle',waypoints=records,required_start=home)
                        return result,dict(reference_point='flange',variant='air_triangle',side_m=SIDE_M,
                            execution_style='segmented_triangle',reference_xyz_m=vertices,planned_xyz_m=tuple(trace),
                            home_q_rad=home,waypoint_names=names,command_segments=len(dest),
                            total_command_segments=len(p.trajectory.segments),collision_configurations=checked,
                            max_tcp_deviation_m=max(deviations),max_tcp_orientation_error_rad=max(rotations),
                            min_sigma_scaled=min(sigmas),sampled_configurations=len(trace),
                            height_above_wall_m=height,side_segments=divisions)
                    except TaskInputError as exc:
                        last_error=str(exc)
    raise TaskInputError('No se encontró un triángulo admisible: '+last_error)
