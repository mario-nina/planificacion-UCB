"""Conservative empirical LSPB shape identification. No robot I/O.

Fit only measured positions. A fit is an equivalent symmetric trapezoid, not
servo telemetry or a proof that ABB follows that model. A reduction is allowed
only after two consistent verified untuned repetitions of the same segment.
"""
from math import isfinite, radians, sqrt
from statistics import median

MIN_SPAN = radians(10)
MATCH = radians(.05)
MIN_SAMPLES = 12
MAX_FIT_ERROR = .025  # normalized displacement
MIN_RATIO = .8  # at most 20% reduction from the pre-existing modal setting


def progress(u, blend):
    u = max(0., min(1., u))
    if u < blend:
        return u*u/(2*blend*(1-blend))
    if u <= 1-blend:
        return (u-blend/2)/(1-blend)
    return 1-(1-u)**2/(2*blend*(1-blend))


def fit_segment(start, goal, duration, blend, samples, sent, phase2, terminal):
    """Return an equivalent ramp estimate only for adequately resolved motion.

    Timestamps are host arrivals. Search a bounded onset interval and ±15%
    duration; never shift or replace the reference displayed in RViz.
    """
    if not (0 < blend <= .5 and duration > 0 and sent <= phase2 <= terminal):
        return None
    delta = tuple(b-a for a,b in zip(start,goal))
    denom = sum(d*d for d in delta)
    if max(map(abs,delta)) < MIN_SPAN:
        return None
    data = []
    for stamp, q in samples:
        if (len(q) != 6 or not isfinite(stamp) or not all(isfinite(x) for x in q)
                or stamp < sent or stamp > terminal or (data and stamp <= data[-1][0])):
            continue
        p = sum((x-a)*d for x,a,d in zip(q,start,delta))/denom
        residual = sqrt(sum((x-a-p*d)**2 for x,a,d in zip(q,start,delta))/denom)
        if residual > .025 or p < -.03 or p > 1.03:
            return None
        data.append((stamp,p))
    moving = [(t,p) for t,p in data if .02 < p < .98]
    if len(moving) < MIN_SAMPLES or not data or data[-1][1] < .98:
        return None
    intervals = [b[0]-a[0] for a,b in zip(moving,moving[1:])]
    gap = median(intervals)
    if gap <= 0 or max(intervals) > max(.04,2.5*gap):
        return None
    if any(b[1] < a[1]-.015 for a,b in zip(data,data[1:])):
        return None
    # First phase-2 observation is an upper bound only to within host sampling.
    upper = min(phase2 + gap, moving[0][0])
    lower = max(sent, phase2 - 2*gap)
    if lower > upper:
        return None
    objective = data[::max(1,len(data)//96)]
    if objective[-1] != data[-1]:
        objective.append(data[-1])
    best = None
    for k in range(11):
        onset = lower + (upper-lower)*k/10
        for j in range(13):
            observed_t = duration*(.85+.025*j)
            if onset+observed_t > terminal+gap:
                continue
            for i in range(5,51):
                f = i/100
                error = sum((p-progress((t-onset)/observed_t,f))**2 for t,p in objective)/len(objective)
                if best is None or error < best[0]:
                    best = (error,onset,observed_t,f)
    if best is None:
        return None
    error,onset,observed_t,f = best
    rmse = sqrt(error)
    ramp = observed_t*f
    accel_count = sum(onset < t < onset+ramp for t,_ in moving)
    decel_count = sum(onset+observed_t-ramp < t < onset+observed_t for t,_ in moving)
    if rmse > MAX_FIT_ERROR or min(accel_count,decel_count) < 3 or ramp < 3*gap:
        return None
    expected = 1/(duration**2*blend*(1-blend))
    measured = 1/(observed_t**2*f*(1-f))
    return dict(rmse_progress=rmse, samples=len(moving), ramp_s=ramp,
                cruise_s=observed_t*(1-2*f), fitted_duration_s=observed_t,
                reference_ramp_s=duration*blend, acceleration_ratio=measured/expected,
                peak_velocity_ratio=(duration*(1-blend))/(observed_t*(1-f)),
                sample_interval_s=gap, timing='host_fit_not_servo')


class AccelerationCalibration:
    """Session-local, exact-direction repeated-segment calibration.

    No extrapolation across postures or profiles, no increase above the original
    setting, no adjustment during an order. Two baseline observations required.
    The first proposed reduction is a single bounded experiment, not a guarantee.
    """
    def __init__(self):
        self.records = []

    @staticmethod
    def matches(a,b):
        return (abs(a[2]-b[2]) <= .005 and abs(a[3]-b[3]) <= .001 and
                all(abs(x-y)<=MATCH for x,y in zip(a[0]+a[1],b[0]+b[1])))

    def record(self, key, fit, now, *, ratio=1.):
        if fit is not None or ratio < 1.:
            self.records.append((key,fit,now,ratio))
            self.records = self.records[-24:]

    @staticmethod
    def score(fit):
        return abs(fit['acceleration_ratio']-1)+abs(fit['peak_velocity_ratio']-1)

    def ratio(self, key, now):
        matches = [(f,r) for k,f,t,r in self.records if 0 <= now-t <= 600 and self.matches(k,key)]
        base = [f for f,r in matches if r == 1. and f is not None]
        tested = [(f,r) for f,r in matches if r < 1.]
        if len(base)<2:
            return 1.,'sin dos ensayos base suficientemente muestreados'
        a,b = base[-2:]
        if tested:
            # Compare against baseline. Reuse only an improvement with no undue time penalty.
            f,r = tested[-1]
            if (f is not None and self.score(f) < .9*min(self.score(a),self.score(b))
                    and abs(f['fitted_duration_s']/key[2]-1) <= .10):
                return r,'reducción previamente observada con mejor ajuste equivalente'
            return 1.,'ensayo de reducción sin mejora demostrada; conservar ajuste base'
        ra,rb = a['acceleration_ratio'],b['acceleration_ratio']
        if (min(ra,rb)<1.15 or abs(ra-rb)>.15*max(ra,rb)
                or any(abs(f['fitted_duration_s']/key[2]-1)>.10 for f in (a,b))):
            return 1.,'sin exceso de aceleración consistente con duración comparable'
        return max(MIN_RATIO,1/max(ra,rb)), 'reducción experimental acotada a partir de dos ensayos equivalentes'


def latest_derivatives(samples):
    """Local quadratic at last actual timestamp; no fabricated observations.

    For nonuniform samples, secants plus the inferred constant acceleration
    give the endpoint velocity. Estimates degrade across phase changes/noise.
    """
    if len(samples) < 3:
        return None
    (ta,a),(tb,b),(tc,c) = tuple(samples)[-3:]
    h1,h2=(tb-ta)/1e9,(tc-tb)/1e9
    if (min(h1,h2)<.015 or not .05<=h1+h2<=.6
            or max(h1,h2)>3*min(h1,h2)):
        return None
    acceleration=tuple(2*((z-y)/h2-(y-x)/h1)/(h1+h2) for x,y,z in zip(a,b,c))
    velocity=tuple((z-y)/h2+v*h2/2 for y,z,v in zip(b,c,acceleration))
    return velocity,acceleration
