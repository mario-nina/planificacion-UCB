"""Host-clock comparison of sampled joint feedback and mathematical reference.

No interpolation of measured data or fit of reference to feedback. The caller
accepts only fresh, correlated samples after observed RAPID phase 2, during
that segment's nominal reference interval. Metrics are diagnostic, never used
to authorize, terminate or accelerate motion.
"""
from math import isfinite, sqrt


def reference_limits(segment):
    """Analytical extrema of a rest-to-rest reference, not servo limits."""
    if getattr(segment, "linear_cartesian", False):
        return dict(shape="LSPB cartesiano", ramp_s=segment.duration*segment.blend_fraction,
                    cruise_s=segment.duration*(1-2*segment.blend_fraction),
                    peak_velocity_rad_s=segment.peak_v, peak_acceleration_rad_s2=segment.peak_a)
    delta = tuple(abs(b - a) for a, b in zip(segment.start, segment.goal))
    t = segment.duration
    if segment.profile == "quintic":
        v = 15 / 8
        a = 10 / sqrt(3)
        shape = "quintic"
        ramp = cruise = None
    else:
        f = segment.blend_fraction
        v = 1 / (1 - f)
        a = v / f
        shape = "triangular" if f == .5 else "trapezoidal"
        ramp = t * f
        cruise = t * (1 - 2 * f)
    return dict(shape=shape, ramp_s=ramp, cruise_s=cruise,
                peak_velocity_rad_s=tuple(d * v / t for d in delta),
                peak_acceleration_rad_s2=tuple(d * a / (t*t) for d in delta))


class TrackingMetrics:
    """Incremental, per-sequence and whole-run sample statistics."""
    def __init__(self):
        self._legs = {}

    def add(self, index, sequence, stamp, error):
        if (type(sequence) is not int or not isfinite(stamp) or len(error) != 6
                or not all(isfinite(e) for e in error)):
            return False
        key = (index, sequence)
        row = self._legs.setdefault(key, dict(n=0, sums=[0.]*6, peaks=[0.]*6,
                                              first=stamp, last=None))
        if row['last'] is not None and stamp <= row['last']:
            return False
        row['last'] = stamp
        row['n'] += 1
        for i, e in enumerate(error):
            row['sums'][i] += e*e
            row['peaks'][i] = max(row['peaks'][i], abs(e))
        return True

    @staticmethod
    def _summary(rows):
        n = sum(row['n'] for row in rows)
        if not n:
            return dict(samples=0, rmse_rad=None, rmse_by_joint_rad=None,
                        max_error_rad=None, max_error_by_joint_rad=None)
        sums = [sum(row['sums'][i] for row in rows) for i in range(6)]
        peaks = [max(row['peaks'][i] for row in rows) for i in range(6)]
        return dict(samples=n, rmse_rad=sqrt(sum(sums)/(6*n)),
                    rmse_by_joint_rad=tuple(sqrt(s/n) for s in sums),
                    max_error_rad=max(peaks), max_error_by_joint_rad=tuple(peaks))

    def report(self, index, sequence):
        key = (index, sequence)
        return dict(basis="Muestras únicas RWS; fase 2 observada; intervalo de referencia; sin ajuste temporal",
                    leg=self._summary([self._legs[key]] if key in self._legs else []),
                    run=self._summary(list(self._legs.values())))


def local_derivatives(samples):
    """Quadratic least-squares fit on up to five recent real samples.

    Evaluate both derivatives at the newest timestamp. Nonuniform host
    timestamps are supported; no synthetic samples or extrapolated positions.
    Reject gaps instead of drawing across missing measurements.
    """
    rows = tuple(samples)[-5:]
    if len(rows) < 3:
        return None
    if any(len(q) != 6 or not all(isfinite(v) for v in q) for _, q in rows):
        return None
    gaps = [(b[0]-a[0])/1e9 for a,b in zip(rows,rows[1:])]
    span = sum(gaps)
    if min(gaps) < .015 or not .05 <= span <= .8 or max(gaps) > .35 or max(gaps) > 3*min(gaps):
        return None
    # Normalize time to [-1,0]. Orthogonalize the linear and quadratic
    # columns against the constant to avoid a poorly scaled normal matrix.
    x = [(t-rows[-1][0])/1e9/span for t,_ in rows]
    n = len(x)
    mean = sum(x)/n
    mean2 = sum(t*t for t in x)/n
    u = [t-mean for t in x]
    v = [t*t-mean2 for t in x]
    uu = sum(t*t for t in u)
    uv = sum(a*b for a,b in zip(u,v))
    w = [b-a*uv/uu for a,b in zip(u,v)]
    ww = sum(t*t for t in w)
    if ww < 1e-10:
        return None
    velocity, acceleration = [], []
    for j in range(6):
        y = [q[j]-rows[-1][1][j] for _,q in rows]
        c = sum(a*b for a,b in zip(w,y))/ww
        b = (sum(a*b for a,b in zip(u,y))-uv*c)/uu
        velocity.append(b/span)
        acceleration.append(2*c/span**2)
    return tuple(velocity), tuple(acceleration)


def reference_at(segment, timestamp, origin):
    """Evaluate the original unscaled reference on the host clock.

    Outside its interval, show rest at start/goal. This must not be used
    to infer a physical arrival or to change a requested motion duration.
    """
    from dataclasses import replace
    t = timestamp-origin
    sample = segment.sample(min(segment.duration, max(0., t)))
    if t < 0 or t >= segment.duration:
        sample = replace(sample, velocities=(0.,)*6, accelerations=(0.,)*6)
    return sample
