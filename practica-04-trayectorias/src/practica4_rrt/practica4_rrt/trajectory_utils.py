import math
from bisect import bisect_right


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def peak_cubic_speed(path, total_time):
    """Peak speed when segment times are proportional to segment length."""
    length = path_length(path)
    if not math.isfinite(total_time) or total_time <= 0.0 or length <= 0.0:
        raise ValueError('Path length and total_time must be positive')
    return 1.5 * length / total_time


def minimum_time_for_speed(path, max_reference_speed):
    if not math.isfinite(max_reference_speed) or max_reference_speed <= 0.0:
        raise ValueError('max_reference_speed must be finite and positive')
    return 1.5 * path_length(path) / max_reference_speed


def segment_times(path, total_time):
    lengths = [math.dist(a, b) for a, b in zip(path, path[1:])]
    if not math.isfinite(total_time) or total_time <= 0:
        raise ValueError('total_time must be finite and positive')
    if not lengths or any(not math.isfinite(v) or v < 1e-9 for v in lengths):
        raise ValueError('Path requires nondegenerate segments')
    times = [total_time*v/sum(lengths) for v in lengths]
    times[-1] = total_time-sum(times[:-1])
    return times


def cubic_reference(a, b, t, td):
    u = max(0.0, min(1.0, t/td))
    s = 3*u*u-2*u*u*u
    return tuple(x+s*(y-x) for x, y in zip(a, b))


def rounded_path(path, grid, corner_distance=0.6, samples_per_corner=20):
    """Round polyline corners with collision-checked quadratic Bezier arcs."""
    if (corner_distance <= 0.0 or samples_per_corner < 2 or
            not grid.validate_path(path)):
        raise ValueError('Invalid path or rounding parameters')
    if len(path) < 3:
        return list(path)

    result = [tuple(path[0])]
    for a, b, c in zip(path, path[1:], path[2:]):
        incoming = math.dist(a, b)
        outgoing = math.dist(b, c)
        trim = min(corner_distance, 0.45 * incoming, 0.45 * outgoing)
        accepted = None
        for _ in range(12):
            entry = tuple(b[k] + trim * (a[k] - b[k]) / incoming for k in (0, 1))
            exit_point = tuple(b[k] + trim * (c[k] - b[k]) / outgoing for k in (0, 1))
            curve = []
            for index in range(samples_per_corner + 1):
                u = index / samples_per_corner
                curve.append(tuple(
                    (1.0-u)**2 * entry[k] +
                    2.0 * (1.0-u) * u * b[k] +
                    u*u * exit_point[k]
                    for k in (0, 1)
                ))
            candidate = [result[-1], *curve]
            if grid.validate_path(candidate):
                accepted = curve
                break
            trim *= 0.5
        if accepted is None:
            raise ValueError('Unable to round path without collision')
        for point in accepted:
            if math.dist(result[-1], point) > 1e-9:
                result.append(point)

    if math.dist(result[-1], path[-1]) > 1e-9:
        result.append(tuple(path[-1]))
    if not grid.validate_path(result):
        raise ValueError('Rounded path collides')
    return result


class ArcLengthPath:
    """Piecewise-linear arc-length lookup for a densely rounded path."""
    def __init__(self, path):
        if not path:
            raise ValueError('Path cannot be empty')
        self.path = [tuple(point) for point in path]
        self.cumulative = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            distance = math.dist(a, b)
            if distance > 1e-12:
                self.cumulative.append(self.cumulative[-1] + distance)
            else:
                raise ValueError('Path contains a degenerate segment')
        self.length = self.cumulative[-1]

    def sample(self, distance):
        if len(self.path) == 1:
            return self.path[0], (0.0, 0.0)
        distance = max(0.0, min(self.length, distance))
        index = min(
            len(self.path)-2,
            max(0, bisect_right(self.cumulative, distance)-1)
        )
        a, b = self.path[index], self.path[index+1]
        segment = self.cumulative[index+1]-self.cumulative[index]
        ratio = (distance-self.cumulative[index])/segment
        point = tuple(a[k]+ratio*(b[k]-a[k]) for k in (0, 1))
        tangent = tuple((b[k]-a[k])/segment for k in (0, 1))
        return point, tangent
