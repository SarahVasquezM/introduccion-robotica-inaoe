"""Conservative occupancy cells and exact segment/AABB collision checks."""
import math
import yaml


class OccupancyMap:
    def __init__(self, config):
        self.config = config
        self.bounds = tuple(float(v) for v in config['bounds'])
        self.resolution = float(config['resolution'])
        self.inflation = float(config['drone_radius']) + float(config['margin'])
        values = (*self.bounds, self.resolution, self.inflation,
                  float(config['drone_radius']), float(config['margin']))
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Map parameters must be finite')
        x0, x1, y0, y1 = self.bounds
        if x0 >= x1 or y0 >= y1 or self.resolution <= 0:
            raise ValueError('Invalid map bounds/resolution')
        if config['drone_radius'] < 0 or config['margin'] < 0:
            raise ValueError('Radius and margin must be nonnegative')
        self.width = round((x1 - x0) / self.resolution)
        self.height = round((y1 - y0) / self.resolution)
        if not (math.isclose(self.width * self.resolution, x1 - x0) and
                math.isclose(self.height * self.resolution, y1 - y0)):
            raise ValueError('Map extents must be multiples of resolution')
        self.rectangles = []
        for o in config['obstacles']:
            if not all(math.isfinite(float(o[k])) for k in
                       ('x', 'y', 'width', 'depth', 'height')):
                raise ValueError('Obstacle values must be finite')
            if min(o['width'], o['depth'], o['height']) <= 0:
                raise ValueError('Obstacle dimensions must be positive')
            r = self.inflation
            self.rectangles.append((o['x'] - o['width']/2-r,
                                    o['x'] + o['width']/2+r,
                                    o['y'] - o['depth']/2-r,
                                    o['y'] + o['depth']/2+r))
        # Mark any cell touching inflated geometry; avoids underestimating corners.
        self.data = []
        for iy in range(self.height):
            for ix in range(self.width):
                cx, cy = self.grid_to_world(ix, iy)
                h = self.resolution / 2
                cell = (cx-h, cx+h, cy-h, cy+h)
                boundary = (cell[0] <= x0+self.inflation or
                            cell[1] >= x1-self.inflation or
                            cell[2] <= y0+self.inflation or
                            cell[3] >= y1-self.inflation)
                occupied = boundary or any(
                    cell[0] <= b and cell[1] >= a and
                    cell[2] <= d and cell[3] >= c
                    for a, b, c, d in self.rectangles)
                self.data.append(100 if occupied else 0)

    @classmethod
    def load(cls, filename):
        with open(filename, encoding='utf-8') as f:
            return cls(yaml.safe_load(f))

    def world_to_grid(self, x, y):
        x0, x1, y0, y1 = self.bounds
        if not (math.isfinite(x) and math.isfinite(y) and x0 <= x < x1 and y0 <= y < y1):
            raise ValueError('Point outside map')
        return (math.floor((x-x0)/self.resolution),
                math.floor((y-y0)/self.resolution))

    def grid_to_world(self, ix, iy):
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            raise ValueError('Cell outside map')
        return (self.bounds[0] + (ix+0.5)*self.resolution,
                self.bounds[2] + (iy+0.5)*self.resolution)

    def point_free(self, p):
        try:
            ix, iy = self.world_to_grid(*p)
        except ValueError:
            return False
        return self.data[iy*self.width+ix] == 0

    @staticmethod
    def intersects(a, b, rect):
        """Closed segment vs closed box via slab intervals (including tangency)."""
        lo, hi = 0.0, 1.0
        for k, lower, upper in ((0, rect[0], rect[1]), (1, rect[2], rect[3])):
            delta = b[k]-a[k]
            if abs(delta) < 1e-14:
                if not lower <= a[k] <= upper:
                    return False
            else:
                t0, t1 = sorted(((lower-a[k])/delta, (upper-a[k])/delta))
                lo, hi = max(lo, t0), min(hi, t1)
                if lo > hi:
                    return False
        return True

    def segment_free(self, a, b):
        if not self.point_free(a) or not self.point_free(b):
            return False
        # Check every occupied cell in the segment bounding box exactly. Sampling
        # alone can miss a thin obstacle or a clipped cell corner.
        ia, ja = self.world_to_grid(*a)
        ib, jb = self.world_to_grid(*b)
        for iy in range(max(0, min(ja, jb)-1), min(self.height, max(ja, jb)+2)):
            for ix in range(max(0, min(ia, ib)-1), min(self.width, max(ia, ib)+2)):
                if self.data[iy*self.width+ix]:
                    x, y = self.grid_to_world(ix, iy)
                    h = self.resolution/2
                    if self.intersects(a, b, (x-h, x+h, y-h, y+h)):
                        return False
        return True

    def validate_path(self, path):
        return bool(path) and all(self.point_free(p) for p in path) and all(
            self.segment_free(a, b) for a, b in zip(path, path[1:]))
