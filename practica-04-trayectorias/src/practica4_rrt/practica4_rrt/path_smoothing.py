import random


def smooth_path(path, grid, attempts=200, seed=11):
    if attempts < 0 or not grid.validate_path(path):
        raise ValueError('Invalid path or smoothing attempts')
    result = list(path)
    rng = random.Random(seed)
    for _ in range(attempts):
        if len(result) < 3:
            break
        i, j = sorted(rng.sample(range(len(result)), 2))
        if j > i+1 and grid.segment_free(result[i], result[j]):
            result[i+1:j] = []
    if not grid.validate_path(result):
        raise ValueError('Smoothed path collides')
    return result
