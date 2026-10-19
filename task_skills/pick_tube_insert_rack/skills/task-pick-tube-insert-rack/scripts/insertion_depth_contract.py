"""Pure depth contract; importing this module never connects to a robot."""
import math


def seated_minimum_depth(insert_depth_m, near_tolerance_m, explicit_minimum=None):
    """Default contact acceptance to the end of this insertion, not a fixed 3mm.

An explicit calibrated minimum is preserved. This is a necessary depth condition,
not sufficient proof of seating: force, release and postconditions stay separate.
"""
    for name, value in (('insert_depth_m', insert_depth_m), ('near_tolerance_m', near_tolerance_m)):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be positive and finite')
    if explicit_minimum is not None:
        if (type(explicit_minimum) not in (int, float) or not math.isfinite(explicit_minimum)
                or not 0 < explicit_minimum <= insert_depth_m):
            raise ValueError('seated_min_depth_m must be positive and no greater than insertion depth')
        return float(explicit_minimum)
    return insert_depth_m - min(near_tolerance_m, .15 * insert_depth_m)
