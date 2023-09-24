import numpy


def ang_diff_min(theta1, theta2, full=2.0 * numpy.pi):
    r"""
    Find the angular difference from `theta1` to `theta2`, with the
    minimum absolute value normalized to :math:`[-\pi, \pi)`.

    The positive difference is the angle going in the positive
    direction from `theta1` to `theta2`, normalized to be in the range
    :math:`[0, 2 \pi)`. The negative difference is the angle going in
    the negative direction. This function returns the smaller of the
    two by absolute value.

    The return value can be computed without branching by rotating by
    half a circle before applying the moduli, then rotating back::

        ang_diff_min = fmod(fmod(theta2 - theta1 + 0.5 * full, full) +
                            full, full) - 0.5 * full

    Inputs can be scalars or arrays. Arrays must broadcast together.

    Parameters
    ----------
    theta1 : array-like
        The start angle or angles, in radians.
    theta2 : array-like
        The end angle or angles, in radians.
    full : float
        The period of a full circle. Defaults to :math:`2 \pi`. Use 360
        for data in degrees, 400 for gradians, 6400 for mils, etc.

    Returns
    -------
    numpy.ndarray :
        An array containing the broadcasted sign-preserving normalized
        difference of the two inputs with the smallest absolute value.
    """
    half = 0.5 * full
    # Broken out for readability
    step1 = numpy.fmod(theta2 - theta1 + half, full)
    return numpy.fmod(step1 + full, full) - half

