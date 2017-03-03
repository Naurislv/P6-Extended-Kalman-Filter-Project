"""Simple pythonic Kalman Filter implementation."""

import math


def dg(mju, sigma2, x):
    """Discrete Gausian value for given inputs."""
    return 1 / math.sqrt(2 * math.pi * sigma2) * math.exp(-1 / 2 * (x - mju) ** 2 / sigma2)


def update(mean1, var1, mean2, var2):
    """Calculate resulting variance and mean values of two gausian distributions.

    mean1: prior mean value
    var1: prior variance value

    mean1: measurement (next) mean value
    var2: measurement (next) variance value
    """
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1 / var2)

    return new_mean, new_var


def predict(mean1, var1, mean2, var2):
    """Calculate resulting motion.

    mean1: prior mean value
    var1: prior variance value

    mean1: motion mean value
    var2: motion variance value
    """
    new_mean = mean1 + mean2
    new_var = var1 + var2

    return new_mean, new_var


def KF1D(measurements, motion, measurement_sig, motion_sig, mu, sig):
    """Calculate Kalman Filter values for 1D case.

    Inputs:
        measurements: sequence of measurements (means)
        motion: sequence of motions (means)
        measurement_sig: measurement variance value
        motion_sig: motion variance value
        mu = initial mean value
        sig = initial variance value
    """
    print('Initial measurement (mean) / motion (variance) {} / {}'.format(mu, sig))
    for n in range(len(measurements)):
        mu, sig = update(mu, sig, measurements[n], measurement_sig)
        print('update:', mu, sig)
        mu, sig = predict(mu, sig, motion[n], motion_sig)
        print('predict:', mu, sig)
