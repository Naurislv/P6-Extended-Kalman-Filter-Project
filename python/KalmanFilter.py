"""Simple python Kalman Filter implementation."""

import math
import numpy as np


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

    Sample inputs:
        measurements = [5., 6., 7., 9., 10.]
        motion = [1., 1., 2., 1., 1.]
        measurement_sig = 4.
        motion_sig = 2.
        mu = 0.
        sig = 10000.
    """
    print('Initial measurement (mean) / motion (variance) {} / {}'.format(mu, sig))
    for n in range(len(measurements)):
        mu, sig = update(mu, sig, measurements[n], measurement_sig)
        print('update:', mu, sig)
        mu, sig = predict(mu, sig, motion[n], motion_sig)
        print('predict:', mu, sig)


def KFnD(x, P, measurements, H, R, I, F, u):
    """"Kalman n dimension implementation.

    Sample inputs:
        measurements = [1, 2, 3]
        x = np.array([[0.], [0.]])  # initial state (location and velocity)
        P = np.array([[1000., 0.], [0., 1000.]])  # initial uncertainty
        u = np.array([[0.], [0.]])  # external motion
        F = np.array([[1., 1.], [0, 1.]])  # next state function
        H = np.array([[1., 0.]])  # measurement function
        R = np.array([[1.]])  # measurement uncertainty
        I = np.array([[1., 0.], [0., 1.]])  # identity matrix
    """
    for n in range(len(measurements)):
        # measurement update
        Y = measurements[n] - np.dot(H, x)
        S = np.dot(np.dot(H, P), H.T) + R
        K = np.dot(np.dot(P, H.T), np.linalg.inv(S))
        x = x + np.dot(K, Y)
        P = np.dot(I - K * H, P)
        # prediction
        x = np.dot(F, x) + u
        P = np.dot(np.dot(F, P), F.T)

    return x, P
