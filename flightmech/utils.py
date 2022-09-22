import numpy as np
from fluids import ATMOSPHERE_1976


def feet(m: float):
    return m * 3.28


def meters(f: float):
    return f * 1 / 3.28


def rho(h=0):
    atm = ATMOSPHERE_1976(Z=h)
    return atm.rho


def c(h):
    atm = ATMOSPHERE_1976(Z=h)
    return (1.4 * 287.058 * atm.T) ** (1 / 2)


# noinspection PyPep8Naming
def Ma(V, h):
    return V / c(h)


def a_to_f(alpha):
    return np.array([
        [np.cos(alpha), 0, np.sin(alpha)],
        [0, 0, 0],
        [-np.sin(alpha), 0, np.cos(alpha)]
    ])


def f_to_a(alpha):
    return np.array([
        [],
        [],
        []
    ])


def a_to_g(gamma):
    return np.array([
        [np.cos(gamma), 0, np.sin(gamma)],
        [0, 0, 0],
        [-np.sin(gamma), 0, np.cos(gamma)]
    ])


def f_to_g(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 0, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
