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
