# This is a sample Python script.
import math

import numpy as np
from scipy.optimize import minimize

from flightmech.aircraft import AircraftState, Aerodynamics, Aircraft
from flightmech.propulsion import Propeller, PropulsionSystem

prop = Propeller(d=2, zeta=0.7, P_max=50e3, n_v=-1)
propulsion = PropulsionSystem(propulsor=prop, n_prop=2)

aerodynamics = Aerodynamics(
    C_A_0=0.10789090909090908,
    C_A_alpha=0.2816363636363635,
    C_W0=0.01260553908834591,
    k=-0.016711757696203162,
    k_1=0.01714693420911262
)

ac = Aircraft(
    S=197,
    b=65,
    propulsion=propulsion,
    aerodynamics=aerodynamics,
    m=2500
)


def accel_z(x):
    ac.state.alpha = x[0]
    ac.state.theta = x[1]
    ac.state.V[0] = x[2]
    ac.state.update(ac.accel(1))
    ac.propulsion.propulsor.P_max = x[3]
    accel = ac.accel(1)[2]
    return accel


if __name__ == '__main__':
    V = np.array([20, 0, 0])

    alpha_0 = math.radians(7)
    theta_0 = alpha_0
    u_g_0 = 20
    P_max_0 = 50e3

    init_state = AircraftState(
        alpha=alpha_0,
        theta=theta_0,
        gamma=0,
        V=np.array([20, 0, 0]),
        a=np.array([0, 0, 0]),
        s=np.array([0, 0, -18288])
    )

    x0 = np.array([alpha_0, theta_0, u_g_0, P_max_0])

    ac.state = init_state

    res = minimize(accel_z, x0)
    print(res)
