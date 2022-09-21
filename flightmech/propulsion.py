from __future__ import annotations

import numpy as np

from flightmech.basemodel import FlightMechBaseModel
from flightmech.utils import rho


# noinspection PyPep8Naming
class Propeller(FlightMechBaseModel):
    d: float
    zeta: float
    P_max: float
    n_v: int = -1

    @property
    def S_rotor(self):
        return np.pi * (self.d / 2) ** 2

    def V_str(self, h, n: float = 1):
        return (self.F_0(n) / (self.S_rotor * rho(h))) ** (1 / 2)

    def F_F(self, h: float, V: float = 0, n: float = 1):
        V_str = self.V_str(h, n)
        return rho(h) * self.S_rotor * V_str * (V_str - V)

    def F_0(self, h: float, n: float = 1):
        return (2 * rho(h) * self.S_rotor * (n * self.P_max * self.zeta) ** 2) ** (1 / 3)


class PropulsionSystem(FlightMechBaseModel):
    propulsor: Propeller
    n_prop: int = 2

    def thrust(self, h, V: float = 0, n=1):
        return self.n_prop * self.propulsor.F_F(h, V, n)
