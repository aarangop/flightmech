from pydantic import BaseModel

from flightmech.utils import rho


# noinspection PyPep8Naming
class Propeller(BaseModel):
    d: float
    zeta: float
    P_max: float
    n_v: int = -1

    def F_F(self, h: float, P: float, V: float = 0):
        _F_0 = self.F_0(h, P)
        return _F_0 - (rho(h) * self.S * _F_0) ** (1 / 2) * V

    def F_0(self, h: float, P):
        return (2 * rho(h) * self.S * (P * self.zeta) ** 2) ** (1 / 3)
