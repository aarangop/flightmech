from pydantic import BaseModel

from flightmech.propeller import Propeller
from flightmech.utils import rho


# noinspection PyPep8Naming
class Aircraft(BaseModel):
    S: float  # Flügelfläche [m^2]
    b: float  # Spannweite [m]
    prop: Propeller
    n_prop: int = 2

    # Aerodynamische Eigenschaften
    C_A_alpha: float
    C_A_0: float
    C_W0: float
    C_ww: float
    k: float

    # A/C State (Zustandsgrößen)
    alpha: float = 0
    alpha_punkt: float = 0
    theta: float = 0
    theta_punkt: float = 0
    gamma: float = 0
    gamma_punkt: float = 0

    def C_A(self):
        return self.C_A_0 + self.C_A_alpha * self.alpha

    def C_A_rel(self):
        return self.C_A() / self.C_A_1()

    def C_A_1(self):
        return (self.C_W0 / self.k) ** (1 / 2)

    def C_A_0_rel(self):
        return self.C_A_0 / self.C_A_1()

    def C_W(self, h, v):
        return self.C_W0 * (1 - 2 * self.C_A_0_rel() * self.C_A_rel() + self.C_A_rel() ** 2)

    def W(self, h, v):
        rho_h = rho(h)
        return rho_h / 2 * v ** 2 * self.S + (self.C_W(h, v))

    def A(self, h, v):
        rho_h = rho(h)
        return self.C_A() * rho_h / 2 * v ** 2 * self.S

    def a_x(self, P, h):
        pass

    def a_z(self, P, h):
        pass
