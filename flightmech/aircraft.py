import numpy as np
from pydantic import BaseModel, Field

from flightmech.basemodel import FlightMechBaseModel
from flightmech.constants import g_0
from flightmech.propulsion import PropulsionSystem
from flightmech.utils import rho


# noinspection PyPep8Naming
class Aerodynamics(FlightMechBaseModel):
    # Aerodynamische Eigenschaften
    C_A_0: float
    C_A_alpha: float
    C_W0: float
    k: float
    k_1: float

    def C_A(self, alpha):
        return self.C_A_0 + self.C_A_alpha * alpha

    def C_W(self, alpha):
        return self.C_W0 + self.k_1 * self.C_A(alpha) + self.k * self.C_A(alpha) ** 2


# noinspection PyPep8Naming
class AircraftState(FlightMechBaseModel):
    # A/C State (Zustandsgrößen)
    alpha: float = 0
    d_alpha: float = 0
    theta: float = 0
    d_theta: float = 0
    gamma: float = 0
    d_gamma: float = 0

    s: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    V: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    a: np.ndarray = Field(default_factory=lambda: np.zeros(3))

    def update(self, dt=0, accel=np.zeros(3), d_theta: float = 0):
        self.a = accel
        self.V = self.V + self.a * dt
        self.s = self.s + self.a / 2 * dt ** 2 + self.V * dt

        gamma_new = np.arctan(self.V[1] / self.V[0])
        self.d_gamma = gamma_new - self.gamma
        self.gamma = gamma_new
        self.d_theta = d_theta
        self.theta += self.d_theta
        self.alpha = self.theta - self.gamma

    def V_a(self):
        return np.linalg.norm(self.V)

    def h(self):
        return - self.s[2]

    def rho(self):
        return rho(self.h())


# noinspection PyPep8Naming
class Aircraft(BaseModel):
    S: float  # Flügelfläche [m^2]
    b: float  # Spannweite [m]
    m: float
    propulsion: PropulsionSystem
    aerodynamics: Aerodynamics
    state: AircraftState = AircraftState()

    def A(self):
        return self.aerodynamics.C_A(self.state.alpha) * self.state.rho() / 2 * self.state.V_a() ** 2 * self.S

    def A_a(self):
        return np.array(
            [
                0,
                0,
                self.A()
            ]
        )

    def A_g(self):
        A = self.A()
        return A * np.array(
            [
                - np.sin(self.state.gamma),
                0,
                - np.cos(self.state.gamma)
            ]
        )

    def W(self):
        return self.aerodynamics.C_W(self.state.alpha) * self.state.rho() / 2 * self.state.V_a() ** 2 * self.S

    def W_a(self):
        return np.array(
            [
                - self.W(),
                0,
                0
            ]
        )

    def W_g(self):
        W = self.W()
        return W * np.array(
            [
                - np.cos(self.state.gamma),
                0,
                np.sin(self.state.gamma)
            ]
        )

    def F_a(self, n: float = 1):
        F = self.propulsion.thrust(self.state.h(), self.state.V_a())
        return self.propulsion.thrust(self.state.h(), self.state.V_a(), n) * np.array(
            [
                np.cos(self.state.alpha),
                0,
                - np.sin(self.state.alpha)
            ]
        )

    def F_g(self, n: float = 1):
        F = self.propulsion.thrust(self.state.h(), self.state.V_a(), n)
        return F * np.array(
            [
                np.cos(self.state.theta),
                0,
                - np.sin(self.state.theta)
            ]
        )

    def accel(self, n: float = 1):
        A = self.A_g()
        F = self.F_g(n)
        W = self.W_g()
        res = A + F + W + np.array([0, 0, self.m * g_0])
        return res / self.m
