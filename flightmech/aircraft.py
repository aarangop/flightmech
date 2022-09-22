import math

import numpy as np
import pandas as pd
from pydantic import BaseModel, Field

from flightmech.basemodel import FlightMechBaseModel
from flightmech.constants import g_0
from flightmech.propulsion import PropulsionSystem
from flightmech.utils import rho, a_to_f, a_to_g, f_to_a, f_to_g


# noinspection PyPep8Naming
def load_aerodynamics(_polar):
    df = pd.read_csv(f"../data/polare/{_polar}.csv",
                     delimiter=",",
                     header=0,
                     names=['alpha', 'Re', 'Cl', 'Cd', 'Cm 0.25', 'TU', 'TL', 'SU', 'SL', 'L/D', 'A.C.', 'C.P.'],
                     dtype={'alpha': np.float64, 'Cl': np.float64, 'Cd': np.float64}
                     )

    k, k_1, C_W0 = np.polyfit(x=df['Cl'], y=df['Cd'], deg=2)

    lift_coefficients = np.polyfit(x=df['alpha'], y=df['Cl'], deg=1)
    C_A_alpha, C_A_0 = lift_coefficients

    return Aerodynamics(
        C_A_0=C_A_0,
        C_A_alpha=C_A_alpha,
        C_W0=C_W0,
        k=k,
        k_1=k_1
    )


# noinspection PyPep8Naming
class Aerodynamics(FlightMechBaseModel):
    # Aerodynamische Eigenschaften
    C_A_0: float
    C_A_alpha: float
    C_W0: float
    k: float
    k_1: float
    alpha_max: float = 10

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

    # Coordinates in earth-bound coordinate system
    s: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    V: np.ndarray = Field(default_factory=lambda: np.zeros(3))
    a: np.ndarray = Field(default_factory=lambda: np.zeros(3))

    @property
    def alpha_deg(self):
        return math.degrees(self.alpha)

    def update(self, dt=0, accel=np.zeros(3), d_theta: float = 0):
        self.a = accel
        self.V = self.V + self.a * dt
        self.s = self.s + self.a / 2 * dt ** 2 + self.V * dt

        gamma_new = np.arctan(self.V[2] / self.V[0])
        self.d_gamma = gamma_new - self.gamma
        self.gamma = gamma_new
        self.alpha = self.theta - self.gamma
        self.theta = self.alpha + self.gamma

    def V_a(self):
        return np.array([np.linalg.norm(self.V), 0, 0])

    def V_f(self):
        return np.matmul(a_to_f(self.alpha), self.V_a())

    def V_g(self):
        return self.V

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

    def G(self):
        return self.m * g_0

    def A(self):
        return self.aerodynamics.C_A(self.state.alpha_deg) * self.state.rho() / 2 * self.state.V_a()[0] ** 2 * self.S

    def A_a(self):
        return self.A() * np.array([0, 0, -1])

    def A_g(self):
        return np.matmul(a_to_g(self.state.gamma), self.A_a())

    def W(self):
        return self.aerodynamics.C_W(self.state.alpha_deg) * self.state.rho() / 2 * self.state.V_a()[0] ** 2 * self.S

    def W_a(self):
        return self.W() * np.array([-1, 0, 0])

    def W_g(self):
        return np.matmul(a_to_g(self.state.gamma), self.W_a())

    def F(self, n: float = 1):
        return self.propulsion.thrust(self.state.h(), self.state.V_f()[0], n)

    def F_f(self, n: float = 1):
        return self.F(n) * np.array([1, 0, 0])

    def F_a(self, n: float = 1):
        return self.F_f(n) * f_to_a(self.state.alpha)

    def F_g(self, n: float = 1):
        F_f = self.F_f(n)
        res = np.matmul(f_to_g(self.state.theta), F_f)
        return res

    def accel(self, n: float = 1):
        A = self.A_g()
        F = self.F_g(n)
        W = self.W_g()
        res = A + F + W + np.array([0, 0, self.m * g_0])
        return res / self.m
