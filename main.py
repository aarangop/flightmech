# This is a sample Python script.
from flightmech.aircraft import Aircraft
from flightmech.propeller import Propeller

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

prop = Propeller(d=2, zeta=0.7, P_max=90e3)
ac = Aircraft(S=197, b=65, prop=prop, n_prop=2, C_A_alpha=0.0947, C_A_0=0.253)

if __name__ == '__main__':
    pass

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
