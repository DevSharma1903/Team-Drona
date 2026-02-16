import numpy as np

G = 9.80665
R = 287.058
L = 0.0065
T0 = 288.15
rho0 = 1.225

def speed_of_sound(h: float) -> float:
    T = T0 - L * h
    return float(np.sqrt(1.4 * R * T))


def density_isa(h: float) -> float: 
    # h = h*3.281
    # return float (-3.3094*h*(10**(-5))) + (3.558*(10**(-6))*(h**2)) - (1.7667*(10**(-7))*(h**3)) + (3.6324*(10**(-8))*(h**4))
    return float(rho0 * ((1 - L * h / T0) ** ((G / (R * L)) - 1)))