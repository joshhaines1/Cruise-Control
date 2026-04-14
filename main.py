import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Parameters from the packet
m = 2000  # mass in kg 
k = 40    # resistive coefficient 
v0 = 25   # initial velocity [cite: 58]

def car_dynamics(v, t, u):
    # derived from mv'(t) = -kv(t) + u(t) 
    dv_dt = (-k * v + u) / m
    return dv_dt

# Open-loop force calculation
r0 = 25 
u_const = k * r0  # [cite: 46, 52]

# Time array (0 to 300 seconds) [cite: 58]
t = np.linspace(0, 300, 1000)

# Solve the ODE
v_values = odeint(car_dynamics, v0, t, args=(u_const,))

def car_with_wind(v, t):
    # Base control force
    u = 1000 
    # Disturbance: 500N headwind after 50s [cite: 53, 57]
    f_disturb = 500 if t > 50 else 0
    
    dv_dt = (-k * v + u - f_disturb) / m
    return dv_dt

# Solve and Plot
v_disturbed = odeint(car_with_wind, v0, t)

plt.plot(t, v_disturbed)
plt.axvline(x=50, color='r', linestyle='--', label='Headwind starts')
plt.title("Cruise Control Response to Headwind (Open-Loop)")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.show()