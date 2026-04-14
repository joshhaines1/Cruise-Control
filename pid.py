import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Plant Parameters
m = 2000 
k = 40   
r0 = 25  # Set speed (m/s)

def simulate_cruise(Kp, Ki, Kd, label):
    # Integral state tracker
    integral_error = [0.0]
    last_error = [0.0]
    last_t = [0.0]

    def pid_dynamics(t, v):
        # 1. Calculate Error
        error = r0 - v
        
        # 2. Update Integral (Simple rectangular approximation)
        dt = t - last_t[0]
        if dt > 0:
            integral_error[0] += error * dt
            derivative = (error - last_error[0]) / dt
        else:
            derivative = 0
            
        # 3. PID Formula: u(t) = Kp*e + Ki*∫e + Kd*de/dt
        u = Kp * error + Ki * integral_error[0] + Kd * derivative
        
        # 4. Save states for next step
        last_error[0] = error
        last_t[0] = t
        
        # 5. Physics: mv' = -kv + u - disturbance
        disturbance = 500 if t >= 50 else 0
        dv_dt = (-k * v + u - disturbance) / m
        return dv_dt

    sol = solve_ivp(pid_dynamics, [0, 150], [25], t_eval=np.linspace(0, 150, 1000), method='RK45')
    return sol.t, sol.y[0]

# --- Testing Different Gain Sets ---
plt.figure(figsize=(12, 6))

# Kd = 800
t, v = simulate_cruise(Kp=1360, Ki=200, Kd=800, label='PID-Control')
plt.plot(t, v, label='PID (Kp=1360, Ki=200, Kd=800)')

# Kd = 400 
t, v = simulate_cruise(Kp=1360, Ki=200, Kd=400, label='PID-Control')
plt.plot(t, v, label='PID (Kp=1360, Ki=200, Kd=400)')

# Kd = 200 
t, v = simulate_cruise(Kp=1360, Ki=200, Kd=200, label='PID-Control')
plt.plot(t, v, label='PID (Kp=1360, Ki=200, Kd=200)')

# Kd = 100 
t, v = simulate_cruise(Kp=1360, Ki=200, Kd=100, label='PID-Control')
plt.plot(t, v, label='PID (Kp=1360, Ki=200, Kd=100)')

# Test 4: "Bad" Oscillation (Exercise 10)
# t, v = simulate_cruise(Kp=100, Ki=200, Kd=0, label='Oscillatory')
# plt.plot(t, v, '--', label='Unstable PI (Kp=100): Bad tuning')

plt.axvline(x=50, color='black', alpha=0.3, label='Headwind Starts')
plt.axhline(y=25, color='r', linestyle=':', label='Setpoint (25 m/s)')
plt.title("Comparison of Cruise Control Strategies")
plt.xlabel("Time (s)"); plt.ylabel("Velocity (m/s)")
# Focus the view on the area around the setpoint (25 m/s)
plt.ylim(0, 50)  # Limits velocity view from 0 to 50 m/s

# Optional: Focus the time on the disturbance window
plt.xlim(40, 150) # Shows 10 seconds before the wind hits until the end
plt.legend(); plt.grid(True); plt.show()