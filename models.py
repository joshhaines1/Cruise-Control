import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Plant Parameters from Project Packet
m = 2000  # mass in kg [cite: 51]
k = 40    # linear resistive coefficient [cite: 51]
r0 = 25   # desired set speed (m/s) [cite: 51]

# ==========================================
# OPEN-LOOP CONTROL (No Feedback to Wind)
# ==========================================
# This system does not monitor v(t). It provides a fixed force u
# based solely on the target r0, assuming no disturbances[cite: 60].
def simulate_open_loop():
    # Force required to maintain r0 on level ground with no wind [cite: 46, 52]
    u_fixed = k * r0 
    
    def dynamics(t, v):
        # The controller doesn't "see" this disturbance 
        disturbance = 500 if t >= 50 else 0 
        return (-k * v + u_fixed - disturbance) / m
    
    sol = solve_ivp(dynamics, [0, 150], [25], t_eval=np.linspace(0, 150, 1000))
    return sol.t, sol.y[0]

# ==========================================
# CLOSED-LOOP CONTROL (Feedback Enabled)
# ==========================================
# These systems monitor the actual velocity v(t) and calculate
# an error signal e(t) = r(t) - v(t) to adjust the force u[cite: 65, 72, 132].
def simulate_closed_loop(Kp, Ki, Kd):
    integral_error = [0.0]
    last_error = [0.0]
    last_t = [0.0]

    def pid_dynamics(t, v):
        # Step 1: Monitor actual speed and calculate error [cite: 132]
        error = r0 - v 
        dt = t - last_t[0]
        
        # Step 2: Accumulate error (Integral) and rate of change (Derivative) [cite: 96, 125]
        if dt > 0:
            integral_error[0] += error * dt 
            derivative = (error - last_error[0]) / dt 
        else:
            derivative = 0
            
        # Step 3: Compute u(t) using PID gains [cite: 135]
        u = Kp * error + Ki * integral_error[0] + Kd * derivative 
        
        last_error[0] = error
        last_t[0] = t
        
        # Step 4: System reacts to both u and the disturbance [cite: 117, 145]
        disturbance = 500 if t >= 50 else 0 
        dv_dt = (-k * v + u - disturbance) / m 
        return dv_dt

    sol = solve_ivp(pid_dynamics, [0, 150], [25], t_eval=np.linspace(0, 150, 1000), method='RK45')
    return sol.t, sol.y[0]

# --- Generate and Compare Plots ---
plt.figure(figsize=(12, 7))

# OPEN-LOOP (Exercise 4 scenario) [cite: 53]
t_ol, v_ol = simulate_open_loop()
plt.plot(t_ol, v_ol, 'k--', linewidth=2, label='OPEN-LOOP: Fixed force (Fails at t=50)')

# CLOSED-LOOP (Exercise 7: Proportional Only) [cite: 88]
t_p, v_p = simulate_closed_loop(Kp=1360, Ki=0, Kd=0)
plt.plot(t_p, v_p, label='CLOSED-LOOP: P-Only (Biased/Steady-state error)')

# Test 2: PI Control (Exercise 8/9)
t, v = simulate_closed_loop(Kp=1360, Ki=200, Kd=0)
plt.plot(t, v, label='PI (Kp=1360, Ki=200): Corrects speed bias')

# CLOSED-LOOP (Exercise 11/12: Full PID) [cite: 141, 144]
t_pid, v_pid = simulate_closed_loop(Kp=1360, Ki=200, Kd=400)
plt.plot(t_pid, v_pid, label='CLOSED-LOOP: PID (Smooth & Accurate recovery)')

# --- Visual Formatting ---
plt.axvline(x=50, color='red', alpha=0.5, linestyle='-', label='Disturbance (Headwind) Starts')
plt.axhline(y=25, color='green', linestyle=':', label='Desired Speed (Setpoint)')
plt.title("Cruise Control: Open-Loop vs. Closed-Loop Response")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.ylim(15, 30) # Zoomed in to see the difference clearly
plt.xlim(40, 150)
plt.legend(loc='lower left')
plt.grid(True)
plt.show()