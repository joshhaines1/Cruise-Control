import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from parameters import models_to_plot, mass, linear_resistance_coefficient, set_speed, initial_velocity, disturbance_force, time_when_wind_starts

# The open loop system does not monitor v(t). It provides a fixed force u based solely on the target set_speed, assuming no disturbances.
def simulate_open_loop():
    # Force required to maintain set_speed on level ground with no wind
    u_fixed = linear_resistance_coefficient * set_speed
    
    def dynamics(t, v):
        # The controller doesn't "see" this disturbance 
        disturbance = disturbance_force if t >= time_when_wind_starts else 0 
        return (-linear_resistance_coefficient * v + u_fixed - disturbance) / mass
    
    sol = solve_ivp(dynamics, [0, 150], [initial_velocity], t_eval=np.linspace(0, 150, 1000))
    return sol.t, sol.y[0]

# These closed-loop systems monitor the actual velocity v(t) and calculate an error signal e(t) = r(t) - v(t) to adjust the force u.
def simulate_closed_loop(Kp, Ki, Kd):
    y0 = [initial_velocity, 0.0] 

    def pid_dynamics(t, y):
        v = y[0]
        itg_err = y[1]
        
        error = set_speed - v
        disturb = disturbance_force if t >= time_when_wind_starts else 0
        
        # Calculate acceleration (dv/dt) using the integrated mass formula
        # This prevents the circular dependency of the D-term
        dv_dt = (-linear_resistance_coefficient * v + Kp * error + Ki * itg_err - disturb) / (mass + Kd)
        
        # The derivative of the integral_error state is just the current error
        return [dv_dt, error]

    sol = solve_ivp(pid_dynamics, [0, 150], y0, t_eval=np.linspace(0, 150, 1000), method='RK45')
    return sol.t, sol.y[0]

if __name__ == "__main__":
    # Plot graphs for all selected models
    plt.figure(figsize=(12, 7))

    # OPEN-LOOP (Exercise 4 scenario)
    if 'open_loop' in models_to_plot:
        t_ol, v_ol = simulate_open_loop()
        plt.plot(t_ol, v_ol, label='OPEN-LOOP: Fixed force (Fails at t=50)')

    # CLOSED-LOOP P Control (Exercise 7: Proportional Only)
    if 'p' in models_to_plot:
        t_p, v_p = simulate_closed_loop(Kp=1360, Ki=0, Kd=0)
        plt.plot(t_p, v_p, label='P-Only (Biased/Steady-state error)')

    # CLOSED-LOOP: PI Control (Exercise 8/9: Integral to eliminate steady-state error)
    if 'pi' in models_to_plot:
        t, v = simulate_closed_loop(Kp=1360, Ki=400, Kd=0)
        plt.plot(t, v, label='PI Corrects speed bias')

    # CLOSED-LOOP: PID Control (Exercise 11/12: Full PID - Adds derivative for better response)
    if 'pid' in models_to_plot:
        t_pid, v_pid = simulate_closed_loop(Kp=1360, Ki=200, Kd=400)
        plt.plot(t_pid, v_pid, label='PID (Smooth & Accurate recovery)')

    plt.axvline(x=time_when_wind_starts, color='red', alpha=1, linestyle=':', label='Disturbance (Headwind) Starts')
    plt.axhline(y=set_speed, color='green', linestyle=':', label='Desired Speed (Setpoint)')
    plt.title("Cruise Control: Open-Loop vs. Closed-Loop Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (mass/s)")
    plt.ylim(set_speed - 2, set_speed + 2) 
    plt.xlim(time_when_wind_starts - 10, time_when_wind_starts + 10)
    plt.legend(loc='lower left')
    plt.grid(True)
    plt.show()