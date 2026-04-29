import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from parameters import models_to_plot, mass, wind_frequency, wind_type, linear_resistance_coefficient, quadratic_resistance_coefficient, set_speed, initial_velocity, disturbance_force, time_when_wind_starts, wind_speed

# The open loop system does not monitor v(t). It provides a fixed force u based solely on the target set_speed, assuming no disturbances.
def simulate_open_loop():
    # Force required to maintain set_speed on level ground with no wind
    u_fixed = linear_resistance_coefficient * set_speed * set_speed
    
    def dynamics(t, v):
        # The controller doesn't "see" this disturbance 
        if wind_type == 'constant':
            disturbance = disturbance_force if t >= time_when_wind_starts else 0
        elif wind_type == 'sinusoidal':
           disturbance = disturbance_force * (1 + 0.5 * np.sin(wind_frequency * (t - time_when_wind_starts))) if t >= time_when_wind_starts else 0
        
        return (-linear_resistance_coefficient * (v - wind_speed if t >= time_when_wind_starts else v) * (v - wind_speed if t >= time_when_wind_starts else v) + u_fixed - disturbance) / mass
    
    sol = solve_ivp(dynamics, [0, 150], [initial_velocity], t_eval=np.linspace(0, 150, 1000), rtol=1e-8, atol=1e-8)
    return sol.t, sol.y[0]

# These closed-loop systems monitor the actual velocity v(t) and calculate an error signal e(t) = r(t) - v(t) to adjust the force u.
def simulate_closed_loop(Kp, Ki, Kd):
    y0 = [initial_velocity, 0.0] 

    def pid_dynamics(t, y):
        v = y[0]
        itg_err = y[1]

        # Error
        error = set_speed - v

        # Disturbance
        if wind_type == 'constant':
            disturbance = disturbance_force if t >= time_when_wind_starts else 0
        elif wind_type == 'sinusoidal':
            disturbance = (
                disturbance_force * (1 + 0.5 * np.sin(wind_frequency * (t - time_when_wind_starts)))
                if t >= time_when_wind_starts else 0
            )
        else:
            disturbance = 0

        v_rel = v - wind_speed if t >= time_when_wind_starts else v
        drag = quadratic_resistance_coefficient * v_rel * abs(v_rel)

        dv_dt_uncontrolled = (-drag - disturbance) / mass
        d_error_dt = -dv_dt_uncontrolled

        u = Kp * error + Ki * itg_err + Kd * d_error_dt
        dv_dt = (-drag + u - disturbance) / mass
        return [dv_dt, error]

    sol = solve_ivp(pid_dynamics, [0, 150], y0, t_eval=np.linspace(0, 150, 1000), method='RK45')
    return sol.t, sol.y[0]

if __name__ == "__main__":
    # Plot graphs for all selected models
    plt.figure(figsize=(12, 7))
    plot_title = "Cruise Control: "
    loops = set()  
    # OPEN-LOOP (Exercise 4 scenario)
    if 'open_loop' in models_to_plot:
        t_ol, v_ol = simulate_open_loop()
        plt.plot(t_ol, v_ol, label='OPEN-LOOP: Fixed force (Fails at t=50)')
        loops.add('Open-Loop')

    # CLOSED-LOOP P Control (Exercise 7: Proportional Only)
    if 'p' in models_to_plot:
        t_p, v_p = simulate_closed_loop(Kp=1360, Ki=0, Kd=0)
        plt.plot(t_p, v_p, label='P-Only (Biased/Steady-state error)')
        loops.add('Closed-Loop')

    # CLOSED-LOOP: PI Control (Exercise 8/9: Integral to eliminate steady-state error)
    if 'pi' in models_to_plot:
        t, v = simulate_closed_loop(Kp=1360, Ki=400, Kd=0)
        plt.plot(t, v, label='PI Corrects speed bias')
        loops.add('Closed-Loop')

    # CLOSED-LOOP: PID Control (Exercise 11/12: Full PID - Adds derivative for better response)
    if 'pid' in models_to_plot:
        t_pid, v_pid = simulate_closed_loop(Kp=1360, Ki=400, Kd=600)
        plt.plot(t_pid, v_pid, label='PID (Smooth & Accurate recovery)')
        loops.add('Closed-Loop')

    if len(loops) > 1:
        plot_title += " & ".join(loops)
    else:        
        plot_title += loops.pop()

    plt.axvline(x=time_when_wind_starts, color='red', alpha=1, linestyle=':', label='Disturbance (Headwind) Starts')
    plt.axhline(y=set_speed, color='green', linestyle=':', label='Desired Speed (Setpoint)') 
    plt.title(plot_title)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (mass/s)")
    plt.ylim(set_speed - 2, set_speed + 2) 
    plt.xlim(time_when_wind_starts - 5, time_when_wind_starts + 15)
    plt.legend(loc='lower left')
    plt.grid(True)
    plt.show()