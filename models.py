import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from parameters import (
    models_to_plot,
    mass,
    wind_frequency,
    wind_type,
    linear_resistance_coefficient,
    set_speed,
    initial_velocity,
    disturbance_force,
    time_when_wind_starts
)

def disturbance(t, wind_type):
    # No disturbance before the wind starts
    if t < time_when_wind_starts:
        return 0

    if wind_type == 'constant':
        return disturbance_force
    elif wind_type == 'sinusoidal':
        return disturbance_force * (1 + 0.5 * np.sin(wind_frequency * (t - time_when_wind_starts)))
    else:
        return 0

# OPEN-LOOP SYSTEM
def simulate_open_loop():
    # Force required to maintain set_speed on level ground with no wind
    u_fixed = linear_resistance_coefficient * set_speed

    def dynamics(t, v):
        d_force = disturbance(t, wind_type)
        return (-linear_resistance_coefficient * v + u_fixed - d_force) / mass

    sol = solve_ivp(dynamics,[0, 150],[initial_velocity], t_eval=np.linspace(0, 150, 1000), rtol=1e-8, atol=1e-8)
    return sol.t, sol.y[0]


# CLOSED-LOOP SYSTEM (PID)
def simulate_closed_loop(Kp, Ki, Kd):
    y0 = [initial_velocity, 0.0]  # [velocity, integral of error]

    def pid_dynamics(t, y):
        v = y[0]
        itg_err = y[1]

        # Error
        error = set_speed - v

        # Disturbance
        d_force = disturbance(t, wind_type)

        # Uncontrolled dynamics (used for derivative term)
        dv_dt_uncontrolled = (-linear_resistance_coefficient * v - d_force) / mass

        # Derivative of error
        d_error_dt = -dv_dt_uncontrolled

        # PID control input
        u = Kp * error + Ki * itg_err + Kd * d_error_dt

        # Final system dynamics
        dv_dt = (-linear_resistance_coefficient * v + u - d_force) / mass

        return [dv_dt, error]

    sol = solve_ivp(pid_dynamics, [0, 150], y0, t_eval=np.linspace(0, 150, 1000), method='RK45', rtol=1e-8, atol=1e-8)
    return sol.t, sol.y[0]


if __name__ == "__main__":
    plt.figure(figsize=(12, 7))
    plot_title = "Cruise Control: "
    loops = set()

    # OPEN-LOOP
    if 'open_loop' in models_to_plot:
        t_ol, v_ol = simulate_open_loop()
        plt.plot(t_ol, v_ol, label='OPEN-LOOP: Fixed force (Fails at disturbance)')
        loops.add('Open-Loop')

    # P-ONLY
    if 'p' in models_to_plot:
        t_p, v_p = simulate_closed_loop(Kp=1360, Ki=0, Kd=0)
        plt.plot(t_p, v_p, label='P-Only (Steady-state error)')
        loops.add('Closed-Loop')

    # PI
    if 'pi' in models_to_plot:
        t_pi, v_pi = simulate_closed_loop(Kp=1360, Ki=400, Kd=0)
        plt.plot(t_pi, v_pi, label='PI (Corrects steady-state error)')
        loops.add('Closed-Loop')

    # PID
    if 'pid' in models_to_plot:
        t_pid, v_pid = simulate_closed_loop(Kp=1360, Ki=400, Kd=600)
        plt.plot(t_pid, v_pid, label='PID (Smooth & accurate recovery)')
        loops.add('Closed-Loop')

    if len(loops) > 1:
        plot_title += " & ".join(loops)
    else:
        plot_title += loops.pop()

    # Visual markers
    plt.axvline(
        x=time_when_wind_starts,
        color='red',
        linestyle=':',
        label='Disturbance Starts'
    )
    plt.axhline(
        y=set_speed,
        color='green',
        linestyle=':',
        label='Set Speed'
    )

    plt.title(plot_title)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.ylim(set_speed - 2, set_speed + 2)
    plt.xlim(time_when_wind_starts - 5, time_when_wind_starts + 15)
    plt.legend(loc='lower left')
    plt.grid(True)
    plt.show()