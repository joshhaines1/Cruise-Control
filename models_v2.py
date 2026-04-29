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

# ------------------ DISTURBANCE ------------------
def disturbance(t, wind_type):
    if t < time_when_wind_starts:
        return 0

    if wind_type == 'constant':
        return disturbance_force
    elif wind_type == 'sinusoidal':
        return disturbance_force * (
            1 + 0.5 * np.sin(wind_frequency * (t - time_when_wind_starts))
        )
    else:
        return 0


# ------------------ OPEN LOOP ------------------
def simulate_open_loop():
    u_fixed = linear_resistance_coefficient * set_speed
    t_eval = np.linspace(0, 150, 1000)

    def dynamics(t, v):
        d_force = disturbance(t, wind_type)
        return (-linear_resistance_coefficient * v + u_fixed - d_force) / mass

    sol = solve_ivp(dynamics, [0, 150], [initial_velocity],
                    t_eval=t_eval, rtol=1e-8, atol=1e-8)

    v = sol.y[0]

    u = np.full_like(t_eval, u_fixed)

    dt = t_eval[1] - t_eval[0]
    gas_used = np.cumsum(u) * dt
    distance = np.cumsum(v) * dt

    efficiency = distance[-1] / gas_used[-1]

    return t_eval, v, gas_used, efficiency


# ------------------ CLOSED LOOP ------------------
def simulate_closed_loop(Kp, Ki, Kd):
    t_eval = np.linspace(0, 150, 1000)
    y0 = [initial_velocity, 0.0]

    def pid_dynamics(t, y):
        v = y[0]
        itg_err = y[1]

        error = set_speed - v
        d_force = disturbance(t, wind_type)

        dv_dt_uncontrolled = (-linear_resistance_coefficient * v - d_force) / mass
        d_error_dt = -dv_dt_uncontrolled

        u = Kp * error + Ki * itg_err + Kd * d_error_dt

        dv_dt = (-linear_resistance_coefficient * v + u - d_force) / mass

        return [dv_dt, error]

    sol = solve_ivp(pid_dynamics, [0, 150], y0,
                    t_eval=t_eval, method='RK45',
                    rtol=1e-8, atol=1e-8)

    v = sol.y[0]

    # Recompute u(t)
    u_vals = []
    itg_err = 0
    dt = t_eval[1] - t_eval[0]

    for i in range(len(t_eval)):
        v_i = v[i]
        t_i = t_eval[i]

        error = set_speed - v_i
        itg_err += error * dt

        d_force = disturbance(t_i, wind_type)
        dv_dt_uncontrolled = (-linear_resistance_coefficient * v_i - d_force) / mass
        d_error_dt = -dv_dt_uncontrolled

        u = Kp * error + Ki * itg_err + Kd * d_error_dt
        u_vals.append(u)

    u_vals = np.array(u_vals)

    gas_used = np.cumsum(u_vals) * dt
    distance = np.cumsum(v) * dt

    efficiency = distance[-1] / gas_used[-1]

    return t_eval, v, gas_used, efficiency


# ------------------ MAIN ------------------
if __name__ == "__main__":

    # --------- GRAPH 1: VELOCITY ---------
    plt.figure(figsize=(12, 6))
    plot_title = "Velocity: "
    loops = set()

    # OPEN LOOP
    if 'open_loop' in models_to_plot:
        t, v, gas, eff = simulate_open_loop()
        plt.plot(t, v, label='OPEN')
        print(f"Open-loop efficiency: {eff:.4f}")
        loops.add('Open-Loop')

    # P
    if 'p' in models_to_plot:
        t, v, gas, eff = simulate_closed_loop(1360, 0, 0)
        plt.plot(t, v, label='P')
        print(f"P efficiency: {eff:.4f}")
        loops.add('Closed-Loop')

    # PI
    if 'pi' in models_to_plot:
        t, v, gas, eff = simulate_closed_loop(1360, 400, 0)
        plt.plot(t, v, label='PI')
        print(f"PI efficiency: {eff:.4f}")
        loops.add('Closed-Loop')

    # PID
    if 'pid' in models_to_plot:
        t, v, gas, eff = simulate_closed_loop(1360, 400, 600)
        plt.plot(t, v, label='PID')
        print(f"PID efficiency: {eff:.4f}")
        loops.add('Closed-Loop')

    plt.axvline(x=time_when_wind_starts, linestyle=':', label='Disturbance')
    plt.axhline(y=set_speed, linestyle=':', label='Set Speed')

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.ylim(set_speed - 2, set_speed + 2)
    plt.xlim(time_when_wind_starts - 5, time_when_wind_starts + 15)
    plt.title(plot_title + " & ".join(loops))
    plt.legend()
    plt.grid(True)
    plt.show()


    # --------- GRAPH 2: GAS USED ---------
    plt.figure(figsize=(12, 6))
    plot_title = "Fuel Usage: "
    loops = set()

    if 'open_loop' in models_to_plot:
        t, v, gas, _ = simulate_open_loop()
        plt.plot(t, gas, label='OPEN')
        loops.add('Open-Loop')

    if 'p' in models_to_plot:
        t, v, gas, _ = simulate_closed_loop(1360, 0, 0)
        plt.plot(t, gas, label='P')
        loops.add('Closed-Loop')

    if 'pi' in models_to_plot:
        t, v, gas, _ = simulate_closed_loop(1360, 400, 0)
        plt.plot(t, gas, label='PI')
        loops.add('Closed-Loop')

    if 'pid' in models_to_plot:
        t, v, gas, _ = simulate_closed_loop(1360, 400, 600)
        plt.plot(t, gas, label='PID')
        loops.add('Closed-Loop')

    plt.axvline(x=time_when_wind_starts, linestyle=':', label='Disturbance')

    plt.xlabel("Time (s)")
    plt.ylabel("Cumulative Gas Used")
    plt.title(plot_title + " & ".join(loops))
    plt.legend()
    plt.grid(True)
    plt.show()