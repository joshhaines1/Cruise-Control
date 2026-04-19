import sys
import os
import matplotlib.pyplot as plt
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from models import simulate_closed_loop
from parameters import time_when_wind_starts, set_speed

def run_sensitivity_analysis():
    kd_values = [-1000, -500, 500, 1000]  # Range of Kd values to analyze
    plt.figure(figsize=(12, 7))

    for value in kd_values:
        t, v = simulate_closed_loop(Kp=1360, Ki=200, Kd=value)
        plt.plot(t, v, label='PID Kd=' + str(value))

    plt.axvline(x=time_when_wind_starts, color='red', alpha=1, linestyle=':', label='Headwind Starts')
    plt.axhline(y=set_speed, color='green', linestyle=':', label='Desired Speed (Setpoint)')
    plt.title("Cruise Control: PID Controller Sensitivity Analysis")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (mass/s)")
    plt.ylim(set_speed - 2, set_speed + 2) 
    plt.xlim(time_when_wind_starts - 10, time_when_wind_starts + 10)
    plt.legend(loc='lower left')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    run_sensitivity_analysis()
