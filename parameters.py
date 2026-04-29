models_to_plot = ['pi', 'p', 'pid', 'open_loop'] # Options are 'open_loop', 'p', 'pi', 'pid'
mass = 2000  # mass in kg 
linear_resistance_coefficient = 40    # linear resistive coefficient 
set_speed = 25   # desired set speed (m/s)
initial_velocity = set_speed   # initial velocity (m/s)
disturbance_force = 1000  # headwind force (N) starting at t=50s
time_when_wind_starts = 50  # seconds
wind_frequency = 0.1   # angular frequency (rad/s) (only used if wind_type is 'sinusoidal')
wind_type = 'constant'  # 'constant' or 'sinusoidal'
wind_speed = -25 #wind speed in direction of car