models_to_plot = ['pi', 'pid', 'open_loop', 'p'] # Options are 'open_loop', 'p', 'pi', 'pid'
mass = 2000  # mass in kg 
linear_resistance_coefficient = 40    # linear resistive coefficient 
set_speed = 25   # desired set speed (m/s)
initial_velocity = set_speed   # initial velocity (m/s)
disturbance_force = 500  # headwind force (N) starting at t=50s
time_when_wind_starts = 10  # seconds