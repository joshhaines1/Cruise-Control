# Ordinary Differential Equations Cruise Control Simulation

## Overview
This project simulates a cruise control system for an Ordinary Differential Equations (ODE) class. Adjust parameters like target speed and gain.

## Installation
- Install Python: [python.org](https://www.python.org/)

## Usage
1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/cruise-control-simulation.git
   cd cruise-control-simulation
   ```
2. **Install dependencies**:
     ```bash
     pip install -r requirements.txt
     ```

3. **Change Parameters**:
   - Edit `parameters.py`:
     ```python
        models_to_plot = ['open_loop', 'p', 'pi', 'pid'] # Options are 'open_loop', 'p', 'pi', 'pid'
        m = 2000  # mass in kg 
        k = 40    # linear resistive coefficient 
        r0 = 25   # desired set speed (m/s)
        disturbance_force = 500  # headwind force (N) starting at t=50s
        time_when_wind_starts = 50  # seconds
     ```

4. **Run the simulation**:
     ```bash
     python models.py
     ```

## License
MIT License - see [LICENSE](LICENSE).