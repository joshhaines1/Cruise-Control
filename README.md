# Ordinary Differential Equations Cruise Control Simulation
## Project Created by Josh Haines and Amos Ebeling
### Overview
The purpose of this project is to design and analyze a control system that maintains a vehicle's velocity at a desired setpoint (r0​=25 m/s) despite external disturbances. In a real-world driving scenario, there are many factors that can affect a system such as headwinds, which we will be analyzing in this project. A human driver can manually adjust the acceleration after seeing on the speedometer that they are going too slow or too fast, but an automated cruise control system must use math to calculate the necessary control force.
The primary challenge addressed in this report is how the car reacts when a constant 500 N headwind is introduced at t=50 s. We evaluate both "Open-Loop" and "Closed-Loop" strategies to determine which provides the most reliable and comfortable passenger experience. An open loop model does not monitor the car’s velocity and is blindly applying force. A closed loop model is constantly monitoring the car’s velocity and applying force based on how far off of the set speed the car is traveling. 


### Installation
- Install Python: [python.org](https://www.python.org/)

### Usage
1. **Clone the repository**:
   ```bash
   git clone https://github.com/joshhaines1/Cruise-Control.git
   cd Cruise-Control
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
