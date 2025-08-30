### Guide to Using the Helicopter Mission Planner Code

This guide provides a detailed walkthrough on how to use your helicopter mission planning code, from setting up a new helicopter design to simulating a mission and analyzing the results.

The code is structured into several interconnected modules:

  * `statistical_design.py`: Defines the helicopter's physical parameters (weight, dimensions) based on high-level mission requirements.
  * `calculating_state.py`: A `Helicopterstate_simulator` class that models the helicopter's performance (thrust, power, fuel burn) at a given point in time.
  * `Mission.py`: The `Mission_planner` class that orchestrates the entire mission simulation, linking the design to the simulator.
  * `main.py`: The main entry point to set up and run a mission.
  * `helper.py`: Contains low-level helper functions for physics, conversions, and geometric calculations.
  * `Heli_engine_data - Sheet1.csv`: A data file for engine performance.

-----

### Step 1: Helicopter Design Configuration

The first step is to define the high-level mission requirements for the helicopter. This is done in the `main.py` file within the `params` dictionary. The `Mission_planner` uses these inputs to call the `HelicopterDesigner` and create a plausible helicopter design.

**`main.py`**

```python
params = {
    # Initial estimate of take-off weight (kg)
    "W0_guess": 4000, 
    # Target payload weight (kg)
    "W_pl_target": 0,
    # Number of crew/passengers
    "crew": 15,
    # Target range (km)
    "Rg_target": 439, 
    # Maximum forward speed (m/s)
    "V_max": 200,
    # Number of main rotor blades
    "Nb": 2,
    # Number of tail rotor blades
    "Nb_tr": 2
}
```

You can modify these values to design different types of helicopters. For instance, to design a heavy-lift helicopter, you would increase `W0_guess` and `W_pl_target`.

-----

### Step 2: Defining a Mission Profile

Once the helicopter is designed, the `Mission_planner` is ready to simulate a flight. The mission is defined as a dictionary passed to the `run_mission` method. The planner executes each phase of the mission in the order you define them.

Here are the supported mission phases:

  * **`climb`**: Simulates a vertical climb from the current altitude.
  * **`hover`**: Simulates hovering at a fixed altitude for a specified duration.
  * **`descent`**: (Not implemented in the provided code, but you could add this).
  * **`cruise`**: (Not implemented in the provided code, but you could add this).

#### A. The `climb` Phase

The `climb` phase can be configured in two ways:

1.  **Time-based Climb (`time` parameter):** The simulator will climb at a constant vertical velocity to reach the `target_altitude` in a specific `time`. This is useful for planning a mission with a fixed schedule.
2.  **Pitch-based Climb (`pitch_ramp_rate` parameter):** The simulator will increase the rotor's collective pitch at a constant rate, and the helicopter will climb as fast as the increasing thrust allows. The simulation will stop when the `target_altitude` is reached or if the blades stall or exceed engine power limits. This is more realistic for performance analysis.

**Example Mission Profiles:**

```python
# Mission Profile 1: Climb to 400m in 20 seconds
mission_1 = {
    "climb": {
        "target_altitude": 400,
        "time": 20
    }
}

# Mission Profile 2: Climb to 500m by ramping up pitch at 0.5 deg/s
mission_2 = {
    "climb": {
        "target_altitude": 500,
        "pitch_ramp_rate": 0.5
    }
}
```

#### B. The `hover` Phase

The `hover` phase requires a target `altitude` and a `duration` in seconds. The simulator will automatically climb to the specified altitude first (using the pitch-based climb method) and then hold that altitude for the duration.

**Example Mission Profile:**

```python
# Mission Profile 3: Climb to 25m, then hover for 100 seconds
mission_3 = {
    "hover": {
        "altitude": 25,
        "duration": 100
    }
}
```

To run a mission, you just need to pass the mission dictionary to the `run_mission` method in `main.py`.

```python
# In main.py:
planner = Mission_planner(params)
result = planner.run_mission(mission_profile=mission_3)
```

-----

### Step 3: Running the Simulation and Analyzing Results

When you run `main.py`, the simulation will begin.

1.  **Real-Time Log:** The terminal will print a detailed, step-by-step log of the simulation. This includes the current time, altitude, fuel consumption, and power output at each `dt` time step. This log gives you a real-time view of the helicopter's performance.

2.  **Visualization:** After the simulation is complete, the `visualize_mission` method generates several plots to help you analyze the mission performance.

      * **Altitude vs. Time:** Shows the helicopter's flight path.
      * **Pitch vs. Time:** Tracks the collective pitch angle, which corresponds to the pilot's control input.
      * **Power vs. Time:** Illustrates the engine power required at each moment. This is crucial for checking if the helicopter's engine is powerful enough for the mission.
      * **Fuel vs. Time:** Shows the rate of fuel consumption over the mission duration. This is used to verify if the helicopter has enough fuel to complete the mission.

By examining these plots, you can quickly assess if the helicopter design can successfully complete the mission profile and identify any potential issues like a lack of power or a high rate of fuel burn.

-----

### README.md

#### Helicopter Mission Planner and Performance Analysis

This repository contains a Python-based tool for the preliminary design, mission planning, and performance analysis of a single-rotor helicopter. The code is structured to allow for easy modification of design parameters and mission profiles.

#### 📁 File Structure

```
├── main.py
├── Mission.py
├── calculating_state.py
├── statistical_design.py
├── helper.py
└── Heli_engine_data - Sheet1.csv
```

#### 🛠️ Prerequisites

To run this code, you need a Python environment with the following libraries installed.

```bash
pip install numpy matplotlib
```

#### ▶️ How to Run

1.  Open the `main.py` file.

2.  Modify the `params` dictionary to define your desired helicopter design and mission requirements.

3.  Modify the `mission_profile` dictionary to define the mission segments (climb, hover, etc.).

4.  Run the script from your terminal:

    ```bash
    python main.py
    ```

#### ⚙️ Configuration

The core configuration is in the `main.py` file.

**Helicopter Design Parameters (`params` dictionary):**

  - `W0_guess`: Initial guess for the gross weight in kg.
  - `W_pl_target`: Target payload weight in kg.
  - `crew`: Number of crew/passengers.
  - `Rg_target`: Target range in km.
  - `V_max`: Maximum forward speed in m/s.
  - `Nb`: Number of main rotor blades.

**Mission Profile (`mission_profile` dictionary):**

The `run_mission` function accepts a dictionary defining a sequence of mission phases. The available phases are:

  - **`climb`**:

      - `target_altitude` (meters)
      - `time` (seconds): For a constant-rate climb.
      - `pitch_ramp_rate` (deg/s): For a pitch-based climb.

  - **`hover`**:

      - `altitude` (meters)
      - `duration` (seconds)

**Example Mission:**

```python
# Climb to 1000m, then hover at 1000m for 300 seconds
mission_profile = {
    "climb": {
        "target_altitude": 1000,
        "pitch_ramp_rate": 0.5
    },
    "hover": {
        "altitude": 1000,
        "duration": 300
    }
}
```

#### 📊 Output and Visualization

Upon running the script, the program will:

1.  **Print a real-time log** to the console, showing the helicopter's state at each time step.
2.  **Generate a series of plots** displaying the mission profile, including altitude, pitch, power, and fuel consumption over time. These plots are essential for a detailed performance analysis.
