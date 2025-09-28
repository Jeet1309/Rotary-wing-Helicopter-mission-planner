# Helicopter Mission Planner: User Manual

## 1\. Introduction

Welcome to the Helicopter Mission Planner\! This program is an easy-to-use tool that allows you to design a helicopter based on your mission requirements and then simulate a flight to see how it performs. You can define various flight phases like climbing, cruising, hovering, and descending.

The program handles all the complex calculations behind the scenes and provides you with a detailed log file and a set of graphical plots to visualize the results.

This manual will guide you through the process, from setting up the project to understanding the final output.

## 2\. Getting Started

### Prerequisites

Before you can run the program, you need to ensure you have Python installed on your computer. You also need a few essential Python libraries, which you can install with the following command in your terminal or command prompt:

```bash
pip install numpy matplotlib pandas
```

### File Setup

Make sure all the following files are located in the **same folder** on your computer:

  * `main.py`
  * `Mission.py`
  * `calculating_state.py`
  * `statistical_design.py`
  * `performance_tool.py`
  * `result_gen.py`
  * `performance_benchmark.py`
  * `Heli_engine_data - Sheet1.csv`
  * `Paper_results.xlsx - 2blade.csv`
  * `Paper_results.xlsx - 3blade.csv`
  * `Paper_results.xlsx - 4blade.csv`
  * `Paper_results.xlsx - 5blade.csv`

### Running the Program

1.  Open your terminal or command prompt.

2.  Navigate to the folder where you saved the files. You can use the `cd` command (e.g., `cd path/to/your/folder`).

3.  Run the main script using the following command:

    ```bash
    python main.py
    ```

## 3\. Step-by-Step Guide

The program will run in two main phases. Just follow the on-screen prompts.

### Phase 1: Helicopter Design Input

The program will first ask you for a few key design parameters. This phase performs an **iterative design calculation** to find a stable helicopter configuration based on your requirements.

You will be prompted to enter the following information. Enter a number for each and press `Enter`.

  * **Enter initial Gross Weight (kg)**: This is your initial guess for the helicopter's total weight.
      * *Example:* `4000`
  * **Enter Payload Weight (kg)**: The weight of the cargo or passengers.
      * *Example:* `500`
  * **Enter Max Speed (m/s)**: The maximum speed the helicopter can achieve in meters per second.
      * *Example:* `200`
  * **Enter number of crew**: The number of crew members. The program assumes each crew member weighs 70 kg.
      * *Example:* `2`
  * **Enter Range (km)**: The total mission range in kilometers.
      * *Example:* `439`
  * **Enter number of main rotor blades**: The number of blades on the main rotor.
      * *Example:* `4`
  * **Enter number of tail rotor blades**: The number of blades on the tail rotor.
      * *Example:* `2`

### Phase 2: Mission Profile Definition

After the design is finalized, the program will prompt you to build your mission profile step-by-step.

You can enter one of the following mission step types: `climb`, `cruise`, `hover`, or `descend`. After entering the type, you will be asked for specific details for that step.

To finish your mission profile and start the simulation, simply type `done`.

Here is a breakdown of the inputs for each step type:

  * **`climb`**
      * **Enter target altitude (m)**: The altitude you want to climb to.
      * **Enter climb velocity (m/s)**: The speed at which you want to ascend.
  * **`cruise`**
      * **Enter target altitude (m)**: The altitude you want to cruise at.
      * **Enter cruise duration (s)**: How long you want to fly at this altitude.
  * **`hover`**
      * **Enter hover altitude (m)**: The altitude where you want to hover.
      * **Enter hover duration (s)**: How long you want to hover.
  * **`descend`**
      * **Enter target altitude (m)**: The altitude you want to descend to.
      * **Enter descend velocity (m/s)**: The speed at which you want to descend.

**Example Mission Profile:**

1.  Type `climb` and press `Enter`.
2.  Enter `1000` for target altitude.
3.  Enter `5` for climb velocity.
4.  Type `cruise` and press `Enter`.
5.  Enter `1000` for target altitude.
6.  Enter `3600` for cruise duration (1 hour).
7.  Type `descend` and press `Enter`.
8.  Enter `0` for target altitude.
9.  Enter `3` for descend velocity.
10. Type `done` to start the simulation.

## 4\. Understanding the Output

Once the mission simulation is complete, the program will generate and save a few key output files and display some information on your screen.

### 📊 Plots

The program will generate and save several **`.svg`** files in the same folder. These vector graphics can be scaled without losing quality. They provide a clear visual summary of the mission.

  * `Altitude_vs_Time.svg`: Shows the helicopter's altitude throughout the mission, with each step clearly defined.
  * `Power_vs_Time.svg`: Graphs the power required by the helicopter at each moment in time. This is useful for identifying the most power-intensive phases.
  * `Fuel_vs_Time.svg`: Shows the cumulative fuel consumption over the mission.
  * `Weight_vs_Time.svg`: Tracks the helicopter's weight as fuel is burned.

A combined plot showing all of these graphs will also be displayed on your screen after the program finishes.

### 📄 Mission Log File

A file named `mission.csv` will be created. This is a detailed log of every time step of the simulation. It's an excellent resource for anyone who wants to perform their own data analysis. The columns include:

  * `time (s)`: The mission time.
  * `height (m)`: The helicopter's altitude.
  * `power_kw`: The power required at that time step.
  * `fuel_kg`: The total fuel burned so far.
  * `weight_kg`: The total weight of the helicopter.
  * `pitch (deg)`: The required collective pitch of the rotor blades.
  * `vertical_velocity (m/s)`: The climb or descent velocity.

### 📝 Console Output

The program will also print the final, converged design parameters to the console. This includes the final gross weight, rotor radius, and the recommended engine from the `Heli_engine_data` dataset. This confirms that the design phase was successful before the simulation began.

This is a detailed guide to the Rotary-wing Helicopter Mission Planner. This manual will explain the purpose of each file, its classes, and its methods, to help you understand, use, and even build upon this project.


## 📁 File-by-File Breakdown

### 1\. `main.py` - The Starting Point

This is the main script that you run to start the application. It orchestrates the entire process, from getting user input to running the simulation and generating results.

#### **`get_user_input()` function:**

  * **Purpose:** Prompts the user to enter the helicopter's design parameters and the desired mission profile.
  * **User Inputs:**
      * **Helicopter Design:** Gross Weight, Payload Weight, Max Speed, number of crew, Range, number of main and tail rotor blades.
      * **Mission Profile:** A series of steps, where each step is either a "climb" (to a certain altitude) or a "hover" (for a specific duration).
  * **Returns:** A tuple containing a dictionary of the design parameters and a list representing the mission profile.

#### **`main()` function:**

  * **Purpose:** The entry point of the program.
  * **Workflow:**
    1.  Calls `get_user_input()` to get the necessary parameters.
    2.  Initializes the `Mission_planner` class from `Mission.py` with the design parameters.
    3.  Runs the mission simulation using the `run_mission()` method.
    4.  If the mission is successful, it calls `visualize_mission()` to generate plots of the results.
    5.  It then calls `run_all_calculations_and_plots()` from `result_gen.py` to generate more detailed plots about the helicopter's performance characteristics.

-----

### 2\. `statistical_design.py` - Designing the Helicopter

This file contains the `HelicopterDesigner` class, which is responsible for the initial design of the helicopter based on statistical data and empirical formulas.

#### **`HelicopterDesigner` class:**

  * **`__init__(self, W0, V_max, Nb, Nb_tr, W_pl, crew, Rg, rho_f)`**

      * **Purpose:** Initializes the designer with the core design parameters provided by the user.
      * **Arguments:**
          * `W0`: Gross weight (kg)
          * `V_max`: Maximum speed (m/s)
          * `Nb`: Number of main rotor blades
          * `Nb_tr`: Number of tail rotor blades
          * `W_pl`: Payload weight (kg)
          * `crew`: Number of crew members
          * `Rg`: Range (km)
          * `rho_f`: Fuel density (kg/L)

  * **`calculate_main_rotor_parameters(self)`:** Calculates main rotor parameters like radius, chord length, and solidity.

  * **`calculate_tail_rotor_parameters(self)`:** Calculates tail rotor parameters.

  * **`calculate_weights(self, W0)`:** Calculates the weights of different helicopter components (engine, fuselage, fuel, etc.) based on the gross weight. It uses statistical formulas to estimate these weights.

  * **`suggest_engine(self, required_power)`:** Suggests a suitable engine from the `Heli_engine_data - Sheet1.csv` file based on the calculated required power.

  * **`iterative_gross_weight_calculator(self, W0_guess)`:** This is a key method. It iteratively refines the helicopter's gross weight. It starts with an initial guess and repeatedly calculates the component weights until the calculated gross weight converges to a stable value.

  * **`run_calculations(self)`:** Runs all the necessary calculations to define the final helicopter design. It returns a dictionary containing all the design parameters.

-----

### 3\. `performance_tool.py` - The Physics Engine

This file is the core of the helicopter's performance calculations. It contains functions for aerodynamics, atmospheric modeling, and the main Blade Element Momentum Theory (BEMT) calculations.

#### **Key Functions:**

  * **Unit Conversions:** A set of helper functions to convert between different units (e.g., `rpm_to_rad_s`, `deg_to_rad`).

  * **`isa(h)`:** Implements the International Standard Atmosphere (ISA) model to calculate air density, pressure, and temperature at a given altitude `h`.

  * **`aerod(V, phi, lift_slope, Theta)`:** Calculates the lift and drag coefficients for an airfoil section.

  * **`TPQ(params, h, main_rotor=True)`:** This is the heart of the performance calculations. It stands for **Thrust, Power, and Torque**.

      * **Purpose:** Implements the Blade Element Momentum Theory (BEMT) to calculate the thrust, power, and torque of the rotor.
      * **Arguments:**
          * `params`: A dictionary of rotor parameters.
          * `h`: The current altitude.
          * `main_rotor`: A boolean to specify if the calculation is for the main rotor or the tail rotor.
      * **Returns:** A tuple containing Thrust (N), Torque (Nm), and Power (kW).

  * **`available_engine_power(power_sl, h)`:** Calculates the available engine power at a given altitude, accounting for the decrease in power with altitude.

  * **`fuel_flow(power, sfc, dt)`:** Calculates the fuel consumed over a time step `dt`, based on the required power and the engine's Specific Fuel Consumption (SFC).

-----

### 4\. `calculating_state.py` - Simulating the Flight

This file simulates the helicopter's state as it progresses through the mission.

#### **`Helicopterstate_simulator` class:**

  * **`__init__(self, design_parameters, g=9.81)`:**

      * **Purpose:** Initializes the simulator with the final design parameters of the helicopter.
      * **Arguments:**
          * `design_parameters`: The dictionary of design parameters from the `HelicopterDesigner` class.
          * `g`: Gravitational acceleration.

  * **`_calculate_blade_properties(...)`:** A helper function that calls the `TPQ` function from `performance_tool.py` to get the rotor's performance characteristics.

  * **`_newton_solver(...)`:** A numerical solver used to find the required blade pitch angle to achieve a desired thrust.

  * **`calculate_hover_performance(self, height_m)`:** Calculates the required pitch and power to hover at a specific altitude.

  * **`update_state_during_climb(self, target_height, dt)`:**

      * **Purpose:** Updates the helicopter's state during a climb.
      * **Logic:**
        1.  Calculates the power required to climb at a certain vertical velocity.
        2.  Adjusts the vertical velocity based on the available power.
        3.  Calculates fuel burned during the time step `dt`.
        4.  Updates the helicopter's weight and current height.

-----

### 5\. `Mission.py` - Managing the Mission

This file defines and executes the mission profile.

#### **`Mission_planner` class:**

  * **`__init__(self, param)`:**

      * **Purpose:** Initializes the mission planner with the user-defined parameters. It also creates an instance of the `HelicopterDesigner` to get the final helicopter design.

  * **`run_mission(self, mission_profile, dt)`:**

      * **Purpose:** Executes the mission step by step.
      * **Logic:**
        1.  Initializes the `Helicopterstate_simulator`.
        2.  Loops through each step in the `mission_profile`.
        3.  For each "climb" step, it calls the simulator's `update_state_during_climb` method repeatedly until the target altitude is reached.
        4.  For each "hover" step, it calculates the hover performance and updates the state for the specified duration.
        5.  It logs the helicopter's state at each time step.
      * **Returns:** A list of dictionaries, where each dictionary represents the state of the helicopter at a specific point in time (the mission log).

#### **`visualize_mission(logs)` function:**

  * **Purpose:** Takes the mission log and generates a set of plots to visualize the mission profile.
  * **Plots:**
      * Altitude vs. Time
      * Pitch vs. Time
      * Power vs. Time
      * Fuel vs. Time
      * Weight vs. Time

-----

### 6\. `result_gen.py` - Visualizing the Results

This file is dedicated to generating more detailed plots about the helicopter's performance characteristics, independent of a specific mission.

#### **`run_all_calculations_and_plots(design_params)` function:**

  * **Purpose:** To generate a series of plots that characterize the helicopter's performance.
  * **Plots:**
      * **Thrust vs. Collective Pitch:** Shows how the thrust of the main and tail rotors changes with the collective pitch angle.
      * **Torque vs. Collective Pitch:** Shows how the torque of the main and tail rotors changes with the collective pitch angle.
      * **Power vs. Thrust:** A key performance plot showing how much power is required to generate a certain amount of thrust.

-----

### 7\. Other Files

  * **`performance_benchmark.py` and `import_paper_results.py`**: These files are used for validating the BEMT code against experimental data from a research paper. They are not essential for running a mission simulation but are important for verifying the accuracy of the physics model.
  * **`Heli_engine_data - Sheet1.csv`**: A CSV file containing a list of helicopter engines, their power output, and their specific fuel consumption.
  * **`Paper_results.xlsx - *.csv`**: These files contain the experimental data used for validation in `performance_benchmark.py`.

-----

## 🛠️ How to Use the Mission Planner

1.  **Run `main.py`:** Execute the `main.py` script from your terminal:

    ```bash
    python main.py
    ```

2.  **Enter Helicopter Design Parameters:** The script will prompt you to enter the initial design parameters for your helicopter.

3.  **Define the Mission Profile:** You will then be asked to define the mission by adding "climb" and "hover" steps.

4.  **Review the Output:**

      * The console will show the progress of the simulation.
      * Once the mission is complete, a series of plots will be displayed, visualizing the mission.
      * More plots will be generated to show the detailed performance characteristics of the designed helicopter.
      * A `mission.csv` file will be created, containing the detailed log of the mission.

This comprehensive guide should give you a solid understanding of the Rotary-wing Helicopter Mission Planner. You can now explore the code in more detail, modify it to suit your needs, or use it as a foundation for your own helicopter simulation projects. Happy flying\! 🚁
