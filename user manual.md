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
