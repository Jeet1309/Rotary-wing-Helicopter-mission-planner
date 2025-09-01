

# Helicopter Design and Mission Simulator

A comprehensive Python tool for the conceptual design, performance analysis, and mission simulation of helicopters.

-----

## 🚁 Table of Contents

  - Introduction
  - Core Concepts & Theory
      - Phase 1: Statistical Design
      - Phase 2: Physics-Based Simulation (BEMT)
  - Project Workflow
  - File Structure
  - Installation and Usage
      - Prerequisites
      - Installation
      - Running the Simulator
  - Understanding the Output
  - Limitations & Future Work

-----

## 📖 Introduction

This project offers a comprehensive workflow for helicopter design, spanning from high-level requirements to detailed mission performance analysis. It's designed for engineers, students, and enthusiasts who want to explore how initial parameters (like payload and range) influence a helicopter's final design and its flight capabilities.

The tool bridges the gap between purely statistical sizing and complex aerodynamic analysis by integrating two key phases:

1.  **Initial Sizing:** It uses empirical formulas derived from a wide range of existing helicopters to generate a realistic baseline design quickly.
2.  **Performance Simulation:** It uses a physics-based model, **Blade Element Momentum Theory (BEMT)**, to accurately simulate the rotor's performance during vertical flight operations like hovering and climbing.

-----

## 🔬 Core Concepts & Theory

The project's methodology is split into two distinct but connected phases.

### Phase 1: Statistical Design

In the early stages of design, it's impractical to run complex simulations for every variable. Instead, we can leverage data from decades of helicopter development. This phase, primarily handled by `statistical_design.py`, uses **empirical equations** to estimate the size, weight, and power requirements of the helicopter.

  - **Iterative Weight Calculation:** The process starts with a guess for the Maximum Take-Off Weight (MTOW). The code then calculates the weight of the airframe, fuel, and systems based on this guess. These calculated weights are summed with the required payload and crew weight to get a new MTOW. This process repeats until the guessed MTOW and the calculated MTOW converge, ensuring the design is internally consistent.
  - **Component Sizing:** Once the final MTOW is known, the script sizes key components like the main and tail rotors (diameter, chord), fuselage, and tail surfaces using formulas that correlate their dimensions to the helicopter's weight and class.

### Phase 2: Physics-Based Simulation (BEMT)

Once the helicopter's physical dimensions are set, we need a more precise way to calculate performance. This is where **Blade Element Momentum Theory (BEMT)** comes in. Handled by `calculating_state.py`, BEMT is a powerful method for analyzing rotor performance.

The core idea is to break the rotor blade into many small, independent sections (blade elements). For each element, the code:

1.  Calculates the local airflow velocity, which is a combination of the blade's rotation speed, the helicopter's climb velocity, and the "induced velocity" (air drawn down through the rotor).
2.  Determines the local angle of attack (`alpha`).
3.  Calculates the small lift and drag forces on that single element using standard aerodynamic equations.
4.  Resolves these forces into thrust (vertical) and torque (rotational resistance) components.

Finally, the forces from all elements are integrated along the blade's length to find the rotor's total **Thrust, Power, and Torque**. This method allows the simulation to accurately model the effects of blade twist, chord variation, and stall.

-----

## ⚙️ Project Workflow

The program follows a logical sequence from user input to final analysis:

1.  **User Input (`main.py`):** The user provides high-level design goals (payload, range, speed, etc.) and a desired mission profile (e.g., hover for 60s, climb to 100m).
2.  **Statistical Design (`statistical_design.py`):**
      - The iterative weight calculator converges on a final MTOW.
      - All major components are sized based on the final MTOW.
      - [cite\_start]A suitable engine is selected from `Heli_engine_data - Sheet1.csv` based on the calculated power requirements[cite: 1].
3.  **Mission Simulation (`Mission.py`):**
      - The `Mission_planner` initializes the `Helicopterstate_simulator` with the final design parameters.
      - It executes the user-defined mission step-by-step (e.g., hover, climb).
      - For each time step (`dt`), it calls the simulator to calculate the helicopter's state.
4.  **State Calculation (`calculating_state.py`):** The simulator uses BEMT to find the required rotor pitch and power to perform the maneuver, accounting for the helicopter's current weight, altitude, and velocity. It also updates fuel consumption.
5.  **Data Logging & Visualization (`Mission.py`, `result_gen.py`):**
      - The results of each time step are logged.
      - [cite\_start]At the end of the mission, plots are generated, and a detailed `mission.csv` file is saved[cite: 2].
      - Additional performance analyses, like stall limits and endurance curves, are calculated and plotted.

-----

## 📂 File Structure

  - `main.py`: The main entry point. It orchestrates the entire process from user input to final output.
  - `statistical_design.py`: The "architect." Contains the `HelicopterDesigner` class, which defines the helicopter's physical form based on statistical data.
  - `calculating_state.py`: The "physics engine." Contains the `Helicopterstate_simulator` class, responsible for all BEMT calculations and determining the forces on the rotors.
  - `Mission.py`: The "pilot." Contains the `Mission_planner` class that interprets the mission profile and directs the simulator to execute the flight plan.
  - `result_gen.py`: The "flight test engineer." This module runs post-design analyses to find performance boundaries like stall angle, maximum thrust, and endurance at different weights.
  - `helper.py`: A "toolbox" containing essential functions for unit conversions, atmospheric properties (ISA model), and core aerodynamic equations used across the project.
  - `Heli_engine_data - Sheet1.csv`: The "engine catalog." [cite\_start]A simple database of engine specifications used for automatic selection[cite: 1].
  - [cite\_start]`mission.csv`: An auto-generated "flight data recorder" log, containing a detailed, second-by-second record of the simulated mission[cite: 2].

-----

## 🛠️ Installation and Usage

### Prerequisites

  - Python 3.6 or newer.

### Installation

1.  Ensure all project files (`.py`, `.csv`) are in the same directory.
2.  Install the required Python libraries by opening a terminal in the project directory and running:
    ```bash
    pip install numpy pandas matplotlib
    ```

### Running the Simulator

1.  Navigate to the project directory in your terminal.
2.  Execute the main script:
    ```bash
    python main.py
    ```
3.  Follow the on-screen prompts to enter your design parameters and define the mission profile. The simulation will begin automatically after you finish entering the mission steps.

-----

## 📊 Understanding the Output

After the simulation is complete, you will receive the following outputs:

  - **Console Output:** The terminal will display a summary of the final helicopter design, followed by real-time status updates as the mission progresses.
  - **Performance Plots:** Several plot windows will appear, visualizing key mission data like altitude, power, pitch, and fuel remaining over time. These plots are also saved as high-quality `.svg` files in the project directory.
  - [cite\_start]**Mission Data Log (`mission.csv`):** A detailed CSV file is generated, containing the time-series data for the entire mission[cite: 2]. This file can be opened in Excel or other data analysis tools for further inspection.

-----

## ❗ Limitations & Future Work

This tool is intended for conceptual design and has several limitations:

  - **Flight Model:** The simulation is currently limited to hover and vertical climb. Forward flight, which involves more complex aerodynamics (like retreating blade stall and flapping), is not implemented.
  - **Aerodynamics:** The model uses a simplified stall model (a hard limit at a 12-degree angle of attack) and does not account for advanced effects like compressibility at high tip speeds.
  - **Tail Rotor:** The tail rotor's primary function (countering torque) is not dynamically coupled to the main rotor's torque output in this simulation version.

**Potential future enhancements could include:**

  - Implementing a forward flight model.
  - Developing a more sophisticated stall and compressibility model.
  - Creating a graphical user interface (GUI) for easier interaction.
  - Expanding the engine and airfoil databases.
