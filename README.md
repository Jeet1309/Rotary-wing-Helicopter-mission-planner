# Helicopter Performance and Mission Simulation

## 📝 Introduction

This project is a comprehensive suite of Python scripts for the preliminary design and performance analysis of a helicopter. It provides a robust framework for:

1.  **Statistical Design**: An iterative process to determine the optimal helicopter gross weight and select a suitable engine based on mission requirements.
2.  **Physics-Based Simulation**: A step-by-step simulation of a user-defined mission profile, calculating key performance metrics like power, fuel consumption, and altitude.
3.  **Performance Analysis**: Generation of detailed plots and data logs to visualize and analyze the mission results.

-----

## ⚙️ Core Concepts & Theory

The project's calculations are grounded in fundamental aerospace engineering principles:

### **Phase 1: Statistical Design**

The core challenge in helicopter design is the interdependency of gross weight and fuel weight. The program addresses this with an **iterative convergence method**. It starts with an initial gross weight guess and calculates the fuel required for a target mission. This new fuel weight is then used to refine the gross weight, and the process repeats until the gross weight value stabilizes. This ensures a feasible design before the mission simulation begins.

### **Phase 2: Physics-Based Simulation**

The simulation engine is built upon **Blade Element Momentum Theory (BEMT)**. This method models the rotor blade as a series of small, independent elements. By calculating the aerodynamic forces (thrust and torque) on each element and summing them up, the program can accurately determine the overall rotor performance. The simulation also incorporates:

  * **International Standard Atmosphere (ISA)**: Calculations are adjusted for changes in air density, pressure, and temperature with altitude.
  * **Engine Performance Modeling**: The simulation uses a specific fuel consumption (SFC) value from the `Heli_engine_data` dataset to calculate fuel burn based on the power required at each time step.

-----

## 🚀 Project Workflow

The project is designed with a clear, sequential workflow:

1.  **User Input (`main.py`)**: The program prompts the user for initial design parameters (e.g., gross weight, payload, number of blades) and defines the mission profile as a series of flight segments (`climb`, `cruise`, `hover`, `descend`).
2.  **Iterative Design (`statistical_design.py`)**: The user-provided parameters are passed to the `HelicopterDesigner` class, which performs the iterative calculation to find the final, converged helicopter design parameters.
3.  **Mission Simulation (`Mission.py`)**: The converged design is used to initialize the `Mission_planner`. This class then executes the mission step by step, calling the `Helicopterstate_simulator` (`calculating_state.py`) to update the helicopter's state at each time step.
4.  **Result Generation (`result_gen.py`)**: After the simulation, the program creates a detailed log file and generates a series of plots that visualize the mission's key performance metrics.
5.  **Benchmark Comparison (`performance_benchmark.py`)**: The program can optionally run performance benchmarks by comparing its calculated results against provided external data from `Paper_results.xlsx`.

-----

## 📂 File Structure

  * `main.py`: The main script that orchestrates the entire simulation, handling user input and calling other modules.
  * `statistical_design.py`: Contains the `HelicopterDesigner` class for the iterative gross weight calculation.
  * `Mission.py`: Defines the `Mission_planner` class, which manages the mission simulation and generates the final mission log and plots.
  * `calculating_state.py`: The core simulation engine, providing the `Helicopterstate_simulator` class to calculate the helicopter's state at each time step.
  * `performance_tool.py`: A library of helper functions for unit conversions, atmospheric calculations, and aerodynamic properties.
  * `result_gen.py`: A script responsible for creating and saving the mission plots in SVG format.
  * `import_paper_results.py`: A script to read and process external benchmark data from the `Paper_results.xlsx` file.
  * `performance_benchmark.py`: A script that runs a performance analysis and compares the simulation results with the imported benchmark data.
  * `Heli_engine_data - Sheet1.csv`: A dataset of various helicopter engines, including power and specific fuel consumption.
  * `Paper_results.xlsx - [2-5]blade.csv`: External data used for validating the model's accuracy against published results for different blade configurations.

-----

## 🛠️ Installation and Usage

### **Prerequisites**

To run the simulator, you need to have Python installed on your system along with the following libraries:

  * `numpy`
  * `matplotlib`
  * `pandas`

### **Installation**

You can install the required libraries using `pip`:

```bash
pip install numpy matplotlib pandas
```

### **Running the Simulator**

1.  Place all the project files in the same directory.
2.  Open a terminal or command prompt.
3.  Navigate to the project directory.
4.  Run the main script:

<!-- end list -->

```bash
python main.py
```

The program will then guide you through providing the necessary design parameters and defining your mission profile.

### **Understanding the Output**

Upon completion, the program generates two main types of output:

1.  **`mission.csv`**: A CSV file containing a detailed log of the simulation. Each row represents a time step and includes data such as time, altitude, required power, fuel burned, and current weight.
2.  **SVG Plots**: Several SVG image files are generated, including a combined mission profile plot and individual plots for altitude, power, fuel, and weight over time. These provide a clear visual representation of the helicopter's performance throughout the mission.

-----

## 🚧 Limitations & Future Work

### **Current Limitations**

  * The simulation is currently limited to the flight phases defined in the mission profile.
  * The engine data is based on a fixed CSV file.
  * The aerodynamic model is a simplified BEMT, not accounting for more complex phenomena like compressibility effects or blade stall.

### **Future Work**

  * Expand the mission profile to include more complex maneuvers like autorotation or turns.
  * Create a dynamic engine database with more detailed performance maps.
  * Integrate a more advanced aerodynamic model for greater accuracy across a wider flight envelope.
