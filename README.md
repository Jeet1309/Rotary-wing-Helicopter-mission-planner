Of course, here is the user manual.

-----

## Simple User Manual

### 📖 1. Introduction

Welcome to the Helicopter Design and Mission Simulator\! This guide will walk you through how to use the tool to design a helicopter and simulate its flight. The program will first ask you for your design goals and then ask for a sequence of flight maneuvers (a "mission").

-----

### ⚙️ 2. Before You Start: Setup

To use this tool, you need a few things set up on your computer.

1.  **Install Python:** If you don't have Python, please install it from [python.org](https://python.org).
2.  **Install Libraries:** You need three libraries. Open your terminal or command prompt and run this command:
    ```bash
    pip install numpy pandas matplotlib
    ```
3.  **Organize Files:** Place all the provided files (`.py` and `.csv`) into a single folder on your computer.

-----

### 🚀 3. Running a Simulation: Step-by-Step Guide

Follow these steps to run a new simulation.

#### Step 1: Launch the Program

  * Open a terminal (on Mac/Linux) or Command Prompt (on Windows).
  * Navigate to the folder where you saved the files. For example:
    ```bash
    cd C:\Users\YourName\Documents\HeliSim
    ```
  * Run the main script with the following command:
    ```bash
    python main.py
    ```

#### Step 2: Enter Helicopter Design Parameters

The program will first ask you to define the helicopter you want to build. It will prompt you for the following information. Just type a number and press Enter.

  * **Initial Gross Weight (kg, e.g., 4000):** A starting guess for the helicopter's total weight. The program will refine this automatically.
  * **Payload Weight (kg, e.g., 0):** The weight of the cargo you want to carry.
  * **Max Speed (m/s, e.g., 200):** The desired top speed.
  * **Number of crew (e.g., 15):** How many people are in the crew.
  * **Range (km, e.g., 439):** How far the helicopter should be able to fly.
  * **Number of main rotor blades (e.g., 4):**
  * **Number of tail rotor blades (e.g., 2):**
  * **Main rotor taper ratio (e.g., 0.8):** How much the blades narrow from root to tip. `1.0` means no taper.
  * **Main rotor twist at root (deg, e.g., 5):** The angle of the blade at the root.
  * **Main rotor twist at tip (deg, e.g., 0):** The angle of the blade at the tip.

#### Step 3: Define the Mission Profile

Next, you will define the flight plan step-by-step. For each step, you'll enter a type and its parameters.

**Mission Step Types:**

  * `hover`: Keep the helicopter at a specific altitude for a set time.
      * **hover altitude (m):** The altitude to hover at. You can leave this blank to hover at the current altitude.
      * **hover duration (s):** How long to hover in seconds.
  * `climb_time`: Climb to a target altitude over a fixed amount of time.
      * **target altitude (m):** The altitude you want to reach.
      * **climb duration (s):** How many seconds the climb should take.

When you have finished adding all the steps for your mission, type **`done`** and press Enter.

#### Example Mission Input:

1.  **Type:** `hover`
2.  **Altitude:** `0`
3.  **Duration:** `60`
    *(This makes the helicopter take off and hover at ground level for 1 minute)*
4.  **Type:** `climb_time`
5.  **Target Altitude:** `100`
6.  **Climb Duration:** `100`
    *(This makes the helicopter climb to 100 meters over 100 seconds)*
7.  **Type:** `hover`
8.  **Altitude:** (leave blank)
9.  **Duration:** `300`
    *(This makes it hover at 100 meters for 5 minutes)*
10. **Type:** `done`

-----

### 📊 4. Understanding the Output

Once you enter "done", the simulation will run. Here's what you'll get:

1.  **Console Messages:** The terminal will show the final design parameters of your helicopter and print live updates as the mission progresses.
2.  **Plots:** Several windows will pop up showing graphs of the mission:
      * Altitude vs. Time
      * Power vs. Time
      * Fuel vs. Time
      * ...and more.
        These plots are also automatically saved as image files (e.g., `Altitude.svg`) in the same folder.
3.  **CSV Log File:** A file named `mission.csv` will be created. This is a spreadsheet file that contains the detailed data for every second of the flight, which you can open in Excel or another program for analysis.
