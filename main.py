import numpy as np
import matplotlib.pyplot as plt
import csv
from statistical_design import HelicopterDesigner
from calculating_state import Helicopterstate_simulator
from performance_tool import isa, deg_to_rad, chord_variation, twist_variation, rad_to_deg, lamda_prandtl, aerod, TPQ, available_engine_power, fuel_flow, degC_to_kel
from Mission import Mission_planner, visualize_mission
import math
import pandas as pd
import json
from result_gen import run_all_calculations_and_plots

g = 9.81  # m/s^2

def get_user_input():
    """
    Prompts the user for helicopter design parameters and mission profile.
    Returns:
        tuple: A tuple containing the design parameters and mission profile.
    """
    print("--- Helicopter Design Input ---")
    W0_guess = float(input("Enter initial Gross Weight (kg, e.g., 4000): "))
    W_pl_target = float(input("Enter Payload Weight (kg, e.g., 0): "))
    V_max = float(input("Enter Max Speed (m/s, e.g., 200): "))
    crew = int(input("Enter number of crew (e.g., 15): "))
    Rg_target = float(input("Enter Range (km, e.g., 439): "))
    Nb = int(input("Enter number of main rotor blades (e.g., 4): "))
    Nb_tr = int(input("Enter number of tail rotor blades (e.g., 2): "))
    
    # Inputs for taper and twist
    taper_ratio = float(input("Enter main rotor taper ratio (e.g., 0.8): "))
    twist_root = float(input("Enter main rotor twist at root (deg, e.g., 5): "))
    twist_tip = float(input("Enter main rotor twist at tip (deg, e.g., 0): "))

    design_params = {
        "W0_guess": W0_guess,
        "W_pl_target": W_pl_target,
        "V_max": V_max,
        "crew": crew,
        "Rg_target": Rg_target,
        "rho_f": 0.8,
        "Nb": Nb,
        "Nb_tr": Nb_tr,
        "taper_ratio": taper_ratio,
        "twist_root": twist_root,
        "twist_tip": twist_tip
    }
    
    print("\n--- Mission Profile Input ---")
    mission_profile = []
    while True:
        step_type = input("Enter mission step type (climb_pitch, climb_time, hover, or done to finish): ").strip().lower()
        if step_type == 'done':
            break
        
        if step_type == 'climb_pitch':
            target_altitude = float(input("   Enter target altitude (m): "))
            pitch_ramp = float(input("   Enter pitch ramp rate (deg/s, e.g., 0.5): "))
            mission_profile.append({
                "type": "climb_pitch",
                "target_altitude": target_altitude,
                "pitch_ramp_rate_deg_per_s": pitch_ramp
            })
        elif step_type == 'climb_time':
            target_altitude = float(input("   Enter target altitude (m): "))
            climb_time = float(input("   Enter climb duration (s): "))
            mission_profile.append({
                "type": "climb_time",
                "target_altitude": target_altitude,
                "climb_time": climb_time
            })
        elif step_type == 'hover':
            altitude_input = input("   Enter hover altitude (m, leave blank for current altitude): ")
            # Corrected logic to handle empty input string
            altitude = float(altitude_input) if altitude_input.strip() else None
            duration = float(input("   Enter hover duration (s): "))
            mission_profile.append({
                "type": "hover",
                "altitude": altitude,
                "duration": duration
            })
        else:
            print("Invalid step type. Please try again.")

    return design_params, mission_profile

def main():
    """
    Main function to run the complete simulation workflow.
    """
    # Get user inputs
    design_params, mission_profile = get_user_input()

    if not mission_profile:
        print("No mission profile defined. Exiting.")
        return

    # Initialize the mission planner
    print("\n--- Initializing Simulator and Running Mission ---")
    planner = Mission_planner(design_params)
    
    
    # Run the mission
    mission_log = planner.run_mission(mission_profile, dt=1.0)
    
    # Check if a log was generated
    if not mission_log:
        print("\nMission failed. No log data was generated.")
        return

    # Save and visualize plots
    print("\n--- Mission Complete. Generating Plots ---")
    
    # Save a combined plot and individual plots as SVG
    visualize_mission(mission_log)
    
    print("\nPlots have been generated and saved as SVG files.")
    print("Mission log saved to 'mission.csv'")
    print("Program finished.")
    print("running required design plots and calculations")
    run_all_calculations_and_plots(params=design_params)

if __name__ == '__main__':
    main()