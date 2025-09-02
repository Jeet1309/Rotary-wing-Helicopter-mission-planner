import numpy as np
import matplotlib.pyplot as plt
import csv
from statistical_design import HelicopterDesigner
from calculating_state import Helicopterstate_simulator
from performance_tool import isa, deg_to_rad, chord_variation, twist_variation, rad_to_deg, lamda_prandtl, aerod, TPQ, available_engine_power, fuel_flow, degC_to_kel
from Mission import Mission_planner, visualize_mission
import math
import pandas as pd

g = 9.81  # m/s^2

def create_mission_profile_plot(logs):
    """
    Generates a multi-panel plot for the mission profile.
    """
    if not logs:
        print("Empty logs, nothing to visualize.")
        return

    # Extract data
    time = [d["time"] for d in logs]
    height = [d.get("height", 0) for d in logs]
    power = [d.get("power_kw", 0) for d in logs]
    pitch = [d.get("pitch", 0) for d in logs]
    fuel = [d.get("fuel_kg", 0) for d in logs]
    weight = [d.get("weight_kg", 0) for d in logs]

    # Create a 3x2 grid
    fig, axs = plt.subplots(3, 2, figsize=(15, 15))
    
    # Flatten the axes array for easier iteration
    axs = axs.flatten()

    # Define plot data and labels
    plot_data = [
        {"data": height, "label": "Altitude (m)", "ylabel": "Altitude (m)"},
        {"data": pitch, "label": "Pitch (deg)", "ylabel": "Pitch (deg)"},
        {"data": power, "label": "Power (kW)", "ylabel": "Power (kW)"},
        {"data": fuel, "label": "Fuel (kg)", "ylabel": "Fuel (kg)"},
        {"data": weight, "label": "Weight (kg)", "ylabel": "Weight (kg)"},
    ]

    # Plot data on each subplot
    for i in range(5):
        ax = axs[i]
        ax.plot(time, plot_data[i]["data"], label=plot_data[i]["label"])
        ax.set_ylabel(plot_data[i]["ylabel"])
        ax.grid(True)
        ax.legend()
        ax.set_xlabel("Time (s)")

    # Hide the last empty subplot
    axs[5].set_visible(False)

    plt.suptitle("Helicopter Mission Profile")
    plt.tight_layout()
    plt.show()

def find_main_rotor_max_thrust_before_stall(heli_sim, altitude=0):
    """Calculates max thrust and stall pitch for the main rotor."""
    pitch_range = np.arange(0.1, 20, 0.1)
    max_thrust = 0.0
    stall_pitch = 0.0
    for pitch in pitch_range:
        try:
            _, _, thrust, _, phi,net_theta = heli_sim._calculate_blade_properties(
                collective_pitch=pitch, 
                height_m=altitude, 
                climb_vel=0,
                main_rotor=True
            )
            max_alpha_deg = np.max(rad_to_deg(deg_to_rad(net_theta) - phi))
            # print(f"main rotor pitch: {pitch:.2f}, thrust: {thrust:.2f}  alpha : {max_alpha_deg:.2f}")
            if max_alpha_deg >= 12:
                stall_pitch = pitch
                print(f"Stall pitch for main rotor found at {stall_pitch:.2f} deg max_alpha = {max_alpha_deg:.2f} deg")
                break
            max_thrust = thrust
        except Exception as e:
            print(f"Error during main rotor stall calculation: {e}")
            break
    return max_thrust, stall_pitch

def find_tail_rotor_max_thrust_before_stall(heli_sim, altitude=0):
    """Calculates max thrust and stall pitch for the tail rotor."""
    pitch_range = np.arange(0.1, 35, 0.1)
    max_thrust = 0.0
    stall_pitch = 0.0
    for pitch in pitch_range:
        try:
            # Note: _calculate_blade_properties needs to be adjusted for tail rotor parameters
            # The 'main_rotor' flag is key here.
            _, _, thrust, _, phi,net_theta = heli_sim._calculate_blade_properties(
                collective_pitch=pitch, 
                height_m=altitude, 
                climb_vel=0,
                main_rotor=False # Crucial change here
            )
            
            max_alpha_deg = np.max(rad_to_deg(deg_to_rad(net_theta) - phi))
            # print(f"Tail rotor pitch: {pitch:.2f}, thrust: {thrust:.2f}  alpha : {max_alpha_deg:.2f}")
            if max_alpha_deg >= 12:
                stall_pitch = pitch
                print(f"Stall pitch for tail rotor found at {stall_pitch:.2f} deg max_alpha = {max_alpha_deg:.2f} deg")
                break
            max_thrust = thrust
        except Exception as e:
            print(f"Error during tail rotor stall calculation: {e}")
            break
    return max_thrust, stall_pitch

def find_tail_rotor_properties(heli_sim, pitch_range, altitude=0):
    """Calculates thrust, torque, and power for the tail rotor over a pitch range."""
    thrusts, torques, powers = [], [], []
    for pitch in pitch_range:
        try:
            _, _, thrust, power, _,_ = heli_sim._calculate_blade_properties(collective_pitch=pitch, height_m=altitude, climb_vel=0, main_rotor=False)
            thrusts.append(thrust)
            # The tail rotor angular velocity from statistical design is in `heli_sim.design_params['Tail Rotor']['Tail Rotor Angular Velocity (rad/s)']`
            # But the _calculate_blade_properties hardcodes it to 50 rad/s. I'll use the hardcoded value for consistency.
            tail_rotor_rad_s = 50
            torques.append(power / tail_rotor_rad_s)
            powers.append(power / 1000) # kW
        except Exception:
            thrusts.append(np.nan)
            torques.append(np.nan)
            powers.append(np.nan)
    return thrusts, torques, powers


def run_all_calculations_and_plots(params):
    """
    Main function to orchestrate all calculations and plot generations.
    """
    

    planner = Mission_planner(params)
    final_design_parameters = planner.final_design_parameters
    heli_sim = planner.heli_sim

    print("\n--- Section 2: Preliminary Helicopter Design ---")
    print("--- 2.1: Design Parameters of your Design ---")
    
    # 2.1 Design Parameters
    main_rotor_radius = final_design_parameters['Main Rotor']['Main Rotor Diameter (m)'] / 2
    tail_rotor_radius = final_design_parameters['Tail Rotor']['Tail Rotor Diameter (m)'] / 2
    
    main_rotor_tip_speed = final_design_parameters['Main Rotor']['Main Rotor Tip Speed (m/s)']
    tail_rotor_tip_speed = final_design_parameters['Tail Rotor']['Tail Rotor Tip Speed (m/s)']
    
    print(f"Main Rotor Radius (m): {main_rotor_radius:.2f}")
    print(f"Tail Rotor Radius (m): {tail_rotor_radius:.2f}")
    print(f"Main Rotor Tip Speed (m/s): {main_rotor_tip_speed:.2f}")
    print(f"Tail Rotor Tip Speed (m/s): {tail_rotor_tip_speed:.2f}")
    print(f"Main Rotor Blades: {final_design_parameters['Main Rotor']['Number of main rotor blades']}")
    print(f"Tail Rotor Blades: {final_design_parameters['Tail Rotor']['Tail rotor blades']}")
    print(f"Main Rotor Chord Variation: linear taper ratio = {params['taper_ratio']}")
    print(f"Tail Rotor Chord Variation: taper ratio = 1")
    print(f"Main Rotor Twist Variation: twist @ root = {params['twist_root']}deg  @tip = {params['twist_tip']}deg")
    print(f"Tail Rotor Twist Variation: no twist")
    print(f"Root Cutout: 0.05 * Radius")

    print("\n--- 2.2: Maximum Thrusts Before Stall ---")
    main_rotor_max_thrust, main_rotor_stall_pitch = find_main_rotor_max_thrust_before_stall(heli_sim)
    print(f"Maximum Main Rotor Thrust before stall: {main_rotor_max_thrust:.2f} N at pitch {main_rotor_stall_pitch:.2f} deg")

    tail_rotor_max_thrust, tail_rotor_stall_pitch = find_tail_rotor_max_thrust_before_stall(heli_sim)
    print(f"Maximum Tail Rotor Thrust before stall: {tail_rotor_max_thrust:.2f} N at pitch {tail_rotor_stall_pitch:.2f} deg")

    print("\n--- Section 3: Hover Mission Test ---")
    altitude_m = 2000.0
    
    
    print("\n--- 3.1: Max Take-off Weight based on blade stall at 2000m AMSL ---")
    main_rotor_stall_thrust_at_alt, _ = find_main_rotor_max_thrust_before_stall(heli_sim, altitude=altitude_m)
    mtow_stall = main_rotor_stall_thrust_at_alt / g
    print(f"Maximum Take-Off Weight (Stall) at {altitude_m}m AMSL: {mtow_stall:.2f} kg")

    print("\n--- 3.2: Max Take-off Weight based on power requirement at 2000m AMSL ---")
    engine_power_sl = final_design_parameters['Engine Suggestion']['Power (kw)'] * 1000 
    power_available_at_alt = available_engine_power(engine_power_sl, altitude_m)
    
    def find_mtow_by_power(heli_sim, power_available, altitude):
        def power_error_func(pitch, h):
            _, _, _, power, _,_ = heli_sim._calculate_blade_properties(pitch, h, 0)
            return power - power_available
        
        pitch_guess = 10 
        try:
            converged_pitch, _ = heli_sim._newton_solver(power_error_func, pitch_guess, altitude)
            _, _, thrust, _, _,_ = heli_sim._calculate_blade_properties(converged_pitch, altitude, 0)
            mtow_power = thrust / g
            return mtow_power
        except Exception as e:
            print(f"Newton solver failed for power MTOW: {e}")
            return None

    mtow_power = find_mtow_by_power(heli_sim, power_available_at_alt, altitude_m)
    print(f"Maximum Take-Off Weight (Power) at {altitude_m}m AMSL: {mtow_power:.2f} kg")
    
    

    print("\n--- 3.3 & 3.4: Fuel Burn Rate & Endurance Plots (at 2000m) ---")
    weights_kg = np.linspace(final_design_parameters["Weights"]['MTOW (kg)'], min(mtow_stall, mtow_power), 50)
    fuel_burn_rates = []
    endurances = []
    sfc = heli_sim.design_params['Engine Suggestion']['SFC']
    fuel_weight_initial_kg = final_design_parameters['Weights']['fuel weight']

    for W_kg in weights_kg:
        heli_sim.reset_state(W0=W_kg*g, altitude=altitude_m)
        try:
            pitch, thrust, power = heli_sim.find_pitch(height_m=altitude_m, climb_vel=0)
            power_W = power
            fuel_burn_kg_min = fuel_flow(power_W, sfc, 60)
            fuel_burn_rates.append(fuel_burn_kg_min)
            if fuel_burn_kg_min > 0:
                endurance_min = fuel_weight_initial_kg / fuel_burn_kg_min
                endurances.append(endurance_min)
            else:
                endurances.append(np.nan)
        except Exception as e:
            print(f"Calculation failed for weight {W_kg} kg: {e}")
            fuel_burn_rates.append(np.nan)
            endurances.append(np.nan)
    
    # Fuel Burn Rate vs Gross Weight Plot
    plt.figure()
    plt.plot(weights_kg, fuel_burn_rates)
    plt.xlabel("Gross Weight (kg)")
    plt.ylabel("Fuel Burn Rate (kg/min)")
    plt.title(f"Fuel Burn Rate vs. Gross Weight at {altitude_m}m")
    plt.grid(True)
    plt.savefig(f"Fuel Burn Rate vs Gross Weight at {altitude_m}.svg", format='svg')
    plt.show()

    # OGE Hover Endurance vs Take-Off-Weight Plot
    plt.figure()
    plt.plot(weights_kg, endurances)
    plt.xlabel("Gross Weight (kg)")
    plt.ylabel("Endurance (minutes)")
    plt.title(f"OGE Hover Endurance vs. Gross Weight at {altitude_m}m")
    plt.grid(True)
    plt.savefig(f"OGE Hover Endurance vs Gross Weight at {altitude_m}.svg", format='svg')
    plt.show()

    print("\n--- Section 2.3: Thrust, Torque, Power Plots (Main and Tail Rotor) ---")
    pitch_range_main = np.arange(0.1, main_rotor_stall_pitch, 0.5)

    # Main Rotor Calculations
    main_thrusts, main_torques, main_powers = [], [], []
    for pitch in pitch_range_main:
        try:
            _, _, thrust, power, _,_ = heli_sim._calculate_blade_properties(collective_pitch=pitch, height_m=0, climb_vel=0, main_rotor=True)
            main_thrusts.append(thrust)
            main_torques.append(power / heli_sim.design_params['Main Rotor']['rad_s'])
            main_powers.append(power / 1000) 
        except Exception:
            main_thrusts.append(np.nan)
            main_torques.append(np.nan)
            main_powers.append(np.nan)
    
    # Tail Rotor Calculations
    pitch_range_tail = np.arange(0.1,tail_rotor_stall_pitch , 0.5)
    tail_thrusts, tail_torques, tail_powers = find_tail_rotor_properties(heli_sim, pitch_range_tail, altitude=0)
    
    # 2.3.1: Thrust vs pitch plot
    plt.figure()
    plt.plot(pitch_range_main, main_thrusts, label="Main Rotor Thrust")
    plt.plot(pitch_range_tail, tail_thrusts, label="Tail Rotor Thrust", linestyle='--')
    plt.xlabel("Collective Pitch (deg)")
    plt.ylabel("Thrust (N)")
    plt.title("Thrust vs. Collective Pitch")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"Thrust vs. Collective Pitch.svg", format='svg')
    plt.show()
    
    # 2.3.2: Torque vs pitch plot
    plt.figure()
    plt.plot(pitch_range_main, main_torques, label="Main Rotor Torque")
    plt.plot(pitch_range_tail, tail_torques, label="Tail Rotor Torque", linestyle='--')
    plt.xlabel("Collective Pitch (deg)")
    plt.ylabel("Torque (Nm)")
    plt.title("Torque vs. Collective Pitch")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"Torque vs. Collective Pitch.svg", format='svg')
    plt.show()

    # 2.3.3: Power vs Thrust plot
    plt.figure()
    plt.plot(main_thrusts, main_powers, label="Main Rotor Power")
    plt.plot(tail_thrusts, tail_powers, label="Tail Rotor Power", linestyle='--')
    plt.xlabel("Thrust (N)")
    plt.ylabel("Power (kW)")
    plt.title("Power vs. Thrust")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"Power vs. Thrust.svg", format='svg')
    plt.show()

if __name__ == '__main__':
    params = {
        "W0_guess": 4000,
        "W_pl_target": 0,
        "V_max": 200,
        "crew": 5,
        "Rg_target": 200,
        "rho_f": 0.8,
        "Nb": 2, 
        "Nb_tr": 2 ,
        'twist_root':0,
        'twist_tip':0,
        'taper_ratio':1,
        
    }
    run_all_calculations_and_plots(params)
    
    