# put at top of your file
import math
import numpy as np
import matplotlib.pyplot as plt
from helper import *
from calculating_state import Helicopterstate_simulator   # your simulator class
from statistical_design import HelicopterDesigner
import csv
import json
import datetime

g = 9.81

class Mission_planner:
    def __init__(self, param):
        # unpack user inputs
        self.W0_guess = param['W0_guess']
        self.W_pl_target = param['W_pl_target']
        self.crew = param['crew']
        self.Rg_target = param['Rg_target']
        self.rho_f = param.get('rho_f', 0.8)
        self.V_max = param['V_max']
        self.Nb = param.get('Nb', 2)
        self.Nb_tr = param.get('Nb_tr', 4)
        
        self.mission_time =0

        # design stage
        design = HelicopterDesigner(
            W0=self.W0_guess, V_max=self.V_max, Nb=self.Nb, Nb_tr=self.Nb_tr,
            W_pl=self.W_pl_target, crew=self.crew, Rg=self.Rg_target, rho_f=self.rho_f
        )
        design.iterative_gross_weight_calculator(self.W0_guess)
        self.final_design_parameters = design.run_calculations()

        # prepare simulator dict
        simulator_params = self.final_design_parameters

        # Main Rotor
        mr = simulator_params['Main Rotor']
        mr['blades'] = self.Nb
        taper_ratio = param.get('taper ratio', 1)
        # taper_ratio = float(taper_ratio) if taper_ratio else 0.8
        mr['Chord_root'] = mr['Main Rotor Chord (m)']
        mr['Chord_tip'] = mr['Main Rotor Chord (m)'] * taper_ratio
        mr['Blade_radius'] = mr['Main Rotor Diameter (m)'] * 0.5
        mr['Root_cutoff'] = mr['Blade_radius'] * 0.05
        mr['rpm'] = rad_s_to_rpm(mr["Main Rotor Angular Velocity (rad/s)"])
        mr['rad_s'] = mr["Main Rotor Angular Velocity (rad/s)"]
        mr['Div'] = 1000
        mr['Twist_root'] = param.get('twist_root',0)
        mr['Twist_tip'] = param.get('twist_tip',0) # Fixed twist
        mr['lift_slope'] = 5.75

        # Tail Rotor
        tr = simulator_params['Tail Rotor']
        tr['blades'] = self.Nb_tr
        tr_taper_ratio = 1 # Fixed for consistency
        tr['Chord_root'] = tr['Tail Rotor Chord (m)']
        tr['Chord_tip'] = tr['Tail Rotor Chord (m)'] * tr_taper_ratio
        tr['Blade_radius'] = tr['Tail Rotor Diameter (m)'] * 0.5
        tr['Root_cutoff'] = tr['Blade_radius'] * 0.05
        tr['rpm'] = rad_s_to_rpm(tr["Tail Rotor Angular Velocity (rad/s)"])
        tr['rad_s'] = tr["Tail Rotor Angular Velocity (rad/s)"]
        tr['Div'] = 1000
        tr['Twist_root'] = 0  # No twist
        tr['Twist_tip'] = 0  # No twist
        tr['lift_slope'] = 5.75

        simulator_params['Weights']['W0'] = self.final_design_parameters['Weights']['MTOW (kg)'] * g
        simulator_params['Weights']['Wf'] = self.final_design_parameters['Weights']["fuel weight"] * g
        simulator_params['Engine Suggestion'] = self.final_design_parameters['Engine Suggestion']

        self.heli_sim = Helicopterstate_simulator(design_parameters=simulator_params)

    # -------------------------
    # Climb: time based
    # -------------------------
    def _climb_time_based(self, target_altitude: float, climb_time: float, dt: float = 1.0, log=None):
        """
        Time-based climb: Corrected logic to properly apply pitch capping and stall checks.
        """
        log = [] if log is None else log

        if self.heli_sim.current_state["curr_height"] == 0:
            hover_zero_res = self._hover(0.0, 1, dt=dt, log=log)
            if hover_zero_res.get('status', '') != 'OK':
                return hover_zero_res
            print(">>> Hover at 0m complete; starting time based climb")
        
        start_h = float(self.heli_sim.current_state.get('curr_height', 0.0))
        if climb_time <= 0:
            raise ValueError("climb_time must be > 0 for time-based climb.")

        climb_rate = (target_altitude - start_h) / climb_time
        t = 0.0
        
        while t < climb_time:
            # 1. First, calculate properties for the current time step
            current_pitch = self.heli_sim.current_state.get('required_pitch', 8.0)
            
            # This is the key change: we manually calculate properties to check for stalls/power issues
            # We don't want to rely on calculate_next_state for this check, as it's a black box.
            _, _, thrust, power, phi,net_theta_deg = self.heli_sim._calculate_blade_properties(
                collective_pitch=current_pitch,
                height_m=self.heli_sim.current_state['curr_height'],
                climb_vel=climb_rate,
                main_rotor=True
            )
            
            # 2. Check for safety caps based on this manual calculation
            max_alpha_deg = np.max(rad_to_deg(deg_to_rad(net_theta_deg) - phi))
            
            pitch_cap = None
            if max_alpha_deg >= 12.0:
                print(f">>> Predicted stall ({max_alpha_deg:.2f} deg) during climb. Capping pitch for next step.")
                pitch_cap = max(0.0, current_pitch - 1.0)
            
            engine_kw = self.final_design_parameters['Engine Suggestion'].get('Power (kw)', None)
            if engine_kw is not None:
                engine_max_w = float(engine_kw) * 1000.0
                if power >= engine_max_w:
                    print(">>> Required power exceeds available engine power. Capping pitch for next step.")
                    pitch_cap = max(0.0, current_pitch - 1.0)

            # 3. Apply the determined pitch cap to the simulator's state for the NEXT time step
            if pitch_cap is not None:
                self.heli_sim.current_state['required_pitch'] = pitch_cap
                print(f"[CLIMB TIME] Applying pitch cap: {pitch_cap:.2f}°")

            # 4. Use calculate_next_state to progress the simulation
            curr_h = float(self.heli_sim.current_state.get('curr_height', 0.0))
            next_h = min(target_altitude, curr_h + climb_rate * dt)
            self.heli_sim.calculate_next_state(dt=dt, next_height=next_h)
            
            # 5. Log the state that was just calculated
            cs = self.heli_sim.current_state
            achieved_h = float(cs.get('curr_height', curr_h))
            fuel_kg = float(cs.get('Wf', 0.0)) / g
            
            log.append({
                "time": self.mission_time,
                "height": achieved_h,
                "v_z": climb_rate,
                "pitch": cs.get('required_pitch', 0.0),
                "power_kw": cs.get('required_power', 0.0) / 1000,
                "fuel_kg": fuel_kg,
                "weight_kg": cs.get('W0', 0) / g
            })
            print(f"[CLIMB TIME] t={self.mission_time:.1f}s | h={achieved_h:.1f}m | v_z={climb_rate:.2f}m/s "
                  f"| pitch={cs.get('required_pitch', 0.0):.2f}° | fuel={fuel_kg:.2f}kg | power={cs.get('required_power', 0.0)/1000:.2f}kW "
                  f"| W={cs.get('W0',0)/g:.2f}kg")
            # print(f"| max_alpha={max_alpha_deg:.2f}deg")
            
            if fuel_kg <= 0.01:
                print("!!! Fuel empty, aborting climb.")
                return {"status": "FUEL_EMPTY", "time": t + dt, "log": log}

            t += dt
            self.mission_time += dt

            if abs(achieved_h - target_altitude) < 1e-3:
                print(f"*** Target altitude {target_altitude} m reached at t={t:.1f}s ***")
                break

        return {"status": "OK", "time": t, "log": log}

    def _climb_pitch_ramp(self, target_altitude: float, dt: float = 1,
                          pitch_ramp_rate_deg_per_s: float = 0.5,
                          log=None):
        """
        Pitch-ramp climb. Corrected logic to properly apply pitch capping and stall checks.
        """
        log = [] if log is None else log
        if self.heli_sim.current_state["curr_height"] == 0:
            hover_zero_res = self._hover(0.0, 1, dt=dt, log=log)
            if hover_zero_res.get('status', '') != 'OK':
                return hover_zero_res
            print(">>> Hover at 0m complete; starting pitch-ramp climb")

        W0_N = float(self.heli_sim.current_state.get('W0', 0.0))
        fuel_N = float(self.heli_sim.current_state.get('Wf', 0.0))
        height = float(self.heli_sim.current_state.get('curr_height', 0.0))
        v_z = float(self.heli_sim.current_state.get('vertical_velocity', 0.0))
        pitch_deg = float(self.heli_sim.current_state.get('required_pitch', 0.0))

        sfc = float(self.heli_sim.design_params['Engine Suggestion'].get('SFC', 0.0))

        if height >= target_altitude:
            return {"status": "ALREADY_AT_TARGET", "time": 0.0, "log": log}

        t = 0.0
        
        while True:
            # First, check for safety conditions based on the current state.
            _, _, _, Power_W, Phi_main,net_theta_deg = self.heli_sim._calculate_blade_properties(
                collective_pitch=pitch_deg, 
                height_m=height, 
                climb_vel=v_z, 
                main_rotor=True
            )
            max_alpha_deg = np.max(rad_to_deg(deg_to_rad(net_theta_deg) - Phi_main))
            
            not_stalled = max_alpha_deg < 12.0
            
            engine_kw = self.final_design_parameters['Engine Suggestion'].get('Power (kw)', None)
            engine_max_w = float(engine_kw) * 1000.0 if engine_kw is not None else float('inf')
            power_exceeded = Power_W >= engine_max_w
            
            # Apply pitch capping logic for the CURRENT step.
            if not not_stalled:
                print(f">>> Stall predicted ({max_alpha_deg:.2f} deg), capping pitch.")
                pitch_deg = max(0.0, pitch_deg - 1.0)
            elif power_exceeded:
                print(f">>> Required power ({Power_W/1000:.2f} kW) exceeds engine rating ({engine_max_w/1000:.2f} kW). Capping pitch.")
                pitch_deg = max(0.0, pitch_deg - 1.0)
            else:
                # Normal pitch ramp if no safety conditions are met
                pitch_deg += pitch_ramp_rate_deg_per_s * dt

            # Use the new pitch to calculate properties for the CURRENT step
            _, _, Thrust_N, Power_W, Phi_main,net_theta_deg = self.heli_sim._calculate_blade_properties(
                collective_pitch=pitch_deg, 
                height_m=height, 
                climb_vel=v_z, 
                main_rotor=True
            )

            # Update next state
            mass_kg = W0_N / g if g != 0 else 1.0
            a_z = (Thrust_N - W0_N) / mass_kg
            v_z_next = v_z + a_z * dt
            height_next = height + v_z_next * dt

            # Final step check
            if height_next >= target_altitude:
                height = target_altitude
                v_z = 0.0
                fuel_burned_kg = fuel_flow(Power_W, sfc, dt)
                fuel_burned_N = fuel_burned_kg * g
                fuel_N = max(0.0, fuel_N - fuel_burned_N)
                W0_N = max(0.0, W0_N - fuel_burned_N)

                t += dt
                self.mission_time += dt
                log.append({
                    "time": self.mission_time,
                    "height": height,
                    "v_z": v_z,
                    "pitch": pitch_deg,
                    "power_kw": Power_W / 1000,
                    "fuel_kg": fuel_N / g,
                    "weight_kg": W0_N / g
                })
                print(f"[CLIMB PITCH] t={self.mission_time:.1f}s | h={height:.1f}m | v_z={v_z:.2f}m/s "
                      f"| pitch={pitch_deg:.2f}° | fuel={fuel_N/g:.2f}kg | power={Power_W/1000:.2f}kW "
                      f"| W={W0_N/g:.2f}kg")
                self.heli_sim.current_state.update({
                    'Wf': fuel_N, 'W0': W0_N, 'curr_height': height,
                    'vertical_velocity': v_z, 'required_pitch': pitch_deg,
                    'required_power': Power_W
                })
                return {"status": "REACHED_ALTITUDE", "time": t, "log": log}

            v_z = v_z_next
            height = max(0.0, height_next)

            fuel_burned_kg = fuel_flow(Power_W, sfc, dt)
            fuel_burned_N = fuel_burned_kg * g
            fuel_N = max(0.0, fuel_N - fuel_burned_N)
            W0_N = max(0.0, W0_N - fuel_burned_N)

            t += dt
            self.mission_time += dt
            log.append({
                "time": self.mission_time,
                "height": height,
                "v_z": v_z,
                "pitch": pitch_deg,
                "power_kw": Power_W / 1000,
                "fuel_kg": fuel_N / g,
                "weight_kg": W0_N / g
            })
            print(f"[CLIMB PITCH] t={self.mission_time:.1f}s | h={height:.1f}m | v_z={v_z:.2f}m/s "
                  f"| pitch={pitch_deg:.2f}° | fuel={fuel_N/g:.2f}kg | power={Power_W/1000:.2f}kW "
                  f"| W={W0_N/g:.2f}kg")
            # print(f"| max_alpha={max_alpha_deg:.2f}deg")

            self.heli_sim.current_state.update({
                'Wf': fuel_N, 'W0': W0_N, 'curr_height': height,
                'vertical_velocity': v_z, 'required_pitch': pitch_deg,
                'required_power': Power_W
            })

            if fuel_N / g <= 10.0:
                return {"status": "FUEL_EMPTY", "time": t, "log": log}


    def _hover(self, hover_altitude: float, duration_s: float, dt: float = 1.0, log=None):
        """
        Hover at hover_altitude for duration_s. Uses _climb_pitch_ramp if altitude differs.
        Added small guards for missing keys and clearer prints.
        """
        log = [] if log is None else log
        if hover_altitude == None:
            hover_altitude = float(self.heli_sim.current_state.get('curr_height', 0.0))

        curr_h = float(self.heli_sim.current_state.get('curr_height', 0.0))
        if abs(curr_h - hover_altitude) > 1e-6:
            climb_res = self._climb_pitch_ramp(target_altitude=hover_altitude, dt=dt, log=log)
            if climb_res.get('status', '') not in ('OK', 'REACHED_ALTITUDE'):
                return climb_res
            print(">>> Reached hover altitude")
            t = float(climb_res.get('time', 0.0))
        else:
            t = 0.0

        t_new = 0.0
        while t_new < duration_s:
            self.heli_sim.calculate_next_state(dt=dt, next_height=hover_altitude)
            t += dt
            self.mission_time += dt
            t_new += dt

            cs = self.heli_sim.current_state
            log.append({
                "time": self.mission_time,
                "height": cs.get('curr_height', hover_altitude),
                "v_z": cs.get('vertical_velocity', 0.0),
                "pitch": cs.get('required_pitch', 0.0),
                "power_kw": cs.get('required_power', 0.0) / 1000,
                "fuel_kg": cs.get('Wf', 0.0) / g,
                "weight_kg": cs.get('W0', 0.0) / g
            })

            if t_new % 10 == 0:
                 print(f"[HOVER] t={self.mission_time:.1f}s | h={cs.get('curr_height', hover_altitude):.1f}m "
          f"| v_z={cs.get('vertical_velocity',0.0):.2f}m/s | pitch={cs.get('required_pitch',0.0):.2f}° "
          f"| fuel={cs.get('Wf',0.0)/g:.2f}kg | power={cs.get('required_power',0.0)/1000:.2f}kW "
          f"| W={cs.get('W0',0.0)/g:.2f}kg")
                 
            if cs.get('Wf', 0.0) / g <= 0.01:
                return {"status": "FUEL_EMPTY_DURING_HOVER", "time": t, "log": log}
            if cs.get('max_alpha') >= 12.0:
                print(f">>> Predicted stall during hover. {cs.get('max_alpha'):.2f} deg")
                return {"status": "STALL_PREDICTED", "time": t, "log": log}

        return {"status": "OK", "time": t, "log": log}

    def run_mission(self, mission_profile, dt: float = 1.0, log_file="mission.csv"):
        log = []

        for i, step in enumerate(mission_profile, start=1):
            step_type = step["type"]
            print(f"\n[MISSION] Step {i}: {step_type.upper()} started...")

            if step_type == "climb_time":
                res = self._climb_time_based(
                    target_altitude=step["target_altitude"],
                    climb_time=step["climb_time"],
                    dt=dt,
                    log=log
                )

            elif step_type == "climb_pitch":
                res = self._climb_pitch_ramp(
                    target_altitude=step["target_altitude"],
                    pitch_ramp_rate_deg_per_s=step.get("pitch_ramp_rate_deg_per_s", 0.5),
                    dt=dt,
                    log=log
                )

            elif step_type == "hover":
                res = self._hover(
                    hover_altitude=step.get("altitude", self.heli_sim.current_state['curr_height']),
                    duration_s=step["duration"],
                    dt=dt,
                    log=log
                )

            else:
                raise ValueError(f"[MISSION] Unknown step type: {step_type}")

            final_altitude = res["log"][-1]["height"] if res.get("log") else float("nan")

            print(f"[MISSION] Step {i}: {step_type.upper()} finished → {res['status']} "
                f"at t={self.mission_time:.1f}s, altitude={final_altitude:.1f}m")

        if log:
            keys = log[0].keys()
            with open(log_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(log)

        return log
    

def visualize_mission(logs):
    if not logs:
        print("Empty logs, nothing to visualize.")
        return

    time = [d["time"] for d in logs]
    height = [d.get("height", 0) for d in logs]
    power = [d.get("power_kw", 0) for d in logs]
    pitch = [d.get("pitch", 0) for d in logs]
    fuel = [d.get("fuel_kg", 0) for d in logs]
    weight = [d.get("weight_kg", 0) for d in logs]

    fig, axs = plt.subplots(3, 2, figsize=(15, 15))
    fig.delaxes(axs[2, 1])
    
    axs = axs.flatten()

    plot_data = {
        0: {"data": height, "label": "Altitude (m)", "ylabel": "Altitude (m)"},
        1: {"data": pitch, "label": "Pitch (deg)", "ylabel": "Pitch (deg)"},
        2: {"data": power, "label": "Power (kW)", "ylabel": "Power (kW)"},
        3: {"data": fuel, "label": "Fuel (kg)", "ylabel": "Fuel (kg)"},
        4: {"data": weight, "label": "Weight (kg)", "ylabel": "Weight (kg)"},
    }

    for i in range(5):
        ax = axs[i]
        ax.plot(time, plot_data[i]["data"], label=plot_data[i]["label"])
        ax.set_ylabel(plot_data[i]["ylabel"])
        ax.grid(True)
        ax.legend()
        ax.set_xlabel("Time (s)")

    plt.suptitle("Helicopter Mission Profile")
    plt.tight_layout()
    plt.show()

    for i in range(5):
        plt.figure(figsize=(10, 6))
        plt.plot(time, plot_data[i]["data"], label=plot_data[i]["label"])
        plt.xlabel("Time (s)")
        plt.ylabel(plot_data[i]["ylabel"])
        plt.title(f"{plot_data[i]['label']} over Time")
        plt.grid(True)
        plt.legend()
        plt.savefig(f"{plot_data[i]['label'].split(' ')[0]}.svg", format='svg')
        plt.close()

if __name__ =="__main__":
    
    params = {
        "W0_guess": 4000,
        "W_pl_target": 0,
        "V_max": 200,
        "crew": 15,
        "Rg_target": 439,
        "rho_f": 0.8,
        'twist_root':4,
        
    }
    planner = Mission_planner(params)
    