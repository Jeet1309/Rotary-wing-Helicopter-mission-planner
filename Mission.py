# put at top of your file
import math
import numpy as np
import matplotlib.pyplot as plt
from helper import *
from calculating_state import Helicopterstate_simulator   # your simulator class
from statistical_design import HelicopterDesigner
import csv

g = 9.81
import json
import datetime

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
        self.Nb_tr = param.get('Nb_tr', 2)
        
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
        mr = simulator_params['Main Rotor']
        mr['blades'] = self.Nb
        mr['Chord_root'] = mr['Main Rotor Chord (m)']
        mr['Chord_tip'] = mr['Main Rotor Chord (m)']
        mr['Blade_radius'] = mr['Main Rotor Diameter (m)'] * 0.5
        mr['Root_cutoff'] = mr['Blade_radius'] * 0.05
        mr['rpm'] = rad_s_to_rpm(mr["Main Rotor Angular Velocity (rad/s)"])
        mr['rad_s'] = mr["Main Rotor Angular Velocity (rad/s)"]
        mr['Div'] = 1000
        mr['Twist_root'] = float(input("Enter the root twist in degrees (e.g., 10): "))
        mr['Twist_tip'] = float(input("Enter the tip twist in degrees (e.g., 0): "))
        mr['lift_slope'] = 5.75

        simulator_params['Weights']['W0'] = self.final_design_parameters['Weights']['MTOW (kg)'] * g
        simulator_params['Weights']['Wf'] = self.final_design_parameters['Weights']["fuel weight"] * g
        simulator_params['Engine Suggestion'] = self.final_design_parameters['Engine Suggestion']

        self.heli_sim = Helicopterstate_simulator(design_parameters=simulator_params)

        # Precompute blade geometry arrays
        p = self.heli_sim.design_params['Main Rotor']
        self.r_arr = np.linspace(p['Root_cutoff'], p['Blade_radius'], int(p['Div']))
        self.chord_array = chord_variation(p['Chord_root'], p['Chord_tip'],
                                           p['Blade_radius'], p['Root_cutoff'], self.r_arr)
        self.theta_array = twist_variation(p['Twist_root'], p['Twist_tip'],
                                           p['Blade_radius'], p['Root_cutoff'], self.r_arr)

        self.rad_s = p['rad_s']
        self.blades = p['blades']
        self.R = p['Blade_radius']
        self.Div = int(p['Div'])
        self.lift_slope = p['lift_slope']

    # -------------------------
    # Climb: time based
    # -------------------------
    def _climb_time_based(self, target_altitude: float, climb_time: float, dt: float = 1.0, log=None):
        """
        Time-based climb: ramp to a steady climb_rate and call simulator to reach successive next_height.
        Added robust prints, pitch capping logic and safe guards.
        """
        log = [] if log is None else log

        if(self.heli_sim.current_state["curr_height"]==0):
            hover_zero_res = self._hover(0.0, 1, dt=dt, log=log)
            if hover_zero_res.get('status', '') != 'OK':
                return hover_zero_res
            print(">>> Hover at 0m complete; starting time based climb")

        start_h = float(self.heli_sim.current_state.get('curr_height', 0.0))
        if climb_time <= 0:
            raise ValueError("climb_time must be > 0 for time-based climb.")

        climb_rate = (target_altitude - start_h) / climb_time
        t = 0.0

        # pitch cap variable (None == no cap)
        max_pitch = None

        while t < climb_time:
            # If a pitch cap was computed in previous iteration, apply it BEFORE next simulate step
            if max_pitch is not None:
                capped_pitch = min(float(self.heli_sim.current_state.get('required_pitch', 0.0)), max_pitch)
                self.heli_sim.current_state['required_pitch'] = capped_pitch
                print(f"[CLIMB TIME] Applying pitch cap: {capped_pitch:.2f}°")

            curr_h = float(self.heli_sim.current_state.get('curr_height', 0.0))
            next_h = min(target_altitude, curr_h + climb_rate * dt)

            # Let simulator compute next state to reach next_h (simulator should use required_pitch if present)
            self.heli_sim.calculate_next_state(dt=dt, next_height=next_h)

            # read safely from current_state with defaults
            cs = self.heli_sim.current_state
            achieved_h = float(cs.get('curr_height', curr_h))
            pitch = float(cs.get('required_pitch', 0.0))
            power = float(cs.get('required_power', 0.0))
            avail_power = float(cs.get('available_power', 1e12))  # very large default if not present
            max_alpha = cs.get('max_alpha', -999.0)

            err_h = next_h - achieved_h
            fuel_kg = float(cs.get('Wf', 0.0)) / g

            # decide caps for next iteration if needed (set max_pitch but do not overwrite current_state)
            if max_alpha is not None and float(max_alpha) >= 12.0:
                print(">>> Predicted stall during climb. capping pitch for next step.")
                max_pitch = max(0.0, pitch - 1.0)

            # engine power check (Power_W in W vs Engine Suggestion in kW)
            engine_kw = self.final_design_parameters['Engine Suggestion'].get('Power (kw)', None)
            if engine_kw is not None:
                engine_max_w = float(engine_kw) * 1000.0
                # If the rotor power demand is higher than engine (with some margin), cap
                if power >= engine_max_w:
                    print(">>> Required power exceeds engine rating. Capping pitch for next step.")
                    max_pitch = max(0.0, pitch - 1.0)
            else:
                # if engine kw missing, use available_power from sim if present
                if avail_power <= power:
                    print(">>> Simulator reports available_power <= required_power. Capping pitch.")
                    max_pitch = max(0.0, pitch - 1.0)

            # log & prints
            log.append({
                "time": self.mission_time,
                "height": achieved_h,
                "v_z": climb_rate,
                "pitch": pitch,
                "power_kw": power / 1000,
                "fuel_kg": fuel_kg,
                "weight_kg": cs.get('W0', 0) / g
            })
            print(f"[CLIMB TIME] t={self.mission_time:.1f}s | h={achieved_h:.1f}m | v_z={climb_rate:.2f}m/s "
      f"| pitch={pitch:.2f}° | fuel={fuel_kg:.2f}kg | power={power/1000:.2f}kW "
      f"| W={cs.get('W0',0)/g:.2f}kg")

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
        Pitch-ramp climb. Uses your aero/TPQ chain but avoids overwriting current_state.
        Replaces a_z integration with the same approach but keeps your prints and fuel logic.
        max_time is a safety to avoid infinite loops.
        """
        log = [] if log is None else log
        if(self.heli_sim.current_state["curr_height"]==0):
            hover_zero_res = self._hover(0.0, 1, dt=dt, log=log)
            if hover_zero_res.get('status', '') != 'OK':
                return hover_zero_res
            print(">>> Hover at 0m complete; starting pitch-ramp climb")

        W0_N = float(self.heli_sim.current_state.get('W0', 0.0))
        fuel_N = float(self.heli_sim.current_state.get('Wf', 0.0))
        height = float(self.heli_sim.current_state.get('curr_height', 0.0))
        v_z = float(self.heli_sim.current_state.get('vertical_velocity', 0.0))
        pitch_deg = float(self.heli_sim.current_state.get('required_pitch', 0.0))
        max_alpha = self.heli_sim.current_state.get("max_alpha")

        sfc = float(self.heli_sim.design_params['Engine Suggestion'].get('SFC', 0.0))

        if height >= target_altitude:
            return {"status": "ALREADY_AT_TARGET", "time": 0.0, "log": log}

        not_stalled = True
        cnt = 0
        t = 0.0
        max_pitch = None  # pitch cap for next step

        while True:
            # ramp pitch (but do not break caps)
            if max_pitch is not None:
                pitch_deg = min(pitch_deg, max_pitch)
            if not_stalled:
                pitch_deg += pitch_ramp_rate_deg_per_s * dt
            else:
                cnt += 1
                if cnt == 1:
                    print(">>> Blade stalled, reducing pitch")
                    pitch_deg = max(0.0, pitch_deg - 1.0)

            # If a pitch cap exists from previous iteration, ensure we don't exceed it
            if max_pitch is not None:
                pitch_deg = min(pitch_deg, max_pitch)

            # aero calculations using current pitch_deg and v_z
            net_theta_rad = deg_to_rad(self.theta_array + pitch_deg)
            sigma = (self.blades * self.chord_array) / (math.pi * self.R)
            lamda_c = v_z / (self.rad_s * self.R) if (self.rad_s * self.R) != 0 else 0.0

            induced_vel = lamda_prandtl(v_z, self.blades, self.r_arr, self.R,
                                        sigma, self.lift_slope, 1e-12,
                                        net_theta_rad, math.pi, lamda_c, self.rad_s)

            Temp, _, rho = isa(height)
            cl, cd, alpha_arr, U_p, U_t, Phi = aerod(self.rad_s, self.r_arr, v_z,
                                                    induced_vel, net_theta_rad, self.lift_slope, Temp=Temp)

            max_alpha_deg = float(np.max(rad_to_deg(alpha_arr)))
            if max_alpha_deg >= 12:
                not_stalled = False

            Thrust_N, Power_W, Torque = TPQ(rho, U_p, U_t, self.chord_array, cl, cd, Phi,
                                            self.R, self.Div, self.blades, self.r_arr, self.rad_s)

            # DEBUG engine power vs rotor power: use consistent units
            engine_kw = self.final_design_parameters['Engine Suggestion'].get('Power (kw)', None)
            if engine_kw is not None:
                engine_max_w = float(engine_kw) * 1000.0
                if Power_W >= engine_max_w:
                    print(">>> Required power exceeds engine rating. Capping pitch.")
                    
                    # reduce pitch immediately to avoid runaway
                    pitch_deg = max(0.0, pitch_deg - 1.0)
                    max_pitch = pitch_deg
                    continue


            # compute acceleration and predicted next height (keep your original physics)
            mass_kg = W0_N / g if g != 0 else 1.0
            a_z = (Thrust_N - W0_N) / mass_kg
            v_z_next = v_z + a_z * dt
            height_next = height + v_z_next * dt

            # Overshoot check: clamp final step
            if height_next >= target_altitude:
                height = target_altitude
                v_z = 0.0
                # burn fuel for the last step
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

            # Normal integration
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

            # push updated values to simulator state so _hover / other calls read them
            self.heli_sim.current_state.update({
                'Wf': fuel_N, 'W0': W0_N, 'curr_height': height,
                'vertical_velocity': v_z, 'required_pitch': pitch_deg,
                'required_power': Power_W
            })

            # safety exits
            if fuel_N / g <= 0.01:
                return {"status": "FUEL_EMPTY", "time": t, "log": log}
            if max_alpha_deg >= 12:
                print(">>> Stall predicted during climb.")
                return {"status": "STALL_PREDICTED", "time": t, "log": log}

        # timeout safety
            print("Warning: climb_pitch_ramp reached max_time without reaching target")
            return {"status": "TIMEOUT", "time": t, "log": log}


    def _hover(self, hover_altitude: float, duration_s: float, dt: float = 1.0, log=None,W0=0):
        """
        Hover at hover_altitude for duration_s. Uses _climb_pitch_ramp if altitude differs.
        Added small guards for missing keys and clearer prints.
        """
        log = [] if log is None else log

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
            # next state at same height
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
            ## update weigth 
            # fuel_bured = fuel_flow(cs.get('required_power', 0.0),)
            

            if t_new % 10 == 0:
                 print(f"[HOVER] t={self.mission_time:.1f}s | h={cs.get('curr_height', hover_altitude):.1f}m "
          f"| v_z={cs.get('vertical_velocity',0.0):.2f}m/s | pitch={cs.get('required_pitch',0.0):.2f}° "
          f"| fuel={cs.get('Wf',0.0)/g:.2f}kg | power={cs.get('required_power',0.0)/1000:.2f}kW "
          f"| W={cs.get('W0',0.0)/g:.2f}kg")
                 
            if cs.get('Wf', 0.0) / g <= 0.01:
                return {"status": "FUEL_EMPTY_DURING_HOVER", "time": t, "log": log}
            if cs.get('max_alpha', -999.0) >= 12.0:
                print(">>> Predicted stall during hover.")
                return {"status": "STALL_PREDICTED", "time": t, "log": log}

        return {"status": "OK", "time": t, "log": log}


    # -------------------------
    # Public API
    # -------------------------
    def run_mission(self, mission_profile, dt: float = 1.0, log_file="mission.csv"):
        """
        Run a sequence of mission steps dynamically from mission_profile.

        Args:
            mission_profile (list[dict]): Each dict describes a mission step.
            dt (float): Time step for simulation.
            log_file (str): Path to save CSV log.
        """
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

            # Pull last altitude safely
            final_altitude = res["log"][-1]["height"] if res.get("log") else float("nan")

            # Step summary
            print(f"[MISSION] Step {i}: {step_type.upper()} finished → {res['status']} "
                f"at t={self.mission_time:.1f}s, altitude={final_altitude:.1f}m")

        # Save CSV if logging is available
        if log:
            keys = log[0].keys()
            with open(log_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(log)

        return log


        

def visualize_mission( logs):
    
    if not logs:
        print("Empty logs, nothing to visualize.")
        return

    time = [d["time"] for d in logs]
    height = [d.get("height", 0) for d in logs]
    power = [d.get("power_kw", 0)  for d in logs]
    pitch = [d.get("pitch", 0) for d in logs]
    fuel = [d.get("fuel_kg", 0) for d in logs]

    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    axs[0].plot(time, height, label="Altitude (m)")
    axs[1].plot(time, pitch, color="orange", label="Pitch (deg)")
    axs[2].plot(time, power, color="red", label="Power (kW)")
    axs[3].plot(time, fuel, color="green", label="Fuel (kg)")

    axs[0].set_ylabel("Altitude (m)")
    axs[1].set_ylabel("Pitch (deg)")
    axs[2].set_ylabel("Power (kW)")
    axs[3].set_ylabel("Fuel (kg)")
    axs[3].set_xlabel("Time (s)")

    for ax in axs:
        ax.grid(True)
        ax.legend()

    plt.suptitle("Helicopter Mission Profile")
    plt.tight_layout()
    plt.show()

if __name__ =="__main__":
    
    params = {
        "W0_guess": 4000,
        "W_pl_target": 0,
        "V_max": 200,
        "crew": 15,
        "Rg_target": 439,
        "rho_f": 0.8
    }
    planner = Mission_planner(params)
    
    result = planner._hover(0,10000)


