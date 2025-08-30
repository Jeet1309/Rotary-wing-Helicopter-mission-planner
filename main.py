from Mission import Mission_planner,visualize_mission,HelicopterDesigner
import json
import datetime
import csv
import numpy as np
from helper import *
# ------------------------------------------------------------------------------------------------------------------------------------------------
def required_hover_power(W0, altitude, params):
    """
    Computes required hover power at a given weight and altitude.

    Args:
        W0 (float): Gross weight [kg]
        altitude (float): Altitude [m]
        params (dict): rotor/engine parameters
    
    Returns:
        dict with thrust, power, stall (bool)
    """
    g = 9.81
    thrust_req = W0 * g

    # Atmosphere at altitude
    T, P, rho = isa(altitude)

    # Rotor parameters
    R = params["R"]            # rotor radius [m]
    b = params["blades"]       # number of blades
    c = params["chord"]        # representative chord [m]
    lift_slope = params["lift_slope"]  # per rad
    rpm = params["rpm"]
    rad_s = rpm_to_rad_s(rpm)

    # Discretize blade
    n_elem = 20
    r = np.linspace(0.2*R, R, n_elem)
    dr = R/n_elem
    chord = np.ones_like(r) * c
    theta = deg_to_rad(params["theta0_deg"])  # initial guess collective

    # Iterative solve for pitch (Newton solver)
    def thrust_error(theta_in, h):
        cl, cd, alpha, U_p, U_t, phi = aerod(rad_s, r, 0.0,  # hover
                                             lamda_prandtl(0, b, r, R, (b*c)/(pi*R), 
                                                           lift_slope, 1e-4, theta_in, pi, 0, rad_s),
                                             theta_in, lift_slope, T-273)
        Cm = chord
        Thrust, Power, Torque = TPQ(rho, U_p, U_t, Cm, cl, cd, phi, R, n_elem, b, r, rad_s)
        return Thrust - thrust_req

    theta_sol, err = newton_solver(thrust_error, theta, altitude)

    # Recompute with final theta
    cl, cd, alpha, U_p, U_t, phi = aerod(rad_s, r, 0.0,
                                         lamda_prandtl(0, b, r, R, (b*c)/(pi*R), lift_slope, 1e-4, theta_sol, pi, 0, rad_s),
                                         theta_sol, lift_slope, T-273)
    Cm = chord
    Thrust, Power, Torque = TPQ(rho, U_p, U_t, Cm, cl, cd, phi, R, n_elem, b, r, rad_s)

    stalled = np.any(alpha > deg_to_rad(params["stall_angle_deg"]))

    return {
        "W0": W0,
        "thrust": Thrust,
        "power": Power,
        "stall": stalled
    }

# ------------------------------------------------------------------------------------------------------------------------------------------------
def get_mtow_stall_2000m(W0_range, params, altitude=2000):
    """
    Returns MTOW limited by stall at 2000m.
    """
    feasible = []
    for W0 in W0_range:
        res = required_hover_power(W0, altitude, params)
        if not res["stall"]:
            feasible.append(W0)
        else:
            break
    return max(feasible) if feasible else None

# ------------------------------------------------------------------------------------------------------------------------------------------------
def get_mtow_power_2000m(W0_range, params, altitude=2000):
    """
    Returns MTOW limited by available power at 2000m.
    """
    feasible = []
    for W0 in W0_range:
        res = required_hover_power(W0, altitude, params)
        Pavail = available_engine_power(params["engine_power_sl"], altitude)
        if res["power"] <= Pavail and not res["stall"]:
            feasible.append(W0)
        else:
            break
    return max(feasible) if feasible else None

    
    
    
if __name__ == "__main__":
#     mission = [
#     {"type": "climb_time", "target_altitude": 100, "climb_time": 100},

#     {"type": "hover",  "duration": 100},
#     {"type": "hover", "altitude": 200, "duration": 100},
# ]
    
    params = {
        "W0_guess": 4000,
        "W_pl_target": 0,
        "V_max": 200,
        "crew": 100,
        "Rg_target": 439,
        "rho_f": 0.8
    }
    planner = Mission_planner(params)
    
    log = planner.run_mission(mission_profile=mission,log_file="miss2.csv")
    

    # Run the test mission
    
    # visualize_mission(log)
    
    
