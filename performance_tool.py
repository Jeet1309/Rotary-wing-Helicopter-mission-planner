import math
import numpy as np
import matplotlib.pyplot as plt

pi = math.pi
# ------------------------------------------------------------------------------------------------------------------------------------------------
def rpm_to_rad_s(rpm):
    return rpm * 2 * pi / 60
# ------------------------------------------------------------------------------------------------------------------------------------------------
def rad_s_to_rpm(rad_s):
    return rad_s * 60 / (2 * pi)
# ------------------------------------------------------------------------------------------------------------------------------------------------
def deg_to_rad(deg):
    return deg * pi / 180
# ------------------------------------------------------------------------------------------------------------------------------------------------
def rad_to_deg(rad):
    return rad * 180 / pi
# ------------------------------------------------------------------------------------------------------------------------------------------------
def degC_to_kel(deg):
    return (273.15 + deg)
# ------------------------------------------------------------------------------------------------------------------------------------------------
def ft_to_m(ft):
     return ft*0.3048
# ------------------------------------------------------------------------------------------------------------------------------------------------
def twist_variation(Twist_root, Twist_tip, Blade_radius, Root_cutoff, r_array):
    slope = (Twist_root - Twist_tip)/(Blade_radius - Root_cutoff)
    Twist_array = Twist_root - (slope * (r_array - Root_cutoff))
    return Twist_array
# ------------------------------------------------------------------------------------------------------------------------------------------------
def chord_variation(Chord_root:float , Chord_tip:float, Blade_radius:float, Root_cutoff:float, r_array: list) -> list:
    slope = (Chord_root - Chord_tip)/(Blade_radius - Root_cutoff)
    Chord_array = Chord_root - slope*(r_array - Root_cutoff)
    return Chord_array

# ------------------------------------------------------------------------------------------------------------------------------------------------
def isa(Height_m):
    # T,P,D at sea level
    Temp_SL = degC_to_kel(15)           # temp at sea level in deg c
    Pre_SL = 101325                     # pressure in pascals [N/m2]
    Den_SL = 1.225                      # density at sea level [kg/m3]
    g0 = 9.807                          # Gravity at sea level
    R = 287

    #T,P,D at Height H in m
    pow = (g0 / (-0.0065 * R))
    Temp_H = Temp_SL - (0.0065 * Height_m)
    Pre_H = Pre_SL * (Temp_H/Temp_SL)**(-pow)
    Den_H = Den_SL * (Temp_H/Temp_SL)**(-pow-1)

    return Temp_H, Pre_H, Den_H

# ------------------------------------------------------------------------------------------------------------------------------------------------

def lamda_prandtl(Climb_velocity, blades, r_array, Blade_radius, sigma, lift_slope, tol, Theta_rad, pi, Lamda_c, rad_s):

    lam = 1 * np.ones_like(r_array)

    # for Hover Case 
    if Climb_velocity == 0:
        #print("V_c is zero")
        for i in range(100):
            f = (blades / 2) * (1 - (r_array/Blade_radius)) / lam
            F = (2/pi)*np.arccos(np.exp(-f))
            F = np.maximum(F, 1e-6)

            lam_new = np.sqrt(((sigma*lift_slope)/(16*F))**2 + ((sigma*lift_slope*Theta_rad*r_array)/(8*F*Blade_radius)) ) - ((sigma*lift_slope)/(16*F))
            # lamda_tiploss
            # print(np.all(np.abs(lam_new - lam) < tol))
            if np.all(np.abs(lam_new - lam) < tol):
                lamda_tiploss = lam_new
                break

            lam = lam_new

    # for Climb Case
    else:
        #print("V_c is not zero")
        for i in range(100):
            f = (blades / 2) * (1 - (r_array/Blade_radius)) / lam
            F = (2/pi)*np.arccos(np.exp(-f))
            F = np.maximum(F, 1e-6)

            lam_new = np.sqrt( (((sigma*lift_slope)/(16*F)) - (Lamda_c/2))**2 + ((sigma*lift_slope*Theta_rad*r_array)/(8*F*Blade_radius)) ) - (((sigma*lift_slope)/(16*F)) - (Lamda_c/2))
            
            if np.all(np.abs(lam_new - lam) < tol):
                lamda_tiploss = lam_new
                break
            
            lam = lam_new
    try:
        Lamda = lamda_tiploss*1.062
    except:
        Lamda = lam# K factor accouting for other various losses 

    induced_vel = (Lamda * rad_s * Blade_radius) - Climb_velocity

    return(induced_vel)

# ------------------------------------------------------------------------------------------------------------------------------------------------

def aerod(rad_s, r_array, climb_vel, induced_vel, Theta_rad, lift_slope,Temp):
    U_t = rad_s * r_array
    U_p = climb_vel + induced_vel
    phi = np.arctan(U_p/U_t)
    alpha_eff = Theta_rad - phi

    # base Cl and Cd 
    C_l = lift_slope * alpha_eff
    C_d = 0.0113 + (0.3 * (alpha_eff**2))

    # finding mach number included temp variations
    sound_speed = math.sqrt(1.4 * 287 * (273 + Temp)) 
    mach_no = (rad_s * r_array)/(sound_speed)
    
    if np.max(mach_no) < 1:
        # Accounting for compressible losses
        C_l = C_l / (np.sqrt(1 - mach_no**2)) 
        C_d = C_d/(np.sqrt(1 - mach_no**2))
    else:
        # clamp or warn
        C_l = C_l
        C_d = C_d
        print("Warning: Mach > 1 region reached, forcing C_l=C_d=0")

    return (C_l, C_d, alpha_eff, U_p, U_t, phi)

# ------------------------------------------------------------------------------------------------------------------------------------------------

def TPQ(rho, U_p, U_t, C_m, C_l, C_d, phi, R, Div, b, r, rad_s):

    ## finding Total Thrust 
    Thrust_per_blade_elem = 0.5 * rho * (U_p**2 + U_t**2) * C_m * (C_l * np.cos(phi) - C_d * np.sin(phi)) * (R/Div)
    Thrust_one_blade = np.sum(Thrust_per_blade_elem)        # Total Thrust for one blade
    Total_Thrust = Thrust_one_blade * b                     # Total Thrust of the rotor

    ## finding Total Drag or Force in X direction
    Fx_per_blade_elem = 0.5 * rho * (U_p**2 + U_t**2) * C_m * (C_l * np.sin(phi) + C_d * np.cos(phi)) * (R/Div)
    Fx_one_blade = np.sum(Fx_per_blade_elem)                # Drag force for one blade
    Total_Fx = Fx_one_blade * b                             # Total Drag force or resistance to spin in X direction

    ## finding Total Torque
    Torque_per_blade_elem = Fx_per_blade_elem * r           # Torque at each balde element sections
    Torque_one_blade = np.sum(Torque_per_blade_elem)        # Total torque per blade
    Total_Torque = Torque_one_blade * b                     # Total Torque for all blades

    ## finding Total Power
    Power_per_blade_elem = Fx_per_blade_elem * r * rad_s    # Power req at each secton of blade element
    Power_one_blade = np.sum(Power_per_blade_elem)          # Total sum of power req at one blade 
    Total_Power = Power_one_blade * b                       # Total Power for all blades


    return Total_Thrust, Total_Power, Total_Torque

# ------------------------------------------------------------------------------------------------------------------------------------------------


def plot_results(time_arr, data_arr, y_label,xlabel, title):
    """
    Plots a single result array against time.
    """
    plt.figure()
    plt.plot(time_arr, data_arr, label=y_label)
    plt.xlabel(xlabel)
    plt.ylabel(y_label)
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.show()
    
# ------------------------------------------------------------------------------------------------------------------------------------------------

def available_engine_power(engine_power_sl, altitude):
    rho0 = 1.225  # kg/m^3
    k = 0.03
    _, _, rho = isa(altitude)
    engine_powe_alt  = engine_power_sl * (rho / rho0)
    return engine_powe_alt
# ------------------------------------------------------------------------------------------------------------------------------------------------

def fuel_flow(P_req, SFC, dt):
    return (P_req * SFC*dt)/ (3600 * 1000)  # kg
# ------------------------------------------------------------------------------------------------------------------------------------------------

def find_pitch(self, height_m: float,climb_vel:float = 0) -> tuple:
        """
        Calculates the required collective pitch and power for a hover at a given height.
        
        Args:
            height_m (float): The altitude in meters.
            
        Returns:
            A tuple of (pitch, thrust, power).
        """
        def thrust_error_func(pitch, h):
            thrust = self._calculate_blade_properties(pitch, h, climb_vel)[2]
            return thrust - self.current_state['W0']
            
        pitch, _ = self._newton_solver(thrust_error_func, 8, height_m)
        _, _, thrust, power = self._calculate_blade_properties(pitch, height_m, climb_vel=0)
        return pitch, thrust, power

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
def newton_solver(target_func, initial_guess: float, *args) -> tuple:
    tol = 1e-3
    max_iter = 50
    """
    A robust Newton-Raphson solver.
    This is a static method to keep the solver independent of the class state.
    """
    x = initial_guess
    delta = 1e-5
    for i in range(max_iter):
        f = target_func(x, *args)
        if abs(f) < tol:
            return x, f
        f_prime = (target_func(x + delta, *args) - target_func(x - delta, *args)) / (2 * delta)
        if abs(f_prime) < 1e-8:
            print("Derivative vanished, stopping.")
            return x, f
        x = x - f / f_prime
    print("Warning: Newton solver did not converge.")
    return x, f