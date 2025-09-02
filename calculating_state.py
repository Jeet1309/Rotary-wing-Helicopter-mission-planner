import numpy as np
import math
from performance_tool import rad_s_to_rpm, rpm_to_rad_s, deg_to_rad, rad_to_deg, fuel_flow, available_engine_power, chord_variation, twist_variation, isa, aerod, TPQ, lamda_prandtl
from statistical_design import HelicopterDesigner

class Helicopterstate_simulator:
    def __init__(self, design_parameters, g=9.81):
        """
        Initializes the simulator with helicopter design parameters.

        Args:
            design_parameters (dict): A dictionary containing all the
                                      necessary helicopter design data.
            g (float): Gravitational acceleration in m/s^2.
        """
        self.g = g
        self.design_params = design_parameters
        self.W0_initial = self.design_params['Weights']['W0']
        self.Wf_initial = self.design_params['Weights']['Wf']
        self.current_state = {
            'W0': self.W0_initial,
            'Wf': self.Wf_initial,
            'curr_height': 0.0,
            'required_pitch': 0.0,
            'required_power': 0.0,
            'vertical_velocity': 0.0,
            'time_step_duration': 0.0
        }
        self.engine_power_sl = self.design_params['Engine Suggestion']['Power (kw)']
        self._initialize_arrays_main_rotor()
        self._initialize_arrays_tail_rotor() # New call for tail rotor

    def _initialize_arrays_main_rotor(self):
        """Initializes the radial and blade property arrays for the main rotor."""
        p = self.design_params['Main Rotor']
        self.r_arr_main = np.linspace(p['Root_cutoff'], p['Blade_radius'], p['Div'])
        self.chord_array_main = chord_variation(p['Chord_root'], p['Chord_tip'], p['Blade_radius'], p['Root_cutoff'], self.r_arr_main)
        self.theta_array_main = twist_variation(p['Twist_root'], p['Twist_tip'], p['Blade_radius'], p['Root_cutoff'], self.r_arr_main)
        self.rad_s_main = p['rad_s']
        self.blades_main = p['blades']
        self.radius_main = p['Blade_radius']
        self.lift_slope_main = p['lift_slope']
        self.div_main = p['Div']

    def _initialize_arrays_tail_rotor(self):
        """Initializes the radial and blade property arrays for the tail rotor."""
        p = self.design_params['Tail Rotor']
        self.r_arr_tail = np.linspace(p['Root_cutoff'], p['Blade_radius'], p['Div'])
        self.chord_array_tail = chord_variation(p['Chord_root'], p['Chord_tip'], p['Blade_radius'], p['Root_cutoff'], self.r_arr_tail)
        self.theta_array_tail = twist_variation(p['Twist_root'], p['Twist_tip'], p['Blade_radius'], p['Root_cutoff'], self.r_arr_tail)
        self.rad_s_tail = p['rad_s']
        self.blades_tail = p['blades']
        self.radius_tail = p['Blade_radius']
        self.lift_slope_tail = p['lift_slope']
        self.div_tail = p['Div']

    def _calculate_blade_properties(self, collective_pitch: float, height_m: float, climb_vel: float, main_rotor: bool = True) -> tuple:
        """
        Calculates blade properties (thrust, power, torque, etc.) for a given state.
        This is a refactored version of the original `calculate_blade_properties`.
        """
        if main_rotor:
            r_arr = self.r_arr_main
            chord_array = self.chord_array_main
            theta_array = self.theta_array_main
            rad_s = self.rad_s_main
            blades = self.blades_main
            radius = self.radius_main
            lift_slope = self.lift_slope_main
            div = self.div_main
        else:
            r_arr = self.r_arr_tail
            chord_array = self.chord_array_tail
            theta_array = self.theta_array_tail
            rad_s = self.rad_s_tail
            blades = self.blades_tail
            radius = self.radius_tail
            lift_slope = self.lift_slope_tail
            div = self.div_tail
            
        net_theta_deg = theta_array + collective_pitch
        net_theta_rad = deg_to_rad(net_theta_deg)
        sigma = (blades * chord_array) / (math.pi * radius)
        tol = 1e-12
        lamda_c = climb_vel / (rad_s * radius)
        induced_vel = lamda_prandtl(climb_vel, blades, r_arr, radius, sigma, lift_slope, tol, net_theta_rad, math.pi, lamda_c, rad_s)
        T, Pressure, rho = isa(height_m)
        cl, cd, alpha, U_p, U_t, Phi = aerod(rad_s, r_arr, climb_vel, induced_vel, net_theta_rad, lift_slope, T)
        
        # Check for stall condition
        max_alpha_deg = np.max(rad_to_deg(alpha))
        if max_alpha_deg > 12:
            print(f"Stalling at {collective_pitch:.2f} deg, alpha = {max_alpha_deg:.2f} deg.")
        
        Thrust, Power, Torque = TPQ(rho, U_p, U_t, chord_array, cl, cd, Phi, radius, div, blades, r_arr, rad_s)
        
        C_t = (2 * Thrust) / (rho * math.pi * (rad_s**2) * (radius**4))
        C_q = (2 * Torque) / (rho * math.pi * (rad_s**2) * (radius**5))
        
        return C_t, C_q, Thrust, Power,Phi,net_theta_deg

    def _newton_solver(self, target_func, initial_guess: float, *args) -> tuple:
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
        _, _, thrust, power,_,_ = self._calculate_blade_properties(pitch, height_m, climb_vel=0)
        return pitch, thrust, power

    def calculate_next_state(self, dt: float = 0.1, next_height: float = 0.0):
        """
        Calculates the next state of the helicopter for a single time step.
        """
        current_height = self.current_state['curr_height']
        current_weight = self.current_state['W0']
        current_fuel_weight = self.current_state['Wf']

        if current_fuel_weight <= 0.01:
            print("Warning: Fuel exhausted!")
            self.current_state['Wf'] = 0.0
            return
            
        v_z = (next_height - current_height) / dt

        power_available = available_engine_power(self.engine_power_sl, current_height) * 1000 # Convert kW to W
        
        # Find the required pitch and power for the climb
        def climb_error_func(pitch: float, h: float, v_climb: float):
            thrust = self._calculate_blade_properties(pitch, h, v_climb, main_rotor=True)[2]
            return thrust - current_weight

        required_pitch, _ = self._newton_solver(climb_error_func, 8, current_height, v_z)
        _, _, _, required_power,PHI,net_theta_deg = self._calculate_blade_properties(required_pitch, current_height, v_z, main_rotor=True)

        # Handle power limitations
        if required_power > power_available:
            print(f"Warning: Required power ({required_power/1000:.2f} kW) exceeds available power ({power_available/1000:.2f} kW). Adjusting climb rate.")
            
            def new_climb_error_func(v_climb: float, h: float):
                _, _, _, power,PHI,_= self._calculate_blade_properties(required_pitch, h, v_climb, main_rotor=True)
                return power - power_available
            
            v_z_new, _ = self._newton_solver(new_climb_error_func, v_z, current_height)
            v_z = v_z_new
            required_power = power_available
            next_height = current_height + (v_z * dt)
            print(f"Adjusted vertical velocity to {v_z:.2f} m/s, new target height: {next_height:.2f} m.")

            

        # Calculate fuel burn
        sfc = self.design_params['Engine Suggestion']['SFC']
        fuel_burned = fuel_flow(required_power, sfc, dt) # kg
        
        # Update state variables
        self.current_state['Wf'] -= fuel_burned*self.g  # Convert kg to N
        self.current_state['W0'] -= fuel_burned*self.g  # Convert kg to N
        self.current_state['curr_height'] = next_height
        
        # Add other calculated parameters for reference
        self.current_state['required_pitch'] = required_pitch
        self.current_state['required_power'] = required_power
        self.current_state['vertical_velocity'] = v_z
        self.current_state['time_step_duration'] = dt
        self.current_state['power_available'] = power_available
        self.current_state['pitch'] = required_pitch
        self.current_state['max_alpha'] = np.max(net_theta_deg)
    def reset_state(self, W0, altitude):
        self.current_state["W0"] = W0, 
        self.current_state['curr_height'] = altitude