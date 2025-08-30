import numpy as np
import math
from helper import rad_s_to_rpm, rpm_to_rad_s, deg_to_rad, rad_to_deg, fuel_flow, available_engine_power, chord_variation, twist_variation, isa, aerod, TPQ, lamda_prandtl
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
        # print(self.design_params['Engine Suggestion'])
        self.engine_power_sl = self.design_params['Engine Suggestion']['Power (kw)']
        self._initialize_arrays()

    def _initialize_arrays(self):
        """Initializes the radial and blade property arrays."""
        p = self.design_params['Main Rotor']
        self.r_arr = np.linspace(p['Root_cutoff'], p['Blade_radius'], p['Div'])
        self.chord_array = chord_variation(p['Chord_root'], p['Chord_tip'], p['Blade_radius'], p['Root_cutoff'], self.r_arr)
        self.theta_array = twist_variation(p['Twist_root'], p['Twist_tip'], p['Blade_radius'], p['Root_cutoff'], self.r_arr)

    def _calculate_blade_properties(self, collective_pitch: float, height_m: float, climb_vel: float) -> tuple:
        """
        Calculates blade properties (thrust, power, torque, etc.) for a given state.
        This is a refactored version of the original `calculate_blade_properties`.
        """
        p = self.design_params['Main Rotor']
        net_theta_deg = self.theta_array + collective_pitch
        net_theta_rad = deg_to_rad(net_theta_deg)
        sigma = (p['blades'] * self.chord_array) / (math.pi * p['Blade_radius'])
        tol = 1e-12
        lamda_c = climb_vel / (p['rad_s'] * p['Blade_radius'])
        induced_vel = lamda_prandtl(climb_vel, p['blades'], self.r_arr, p['Blade_radius'], sigma, p['lift_slope'], tol, net_theta_rad, math.pi, lamda_c, p['rad_s'])
        T, Pressure, rho = isa(height_m)
        cl, cd, alpha, U_p, U_t, Phi = aerod(p['rad_s'], self.r_arr, climb_vel, induced_vel, net_theta_rad, p['lift_slope'], T)
        
        # Check for stall condition
        max_alpha_deg = np.max(rad_to_deg(alpha))
        if max_alpha_deg > 12:
            print(f"Stalling at {collective_pitch:.2f} deg, alpha = {max_alpha_deg:.2f} deg.")
        
        Thrust, Power, Torque = TPQ(rho, U_p, U_t, self.chord_array, cl, cd, Phi, p['Blade_radius'], p['Div'], p['blades'], self.r_arr, p['rad_s'])
        
        C_t = (2 * Thrust) / (rho * math.pi * (p['rad_s']**2) * (p['Blade_radius']**4))
        C_q = (2 * Torque) / (rho * math.pi * (p['rad_s']**2) * (p['Blade_radius']**5))
        
        return C_t, C_q, Thrust, Power,Phi

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
        _, _, thrust, power,_ = self._calculate_blade_properties(pitch, height_m, climb_vel=0)
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
            thrust = self._calculate_blade_properties(pitch, h, v_climb)[2]
            return thrust - current_weight

        required_pitch, _ = self._newton_solver(climb_error_func, 8, current_height, v_z)
        _, _, _, required_power,PHI = self._calculate_blade_properties(required_pitch, current_height, v_z)

        # Handle power limitations
        if required_power > power_available:
            print(f"Warning: Required power ({required_power/1000:.2f} kW) exceeds available power ({power_available/1000:.2f} kW). Adjusting climb rate.")
            
            def new_climb_error_func(v_climb: float, h: float):
                _, _, _, power,PHI= self._calculate_blade_properties(required_pitch, h, v_climb)
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
        self.current_state['max_alpha'] = np.max(required_pitch - rad_to_deg(PHI))
    def reset_state(self, W0, altitude):
        self.current_state["W0"] = W0, 
        self.current_state['curr_height'] = altitude



    # g = 9.81  # m/s^2

    # # Design parameters (from your original code)
    # W0_guess = 4000
    # W_pl_target = 0
    # crew = 15
    # Rg_target = 439
    # rho_f = 0.8
    # V_max = 200
    # Nb = 2
    # Nb_tr = 2
    # design = HelicopterDesigner(W0=W0_guess, V_max=V_max, Nb=Nb, Nb_tr=Nb_tr, W_pl=W_pl_target, crew=crew, Rg=Rg_target, rho_f=rho_f)
    # design.iterative_gross_weight_calculator(W0_guess)
    # final_design_parameters = design.run_calculations()

    # # Prepare a single, cohesive dictionary for the simulator
    # simulator_params = final_design_parameters
    # simulator_params['Main Rotor']['blades'] = Nb
    # simulator_params['Main Rotor']['Chord_root'] = final_design_parameters['Main Rotor']['Main Rotor Chord (m)']
    # simulator_params['Main Rotor']['Chord_tip'] = final_design_parameters['Main Rotor']['Main Rotor Chord (m)']
    # simulator_params['Main Rotor']['Blade_radius'] = final_design_parameters['Main Rotor']['Main Rotor Diameter (m)'] * 1.0 / 2
    # simulator_params['Main Rotor']['Root_cutoff'] = (final_design_parameters['Main Rotor']['Main Rotor Diameter (m)'] * 1.0 / 2) * 0.05
    # simulator_params['Main Rotor']['rpm'] = rad_s_to_rpm(final_design_parameters['Main Rotor']["Main Rotor Angular Velocity (rad/s)"])
    # simulator_params['Main Rotor']['rad_s'] = final_design_parameters['Main Rotor']["Main Rotor Angular Velocity (rad/s)"]
    # simulator_params['Main Rotor']['W0'] = design.W0 * g
    # simulator_params['Weights']['W0'] = design.W0 * g
    # simulator_params['Weights']['Wf'] = final_design_parameters['Weights']["fuel weight"]
    # simulator_params['Main Rotor']['curr_height'] = 0
    # simulator_params['Main Rotor']['V_c'] = 0
    # simulator_params['Main Rotor']['Div'] = 1000
    # simulator_params['Main Rotor']['Twist_root'] = 0
    # simulator_params['Main Rotor']['Twist_tip'] = 0
    # simulator_params['Main Rotor']['lift_slope'] = 5.75

    # # --- Simulation ---
    # heli_sim = Helicopterstate_simulator(design_parameters=simulator_params)

    # # Example 1: Calculate pitch and power for hover
    # pitch_hover, thrust_hover, power_hover = heli_sim.find_hover_pitch(height_m=0)
    # print(f"Required collective pitch for hover at 0m: {pitch_hover:.2f} deg")
    # print(f"Required power for hover at 0m: {power_hover/1000:.2f} kW")

    # # Example 2: Simulate a vertical climb
    # print("\nSimulating a climb from 0m to 100m in 100 seconds.")
    # dt = 1.0
    # total_time = 100
    # current_time = 0

    # while current_time < total_time:
    #     next_height = (current_time + dt) / total_time * 100  # Simple linear climb profile
    #     heli_sim.calculate_next_state(dt=dt, next_height=next_height)
    #     print(f"Time: {current_time+dt:.1f}s, Height: {heli_sim.current_state['curr_height']:.2f}m, Pitch: {heli_sim.current_state['required_pitch']:.2f} deg, Power: {heli_sim.current_state['required_power']/1000:.2f} kW")
    #     current_time += dt

    # print("\nFinal state after simulation:")
    # print(f"Final height: {heli_sim.current_state['curr_height']:.2f} m")
    # print(f"Final gross weight: {heli_sim.current_state['W0']:.2f} N")
    # print(f"Remaining fuel weight: {heli_sim.current_state['Wf']:.2f} kg")