import math
import pandas as pd
import numpy as np

class HelicopterDesigner:
    """
    A class to calculate and manage the design parameters of a helicopter 
    based on initial inputs like gross weight, max speed, and mission requirements.
    """

    def __init__(self, W0, V_max, Nb, Nb_tr, W_pl, crew, Rg, rho_f):
        """
        Initializes the HelicopterDesigner with core design parameters.

        Args:
            W0 (float): Gross weight in kg.
            V_max (float): Maximum speed in m/s.
            Nb (int): Number of main rotor blades.
            Nb_tr (int): Number of tail rotor blades.
            W_pl (float): Payload weight in kg.
            Wc (float): Crew weight in kg.
            Rg (float): Range in km.
            rho_f (float): Fuel density in kg/L.
        """
        self.W0 = W0
        self.V_max = V_max
        self.Nb = Nb
        self.Nb_tr = Nb_tr
        self.W_pl = W_pl
        self.Wc = crew * 110
        self.Rg = Rg
        self.rho_f = rho_f

        # Attributes to store calculated results
        self.results = {}

    # --- Rotor and Airframe Component Calculations ---
    
    def calculate_main_rotor_parameters(self):
        """Calculates main rotor characteristics based on gross weight and max speed."""
        DL = 2.12 * (self.W0**(1/3) - 0.57)
        D = 9.133 * ((self.W0**0.380) / (self.V_max**0.515))
        c = 0.0108 * ((self.W0**0.539) / (self.Nb**0.714))
        V_tip = 140 * (D**0.171)
        ang_vel = V_tip * 2 / D
        
        if (V_tip > 236): 
            print("main rotor tip velocity is very high")
        
        self.results['Main Rotor'] = {
            "Disc Loading (kg/m^2)": DL,
            "Main Rotor Diameter (m)": D,
            "Main Rotor Chord (m)": c,
            "Main Rotor Tip Speed (m/s)": V_tip,
            "Main Rotor Angular Velocity (rad/s)": ang_vel,
        }
        
    def calculate_tail_rotor_parameters(self):
        """Calculates tail rotor characteristics."""
        D_tr = 0.0886 * (self.W0**0.393)
        a_mt = 0.5107 * (self.results['Main Rotor']['Main Rotor Diameter (m)']**1.061)
        V_tip_tail = 182 * (D_tr**0.172)
        ang_vel_tail = V_tip_tail * 2 / D_tr
        c_tr = 0.0058 * (self.W0**0.506) / (self.Nb_tr**0.720)
        if (V_tip_tail > 236): 
            print("tail rotor tip velocity is very high")

        self.results['Tail Rotor'] = {
            "Tail Rotor Diameter (m)": D_tr,
            "Tail Rotor Arm (m)": a_mt,
            "Tail Rotor Tip Speed (m/s)": V_tip_tail,
            "Tail Rotor Angular Velocity (rad/s)": ang_vel_tail,
            "Tail Rotor Chord (m)": c_tr,
        }

    def calculate_airframe_dimensions(self):
        """Calculates main airframe and tail dimensions."""
        D = self.results['Main Rotor']['Main Rotor Diameter (m)']
        D_tr = self.results['Tail Rotor']['Tail Rotor Diameter (m)']
        
        # Airframe dimensions
        FL = 0.824 * (D**1.056)
        L_rt = 1.09 * (D**1.03)
        FH = 0.642 * (D**0.677)
        FW = 0.436 * (D**0.697)
        
        # Horizontal tail
        a_ht = 0.4247 * (self.W0**0.327)
        S_ht = 0.0021 * (self.W0**0.758)
        
        # Vertical tail
        a_vt = 0.5914 * (D**0.995)
        S_vt = 0.5914 * (D**0.995)
        c_vt = 0.1605 * (D_tr**1.745) if D_tr < 3.5 else 0.297 * (D_tr**1.06)

        self.results['Airframe and Tail'] = {
            "Fuselage Length (m)": FL,
            "Rotor-to-Tail-End Length (m)": L_rt,
            "Helicopter Height (m)": FH,
            "Helicopter Width (m)": FW,
            "Horizontal Tail Arm (m)": a_ht,
            "Horizontal Tail Surface Area (m^2)": S_ht,
            "Vertical Tail Arm (m)": a_vt,
            "Vertical Tail Surface Area (m^2)": S_vt,
            "Average Vertical Tail Chord (m)": c_vt,
        }

    # --- Weight and Performance Calculations ---

    def calculate_weights(self):
        """Calculates useful weight, empty weight, and performs a simple check."""
        We = 0.4854 * (self.W0**1.015)
        Wf = (0.0038 * (self.W0**0.976) * (self.Rg**0.650)) * self.rho_f
        Wu_possible = 0.4709 * (self.W0**0.99)
        Wu_actual = Wf + self.W_pl + self.Wc
        
        msg = "all good"
        
        if (Wu_actual - Wu_possible) / Wu_actual > 0.1:
            msg = "not enough useful weight margin; consider changing W0 or reducing range."

        self.results['Weights'] = {
            "MTOW (kg)": self.W0,
            "Useful Weight (kg)": Wu_actual,
            "fuel weight": Wf,
            "Empty Weight (kg)": We,
            "Weight Check Message": msg,
        }

    def calculate_performance(self):
        """Calculates never-exceed and long-range speeds."""
        V_ne = 0.8215 * (self.V_max**1.056)
        V_lr = 0.5475 * (self.V_max**1.0899)
        
        self.results['Performance'] = {
            "Never Exceed Speed (m/s)": V_ne,
            "Long Range Speed (m/s)": V_lr,
        }

    def calculate_power(self):
        """Calculates required power and transmission specs."""
        P_to = 0.0764 * (self.W0**1.1455)
        T_to = 0.0366 * (self.W0**1.2107)
        P_mc = 0.00126 * (self.W0**0.9876) * (self.V_max**0.9760)
        T_mc = 0.000141 * (self.W0**0.9771) * (self.V_max**1.3393)
        
        self.results['Power and Transmission'] = {
            "Take-off Power (P_to)(kW)": P_to,
            "Take-off Transmission (T_to)(kW)": T_to,
            "Main continuous Power (P_mc)(kW)": P_mc,
            "Main continuous Transmission (T_mc)(kW)": T_mc,
        }

    def suggest_engine(self):
        """
        Suggests a single engine with power between 1.2 and 2 times the required
        take-off power. If multiple engines meet the criteria, the one with
        the highest power is selected.
        """
        
        # Check if 'Take-off Power (P_to)(kW)' exists in the results dictionary
        try:
            p_to = self.results['Power and Transmission']['Take-off Power (P_to)(kW)']
        except KeyError:
            print("Error: Take-off power data not found. Please run power calculations first.")
            self.results['Engine Suggestion'] = "Power data unavailable."
            return

        engine_power_requirement = p_to * 1.2

        # Using a robust file path or handling potential file errors
        try:
            # Assumes the CSV file is in the same directory or a known path
            df = pd.read_csv('Heli_engine_data - Sheet1.csv')
        except FileNotFoundError:
            print("Error: Engine data file 'Heli_engine_data - Sheet1.csv' not found.")
            self.results['Engine Suggestion'] = "Engine database unavailable."
            return

        # Filter for all engines that meet the power criteria
        min_power = p_to * 1.2
        max_power = p_to * 2.0
        
        suitable_engines = df[(df['Power (kw)'] >= min_power) & (df['Power (kw)'] <= max_power)].copy()
        
        if suitable_engines.empty:
            suggestion = "No suitable engine found in the specified power range."
            self.results['Engine Suggestion'] = suggestion
        else:
            # Find the engine with the highest power from the suitable list
            best_engine = suitable_engines.loc[suitable_engines['SFC'].idxmax()]
            
            # Save only the single best engine as a dictionary
            self.results['Engine Suggestion'] = best_engine.to_dict()
    def run_calculations(self):
        """Orchestrates all calculation methods and stores the results."""
        self.calculate_main_rotor_parameters()
        self.calculate_tail_rotor_parameters()
        self.calculate_airframe_dimensions()
        self.calculate_weights()
        self.calculate_performance()
        self.calculate_power()
        self.suggest_engine()
        return self.results
    
    def iterative_gross_weight_calculator(self, W0_guess, tolerance=1e-6, max_iterations=1000):
        """
        Iteratively calculates the gross weight (W0) until it converges,
        based on useful weight requirements.
        """
        error = 1
        iteration = 1
        while (error > tolerance):
            W0_new = (0.4854 * (W0_guess ** 1.015)) + (0.0038 * (W0_guess ** 0.976) * (self.Rg ** 0.65) * self.rho_f) + self.Wc + self.W_pl
            error = abs(W0_new - W0_guess) / W0_guess
            W0_guess = W0_new
                
            iteration += 1
            print(f"Iteration: {iteration}, Gross Weight Guess: {W0_guess:.2f} kg, New W0: {W0_new:.2f} kg")

            if iteration >= max_iterations:
                print(f"Warning: Maximum iterations ({max_iterations}) reached before convergence. "
                      "Consider reducing range or payload.")
                break  
        if (iteration < max_iterations):
            print(f"\nConvergence achieved in {iteration} iterations.")
            print(f"The final converged gross weight is: {W0_new:.2f} kg")
        
        self.W0 = W0_new
        
def main():
    """
    Main function to run the full helicopter design process, including
    the iterative gross weight calculation.
    """
    W0_guess = 4000 
    W_pl_target = 0
    crew = 15
    Rg_target = 439
    rho_f = 0.8
    V_max = 200
    Nb = 2
    Nb_tr = 2 

    print("\n--- Final Helicopter Design Parameters ---")
    designer = HelicopterDesigner(
        W0=W0_guess,
        V_max=V_max,
        Nb=Nb,
        Nb_tr=Nb_tr,
        W_pl=W_pl_target,
        crew=crew,
        Rg=Rg_target,
        rho_f=rho_f
    )
    designer.iterative_gross_weight_calculator(W0_guess)
    
    final_design_parameters = designer.run_calculations()
    # print(final_design_parameters['Engine Suggestion'])

    # Step 3: Print the final results in a structured format
    for section, params in final_design_parameters.items():
        print(f"\n--- {section} ---")
        for key, value in params.items():
            if isinstance(value, float):
                print(f"{key}: {value:.2f}")
            else:
                print(f"{key}: {value}")

if __name__ == '__main__':
    main()