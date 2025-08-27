import math

class HelicopterDesigner:
    """
    A class to calculate and manage the design parameters of a helicopter 
    based on initial inputs like gross weight, max speed, and mission requirements.
    """

    def __init__(self, W0, V_max, Nb, Nb_tr, W_pl, Wc, Rg, rho_f):
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
        self.Wc = Wc
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
        V_tip = 140 / (D**0.171)
        ang_vel = V_tip * 2 / D
        
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
        V_tip_tail = 182 / (D_tr**0.172)
        ang_vel_tail = V_tip_tail * 2 / D_tr
        c_tr = 0.0058 * (self.W0**0.506) / (self.Nb_tr**0.720)

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
            "Gross Weight (kg)": self.W0,
            "Useful Weight (kg)": Wu_actual,
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

    # --- Main Calculation and Helper Methods ---

    def run_calculations(self):
        """Orchestrates all calculation methods and stores the results."""
        self.calculate_main_rotor_parameters()
        self.calculate_tail_rotor_parameters()
        self.calculate_airframe_dimensions()
        self.calculate_weights()
        self.calculate_performance()
        self.calculate_power()
        return self.results

    def iterative_gross_weight_calculator(self,W0_guess, tolerance=1e-3, max_iterations=1000):
        """
        Iteratively calculates the gross weight (W0) until it converges,
        based on useful weight requirements.
        
        This function is kept separate as a utility since it doesn't depend on
        the full set of HelicopterDesigner parameters.
        """

        W0_new = 0
        iteration = 0

        while abs(W0_new - W0_guess) > tolerance and iteration < max_iterations:
            if iteration > 0:
                W0_guess = W0_new
            
            # Calculate required useful weight components
            Wf = (0.0038 * (W0_guess**0.976) * (self.Rg**0.650)) * self.rho_f
            Wu = Wf + self.W_pl + self.Wc
            
            # Recalculate W0 based on the useful weight approximation
            W0_new = (Wu / 0.4709)**(1 / 0.99)
            
            iteration += 1
            print(f"Iteration: {iteration}, Gross Weight Guess: {W0_guess:.2f} kg, New W0: {W0_new:.2f} kg")

        if iteration >= max_iterations:
            print(f"Warning: Maximum iterations ({max_iterations}) reached before convergence. "
                "Consider reducing range or payload.")
        else:
            print(f"\nConvergence achieved in {iteration} iterations.")
            print(f"The final converged gross weight is: {W0_new:.2f} kg")
            
        self.W0 = W0_new
        

        # self.results['Weights']["Gross Weight (kg)"] = W0_new

def main():
    """
    Main function to run the full helicopter design process, including
    the iterative gross weight calculation.
    """
    # Define mission requirements
    W0_guess = 5000 
    W_pl_target = 1500  # Target payload (kg)
    Wc_target = 100     # Target crew weight (kg)
    Rg_target = 439     # Target range (km)
    rho_f = 0.8         # Fuel density (kg/L)
    V_max = 200         # Maximum speed (m/s)
    Nb = 2              # Number of main rotor blades
    Nb_tr = 2           # Number of tail rotor blades

    # Step 1: Iteratively find the optimal gross weight (W0)
    # print("--- Calculating Iterative Gross Weight (W0) ---")
    # final_W0 = iterative_gross_weight_calculator(
    #     W_pl=W_pl_target, 
    #     Wc=Wc_target, 
    #     Rg=Rg_target, 
    #     rho_f=rho_f
    # )

    # Step 1: Use the final W0 to instantiate the designer and calculate all parameters
    print("\n--- Final Helicopter Design Parameters ---")
    designer = HelicopterDesigner(
        W0=W0_guess,
        V_max=V_max,
        Nb=Nb,
        Nb_tr=Nb_tr,
        W_pl=W_pl_target,
        Wc=Wc_target,
        Rg=Rg_target,
        rho_f=rho_f
    )
    # final_design_parameters = designer.run_calculations()

    #Step 2: Iteratively find the optimal gross weight (W0)
    designer.iterative_gross_weight_calculator(W0_guess)
    
    final_design_parameters = designer.run_calculations()

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
