
## How it Works 🧠

The program follows a two-step process:

1.  **Iterative Gross Weight Calculation:** The script first determines the optimal **gross weight** (the total weight of the helicopter with all its components, crew, payload, and fuel). It uses an iterative method, which means it starts with an initial guess and repeatedly refines it until the weight converges to a stable, realistic value that meets your mission's useful weight requirements.

2.  **Comprehensive Parameter Calculation:** Once the final gross weight is determined, the program uses it as a core input to calculate all other design specifications. This includes:
    * **Rotor Dimensions:** Main and tail rotor diameters, tip speeds, and blade chords.
    * **Airframe Size:** Fuselage length, height, and width.
    * **Weight Distribution:** Empty weight and useful weight (the sum of fuel, crew, and payload).
    * **Performance Metrics:** Never-exceed speed and long-range speed.
    * **Power Requirements:** Take-off and continuous power, and transmission specifications.

## How to Use It 🚀

1.  **Open the Code:** Open the `statistical_design.py` file in a text editor or an IDE (like Visual Studio Code).

2.  **Modify Mission Requirements:** Locate the `main()` function near the bottom of the script. In this section, you can define your helicopter's mission by changing the values for payload, range, and other parameters.

    ```python
    def main():
        # Define mission requirements
       W_pl_target = 0      # Target payload (kg)
       crew = 15            # Number of crew members
       Rg_target = 439      # Target range (km)
       rho_f = 0.8          # Fuel density (kg/L)
       V_max = 200          # Maximum speed (km/h)
       Nb = 2               # Number of main rotor blades
       Nb_tr = 2            # Number of tail rotor blades
    ```

3.  **Run the Program:** Save the file and execute it from your terminal or command prompt using the following command:

    ```bash
    python statistical_design.py
    ```

## Output 📊

The program will print the results directly to the console in a structured format. You will first see the iterative process of the gross weight calculation, followed by a detailed summary of all the final design parameters, organized into logical sections.

---

## Code Description

The provided Python script is an object-oriented program designed to perform preliminary calculations for helicopter design. The core functionality is encapsulated within the `HelicopterDesigner` class, which acts as a blueprint for a helicopter model. The script also includes a `main` function to demonstrate how to use this class.

---

### `HelicopterDesigner` Class

This class is the central component of the program. It manages all the data and calculations related to a single helicopter design.

* `__init__(self, V_max, Nb, Nb_tr, W_pl, crew, Rg, rho_f)`: This is the **constructor** of the class. It initializes a new `HelicopterDesigner` object with all the fundamental design inputs. These inputs, such as  **maximum speed (V_max)**, **payload weight (W_pl)**, and **crew** the number of crew/people,  are stored as attributes of the object using `self.`. It also initializes an empty dictionary `self.results` to store all the calculated parameters.

* `solve_gross_weight_with_solver(self)`: This method performs the iterative calculation to find the optimal gross weight. It takes an initial guess for the gross weight (`W0_guess`) and refines it in a loop until the value converges (the change between iterations is smaller than the `tolerance`). The core of this loop is a formula that relates the **useful weight (Wu)**—which includes payload, crew, and fuel—to the gross weight. Once the final weight is found, it updates the object's `self.W0` attribute.

* `calculate_main_rotor_parameters(self)`, `calculate_tail_rotor_parameters(self)`, `calculate_airframe_dimensions(self)`, `calculate_weights(self)`, `calculate_performance(self)`, `calculate_power(self)`: These methods are responsible for a specific set of calculations. Each method uses the attributes stored in `self.` (like `self.W0` and `self.V_max`) to compute various design parameters. The results of these calculations are then stored in the `self.results` dictionary, which is organized into logical sub-dictionaries (e.g., `'Main Rotor'`, `'Weights'`).

* `run_calculations(self)`: This is a master method that acts as a workflow manager. It calls all the individual `calculate_...` methods in the correct sequence. By calling this single method, a user can trigger all the necessary computations to complete the helicopter design.

---

### `main()` Function

The `main()` function serves as the entry point for the program and provides a clear example of how to use the `HelicopterDesigner` class.

* **Define Mission Requirements**: It starts by setting all the input values for the helicopter's design mission.
* **Instantiate the Class**: It creates an instance of the `HelicopterDesigner` class, passing the initial mission requirements to its `__init__` constructor.
* **Run Iteration and Calculations**: It first calls `designer.iterative_gross_weight_calculator()` to find the optimal gross weight. Then, it calls `designer.run_calculations()` to perform all the other design computations based on that final weight.
* **Display Results**: It iterates through the `designer.results` dictionary and prints all the calculated parameters in a formatted, easy-to-read manner.

---

### Key Concepts

* **Object-Oriented Programming (OOP)**: The code uses a class (`HelicopterDesigner`) to group related data and functions. This approach improves code organization and reusability.
* **Modularity**: The separation of calculations into distinct methods (e.g., `calculate_main_rotor_parameters`) makes the code easier to read, debug, and maintain. 
* **Encapsulation**: The class encapsulates all the data and behavior related to a helicopter design, so a user only needs to interact with the class methods without needing to understand the underlying formulas.
* **Iterative Process**: The gross weight calculation is an example of an iterative algorithm, which is a powerful technique for solving problems that do not have a direct, single-step solution.

