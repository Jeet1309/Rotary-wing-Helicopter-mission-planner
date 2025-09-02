import math
import numpy as np
import matplotlib.pyplot as plt

from performance_tool import *
from import_paper_results import *
# For loop conditions
Theta_list = [2,4,6,8,10,12]

# Fixed Parameters fr helicopter

params = {
    "blades": 2,                # No of blades          [Nos]
    "Chord_root": 0.0508,       # Root_chord            [m]
    "Chord_tip": 0.0508,        # Tip_chord             [m]
    "Blade_radius": 0.762,      # Blade radius          [m] 
    "Div": 1000,                 # Div for blade element [nos] 
    "Twist_root": 0,            # Root Twist            [deg]
    "Twist_tip": 0,             # Tip Twist             [deg]
    "V_c": 0,                   # climb velocity        [m/s]
    "rpm": 960,                 # rotor RPM             [rpm]
    "lift_slope": 5.75,         # Lift Slope            [per radian]    
    "Theta_not": 2              # Collective pitch      [deg]
}
pi = math.pi

## finding the chord varaition along the radius

b = params["blades"]
C_root = params["Chord_root"]
C_tip = params["Chord_tip"]
Radius = params["Blade_radius"]
R_cutoff = Radius*0.05
Div = params["Div"]
Theta_root = params["Twist_root"]
Theta_tip = params["Twist_tip"]
Climb_vel = params["V_c"]
rpm = params["rpm"]
l_slope = params["lift_slope"] 
Collective_pitch = params["Theta_not"]

r_arr = np.linspace(R_cutoff,Radius,Div)

print(f"Number of blades       : {b}")
print(f"Root chord             : {C_root} m")
print(f"Tip chord              : {C_tip} m")
print(f"Blade radius           : {Radius} m")
print(f"Root cutoff radius     : {R_cutoff} m")
print(f"Divisions              : {Div}")
print(f"Root twist             : {Theta_root} deg")
print(f"Tip twist              : {Theta_tip} deg")
print(f"Climb Velocity         : {Climb_vel} m/s")
print(f"Rotor speed            : {rpm} RPM")
print(f"Lift curve slope       : {l_slope} per rad")
print(f"Collective_pitch       : {Collective_pitch} deg")


## Chord variation
Chord_array = chord_variation(C_root,C_tip,Radius,R_cutoff,r_arr)

## Geometric Twist Variation
Theta_array = twist_variation(Theta_root,Theta_tip,Radius,R_cutoff,r_arr) 

rad_s = rpm_to_rad_s(rpm)

pitch_angles = [2,4,6,8,10,12]
blades = [2,3,4,5]

def paper_out(b, C_r, pi, R, V_i, rad_s, Theta_not_rad, Total_Thrust, rho, Total_Torque, C_d0):

    sigma = (b * C_r) / (pi * R)
    phi_paper = V_i/(rad_s * R)                                         # ratio of induced velocity and tangential speed
    phi_sigma_paper = phi_paper/sigma                                   # ratio of Phi and solidity
    Theta_paper = Theta_not_rad                                             # no twist only collective pitch angle 
    Theta_sigma_paper = Theta_paper/sigma                               # ratio of Theta and solidity 
    C_t_paper = (2 * Total_Thrust)/(rho * pi * (rad_s**2) * (R**4))     # Ct coefficient of thrust as per paper
    T_sigma_paper = C_t_paper/(sigma**2)                                # ratio of Ct and sigma^2
    C_q_paper = (2 * Total_Torque)/(rho * pi * (rad_s**2) * (R**5))     # Cq coefficient of torque as per paper
    C_q_dash_paper = ((2 * Total_Torque)/(rho * pi * (R**5) * (rad_s**2))) - ((C_d0*sigma)/4)
    Q_sigma_paper = C_q_paper / (sigma**3)
    Q_sigma_dash_paper = Q_sigma_paper - (C_d0/(4 * (sigma**2)))

    return C_t_paper,  C_q_paper

def Coefficient(b, Collective_pitch):
    ## net theta with Collective pitch input 
    net_Theta_deg = Theta_array + Collective_pitch
    net_Theta_rad = deg_to_rad(net_Theta_deg)

    ## Finding Induced Velocity
    Sigma = (b * Chord_array)/(pi * Radius)
    tol = 1e-12
    rad_s = rpm_to_rad_s(rpm)
    lamda_c = Climb_vel / ( rad_s * Radius)
    Induced_vel = lamda_prandtl(Climb_vel,b,r_arr,Radius,Sigma,l_slope,tol,net_Theta_rad,pi,lamda_c,rad_s)

    ## finding Cl, Cd, effective AOA, perpendicular, tangential velocity component and induced AOA
    T = 15
    Cl , Cd, alpha, U_p, U_t, Phi = aerod(rad_s, r_arr, Climb_vel, Induced_vel, net_Theta_rad, l_slope, T)

    ## Total Thrust,Power and Torque
    rho = 1.225
    Thrust, Power, Torque = TPQ(rho,U_p, U_t, Chord_array, Cl, Cd, Phi, Radius, Div, b, r_arr, rad_s)

    ## finding values from paper
    C_t, C_q = paper_out(b, C_root, pi, Radius, Induced_vel, rad_s, net_Theta_rad, Thrust, rho, Torque, C_d0=0.0113)

    return C_t, C_q, Thrust, Power

Thrust_at_theta = {pitch: [] for pitch in pitch_angles} 
Power_at_theta = {pitch:[] for pitch in pitch_angles}

for b in blades:

    C_t_list = []
    C_q_list = []
    C_p_list = []
    Thrust_list = []
    Power_List = []
    

    
    for pitch_angle in pitch_angles:
        C_t , C_q, Thrust, Power  = Coefficient(b, pitch_angle)
        C_p = rpm_to_rad_s(rpm) * C_q

        C_t_list.append(C_t)
        C_q_list.append(C_q)
        C_p_list.append(C_p)
        Thrust_list.append(Thrust)
        Power_List.append(Power)

        # store thrust for current pitch
        Thrust_at_theta[pitch_angle].append(Thrust)
        Power_at_theta[pitch_angle].append(Power)


           
    if b == 2:
        theta_paper, Ct_paper, Cq_paper, C_p_paper = theta_2, Ct_2, Cq_2, Cq_2*rad_s 
    elif b == 3:
        theta_paper, Ct_paper, Cq_paper, C_p_paper = theta_3, Ct_3, Cq_3, Cq_3*rad_s
    elif b == 4:
        theta_paper, Ct_paper, Cq_paper, C_p_paper = theta_4, Ct_4, Cq_4, Cq_4*rad_s
    elif b == 5:
        theta_paper, Ct_paper, Cq_paper, C_p_paper = theta_5, Ct_5, Cq_5, Cq_5*rad_s    

    # Plot CT vs Theta
    plt.figure(1)
    plt.plot(pitch_angles, C_t_list, marker='o', label=f'{b} Blades (BEMT)')
    plt.plot(theta_paper, Ct_paper, marker='x', linestyle='--', label=f'{b} Blades (Paper)')


    # Plot Cq vs Theta
    plt.figure(2)
    plt.plot(pitch_angles, C_q_list, marker='s', label=f'{b} Blades (BEMT)')
    plt.plot(theta_paper, Cq_paper, marker='^', linestyle='--', label=f'{b} Blades (Paper)')

    # PLot Thrust vs Power
    plt.figure(3)
    plt.plot(C_t_list, C_p_list , marker='o', label=f'{b} Blades (BEMT)')
    plt.plot(C_t_list, C_p_paper, marker='^', linestyle='--', label=f'{b} Blades (Paper)')

# Formatting
plt.figure(1)
plt.xlabel("Theta [deg]")
plt.ylabel("C_T")
plt.title("C_T vs Theta (Comparison)")
plt.legend()
plt.grid(True)
#plt.savefig(r"D:\IITB Mtech\Rotary Wing Aerodynamics - AE 667\Assignment 1\BEMT code\Plots\Ct_vs_Theta.svg", format="svg", dpi=300)

plt.figure(2)
plt.xlabel("Theta [deg]")
plt.ylabel("C_Q")
plt.title("C_Q vs Theta (Comparison)")
plt.legend()
plt.grid(True)
#plt.savefig(r"D:\IITB Mtech\Rotary Wing Aerodynamics - AE 667\Assignment 1\BEMT code\Plots\Cq_vs_Theta.svg", format="svg", dpi=300)

plt.figure(3)
plt.xlabel("C_T")
plt.ylabel("C_p")
plt.title("C_T vs C_P ")
plt.legend()
plt.grid(True)
#plt.savefig(r"D:\IITB Mtech\Rotary Wing Aerodynamics - AE 667\Assignment 1\BEMT code\Plots\Ct_vs_CP.svg", format="svg", dpi=300)

plt.figure(4)
for pitch_angle in pitch_angles:
    plt.plot(blades, Thrust_at_theta[pitch_angle], marker='o', label=f'{pitch_angle}° Pitch')

plt.xlabel("No of blades [n]")
plt.ylabel("Total Thrust [N]")
plt.title("No of blades vs Total Thrust at different pitch angles")
plt.legend()
plt.grid(True)
#plt.savefig(r"D:\IITB Mtech\Rotary Wing Aerodynamics - AE 667\Assignment 1\BEMT code\Plots\Thrust_vs_Blades.svg", format="svg", dpi=300)


plt.figure(5)
for pitch_angle in pitch_angles:
    plt.plot(blades, Power_at_theta[pitch_angle], marker='o', label=f'{pitch_angle}° Pitch')

plt.xlabel("No of blades [n]")
plt.ylabel("Total Power [W]")
plt.title("No of blades vs Total Power at different pitch angles")
plt.legend()
plt.grid(True)
#plt.savefig(r"D:\IITB Mtech\Rotary Wing Aerodynamics - AE 667\Assignment 1\BEMT code\Plots\Power_vs_Blades.svg", format="svg", dpi=300)


plt.show()


