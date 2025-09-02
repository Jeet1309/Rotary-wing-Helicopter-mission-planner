import pandas as pd


file = "Paper_results.xlsx"

# Read each sheet separately
df2 = pd.read_excel(file, sheet_name="2blade")
df3 = pd.read_excel(file, sheet_name="3blade")
df4 = pd.read_excel(file, sheet_name="4blade")
df5 = pd.read_excel(file, sheet_name="5blade")

# Convert to numpy arrays
theta_2 = df2["Theta_not (deg)"].to_numpy()
Ct_2    = df2["C_t_paper"].to_numpy()
Cq_2    = df2["C_q_paper"].to_numpy()

theta_3 = df3["Theta_not (deg)"].to_numpy()
Ct_3    = df3["C_t_paper"].to_numpy()
Cq_3    = df3["C_q_paper"].to_numpy()

theta_4 = df4["Theta_not (deg)"].to_numpy()
Ct_4    = df4["C_t_paper"].to_numpy()
Cq_4    = df4["C_q_paper"].to_numpy()

theta_5 = df5["Theta_not (deg)"].to_numpy()
Ct_5    = df5["C_t_paper"].to_numpy()
Cq_5    = df5["C_q_paper"].to_numpy()
