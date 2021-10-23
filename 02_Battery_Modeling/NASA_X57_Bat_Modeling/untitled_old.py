import numpy as np
import matplotlib.pylab as plt
from scipy.interpolate import interp1d, interp2d
from scipy.integrate import odeint 
import time

def main():
    
    
    return 


if __name__ == '__main__': 
    main()    
    plt.show()
    
st = time.time() 
fig, ax = plt.subplots()

with open('MPT battery Data/X57_MPT_1C_lot1_030_0C.csv', 'r') as data_file:

    header = data_file.readline().strip().split(",")

    time_col = header.index("Test_Time(s)")
    current_col = header.index("Current(A)")
    voltage_col = header.index("Voltage(V)")


    data = []
    for line in data_file:
        row = line.strip().split(",")
        data.append([float(x) for x in (row[time_col], row[current_col], row[voltage_col])])

    start_idx = 70
    end_idx = 14000

    test_data = np.array(data)[start_idx:end_idx]
    test_data[:,0] -= test_data[0,0]
    test_data[:,1] *= -1


current_interp = interp1d(test_data[:,0], test_data[:,1], bounds_error=False, kind='linear')


T_bp = [20., 40.] # % Deg C
SOC_bp = np.linspace(0,1,30) # %[0., 0.1, 0.2, 0.25, 0.5, 0.75, 0.9, 0.953, 1.] #%  # SOC break points



tU_oc = [[3.044, 3.123,  3.205, 3.261,  3.338, 3.403, 3.459, 3.505, 3.529, 3.550, 3.601, 3.628, 3.663, 3.693, 3.733, 3.761, 3.798, 3.825, 3.845, 3.864, 3.893, 3.920, 3.967, 4.005, 4.044, 4.070, 4.076, 4.086, 4.106, 4.161],
         [3.044, 3.148,  3.215, 3.271,  3.338, 3.403, 3.459, 3.505, 3.529, 3.555, 3.601, 3.648, 3.683, 3.713, 3.743, 3.771, 3.798, 3.825, 3.853, 3.874, 3.913, 3.950, 3.987, 4.025, 4.054, 4.070, 4.076, 4.086, 4.106, 4.161]]  # volts, Open Circuit Voltage

tC_Th = [[2000., 2000.,  2000., 2000.,  2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000.],
         [2000., 2000.,  2500., 2200.,  1200., 20000., 1400., 120., 1000., 2000., 2000., 2500., 2200., 1200., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2200., 2000.]]  # farads

tR_Th= [[0.070,  0.070,  0.060, 0.060,  0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.090, 0.080, 0.080, 0.080, 0.080, 0.080, 0.070, 0.060, 0.050, 0.040, 0.060, 0.100, 0.070, 0.075, 0.10,  0.075, 0.055, 0.055, 0.040, 0.040],
        [0.011,  0.007,  0.006, 0.005,  0.004, 0.004, 0.004, 0.039, 0.038, 0.070, 0.006, 0.005, 0.004, 0.004, 0.004, 0.011, 0.007, 0.006, 0.005, 0.004, 0.004, 0.004, 0.039, 0.038, 0.070, 0.060, 0.005, 0.040, 0.040, 0.040]] # ohm  Ohmic Losses

tR_0 = [[0.18,    0.14,   0.12,  0.09,   0.08, 0.075,  0.07,  0.07,  0.07,  0.07,  0.07,  0.07, 0.067, 0.067, 0.067, 0.067, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065, 0.065],
        [0.15,    0.08,   0.08,  0.08,   0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.15,  0.08,  0.08,  0.07,  0.05,  0.05,  0.05,  0.05,  0.06,  0.06,  0.06,  0.06,  0.08,  0.06, 0.06]]  # ohm
                             #                                                                                                        

X,Y = np.meshgrid(SOC_bp, T_bp);

U_oc_interp = interp2d(X, Y, tU_oc, kind='linear') # % need Deg C
C_Th_interp = interp2d(X, Y, tC_Th, kind='linear')
R_Th_interp = interp2d(X, Y, tR_Th, kind='linear')
R_0_interp = interp2d(X, Y, tR_0, kind='linear')


def ode_func(y, t):

    # print(y, t)
    SOC = y[0]
    U_Th = y[1]

    T_batt = 20;
    Q_max = 2.85;
    mass_cell = 0.045;
    Cp_cell = 1020;
    eff_cell = 0.95;
    Pack_Loss = 1.0;

    I_Li = current_interp(t)


    U_oc = U_oc_interp(SOC, T_batt)
    C_Th = C_Th_interp(SOC, T_batt)
    R_Th = R_Th_interp(SOC, T_batt)
    R_0 = R_0_interp(SOC, T_batt)

    dXdt_SOC = -I_Li / (3600.0 * Q_max);
    dXdt_U_Th = -U_Th / (R_Th * C_Th) + I_Li / (C_Th);
    # U_L = U_oc - U_Th - (I_Li * R_0);\

    return [dXdt_SOC, dXdt_U_Th]


def compute_other_vars(y, t):
    SOC = y[:,0]
    U_Th = y[:,1]
    T_batt = 20 

    I_Li = current_interp(t)

    U_oc_vals = []
    R_0_vals = []
    for i in range(len(SOC)):
        U_oc = U_oc_interp(SOC[i], T_batt)
        # C_Th = C_Th_interp(SOC[i], T_batt)
        # R_Th = R_Th_interp(SOC[i], T_batt)
        R_0 = R_0_interp(SOC[i], T_batt)
        U_oc_vals.append(U_oc[0])
        R_0_vals.append(R_0[0])

    U_oc = np.array(U_oc_vals)
    R_0 = np.array(R_0_vals)

    U_L = U_oc - U_Th - (I_Li * R_0)

    return U_L

sim_states = odeint(ode_func, y0=[1, 0], t=test_data[:,0], hmax=4) # , atol=1e-12)
sim_data = compute_other_vars(sim_states, test_data[:,0])

print('sim time', time.time() - st)
print(sim_states[:,0])

fig, ax = plt.subplots()

ax.plot(test_data[:,0], test_data[:,2])
#ax.plot(test_data[:,0], sim_data)


with open('MPT battery Data/X57_MPT_1C_lot1_027_20C.csv', 'r') as data_file:

    header = data_file.readline().strip().split(",")

    time_col = header.index("Test_Time(s)")
    current_col = header.index("Current(A)")
    voltage_col = header.index("Voltage(V)")


    data = []
    for line in data_file:
        row = line.strip().split(",")
        data.append([float(x) for x in (row[time_col], row[current_col], row[voltage_col])])

    start_idx = 70
    end_idx = 14000

    test_data = np.array(data)[start_idx:end_idx]
    test_data[:,0] -= test_data[0,0]
    test_data[:,1] *= -1


current_interp = interp1d(test_data[:,0], test_data[:,1], bounds_error=False, kind='linear')


T_bp = [20., 40.] # % Deg C
SOC_bp = np.linspace(0,1,30) # %[0., 0.1, 0.2, 0.25, 0.5, 0.75, 0.9, 0.953, 1.] #%  # SOC break points



tU_oc = [[3.044, 3.148, 3.215, 3.271, 3.338, 3.403, 3.459, 3.505, 3.529, 3.550, 3.601, 3.648, 3.683, 3.713, 3.743, 3.771, 3.798, 3.825, 3.853, 3.874, 3.913, 3.950, 3.987, 4.025, 4.054, 4.070, 4.076, 4.086, 4.106, 4.161],
         [3.044, 3.148,  3.215, 3.271,  3.338, 3.403, 3.459, 3.505, 3.529, 3.555, 3.601, 3.648, 3.683, 3.713, 3.743, 3.771, 3.798, 3.825, 3.853, 3.874, 3.913, 3.950, 3.987, 4.025, 4.054, 4.070, 4.076, 4.086, 4.106, 4.161]]  # volts, Open Circuit Voltage

tC_Th = [[2000., 2000.,  2000., 2000.,  2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000.],
         [2000., 2000.,  2500., 2200.,  1200., 20000., 1400., 120., 1000., 2000., 2000., 2500., 2200., 1200., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2000., 2200., 2000.]]  # farads

tR_Th= [[0.070,  0.070,  0.060, 0.060,  0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.060, 0.050, 0.040, 0.060, 0.060, 0.060, 0.055, 0.050, 0.025, 0.025, 0.020, 0.020, 0.020],
        [0.011,  0.007,  0.006, 0.005,  0.004, 0.004, 0.004, 0.039, 0.038, 0.070, 0.006, 0.005, 0.004, 0.004, 0.004, 0.011, 0.007, 0.006, 0.005, 0.004, 0.004, 0.004, 0.039, 0.038, 0.070, 0.060, 0.005, 0.040, 0.040, 0.040]] # ohm  Ohmic Losses

tR_0 = [[0.08,    0.08,   0.06,  0.04,  0.02,   0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.02,  0.03,  0.03,  0.03,  0.03,  0.03, 0.04,  0.03,  0.03, 0.03],
        [0.15,    0.08,   0.08,  0.08,   0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.08,  0.15,  0.08,  0.08,  0.07,  0.05,  0.05,  0.05,  0.05,  0.06,  0.06,  0.06,  0.06,  0.08,  0.06, 0.06]]  # ohm
                             #                                                                                                        

X,Y = np.meshgrid(SOC_bp, T_bp);

U_oc_interp = interp2d(X, Y, tU_oc, kind='linear') # % need Deg C
C_Th_interp = interp2d(X, Y, tC_Th, kind='linear')
R_Th_interp = interp2d(X, Y, tR_Th, kind='linear')
R_0_interp = interp2d(X, Y, tR_0, kind='linear')


def ode_func(y, t):

    # print(y, t)
    SOC = y[0]
    U_Th = y[1]

    T_batt = 20;
    Q_max = 2.85;
    mass_cell = 0.045;
    Cp_cell = 1020;
    eff_cell = 0.95;
    Pack_Loss = 1.0;

    I_Li = current_interp(t)


    U_oc = U_oc_interp(SOC, T_batt)
    C_Th = C_Th_interp(SOC, T_batt)
    R_Th = R_Th_interp(SOC, T_batt)
    R_0 = R_0_interp(SOC, T_batt)

    dXdt_SOC = -I_Li / (3600.0 * Q_max);
    dXdt_U_Th = -U_Th / (R_Th * C_Th) + I_Li / (C_Th);
    # U_L = U_oc - U_Th - (I_Li * R_0);\

    return [dXdt_SOC, dXdt_U_Th]


def compute_other_vars(y, t):
    SOC = y[:,0]
    U_Th = y[:,1]
    T_batt = 20 

    I_Li = current_interp(t)

    U_oc_vals = []
    R_0_vals = []
    for i in range(len(SOC)):
        U_oc = U_oc_interp(SOC[i], T_batt)
        # C_Th = C_Th_interp(SOC[i], T_batt)
        # R_Th = R_Th_interp(SOC[i], T_batt)
        R_0 = R_0_interp(SOC[i], T_batt)
        U_oc_vals.append(U_oc[0])
        R_0_vals.append(R_0[0])

    U_oc = np.array(U_oc_vals)
    R_0 = np.array(R_0_vals)

    U_L = U_oc - U_Th - (I_Li * R_0)

    return U_L

sim_states = odeint(ode_func, y0=[1, 0], t=test_data[:,0], hmax=4) # , atol=1e-12)
sim_data = compute_other_vars(sim_states, test_data[:,0])

print('sim time', time.time() - st)
print(sim_states[:,0])


ax.plot(test_data[:,0], test_data[:,2])
#ax.plot(test_data[:,0], sim_data)

plt.show()