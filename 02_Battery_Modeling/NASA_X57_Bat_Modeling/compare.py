import numpy as np
import matplotlib.pylab as plt
from scipy.interpolate import interp1d, interp2d
from scipy.integrate import odeint

import time
st = time.time()

def main():
            
    # allows any number of data cases to be read and overlaid
    # also allows sim of model and overlay
    # need to define start/stop for each case
    
    # X57_MPT_1C_lot1_027_20C.csv
    # X57_MPT_1C_lot1_028_20C.csv
    # X57_MPT_1C_lot1_029_20C.csv
    # X57_MPT_1o2C_lot1_027_20C.csv
    
    # X57_MPT_1C_lot1_030_0C.csv
    tests = ['X57_MPT_1o2C_lot1_030_0C.csv','X57_MPT_1o2C_lot1_031_0C.csv','X57_MPT_1o2C_lot1_032_0C.csv']#, 'X57_MPT_1C_lot1_028_20C.csv', 'X57_MPT_1C_lot1_029_20C.csv']
    # compare 3 cells across all temp ranges
    #tests = ['X57_MPT_1C_lot1_043_45C.csv','X57_MPT_1C_lot1_044_45C.csv','X57_MPT_1C_lot1_045_45C.csv'] # start_list = [72,70,71] end_list = [15000, 15000, 15000]
    #tests = ['X57_MPT_1C_lot1_040_30C.csv','X57_MPT_1C_lot1_041_30C.csv','X57_MPT_1C_lot1_042_30C.csv'] # start_list = [71,71,70] end_list = [15000, 15000, 15000]
    #tests = ['X57_MPT_1C_lot1_027_20C.csv','X57_MPT_1C_lot1_028_20C.csv','X57_MPT_1C_lot1_029_20C.csv']  # start_list = [71,71,70] end_list = [14000, 14000, 14000]
    tests = ['X57_MPT_1C_lot1_030_0C.csv','X57_MPT_1C_lot1_031_0C.csv','X57_MPT_1C_lot1_032_0C.csv']  # start_list = [70,70,71] end_list = [14000, 14000, 14000]
    # everything
    tests = ['X57_MPT_1C_lot1_030_0C.csv','X57_MPT_1C_lot1_031_0C.csv','X57_MPT_1C_lot1_032_0C.csv', 'X57_MPT_1C_lot1_027_20C.csv','X57_MPT_1C_lot1_028_20C.csv','X57_MPT_1C_lot1_029_20C.csv', 'X57_MPT_1C_lot1_040_30C.csv','X57_MPT_1C_lot1_041_30C.csv','X57_MPT_1C_lot1_042_30C.csv', 'X57_MPT_1C_lot1_043_45C.csv','X57_MPT_1C_lot1_044_45C.csv','X57_MPT_1C_lot1_045_45C.csv']
    #start_list = [70,70,71,  71,71,70,  71,71,70,  72,70,71]
    #end_list = [14000, 14000, 14000, 14000, 14000, 14000, 15000, 15000, 15000, 15000, 15000, 15000]
    
    # compare 5 temps
    tests = ['X57_MPT_1C_lot1_030_0C.csv', 'X57_MPT_1C_lot1_027_20C.csv', 'X57_MPT_1C_lot1_040_30C.csv', 'X57_MPT_1C_lot1_043_45C.csv', 'X57_MPT_1C_lot1_048_60C.csv']
    # 60 C compare
    #tests = ['X57_MPT_1C_lot1_048_60C.csv','X57_MPT_1C_lot1_028_60C.csv'] # start_list = [76,62] end_list = [15100, 15000]
    # different discharge
    
    
    fig, ax = plt.subplots()
    for idx,test in enumerate(tests):
        with open('Data Archive/%s'%test, 'r') as data_file:
    
            header = data_file.readline().strip().split(",")
    
            time_col = header.index("Test_Time(s)")
            current_col = header.index("Current(A)")
            voltage_col = header.index("Voltage(V)")
            temp_col = header.index("Temperature (C)_1")
    
            data = []
            for line in data_file:
                row = line.strip().split(",")
                data.append([float(x) for x in (row[time_col], row[current_col], row[voltage_col], row[temp_col])])
    
            #start_list = [76,62]
            #                0C        20C         30C       45C
            start_list = [70,  71,  71,  72, 72]
            #end_list = [15100, 15000]
            end_list = [14000,  14000,  15000,  15000, 15100]
            start_idx = start_list[idx]
            end_idx = end_list[idx]
    
            # prune data
            test_data = np.array(data)[start_idx:end_idx]
            test_data[:,0] -= test_data[0,0] # baseline data to 0
            test_data[:,1] *= -1 # flip sign on current
    
    
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


    
    sim_states = odeint(ode_func, y0=[1, 0], t=test_data[:,0], hmax=4) # , atol=1e-12)
    sim_data = compute_other_vars(sim_states, test_data[:,0])

    print('sim time', time.time() - st)
    print(sim_states[:,0])


    ax.plot(test_data[:,0], test_data[:,2])   # voltage
    #ax.plot(test_data[:,0], test_data[:,3])  # temperature
    #ax.plot(test_data[:,0], sim_data)        # sim voltage
    #ax.plot(test_data[:,0], sim_states[:,0])  # sim SOC
    ax.legend(['0$^\circ$C','20$^\circ$C','30$^\circ$C','45$^\circ$C','60$^\circ$C'])
    ax.set_ylabel("V")
    ax.set_xlabel("time (0.1 seconds)") 
    
    return 

def ode_func(y, t):

        # print(y, t)
        SOC = y[0]
        U_Th = y[1]

        T_batt = 20;
        Q_max = 3.;#2.85;
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
        C_Th = C_Th_interp(SOC[i], T_batt)
        R_Th = R_Th_interp(SOC[i], T_batt)
        R_0 = R_0_interp(SOC[i], T_batt)
        U_oc_vals.append(U_oc[0])
        R_0_vals.append(R_0[0])

    U_oc = np.array(U_oc_vals)
    R_0 = np.array(R_0_vals)

    U_L = U_oc - U_Th - (I_Li * R_0)

    return U_L

if __name__ == '__main__': 
    main()    
    plt.show()


    
