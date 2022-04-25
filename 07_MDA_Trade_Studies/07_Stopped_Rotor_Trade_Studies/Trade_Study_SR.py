# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units 
import numpy as np   
import matplotlib.pyplot        as plt  
import pickle  
import os
from SUAVE.Core                                                  import Data 
from SUAVE.Methods.Weights.Correlations.Propulsion               import nasa_motor
from SUAVE.Methods.Propulsion.electric_motor_sizing              import size_optimal_motor
from SUAVE.Plots.Performance.Mission_Plots                       import *  
from SUAVE.Plots.Geometry                                        import *   
from copy import deepcopy
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars \
     import compute_airfoil_polars
import time 

import sys 
sys.path.append('../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Stopped_Rotor  import  missions_setup , base_analysis , vehicle_setup , analyses_setup , \
     configs_setup , missions_setup ,full_mission_setup,approach_departure_mission_setup,\
     hover_mission_setup,save_results

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():

    simulated_days   = 1
    flights_per_day  = 1 
    aircraft_range   = 70 *Units.nmi
    reserve_segment  = False  
    recharge_battery = False 
    control_points   = 10
    N_gm_x           = 10
    N_gm_y           = 5 

    alpha_weights    = np.array([0.5,0.0]) # ,1.0 ,0.74,0.5,0.25,0.02, completed 1.0,0.5,0.0
    
    for a_i in range(len(alpha_weights)):
        alpha             = alpha_weights[a_i]  
        alpha_opt_weight  = str(alpha)
        alpha_opt_weight  = alpha_opt_weight.replace('.','_')         
        # modify vehicle 
        base_vehicle      = vehicle_setup()
        vehicle           = modify_vehicle(base_vehicle,alpha)
        configs           = configs_setup(vehicle)   
        
        # ------------------------------------------------------------------------------------------------
        # Full Mission 
        # ------------------------------------------------------------------------------------------------  
        run_noise_model   = False
        hover_noise_test  = False     
         
        min_x = 0
        max_x = 1
        min_y = 0
        max_y = 1
         
        configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
        noise_mission     = full_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
        missions_analyses = missions_setup(noise_mission) 
        analyses          = SUAVE.Analyses.Analysis.Container()
        analyses.configs  = configs_analyses
        analyses.missions = missions_analyses 
        configs.finalize()
        analyses.finalize()      
        noise_mission     = analyses.missions.base
        noise_results     = noise_mission.evaluate()   
        filename          = 'Stopped_Rotor_Full_Mission' + '_alpha' + alpha_opt_weight  
        save_results(noise_results,filename)   
        
        # ------------------------------------------------------------------------------------------------
        # Approach and Departure Noise Mission 
        # ------------------------------------------------------------------------------------------------    
        run_noise_model   = True
        hover_noise_test  = False                
        Y_LIM = np.linspace(1E-6,0.5*Units.nmi,3)     
        X_LIM = np.linspace(0, 5.79*Units.nmi,3)    
        
        Q_idx             = 1 
        for i in range(len(X_LIM)-1):
            for j in range(len(Y_LIM)-1): 
                print('Running Quardant:' + str(Q_idx))
                ti_quad  = time.time()
                min_x = X_LIM[i]
                max_x = X_LIM[i+1]
                min_y = Y_LIM[j]
                max_y = Y_LIM[j+1]
                
                configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
                noise_mission     = approach_departure_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
                missions_analyses = missions_setup(noise_mission) 
                analyses          = SUAVE.Analyses.Analysis.Container()
                analyses.configs  = configs_analyses
                analyses.missions = missions_analyses 
                configs.finalize()
                analyses.finalize()      
                noise_mission     = analyses.missions.base
                noise_results     = noise_mission.evaluate()   
                filename          = 'Stopped_Rotor_Approach_Departure_Noise_Q' + str(Q_idx) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight   
                save_results(noise_results,filename)  
                Q_idx += 1   
     
                tf_quad = time.time() 
                print ('time taken: '+ str(round(((tf_quad-ti_quad)/60),3)) + ' mins')                        
     
    return 

# ------------------------------------------------------------------
#   Modify Vehicle
# ------------------------------------------------------------------     
def modify_vehicle(vehicle,alpha):
    
    net = vehicle.networks.lift_cruise
    bat = net.battery
    
    # delete prop-rotors 
    for rotor in net.lift_rotors:
        del net.lift_rotors[rotor.tag]
    
    # get name of optimized prop-rotor 

    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator  
    
    optimizer             = 'SLSQP'
    design_thrust         = 19E3/12 # (2700*9.81/(12))      
    alpha_opt_weight      = str(alpha)
    alpha_opt_weight      = alpha_opt_weight.replace('.','_')     
    file_name             = rel_path +  '../07_Rotor_Blade_Optimization/Lift_Rotor_Design/Rotor_Designs_2' + separator +  'Rotor_T_' + str(int(design_thrust))  + '_Alpha_' + alpha_opt_weight + '_Opt_' + optimizer
    
    
    rotor                 = load_blade_geometry(file_name)  
    rotor.airfoil_geometry =  [rel_path + '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars   = [[rel_path + '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                               rel_path + '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                               rel_path + '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                               rel_path + '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                               rel_path + '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]] 
    
    number_of_airfoil_section_points = 100
    airfoil_data                     = compute_airfoil_polars(rotor.airfoil_geometry, rotor.airfoil_polars,npoints = number_of_airfoil_section_points)   
    rotor.airfoil_polar_stations     = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    rotor.airfoil_cl_surrogates      = airfoil_data.lift_coefficient_surrogates 
    rotor.airfoil_cd_surrogates      = airfoil_data.drag_coefficient_surrogates   
    # Rotors Locations  
    origins   = [[ 0.226, 1.413, 1.1] ,[  0.226, -1.413 , 1.1],
                 [ 4.630 , 1.413 , 1.1] ,[ 4.630 , -1.413 , 1.1],
                 [0.409 , 4.022 , 1.2] ,[ 0.409 , -4.022 , 1.2],
                 [ 4.413 , 4.022 , 1.2] ,[ 4.413 , -4.022 , 1.2],
                 [ 0.409 , 6.630 , 1.3] ,[ 0.409 , -6.630 , 1.3],
                 [ 4.413 , 6.630 , 1.3] ,[ 4.413 , -6.630 , 1.3]]  
    
    # append prop rotors 
    for ii in range(12):
        lift_rotor          = deepcopy(rotor)
        lift_rotor.tag      = 'rot_' + str(ii+1)
        lift_rotor.origin   = [origins[ii]]
        net.lift_rotors.append(lift_rotor)    
         
    return vehicle

# ------------------------------------------------------------------
#   Load Blade Geometry
# ------------------------------------------------------------------   
def load_blade_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        rotor = pickle.load(file) 
    return rotor

if __name__ == '__main__': 
    main()    
    plt.show()

