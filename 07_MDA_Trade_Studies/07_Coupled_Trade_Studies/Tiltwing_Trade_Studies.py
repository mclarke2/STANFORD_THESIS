# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units 
import numpy as np   
import matplotlib.pyplot        as plt  
import pickle 
import time 
from SUAVE.Core                                                  import Data 
from SUAVE.Plots.Performance.Mission_Plots                       import *  
from SUAVE.Plots.Geometry                                        import *   
from copy import deepcopy

import sys 
sys.path.append('../../../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Tiltwing  import  missions_setup , base_analysis , vehicle_setup , analyses_setup , \
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
    control_points   = 15
    N_gm_x           = 15
    N_gm_y           = 5  

    alpha_weights    = np.linspace(0.0,1.0,6) 
    beta_weights     = np.linspace(0.0,1.0,6)     
    
    for a_i in range(len(alpha_weights)):
        for b_j in range(len(beta_weights)):
            alpha             = alpha_weights[a_i] 
            beta              = beta_weights[b_j] 
            alpha_opt_weight  = str(alpha)
            alpha_opt_weight  = alpha_opt_weight.replace('.','_')    
            beta_opt_weight   = str(beta)
            beta_opt_weight   = beta_opt_weight.replace('.','_')   
            
            # modify vehicle 
            vehicle           = vehicle_setup()
            vehicle           = modify_vehicle(vehicle,alpha,beta)
            configs           = configs_setup(vehicle)  
            configs           = modify_configs(configs,alpha,beta)
            
            # ------------------------------------------------------------------------------------------------
            # Full Mission 
            # ------------------------------------------------------------------------------------------------  
            run_noise_model   = False
            hover_noise_test  = False   
            Y_LIM             = np.linspace(1E-3,5*Units.nmi,3)    
            end_distance      = aircraft_range/((N_gm_x-2)*2)
            X_LIM             = np.linspace(-end_distance+1E-3,aircraft_range + end_distance+1E-3,3)     
            
            Q_idx             = 1 
            for i in range(len(X_LIM)-1):
                for j in range(len(Y_LIM)-1): 
                    print('Running Quardant:' + str(Q_idx))
                    min_x = X_LIM[i]
                    max_x = X_LIM[i+1]
                    min_y = Y_LIM[j]
                    max_y = Y_LIM[j+1]
                     
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
                    filename          = 'Tiltwing_Full_Mission_Noise_Q' + str(Q_idx) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight + '_beta' + beta_opt_weight
                    save_results(noise_results,filename)  
                    Q_idx += 1  
            
            # ------------------------------------------------------------------------------------------------
            # Approach and Departure Noise Mission 
            # ------------------------------------------------------------------------------------------------    
            run_noise_model   = True
            hover_noise_test  = False                
            Y_LIM             = np.linspace(1E-6,0.5*Units.nmi,3) 
            X_LIM             = np.linspace(1E-6, 4.38*Units.nmi,3)    
            
            Q_idx             = 1 
            for i in range(len(X_LIM)-1):
                for j in range(len(Y_LIM)-1): 
                    print('Running Quardant:' + str(Q_idx))
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
                    filename          = 'Tiltwing_Approach_Departure_Noise_Q' + str(Q_idx) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight  + '_beta' + beta_opt_weight
                    save_results(noise_results,filename)  
                    Q_idx += 1  

            # ------------------------------------------------------------------------------------------------
            # Hover Noise Mission 
            # ------------------------------------------------------------------------------------------------     
        
            hover_noise_test  = True 
            run_noise_model   = True                
            min_y             = -0.25*Units.nmi
            max_y             = 0.25*Units.nmi
            min_x             = -0.25*Units.nmi
            max_x             = 0.25*Units.nmi
             
            configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
            base_mission      = hover_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
            missions_analyses = missions_setup(base_mission) 
            analyses          = SUAVE.Analyses.Analysis.Container()
            analyses.configs  = configs_analyses
            analyses.missions = missions_analyses 
            configs.finalize()
            analyses.finalize()     
            mission           = analyses.missions.base
            hover_results     = mission.evaluate()   
            filename          = 'Tiltwing_Hover_Mission' + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight + '_beta' + beta_opt_weight
            save_results(hover_results,filename)   
    return 

# ------------------------------------------------------------------
#   Modify Vehicle
# ------------------------------------------------------------------     
def modify_vehicle(vehicle,alpha,beta):
    
    net = vehicle.networks.battery_propeller
    
    # delete prop-rotors 
    for prop in net.propellers:
        del net.propellers[prop.tag]
    
    # get name of optimized prop-rotor 
    optimizer             = 'SLSQP'
    design_thrust_hover   = (2300*9.81/(8-2))                       
    design_thrust_cruise  = 1410/8 
    alpha_opt_weight      = str(alpha)
    alpha_opt_weight      = alpha_opt_weight.replace('.','_')    
    beta_opt_weight       = str(beta)
    beta_opt_weight       = beta_opt_weight.replace('.','_')    

    name                  = 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\ 
                             '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer
    # load geometry
    prop_rotor = load_blade_geometry(name)


    # Rotors Locations  
    origins   = [[-0.3, 2.0, 0.0], [-0.3, 4.8, 0.0],[-0.3, -2.0, 0.0], [-0.3, -4.8, 0.0],\
               [4.7, 2.0 ,1.4], [4.7, 4.8, 1.4],[4.7, -2.0, 1.4], [4.7, -4.8, 1.4]]      
   
    # append prop rotors 
    for ii in range(8):
        prop_rotor          = deepcopy(prop)
        prop_rotor.tag      = 'prop_' + str(ii+1)
        prop_rotor.origin   = [origins[ii]]
        net.propellers.append(prop_rotor)     
         
    return vehicle


# ------------------------------------------------------------------
#   Modify Configs
# ------------------------------------------------------------------   
def modify_configs(configs): 
    
    # hover 
    for prop in configs.vertical_climb.networks.battery_propeller.propellers: 
        prop.inputs.pitch_command = prop.inputs.pitch_command_hover  
        
    # climb
    for prop in configs.climb.networks.battery_propeller.propellers: 
        prop.inputs.pitch_command = prop.inputs.pitch_command_cruise 
    
    # cruise
    for prop in configs.cruise.networks.battery_propeller.propellers: 
        prop.inputs.pitch_command = prop.inputs.pitch_command_cruise 
            
    # descent
    for prop in configs.vertical_descent.networks.battery_propeller.propellers: 
        prop.inputs.pitch_command = prop.inputs.pitch_command_hover          
    
    return configs  

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

