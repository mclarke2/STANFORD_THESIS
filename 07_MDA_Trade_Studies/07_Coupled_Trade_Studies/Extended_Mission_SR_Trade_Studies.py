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
from SUAVE.Plots.Performance.Mission_Plots                       import *  
from SUAVE.Plots.Geometry                                        import *   
from copy import deepcopy

import sys 
sys.path.append('../../../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Stopped_Rotor  import  missions_setup , base_analysis , vehicle_setup , analyses_setup , \
     configs_setup , missions_setup ,full_mission_setup,approach_departure_mission_setup,\
     hover_mission_setup,save_results

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
        
    simulated_days   = 365
    flights_per_day  = 8 
    aircraft_range   = 70 *Units.nmi
    reserve_segment  = False  
    recharge_battery = False 
    control_points   = 15
    N_gm_x           = 15
    N_gm_y           = 5  

    alpha_weights    = np.array([1.0,0.5,0.0])
    
    for a_i in range(len(alpha_weights)):
        alpha             = alpha_weights[a_i]  
        alpha_opt_weight      = str(alpha)
        alpha_opt_weight      = alpha_opt_weight.replace('.','_')         
        # modify vehicle 
        vehicle           = vehicle_setup()
        vehicle           = modify_vehicle(vehicle,alpha)
        configs           = configs_setup(vehicle)  
        configs           = modify_configs(configs,alpha)
        
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
        filename          = 'Stopped_Rotor_Full_Mission' + '_alpha' + alpha_opt_weight  + 'Days' + str(simulated_days)
        save_results(noise_results,filename)    
        
    return 

# ------------------------------------------------------------------
#   Modify Vehicle
# ------------------------------------------------------------------     
def modify_vehicle(vehicle,alpha):
    
    net = vehicle.networks.lift_cruise
    
    # delete prop-rotors 
    for rotor in net.lift_rotors:
        del net.lift_rotors[rotor.tag]
    
    # get name of optimized prop-rotor 

    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator  
    
    optimizer             = 'SLSQP'
    design_thrust         = (2700*9.81/(12-2))      
    alpha_opt_weight      = str(alpha)
    alpha_opt_weight      = alpha_opt_weight.replace('.','_')     
    file_name             = rel_path +  '../07_Rotor_Blade_Optimization/Lift_Rotor_Design/Data' + separator +  'Rotor_T_' + str(int(design_thrust))  + '_Alpha_' + alpha_opt_weight + '_Opt_' + optimizer
    rotor                  = load_blade_geometry(file_name)


    # Rotors Locations  
    origins   = [[ 0.226, 1.413, 1.1] ,[  0.226, -1.413 , 1.1],
                 [ 4.630 , 1.413 , 1.1] ,[ 4.630 , -1.413 , 1.1],
                 [0.409 , 4.022 , 1.2] ,[ 0.409 , -4.022 , 1.2],
                 [ 4.413 , 4.022 , 1.2] ,[ 4.413 , -4.022 , 1.2],
                 [ 0.409 , 6.630 , 1.3] ,[ 0.409 , -6.630 , 1.3],
                 [ 4.413 , 6.630 , 1.3] ,[ 4.413 , -6.630 , 1.3]]     
    # append prop rotors 
    for ii in range(8):
        lift_rotor          = deepcopy(rotor)
        lift_rotor.tag      = 'prop_' + str(ii+1)
        lift_rotor.origin   = [origins[ii]]
        net.lift_rotors.append(rotor)     
         
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

