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
sys.path.append('../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Tiltwing  import  missions_setup , base_analysis , vehicle_setup , analyses_setup , \
      missions_setup ,full_mission_setup,approach_departure_mission_setup,\
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

    alpha_weights        = np.array([1.0,1.0,1.0,0.5])  
    beta_weights         = np.array([0.1,0.9,0.3,0.3])  
    
    for a_i in range(len(alpha_weights)):
        alpha             = alpha_weights[a_i]  
        alpha_opt_weight  = str(alpha)
        alpha_opt_weight  = alpha_opt_weight.replace('.','_')
        beta              = beta_weights[a_i]  
        beta_opt_weight   = str(beta)
        beta_opt_weight   = beta_opt_weight.replace('.','_')
        
        
        # modify vehicle 
        vehicle           = vehicle_setup()
        vehicle           = modify_vehicle(vehicle,alpha,beta )
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
        filename          = 'Tiltwing_Full_Mission' + '_alpha' + alpha_opt_weight   + '_beta' + beta_opt_weight
        save_results(noise_results,filename)   
        
        # ------------------------------------------------------------------------------------------------
        # Approach and Departure Noise Mission 
        # ------------------------------------------------------------------------------------------------    
        run_noise_model   = False # True
        hover_noise_test  = False                
        Y_LIM = np.linspace(1E-6,0.5*Units.nmi,3)     
        X_LIM = np.linspace(0, 4.38*Units.nmi,3)    
        
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
                filename          = 'Tiltwing_Approach_Departure_Noise_Q' + str(Q_idx) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight   + '_beta' + beta_opt_weight 
                save_results(noise_results,filename)  
                Q_idx += 1  
     
    return 

# ------------------------------------------------------------------
#   Modify Vehicle
# ------------------------------------------------------------------     
def modify_vehicle(vehicle,alpha,beta ):
    
    net = vehicle.networks.battery_propeller
    
    # delete prop-rotors 
    for prop_rotor in net.propellers:
        del net.propellers[prop_rotor.tag]
    
    # get name of optimized prop-rotor 

    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator  
    
    optimizer            = 'SLSQP'
    design_thrust_hover  = (2300*9.81/(8))    
    design_thrust_cruise = 1410/8   
    
    alpha_opt_weight = str(alpha)
    alpha_opt_weight = alpha_opt_weight.replace('.','_')    
    beta_opt_weight  = str(beta)
    beta_opt_weight  = beta_opt_weight.replace('.','_')    
    file_name        =  rel_path +  '../07_Rotor_Blade_Optimization/Prop_Rotor_Design/Rotor_Designs' + separator + 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                 '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer   
    prop_rotor       = load_blade_geometry(file_name)


    # Rotors Locations  
    origins   = [[-0.3, 2.0, 0.0], [-0.3, 4.8, 0.0],[-0.3, -2.0, 0.0], [-0.3, -4.8, 0.0],\
                 [4.7, 2.0 ,1.4], [4.7, 4.8, 1.4],[4.7, -2.0, 1.4], [4.7, -4.8, 1.4]]      

    for ii in range(8):
        tw_prop_rotor          = deepcopy(prop_rotor)
        tw_prop_rotor.tag      = 'prop_' + str(ii+1)
        tw_prop_rotor.origin   = [origins[ii]]
        net.propellers.append(tw_prop_rotor)   
         
    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------
def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one propeller operates at varying flight conditions, one must perscribe  the 
    pitch command of the propeller which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                     = SUAVE.Components.Configs.Config.Container() 
    base_config                                 = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                             = 'base'
    configs.append(base_config)
 
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_hover
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    configs.append(config) 

    # ------------------------------------------------------------------
    #  Vertical Transition 1 
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 35.0  * Units.degrees
    config.tag                                        = 'vertical_transition_1'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_hover
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    


    # ------------------------------------------------------------------
    # Vertical Transition 2    
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 25.0  * Units.degrees
    config.tag                                        = 'vertical_transition_2'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_cruise*0.6
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    
    

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'climb_transition'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_cruise*0.8
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

     
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_cruise
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)
   

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------      
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_cruise
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)
  

    # ------------------------------------------------------------------
    #   Approach Configuration
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'approach'  
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_cruise
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

    
    
    # ------------------------------------------------------------------
    #  Descent Transition
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 45.0  * Units.degrees
    config.tag                                        = 'descent_transition'  
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = (prop.inputs.pitch_command_cruise +  prop.inputs.pitch_command_hover)/2
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

    

    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = prop.inputs.pitch_command_hover
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config) 

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

