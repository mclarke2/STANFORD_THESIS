# Procedure.py
# 
# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
import numpy as np 
from SUAVE.Analyses.Process import Process  
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_circuit_configuration 

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():

    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 

    # size the base config
    procedure = Process()
    procedure.modify_vehicle  = modify_vehicle

    # finalizes the data dependencies
    procedure.finalize        = finalize

    # performance studies
    procedure.missions        = Process()
    procedure.missions.base   = evaluate_mission

    # Post process the results
    procedure.post_process    = post_process

    return procedure

# ----------------------------------------------------------------------        
#   Evaluate Mission
# ----------------------------------------------------------------------    

def evaluate_mission(nexus): 
    # Evaluate the missions and save to results    
    mission         = nexus.missions.mission
    results         = nexus.results
    results.mission = mission.evaluate()   
    return nexus

# ----------------------------------------------------------------------        
#  Modify Vehicle
# ----------------------------------------------------------------------    

def modify_vehicle(nexus):
 

    vehicle = nexus.vehicle_configurations.base
    net     = vehicle.networks.lift_cruise
    bat     = net.battery 
  
    total_cells                          = 140*100
    bat.pack_config.series               = int(np.ceil(bat.pack_config.series))
    bat.pack_config.parallel             = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat)    
    net.voltage                          = bat.max_voltage  
    bat.module_config.number_of_modules  = int(np.ceil(bat.module_config.number_of_modules))
    bat.module_config.total              = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage            = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor  
    bat.module_config.normal_count       = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count     = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                          = bat     
 
 
    return nexus

# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):

    nexus.analyses.finalize()   

    return nexus         

# ----------------------------------------------------------------------
#   Post Process results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
 
    res     = nexus.results.mission  
    summary = nexus.summary   
    vehicle = nexus.vehicle_configurations.base  
    bat     = vehicle.networks.lift_cruise.battery 

    num_ctrl_pts   = len(res.segments[0].conditions.frames.inertial.time[:,0] )
    num_segments   = len(res.segments.keys()) 
    data_dimension = num_ctrl_pts*num_segments 
    
               
    bat_energy      = np.zeros(data_dimension) 
    C_rate          = np.zeros(data_dimension)  
    cell_temp       = np.zeros(data_dimension)  
     
    for i in range(num_segments):    
        energy          = res.segments[i].conditions.propulsion.battery_energy[:,0]*0.000277778 
        volts           = res.segments[i].conditions.propulsion.battery_voltage_under_load[:,0] 
        pack_temp       = res.segments[i].conditions.propulsion.battery_pack_temperature[:,0] 
        current         = res.segments[i].conditions.propulsion.battery_current[:,0]           
        battery_amp_hr  = (energy)/volts 
        C_rating        = current /battery_amp_hr      
                  
        bat_energy[i*num_ctrl_pts:(i+1)*num_ctrl_pts]      = energy   
        cell_temp[i*num_ctrl_pts:(i+1)*num_ctrl_pts]       = pack_temp  
        C_rate[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = C_rating  
  
    max_module_voltage = 50
    safety_factor      = 1.5 
    voltage_residual   = max_module_voltage/safety_factor - bat.module_config.voltage
    
    # Pack up 
    summary.battery_energy              = (bat_energy[0] - bat_energy[-1])/10000
    summary.max_module_temperature      = max(cell_temp)
    summary.max_module_voltage_residual = voltage_residual
    summary.max_C_rate                  = max(C_rate)
    
    # -------------------------------------------------------
    # PRINT ITERATION PERFOMRMANCE
    # -------------------------------------------------------  
    print("Used Battery Energy    : " + str(summary.battery_energy))
    print("Max Module Temperature : " + str(summary.max_module_temperature))  
    print("Module Voltage         : " + str(summary.max_module_voltage_residual))  
    print("Max C-Rate             : " + str(summary.max_C_rate))  
    
    return nexus 