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
import Missions
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
    bat.pack_config.parallel             = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat)    
    net.voltage                          = bat.max_voltage  
    bat.module_config.number_of_modules  = 16 # CHANGE IN OPTIMIZER 
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
 
    res  = nexus.results.mission  

    num_ctrl_pts   = len(res.segments[0].conditions.frames.inertial.time[:,0] )
    num_segments   = len(res.segments.keys()) 
    data_dimension = num_ctrl_pts*num_segments 
    
    PD                 = Data() 
    PD.energy          = np.zeros(data_dimension) 
    PD.C_rating        = np.zeros(data_dimension)  
    PD.pack_temp       = np.zeros(data_dimension)   
    
    PD.aircraft_pos =  np.zeros((data_dimension,3)) 
    dim_segs        = len(res.segments)    
    PD.num_segments = dim_segs
    PD.num_ctrl_pts = num_ctrl_pts
     
    for i in range(num_segments):    
        energy          = res.segments[i].conditions.propulsion.battery_energy[:,0]*0.000277778 
        volts           = res.segments[i].conditions.propulsion.battery_voltage_under_load[:,0] 
        pack_temp       = res.segments[i].conditions.propulsion.battery_pack_temperature[:,0] 
        current         = res.segments[i].conditions.propulsion.battery_current[:,0]           
        battery_amp_hr  = (energy)/volts 
        C_rating        = current /battery_amp_hr      
                  
        PD.energy[i*num_ctrl_pts:(i+1)*num_ctrl_pts]          = energy   
        PD.pack_temp[i*num_ctrl_pts:(i+1)*num_ctrl_pts]       = pack_temp  
        PD.C_rating[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = C_rating  
  
    # Pack up
    summary = nexus.summary
    summary.max_C_rate           = max(PD.C_rating)
    summary.max_pack_temperature = max(PD.pack_temp)
    return nexus 