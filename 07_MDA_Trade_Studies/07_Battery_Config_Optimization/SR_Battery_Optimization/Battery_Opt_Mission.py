# Missions.py
# 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units  

import sys 
sys.path.append('../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Stopped_Rotor  import   missions_setup ,full_mission_setup,approach_departure_mission_setup,\ 

# ----------------------------------------------------------------------
# Define the Mission
# ----------------------------------------------------------------------
def setup(analyses, vehicle):

    simulated_days   = 1
    flights_per_day  = 1  
    reserve_segment  = False  
    recharge_battery = False 
    control_points   = 10 
    aircraft_range   = 70 *Units.nmi  
    hover_noise_test  = False    
    
    missions      = SUAVE.Analyses.Mission.Mission.Container()
    missions.base = approach_departure_mission_setup(analyses,vehicle,simulated_days,flights_per_day,
                                                     aircraft_range,reserve_segment,control_points,
                                                     recharge_battery,hover_noise_test)
    
    return missions
 