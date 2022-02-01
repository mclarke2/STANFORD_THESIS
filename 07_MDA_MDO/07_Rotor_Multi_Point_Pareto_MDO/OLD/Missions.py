# Missions.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units 
import numpy as np

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def hover_setup(analyses,vehicle): 
    missions       = SUAVE.Analyses.Mission.Mission.Container()   
                  
                  
    mission        = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag    = 'mission'     
                  
    Segments       = SUAVE.Analyses.Mission.Segments 
    base_segment   = Segments.Segment()  
    segment                           = Segments.Hover.Climb(base_segment)
    segment.tag                       = 'Hover' 
    segment.altitude_start            = 0.0  * Units.ft 
    segment.altitude_end              = 40.  * Units.ft  
    segment.climb_rate                = 300. * Units['ft/min']       
    mission.append_segment(segment)  
    missions.hover  =  mission  
    return missions   

def cruise_setup(analyses,vehicle): 
    missions       = SUAVE.Analyses.Mission.Mission.Container()   
                  
                  
    mission        = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag    = 'mission'     
                  
    Segments       = SUAVE.Analyses.Mission.Segments 
    base_segment   = Segments.Segment() 
  
    segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                      = "Cruise" 
    segment.altitude                                 = 2500.0 * Units.ft
    segment.air_speed                                = 175.   * Units['mph'] 
    segment.distance                                 = 10*Units.nmi      
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
    mission.append_segment(segment) 
    missions.cruise  =  mission  
 
    return missions    
 