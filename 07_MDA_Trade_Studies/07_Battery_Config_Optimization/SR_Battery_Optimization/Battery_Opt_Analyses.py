# Analyses.py
# 
# ----------------------------------------------------------------------        
# Imports
# ----------------------------------------------------------------------  
import SUAVE
from SUAVE.Core import Units 
import sys 
sys.path.append('../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Stopped_Rotor  import  base_analysis 

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ---------------------------------------------------------------------- 
def analyses_setup(configs): 
 
    aircraft_range   = 70 *Units.nmi 
    N_gm_x           = 10
    N_gm_y           = 5 
    run_noise_model   = False
    hover_noise_test  = False     
     
    min_x = 0
    max_x = 1
    min_y = 0
    max_y = 1
    
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,
                                 aircraft_range,run_noise_model,hover_noise_test)
        analyses[tag] = analysis

    return analyses

  