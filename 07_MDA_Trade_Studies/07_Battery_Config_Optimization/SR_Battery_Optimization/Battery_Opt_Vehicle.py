# Vehicles.py
#
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------  
import sys 
sys.path.append('../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from Stopped_Rotor  import   vehicle_setup ,configs_setup  

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = vehicle_setup()
    configs = configs_setup(base_vehicle)
    
    return configs
  