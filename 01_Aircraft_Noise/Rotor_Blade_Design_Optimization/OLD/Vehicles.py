# Vehicles.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------  
import SUAVE  

# ----------------------------------------------------------------------
#   Define the Vehicle
# ---------------------------------------------------------------------- 

def rotor_blade_setup(rotor): 
    vehicle     = vehicle_setup(rotor)  
    configs     = SUAVE.Components.Configs.Config.Container() 
    base_config = SUAVE.Components.Configs.Config(vehicle) 
    config      = SUAVE.Components.Configs.Config(base_config)
    config.tag  = 'rotor_testbench'
    configs.append(config)  
    
    return configs  

def vehicle_setup(rotor):  
    #   Initialize the Vehicle 
    vehicle                             = SUAVE.Vehicle()  
    net                                 = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_lift_rotor_engines    = 1
    net.identical_lift_rotors           = True  
    net.lift_rotors.append(rotor)  
    vehicle.append_component(net)
    return vehicle 


 