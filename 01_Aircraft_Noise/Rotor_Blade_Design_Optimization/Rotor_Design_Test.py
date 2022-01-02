import SUAVE 
from SUAVE.Core import Units

from low_noise_rotor_design import low_noise_rotor_design 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    # Design a Rotor with airfoil  geometry defined  
    rot                          = SUAVE.Components.Energy.Converters.Rotor()  
    rot.tip_radius               = 1.2 
    rot.hub_radius               = 0.2 * rot.tip_radius
    rot.number_of_blades         = 3  
    rot.freestream_velocity      = 1.24  
    rot.angular_velocity         = 141.54523785   
    rot.design_Cl                = 0.7
    rot.design_altitude          = 500 * Units.feet                  
    rot.design_thrust            = 2697.75 
    rot.induced_hover_velocity   = 15.63326
    rot.VTOL_flag                = True 
    rot.airfoil_geometry          = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rot.airfoil_polars            = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                      '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                      '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                      '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                      '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    rot.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
      
    rot  = low_noise_rotor_design(rot)  
 
    return  
 

if __name__ == '__main__': 
    main()     