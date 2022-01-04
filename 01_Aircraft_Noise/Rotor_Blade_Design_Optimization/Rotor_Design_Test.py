import SUAVE 
from SUAVE.Core import Units
import numpy as np 
from Rotor_Blade_Optimization import  low_noise_rotor_design 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    # Design a Rotor with airfoil  geometry defined  
    rot                                  = SUAVE.Components.Energy.Converters.Rotor()  
    rot.tip_radius                       = 1.2 
    rot.hub_radius                       = 0.2 * rot.tip_radius
    rot.number_of_blades                 = 3  
    rot.freestream_velocity              = 1.24  
    rot.angular_velocity                 = 141.54523785   
    rot.design_Cl                        = 0.7
    rot.design_altitude                  = 500 * Units.feet                  
    rot.design_thrust                    = 500  
    rot.number_of_airfoil_section_points = 200
    rot.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rot.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    rot.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    

    rot.taper                            = 0.3
    rot.design_figure_of_merit           = 0.75
    rot.max_sectional_Cl                 = 1.1
    rot.blade_optimization_pivots        = 5
    rot.linear_interp_flag               = True 
    rot.radial_pivots                    = np.linspace(0,1,rot.blade_optimization_pivots) 
    rot.chord_pivots                     = np.linspace(0.4,0.1,rot.blade_optimization_pivots) 
    rot.twist_pivots                     = np.linspace(20,0,rot.blade_optimization_pivots)
    rot                                  = low_noise_rotor_design(rot)  
 
    return  
 

if __name__ == '__main__': 
    main()     