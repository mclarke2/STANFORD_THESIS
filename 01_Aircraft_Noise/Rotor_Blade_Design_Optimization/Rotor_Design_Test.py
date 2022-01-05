import SUAVE 
from SUAVE.Core import Units
import numpy as np 
from Rotor_Blade_Optimization import  low_noise_rotor_design 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    
    g                                     = 9.81                                   # gravitational acceleration  
    speed_of_sound                        = 340                                    # speed of sound 
    Hover_Load                            = 24191       # hover load    
    rho                                   = 1.2 
              
    rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
    rotor.tag                             = 'rotor'     
    rotor.tip_radius                      = 1.2 
    rotor.hub_radius                      = 0.1 * rotor.tip_radius
    rotor.number_of_blades                = 4
    rotor.design_tip_mach                 = 0.7
    rotor.nacelle_angle                   = 0. * Units.degrees
    rotor.number_of_engines               = 6
    rotor.disc_area                       = np.pi*(rotor.tip_radius**2)        
    rotor.induced_hover_velocity          = np.sqrt(Hover_Load/(2*rho*rotor.disc_area*6)) 
    rotor.freestream_velocity             = 5
    rotor.angular_velocity                = rotor.design_tip_mach* speed_of_sound /rotor.tip_radius   
    rotor.design_Cl                       = 0.7
    rotor.design_altitude                 = 500 * Units.feet                             
    rotor.design_thrust                   = (Hover_Load/(6-1))      
    rotor.number_of_airfoil_section_points = 200
    rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]     
    rotor.taper                            = 0.3
    rotor.design_figure_of_merit           = 0.75
    rotor.max_sectional_Cl                 = 1.1
    rotor.solidity                         = 0.3
    rotor.optimization_slack_constaint     = 0.01 # slack constraint
    rotor.blade_optimization_pivots        = 4 # either 4 or 5 
    rotor.noise_aero_acoustic_obj_weight   = 1 # 1 means only perfomrance optimization 0.5 to weight noise equally
    rotor.linear_interp_flag               = True 
    rotor.radial_pivots                    = np.linspace(0,1,rotor.blade_optimization_pivots) 
    rotor.chord_pivots                     = np.linspace(0.4,0.1,rotor.blade_optimization_pivots) 
    rotor.twist_pivots                     = np.linspace(30,10,rotor.blade_optimization_pivots)*Units.degrees
    rotor                                  = low_noise_rotor_design(rotor,solver_name='SLSQP')  
 
    return  
 
if __name__ == '__main__': 
    main() 