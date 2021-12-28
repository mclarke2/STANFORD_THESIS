# Imports
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller
from SUAVE.Methods.Propulsion                                                             import propeller_design 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars 
from scipy.interpolate import interp1d 
import numpy as np 
import matplotlib.pyplot as plt 
from SUAVE.Plots.Geometry.plot_propeller import plot_propeller

# design propeller 
def design_test_rotor():
    prop                            = SUAVE.Components.Energy.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'test_propeller'  
    prop.tip_radius                 = 1
    prop.hub_radius                 = prop.tip_radius * 0.1
    prop.number_of_blades           = 3 
    prop.thrust_angle               = 0.0   
    prop.twist_distribution         = np.ones(20)* 0 * Units.degrees    
    prop.chord_distribution         = np.ones(20)* 0.1
    prop.radius_distribution        = np.linspace(0.1,0.999,20)
    prop.max_thickness_distribution = np.ones(20)* 0.01
    prop.thickness_to_chord         = np.ones(20)* (0.01/0.1)
    prop.airfoil_geometry           = [ 'Propellers_Rotors/NACA_15.txt']
    prop.airfoil_polars             = [['Propellers_Rotors/NACA_15_polar_Re_50000.txt','Propellers_Rotors/NACA_15_polar_Re_100000.txt',
                                        'Propellers_Rotors/NACA_15_polar_Re_200000.txt','Propellers_Rotors/NACA_15_polar_Re_500000.txt',
                                        'Propellers_Rotors/NACA_15_polar_Re_1000000.txt']]   
    prop.airfoil_polar_stations     = list(np.zeros(20))
    airfoil_polars                  = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs    
    prop.mid_chord_aligment         = np.zeros_like(prop.chord_distribution) #  np.zeros_like(prop.chord_distribution) # prop.chord_distribution/4. - prop.chord_distribution[0]/4.
    prop.airfoil_data               = import_airfoil_geometry(prop.airfoil_geometry, npoints = 402)
    prop.airfoil_flag               = True 
    plot_propeller(prop) 
    
    return prop
