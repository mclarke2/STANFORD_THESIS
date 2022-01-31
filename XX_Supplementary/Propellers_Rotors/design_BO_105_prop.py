# Imports
import SUAVE
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry     
import numpy as np  
import os 

# design propeller 

def design_BO_105_prop():          
    prop                            = SUAVE.Components.Energy.Converters.Rotor()
    prop.inputs                     = Data() 
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'BO_105_40_percent_scale'
    prop.tip_radius                 = 2
    prop.hub_radius                 = prop.tip_radius*0.1 
    prop.number_of_blades           = 4  
    prop.thrust_angle               = 0.
    prop.airfoil_flag               = True
    num_sec                         = 20
    delta_beta                      = 1.5 * Units.degrees 
    non_dim_r                       = np.linspace(prop.hub_radius,0.99,num_sec)
    prop.radius_distribution        = non_dim_r*prop.tip_radius
    prop.thickness_to_chord         = np.ones(num_sec)*0.12
    prop.chord_distribution         = np.ones(num_sec)*0.121 
    prop.max_thickness_distribution = prop.thickness_to_chord* prop.chord_distribution
    prop.twist_distribution         = 8*np.flip(np.linspace(0,1,num_sec))*Units.degrees + delta_beta
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    prop.airfoil_geometry           = [ rel_path +'../Airfoils/NACA_23012.txt']
    prop.airfoil_polars             = [[rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_50000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_100000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_200000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_500000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_23012_polar_Re_1000000.txt']] 
    prop.airfoil_polar_stations     = list(np.zeros(num_sec).astype(int))
    airfoil_polars                  = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs 
    prop.mid_chord_alignment         = np.zeros_like(prop.chord_distribution)  
    prop.number_of_airfoil_section_points = 100
    prop.airfoil_data               = import_airfoil_geometry(prop.airfoil_geometry, npoints = prop.number_of_airfoil_section_points)
    prop.airfoil_flag               = True  
    
    return prop