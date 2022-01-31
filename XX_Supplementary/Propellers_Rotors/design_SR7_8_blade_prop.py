# Imports
import SUAVE
from SUAVE.Core import Units, Data   
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry    
from scipy.interpolate import interp1d
import os
import numpy as np 
import os 

# design propeller 
def design_SR7_8_blade_prop():
    prop                            = SUAVE.Components.Energy.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'SR7_8_blade_Propeller'  
    prop.tip_radius                 = 0.622/2
    prop.hub_radius                 = prop.tip_radius * 0.2476 
    prop.number_of_blades           = 8 
    prop.thrust_angle               = 0.0 
    
    r_R_data                        = np.array([0.2476,0.2927,0.3813, 0.4698,0.5596,0.6482,0.7368,0.8268,0.9141,0.99 ])
    t_b_data                        = np.array([0.235,0.113,0.067,0.046,0.035,0.027,0.022,0.020,0.020,0.019 ])
    b_D_data                        = np.array([0.149,0.166,0.183,0.197,0.204,0.204,0.190,0.158,0.105,0.047 ])     
    delta_beta                      = np.array([18.240,17.466,14.495,10.264,6.5262,3.3360,0.3102,-2.387,-5.248,-7.616 ])
    phi_prime                       = np.array([15.17,12.64,8.797,6.593,5.101,3.829,2.501,1.338,0.504,0.109  ]) 
    
    
    beta_data                       = delta_beta  # did not add blade twist yet + 37.8
    blade_sweep                     = 41 * Units.degrees
    dim                             = 30
    new_radius_distribution         = np.linspace(0.239,0.98,dim)
    func_twist_distribution         = interp1d(r_R_data, beta_data*Units.degrees , kind='cubic')
    func_chord_distribution         = interp1d(r_R_data, b_D_data*2*prop.tip_radius   , kind='cubic')
    func_radius_distribution        = interp1d(r_R_data, r_R_data *prop.tip_radius  , kind='cubic')
    func_max_thickness_distribution = interp1d(r_R_data, t_b_data*b_D_data*2*prop.tip_radius, kind='cubic')  
    
    prop.twist_distribution         = func_twist_distribution(new_radius_distribution)     
    prop.chord_distribution         = func_chord_distribution(new_radius_distribution)         
    prop.radius_distribution        = func_radius_distribution(new_radius_distribution)        
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    prop.airfoil_geometry           = [ rel_path +'../Airfoils/NACA_65_215.txt',rel_path +'../Airfoils/NACA_15.txt']
    prop.airfoil_polars             = [[rel_path +'../Airfoils/Polars/NACA_65_215_polar_Re_50000.txt',rel_path +'../Airfoils/Polars/NACA_65_215_polar_Re_100000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_65_215_polar_Re_200000.txt',rel_path +'../Airfoils/Polars/NACA_65_215_polar_Re_500000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_65_215_polar_Re_1000000.txt'],[rel_path +'../Airfoils/Polars/NACA_15_polar_Re_50000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_15_polar_Re_100000.txt',rel_path +'../Airfoils/Polars/NACA_15_polar_Re_200000.txt',
                                        rel_path +'../Airfoils/Polars/NACA_15_polar_Re_500000.txt',rel_path +'../Airfoils/Polars/NACA_15_polar_Re_1000000.txt']] 

    airfoil_polar_stations          = np.zeros(dim)
    prop.airfoil_polar_stations     = list(airfoil_polar_stations.astype(int) )      
    airfoil_polars                  = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs    
    prop.mid_chord_alignment         = np.zeros_like(prop.radius_distribution * np.tan(blade_sweep) ) 
    prop.number_of_airfoil_section_points = 100
    prop.airfoil_data               = import_airfoil_geometry(prop.airfoil_geometry, npoints = prop.number_of_airfoil_section_points)
    prop.airfoil_flag               = True 
    
    return prop
