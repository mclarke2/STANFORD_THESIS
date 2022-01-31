# Imports
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry    
from scipy.interpolate import interp1d
import os
import numpy as np  

# design propeller   
def design_APC_11x45_prop():     
    prop                            = SUAVE.Components.Energy.Converters.Rotor()
    prop.inputs                     = Data() 
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'APC_11x45_Propeller'
    prop.tip_radius                 = 0.28/2
    prop.hub_radius                 = prop.tip_radius*0.15 
    prop.number_of_blades           = 2  
    prop.thrust_angle               = 0 * Units.degrees     
    r_R                             = np.array([0.15,0.2,0.25, 0.3,0.35,0.4,0.45,0.5,0.55	
                                                ,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,0.99  ]) 
    b_R                             = np.array([ 0.124,0.146,0.167,0.18,0.188,0.19,0.189,0.184,0.175,0.164,
                                                0.151,0.136,0.121,0.105,0.09,0.077,0.053,0.029 ])
    beta                            = np.array([ 41.11,41.01,34.01,28.76,24.78,21.7,19.4,17.65,16.19,
                                             15.03,14.1,13.26,12.61,12.0,11.34,10.55,9.33,8.07 ])     
    
    prop.twist_distribution         = beta*Units.degrees
    prop.chord_distribution         = b_R*prop.tip_radius    
    prop.radius_distribution        = r_R*prop.tip_radius    
    
    # estimate thickness 
    r_R_data                        = np.array([0.15,0.275,0.367,0.449,0.5,0.55,
                                                0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,0.99])
    t_b_data                        = np.array([0.122,0.105,0.077,0.061,0.055,0.049,0.045,0.041,0.038
                                                ,0.035,0.033,0.031,0.029,0.027,0.026]) 
    b_D_data                        = np.array([0.14485,0.14587,0.1481,
                                                0.1499,0.15061,0.15058,0.14981,0.14831,0.1468,0.14529,0.14268,
                                                0.13764,0.12896,0.11304,0.085])    
    func_max_thickness_distribution = interp1d(r_R_data, t_b_data*b_D_data*2*prop.tip_radius, kind='cubic')   
    prop.max_thickness_distribution = func_max_thickness_distribution(r_R) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    prop.airfoil_geometry           = [ rel_path +'../Airfoils/E63.txt',rel_path +'../Airfoils/Clark_y.txt']
    prop.airfoil_polars             = [[rel_path +'../Airfoils/Polars/E63_polar_Re_50000.txt'     ,rel_path +'../Airfoils/Polars/E63_polar_Re_100000.txt',
                                        rel_path +'../Airfoils/Polars/E63_polar_Re_200000.txt'    ,rel_path +'../Airfoils/Polars/E63_polar_Re_500000.txt',
                                        rel_path +'../Airfoils/Polars/E63_polar_Re_1000000.txt'] ,[rel_path +'../Airfoils/Polars/Clark_y_polar_Re_50000.txt',
                                        rel_path +'../Airfoils/Polars/Clark_y_polar_Re_100000.txt',rel_path +'../Airfoils/Polars/Clark_y_polar_Re_200000.txt',
                                        rel_path +'../Airfoils/Polars/Clark_y_polar_Re_500000.txt',rel_path +'../Airfoils/Polars/Clark_y_polar_Re_1000000.txt']] 
    prop.airfoil_polar_stations     = [0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1]   
    airfoil_polars                  = compute_airfoil_polars( prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs 
    prop.mid_chord_alignment         = np.zeros_like(prop.chord_distribution) #  prop.chord_distribution/4. - prop.chord_distribution[0]/4.  
    prop.number_of_airfoil_section_points = 100
    prop.airfoil_data               = import_airfoil_geometry(prop.airfoil_geometry, npoints = prop.number_of_airfoil_section_points)
    prop.airfoil_flag               = True 
        
 
    return prop