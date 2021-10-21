# Imports
import SUAVE
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from scipy.interpolate import interp1d
from scipy.interpolate import Rbf, InterpolatedUnivariateSpline
import numpy as np 
import matplotlib.pyplot as plt 

# design propeller 
                                        
def design_F8745D4_prop():  
    prop                            = SUAVE.Components.Energy.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'F8745_D4_Propeller'  
    prop.tip_radius                 = 2.03/2
    prop.hub_radius                 = prop.tip_radius*0.20
    prop.number_of_blades           = 2  
    r_R_data                        = np.array([ 0.2,0.300,0.450,0.601,0.747,0.901,0.950,0.975,0.998   ])    
    t_c_data                        = np.array([ 0.3585,0.1976,0.1148,0.0834,0.0648,0.0591,0.0562,0.0542,0.0533    ])    
    b_R_data                        = np.array([0.116,0.143,0.163,0.169,0.166,0.148,0.135,0.113,0.075  ])    
    beta_data                       = np.array([  0.362,0.286,0.216,0.170,0.135,0.112,0.105,0.101,0.098  ])* 100 
  
    dim = 30          
    new_radius_distribution         = np.linspace(0.2,0.98 ,dim)
    func_twist_distribution         = interp1d(r_R_data, (beta_data)* Units.degrees   , kind='cubic')
    func_chord_distribution         = interp1d(r_R_data, b_R_data * prop.tip_radius , kind='cubic')
    func_radius_distribution        = interp1d(r_R_data, r_R_data * prop.tip_radius , kind='cubic')
    func_max_thickness_distribution = interp1d(r_R_data, t_c_data * b_R_data , kind='cubic')  
    
    prop.twist_distribution         = func_twist_distribution(new_radius_distribution)     
    prop.chord_distribution         = func_chord_distribution(new_radius_distribution)         
    prop.radius_distribution        = func_radius_distribution(new_radius_distribution)        
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution 
    prop.airfoil_geometry           = [ 'Propellers_Rotors/Clark_y.txt']
    prop.airfoil_polars             = [['Propellers_Rotors/Clark_y_polar_Re_50000.txt' ,'Propellers_Rotors/Clark_y_polar_Re_100000.txt','Propellers_Rotors/Clark_y_polar_Re_200000.txt',
                                        'Propellers_Rotors/Clark_y_polar_Re_500000.txt','Propellers_Rotors/Clark_y_polar_Re_1000000.txt']]
    prop.airfoil_polar_stations     = list(np.zeros(dim))  
    airfoil_polars                  = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs    
    prop.mid_chord_aligment         = np.zeros_like(prop.chord_distribution) #  prop.chord_distribution/4. - prop.chord_distribution[0]/4.    
    
    return prop