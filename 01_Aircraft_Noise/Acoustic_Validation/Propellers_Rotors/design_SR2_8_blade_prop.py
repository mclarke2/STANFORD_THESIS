# Imports
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller
from SUAVE.Methods.Propulsion                                                             import propeller_design 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry    
from scipy.interpolate import interp1d
from scipy.interpolate import Rbf, InterpolatedUnivariateSpline
import numpy as np 
import os
import matplotlib.pyplot as plt 

# design propeller 
def design_SR2_8_blade_prop():
    prop                            = SUAVE.Components.Energy.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'SR2_8_blade_Propeller'  
    prop.tip_radius                 = 0.622/2
    prop.hub_radius                 = prop.tip_radius * 0.239
    prop.number_of_blades           = 8  
    prop.thrust_angle               = 0.0     
    r_R_data                        = np.array([0.239,0.275,0.367,0.449,0.5,0.55,
                                                0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,0.99])
    t_b_data                        = np.array([0.19804,0.12503,0.06898,
                                                0.05097,0.04377,0.03762,0.03279,0.02907,0.02625,0.02364,
                                                0.02234,0.02083,0.0208,0.0204,0.02])
    b_D_data                        = np.array([0.14485,0.14587,0.1481,
                                                0.1499,0.15061,0.15058,0.14981,0.14831,0.1468,0.14529,0.14268,
                                                0.13764,0.12896,0.11304,0.085])     
    delta_beta                      = np.array([23.325,20.851,14.355,10.098,
                                         8.185,6.394,4.726,3.058,1.483,0.000,-1.405,-3.243,-5.188,
                                         -6.394 ,-7.083 ])
       
    dim = 20
    new_radius_distribution         = np.linspace(0.239,0.98,dim)
    func_twist_distribution         = interp1d(r_R_data, delta_beta *Units.degrees , kind='cubic')
    func_chord_distribution         = interp1d(r_R_data, b_D_data*2*prop.tip_radius   , kind='cubic')
    func_radius_distribution        = interp1d(r_R_data, r_R_data *prop.tip_radius  , kind='cubic')
    func_max_thickness_distribution = interp1d(r_R_data, t_b_data*b_D_data*2*prop.tip_radius, kind='cubic')  
    
    prop.twist_distribution         = func_twist_distribution(new_radius_distribution)     
    prop.chord_distribution         = func_chord_distribution(new_radius_distribution)         
    prop.radius_distribution        = func_radius_distribution(new_radius_distribution)        
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution) 
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution
    path = os.path.dirname(__file__)
    prop.airfoil_geometry           = [ path+'/NACA_65_215.txt',path+'/NACA_15.txt']
    prop.airfoil_polars             = [[path+'/NACA_65_215_polar_Re_50000.txt',path+'/NACA_65_215_polar_Re_100000.txt',
                                        path+'/NACA_65_215_polar_Re_200000.txt',path+'/NACA_65_215_polar_Re_500000.txt',
                                        path+'/NACA_65_215_polar_Re_1000000.txt'],[path+'/NACA_15_polar_Re_50000.txt',
                                        path+'/NACA_15_polar_Re_100000.txt',path+'/NACA_15_polar_Re_200000.txt',
                                        path+'/NACA_15_polar_Re_500000.txt',path+'/NACA_15_polar_Re_1000000.txt']] 
    airfoil_polar_stations     =  np.zeros(dim)
    n                               = len(prop.twist_distribution)  
    airfoil_polar_stations[round(n*0.40):] = 1
    prop.airfoil_polar_stations     = list(airfoil_polar_stations.astype(int) ) 
    airfoil_polars                  = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs    
    prop.mid_chord_aligment         = np.zeros_like(prop.chord_distribution) #  np.zeros_like(prop.chord_distribution) # prop.chord_distribution/4. - prop.chord_distribution[0]/4.
    prop.airfoil_data               = import_airfoil_geometry(prop.airfoil_geometry, npoints = 402)
 
    return prop
