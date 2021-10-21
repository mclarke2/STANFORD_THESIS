
import SUAVE
from SUAVE.Core import Units 
import numpy as np 
import copy 
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Propulsion import propeller_design   
import matplotlib.pyplot as plt
import matplotlib.cm as cm 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from scipy.interpolate import interp1d 
import numpy as np   

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    
    #------------------------------------------------------------------
    # Design Rotors  
    #------------------------------------------------------------------ 
    # atmosphere and flight conditions for propeller/rotor design
    g               = 9.81                                   # gravitational acceleration  
    speed_of_sound  = 340                                    # speed of sound 
    rho             = 1.22                                   # reference density
    Hover_Load      = 1000*g      # hover load   
    design_tip_mach = 0.7                                    # design tip mach number 
    
    rotor                        = SUAVE.Components.Energy.Converters.Rotor() 
    rotor.tip_radius             = 3.95 * Units.feet
    rotor.hub_radius             = 0.6  * Units.feet 
    rotor.disc_area              = np.pi*(rotor.tip_radius**2) - np.pi*(rotor.hub_radius**2) 
    rotor.number_of_blades       = 3
    rotor.freestream_velocity    = 500. * Units['ft/min']  
    rotor.angular_velocity       = (design_tip_mach*speed_of_sound)/rotor.tip_radius   
    rotor.design_Cl              = 0.8
    rotor.design_altitude        = 1000 * Units.feet                   
    rotor.design_thrust          = (Hover_Load/4)     
    rotor                        = propeller_design(rotor)    
    

    ## RPM    CT       CP
    #static_results = np.array([[2502,0.0988,0.0880],[2752,0.0985,0.0855],[3053,0.1014,0.0867],[3350,0.1021,0.0872]
                               #,[3637,0.1011,0.0835],[3906,0.1017,0.0853],[4216,0.1015,0.0841],[4473,0.1051,0.0859]
                               #,[4764,0.1043,0.0850],[5026,0.1059,0.0854],[5324,0.1063,0.0857],[5573,0.1074,0.0857]
                               #,[5865,0.1082,0.0856],[6164,0.1091,0.0846],[6444,0.1103,0.0847],[6705,0.1113,0.0846]]) 
       
    
    
    # Find the operating conditions
    atmosphere            = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions =  atmosphere.compute_values(rotor.design_altitude) 
    conditions                                          = Data()
    conditions.freestream                               = Data()
    conditions.propulsion                               = Data()
    conditions.frames                                   = Data()
    conditions.frames.body                              = Data()
    conditions.frames.inertial                          = Data()
    conditions.freestream.update(atmosphere_conditions)
    conditions.freestream.dynamic_viscosity             = atmosphere_conditions.dynamic_viscosity
    conditions.frames.inertial.velocity_vector          = np.array([[rotor.freestream_velocity,0,0]])
    conditions.propulsion.throttle                      = np.array([[1.0]])
    conditions.frames.body.transform_to_inertial        = np.array([np.eye(3)]) 
    
    # Create and attach this propeller 
    rotor.inputs.omega  = np.array(Hover_Load,ndmin=2)  
    
    # propeller with airfoil results 
    F, Q, P, Cplast ,output , etap = rotor.spin(conditions)  
    
    vi_BEMT = np.mean(output.blade_axial_velocity)
    print('BEMT: ', vi_BEMT)
    
    vi = np.sqrt(F/(2*rotor.disc_area*rho))
    
    print('Momentum Theory: ', vi)
    # ----------------------------------------------
    
    # ----------------------------------------------------------------------------	
    # 2D - Plots  Plots    	
    # ---------------------------------------------------------------------------- 	
    # perpendicular velocity, up Plot 	
    fig1 = plt.figure(1)     	
    fig1.set_size_inches(8, 4)  
    axes12 = fig1.add_subplot(2,1,2)           	
    axes12.plot(rotor.radius_distribution, rotor.twist_distribution ,'bo-'  )       	 
    axes12.set_xlabel('Radial Location')	
    axes12.set_ylabel('twist distribution') 	
    axes12.legend(loc='lower right')    	
 
    axes21 = fig1.add_subplot(2,1,1)      	
    axes21.plot(rotor.radius_distribution, rotor.chord_distribution,'bo-')      	
    axes21.set_xlabel('Radial Location')	
    axes21.set_ylabel('chord distribution')	  
    
    return 
    
# design propeller  
def design_APC_11x8_prop():      
    prop                            = SUAVE.Components.Energy.Converters.Rotor()
    prop.inputs                     = Data() 
    prop.tag                        = 'APC_11x8_Propeller'
    prop.tip_radius                 = (11/2)*Units.inches
    prop.hub_radius                 = prop.tip_radius*0.15 
    prop.number_of_blades           = 2  
    prop.thrust_angle               = 0.
    prop.VTOL_flag                  = True 
    r_R                             = np.array([0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.55,
                                                0.60,0.65,0.70,0.75,0.80,0.85,0.90,0.95,0.99 ])
    b_R                             = np.array([0.167,0.166,0.166,0.164,0.161,0.161,0.162,0.162,
                                                0.162,0.162,0.159,0.152,0.143,0.129,0.109,0.084,0.051,0.017 ]) 
    beta                            = np.array([33.69,39.30,42.13,40.42,37.05,33.81,30.80,27.97,25.52,23.28,
                                                21.40,19.68,18.01,16.52,15.07,13.26,11.87,10.54 ]) 
    
    
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
    prop.airfoil_geometry           = ['Propellers_Rotors/E63.txt','Propellers_Rotors/Clark_y.txt']
    prop.airfoil_polars             = [['Propellers_Rotors/E63_polar_Re_50000.txt','Propellers_Rotors/E63_polar_Re_100000.txt',
                                      'Propellers_Rotors/E63_polar_Re_200000.txt'    ,'Propellers_Rotors/E63_polar_Re_500000.txt',
                                      'Propellers_Rotors/E63_polar_Re_1000000.txt'] ,['Propellers_Rotors/Clark_y_polar_Re_50000.txt',
                                      'Propellers_Rotors/Clark_y_polar_Re_100000.txt','Propellers_Rotors/Clark_y_polar_Re_200000.txt',
                                      'Propellers_Rotors/Clark_y_polar_Re_500000.txt','Propellers_Rotors/Clark_y_polar_Re_1000000.txt']] 
    prop.airfoil_polar_stations     = [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1]   
    airfoil_polars                  = compute_airfoil_polars(prop.airfoil_geometry, prop.airfoil_polars)  
    airfoil_cl_surs                 = airfoil_polars.lift_coefficient_surrogates 
    airfoil_cd_surs                 = airfoil_polars.drag_coefficient_surrogates         
    prop.airfoil_cl_surrogates      = airfoil_cl_surs
    prop.airfoil_cd_surrogates      = airfoil_cd_surs 
    prop.mid_chord_aligment         = np.zeros_like(prop.chord_distribution) #  prop.chord_distribution/4. - prop.chord_distribution[0]/4. 
 
    
    return prop

if __name__ == '__main__': 
    main()    
    plt.show()   