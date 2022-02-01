import SUAVE 
from SUAVE.Core import Units , Data 
import numpy as np 
import matplotlib.pyplot as plt
from Rotor_Blade_Optimization import  low_noise_rotor_design 
import pickle
from SUAVE.Plots.Geometry import plot_propeller
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------


def main():
    #test_planform()
    single_design_point()
    #pareto_fronteir()
    return 


def test_planform():
    
    c_r = 0.3
    c_t = 0.1 
    
    

    b       = 1 # span 
    r       = 11
    N       = r-1                        # number of spanwise divisions
    n       = np.linspace(N,0,r)          # vectorize
    theta_n = n*(np.pi/2)/r              # angular stations
    y_n     = b*np.cos(theta_n)         # y locations based on the angular spacing
    eta_n   = np.abs(y_n/b)            # normalized coordinates
    
 
    p          = [0.25,0.5,1,2] 
    markers    = ['s','o','P','D','v']
    
    q          = [0.25,0.5,1,1.5] # q must be positive 
    colors     = ['red','blue','black','green','orange']
    
    # green really (q = 1.5) bad: p 
    # bad q = 2 (green), p = less than 1
    
    # black  (q = 1) bad when p = 0.25   
    # green  (q = 1.5) bad when p = 0.25,0.5    # let q/p > 2
    
    fig = plt.figure()
    axis = fig.add_subplot(1,1,1)
    
    for i in range(len(p)):
        for j in range(len(q)):
            c_n = c_r*(1 - eta_n**p[i])**q[j] + c_t*eta_n
            line_label = 'p = ' + str(p[i]) +  ', q = ' + str(q[j]) 
            axis.plot(y_n,c_n,linestyle = '-', marker = markers[i],color = colors[j], label  = line_label)            
    #plt.legend(loc='upper left') 
    return 

def single_design_point():
 
     # DEFINE ROTOR OPERATING CONDITIONS 
    rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
    rotor.tag                             = 'rotor'     
    rotor.tip_radius                      = 1.3
    rotor.hub_radius                      = 0.15 * rotor.tip_radius
    rotor.design_tip_mach                 = 0.65 # gives better noise results and more realistic blade 
    rotor.number_of_blades                = 3  
    inflow_ratio                          = 0.1
    rotor.angular_velocity                = rotor.design_tip_mach*343 /rotor.tip_radius
    #rotor.freestream_velocity             = 0.1 # 500* Units['ft/min'] #130 * Units.mph  
    rotor.freestream_velocity             = inflow_ratio*rotor.angular_velocity*rotor.tip_radius   
    Hover_Load                            = 2300*9.81      # hover load   
    rotor.design_altitude                 = 0 * Units.feet                             
    rotor.design_thrust                   = (Hover_Load/(8))    
    rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    
     # OPTIMIZATION PARAMETERS 
    rotor.optimization_parameters          = Data()
    opt_params                             = rotor.optimization_parameters
    opt_params.slack_constaint             = 1E-6 # slack constraint 
    opt_params.ideal_SPL                   = 45
    opt_params.aeroacoustic_weight         = 0    # 1 means only perfomrance optimization 0.5 to weight noise equally
    
    # DESING ROTOR 
    rotor                                  = low_noise_rotor_design(rotor,number_of_airfoil_section_points=100)  
  
    # save rotor geomtry
    opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
    opt_weight = opt_weight.replace('.','_')    
    name       = 'Single_Point_Rotor_Design_Thrust_' + str(int(rotor.design_thrust)) + '_Opt_Weight_' + opt_weight
    save_blade_geometry(rotor,name)
    
    plot_propeller(rotor)  
    
    return 

def pareto_fronteir():   
    
    
    # Add to rotor definition  
    objective_weights = [0,1,0.25,0.5,0.75]
    
    for i in range(len(objective_weights)): 
    
        # DEFINE ROTOR OPERATING CONDITIONS 
        rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
        rotor.tag                             = 'rotor'     
        rotor.tip_radius                      = 1.3
        rotor.hub_radius                      = 0.15 * rotor.tip_radius
        rotor.design_tip_mach                 = 0.65 # gives better noise results and more realistic blade 
        rotor.number_of_blades                = 3  
        inflow_ratio                          = 0.1
        rotor.angular_velocity                = rotor.design_tip_mach*343 /rotor.tip_radius
        #rotor.freestream_velocity             = 0.1 # 500* Units['ft/min'] #130 * Units.mph  
        rotor.freestream_velocity             = inflow_ratio*rotor.angular_velocity*rotor.tip_radius   
        Hover_Load                            = 2300*9.81      # hover load   
        rotor.design_altitude                 = 0 * Units.feet                             
        rotor.design_thrust                   = (Hover_Load/(8))    
        rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
        rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                 '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                 '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                 '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                 '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
        rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
        
         # OPTIMIZATION PARAMETERS 
        rotor.optimization_parameters          = Data()
        opt_params                             = rotor.optimization_parameters
        opt_params.slack_constaint             = 1E-6 # slack constraint 
        opt_params.ideal_SPL                   = 45
        opt_params.aeroacoustic_weight         = objective_weights[i]     
        
        # DESING ROTOR 
        rotor                                  = low_noise_rotor_design(rotor,number_of_airfoil_section_points=100)  
      
        # save rotor geomtry
        opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
        opt_weight = opt_weight.replace('.','_')    
        name       = 'Pareto_Rotor_Design_Thrust_' + str(int(rotor.design_thrust)) + '_Opt_Weight_' + opt_weight
        save_blade_geometry(rotor,name)
          
        if objective_weights[i] == 0: 
            opt_params.ideal_SPL                      = rotor.design_SPL_dBA 
   
    return  


def save_blade_geometry(rotor,filename):

    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(rotor, file)     
    
    return 

if __name__ == '__main__': 
    main() 
    plt.show()