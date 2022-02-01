import SUAVE 
from SUAVE.Core import Units
import numpy as np 
import matplotlib.pyplot as plt
from Rotor_Blade_Optimization import  low_noise_rotor_design 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------


def main():
    test_planform()
    #test_optimizer()
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
    q          = [0.25,0.5,1,2] # q must be positive 
    colors     = ['red','blue','black','green']
    markers    = ['s','o','P','D']
    
    
    fig = plt.figure()
    axis = fig.add_subplot(1,1,1)
    
    for i in range(4):
        for j in range(4):
            c_n = c_r*(1 - eta_n**p[i])**q[j] + c_t*eta_n
            line_label = 'p = ' + str(p[i]) +  ', q = ' + str(q[j]) 
            axis.plot(y_n,c_n,linestyle = '-', marker = markers[i],color = colors[j], label  = line_label)            
    #plt.legend(loc='upper left') 
    return 

def test_optimizer():   
    
    g                                     = 9.81                                   # gravitational acceleration  
    speed_of_sound                        = 340                                    # speed of sound 
    Hover_Load                            = 4191       # hover load    
    rho                                   = 1.2 
              
    rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
    rotor.tag                             = 'rotor'     
    rotor.tip_radius                      = 1.2 
    rotor.hub_radius                      = 0.15 * rotor.tip_radius
    rotor.number_of_blades                = 4
    rotor.design_tip_mach                 = 0.7
    rotor.freestream_velocity             = 5
    rotor.angular_velocity                = rotor.design_tip_mach* speed_of_sound /rotor.tip_radius   
    rotor.design_Cl                       = 0.7
    rotor.design_altitude                 = 500 * Units.feet                             
    rotor.design_thrust                   = (Hover_Load/(6-1))    
    rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    rotor.design_figure_of_merit           = 0.75
    rotor.optimization_slack_constaint     = 0.01 # slack constraint
    rotor.blade_optimization_pivots        = 5 # either 4 or 5 
    rotor.noise_aero_acoustic_obj_weight   = 1 # 1 means only perfomrance optimization 0.5 to weight noise equally
    rotor.linear_interp_flag               = True 
    rotor                                  = low_noise_rotor_design(rotor,solver_name='SLSQP')  
 
    return  
 
if __name__ == '__main__': 
    main() 
    plt.show()