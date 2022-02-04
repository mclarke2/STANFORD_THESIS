import SUAVE 
from SUAVE.Core import Units , Data 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt
from SUAVE.Methods.Propulsion import  rotor_design
import pickle
from SUAVE.Plots.Geometry import plot_propeller
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------


def main():
    #test_planform()
    single_design_point()
    #plot_results_and_pareto_fronteir()
    return 


def test_planform(): 
    c_r     = 0.3
    c_t     = 0.1   
    b       = 1 # span 
    r       = 11
    N       = r-1                      # number of spanwise divisions
    n       = np.linspace(N,0,r)       # vectorize
    theta_n = n*(np.pi/2)/r            # angular stations
    y_n     = b*np.cos(theta_n)        # y locations based on the angular spacing
    eta_n   = np.abs(y_n/b)            # normalized coordinates 
    p       = [0.25,0.5,1,2] 
    markers = ['s','o','P','D','v'] 
    q       = [0.25,0.5,1,1.5]         # q must be positive 
    colors  = ['red','blue','black','green','orange']
    
    fig = plt.figure()
    axis = fig.add_subplot(1,1,1) 
    for i in range(len(p)):
        for j in range(len(q)):
            c_n = c_r*(1 - eta_n**p[i])**q[j] + c_t*eta_n
            line_label = 'p = ' + str(p[i]) +  ', q = ' + str(q[j]) 
            axis.plot(y_n,c_n,linestyle = '-', marker = markers[i],color = colors[j], label  = line_label)    
    return 

def single_design_point():
    
    objective_weights = np.linspace(0.05,0.95,30)
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
        opt_params                             = rotor.optimization_parameters 
        opt_params.aeroacoustic_weight         = objective_weights[i]   # 1 means only perfomrance optimization 0.5 to weight noise equally
        
        # DESING ROTOR 
        rotor                                  = rotor_design(rotor,number_of_airfoil_section_points=100)  
      
        # save rotor geomtry
        opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
        opt_weight = opt_weight.replace('.','_')    
        name       = 'Single_Point_Rotor_Design_Thrust_' + str(int(rotor.design_thrust)) + '_Opt_Weight_' + opt_weight
        save_blade_geometry(rotor,name)
        
        #plot_propeller(rotor)  
    
    return 

def plot_results_and_pareto_fronteir():   
    
     
    objective_weights = [0.0,0.25,0.5,0.75,1.0]
    design_thrust     = (2300*9.81/(8))   
     
    
    # Add to rotor definition  

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'legend.fontsize': 22,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 2
    plot_parameters.line_style       = '-'
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 7
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.plot_grid        = True  
    plot_parameters.colors           = cm.viridis(np.linspace(0,1,len(objective_weights)))
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
    
    axis_1,axis_2,axis_3,axis_4,fig_1,fig_2,fig_3,fig_4 = set_up_axes(plot_parameters,design_thrust)
    
    for i in range(len(objective_weights)):   
        # save rotor geomtry
        opt_weight      = str(objective_weights[i])
        opt_weight      = opt_weight.replace('.','_')    
        rotor_file_name = 'Single_Point_Rotor_Design_Thrust_' + str(int(design_thrust)) + '_Opt_Weight_' + opt_weight
        rotor           = load_blade_geometry(rotor_file_name)  
        
        rotor_name  = r'$\alpha$ = ' + str(objective_weights[i]) 
        propeller_comparison_plots(rotor,axis_1,axis_2,axis_3,axis_4,plot_parameters,i, rotor_name)   
    
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()

    
    axis_1.legend(loc='upper right', ncol= 2, prop={'size': plot_parameters.legend_font_size})  
    axis_2.legend(loc='upper right', ncol= 2, prop={'size': plot_parameters.legend_font_size}) 
    axis_3.legend(loc='upper right', ncol= 2, prop={'size': plot_parameters.legend_font_size}) 
    axis_4.legend(loc='upper right', ncol= 2, prop={'size': plot_parameters.legend_font_size})    
    fig_1_name = "Rotor_Twist_Comparson_" + str(int(design_thrust))  + '_N' 
    fig_2_name = "Rotor_Chord_Comparson_" + str(int(design_thrust))  + '_N' 
    fig_3_name = "Rotor_Thickness_Comparson_" + str(int(design_thrust))  + '_N' 
    fig_4_name = "Rotor_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'         
    fig_1.savefig(fig_1_name  + '.pdf')               
    fig_2.savefig(fig_2_name  + '.pdf')               
    fig_3.savefig(fig_3_name  + '.pdf')               
    fig_4.savefig(fig_4_name  + '.pdf')          
    return  
 

def propeller_comparison_plots(rotor,axis_1,axis_2,axis_3,axis_4,PP,idx,label_name = 'prop'):  
         
    axis_1.plot(rotor.radius_distribution, rotor.twist_distribution/Units.degrees,
                color      = PP.colors[idx],
                marker     = PP.markers[idx],
                linestyle  = PP.line_style,
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_2.plot(rotor.radius_distribution, rotor.chord_distribution/rotor.tip_radius,
                color      = PP.colors[idx],
                marker     = PP.markers[idx],
                linestyle  = PP.line_style,
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_3.plot(rotor.radius_distribution  , rotor.max_thickness_distribution/rotor.chord_distribution,
                color      = PP.colors[idx],
                marker     = PP.markers[idx],
                linestyle  = PP.line_style,
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_4.scatter(rotor.design_power/1E6, rotor.design_SPL_dBA,
                   color  = PP.colors[idx],
                   marker = 'o',
                   s      = 150,
                   label  = label_name )    
    
    return  

# ------------------------------------------------------------------ 
# Setup Axes 
# ------------------------------------------------------------------ 
def set_up_axes(PP,design_thrust):
    
    # ------------------------------------------------------------------
    #   Twist Distribition
    # ------------------------------------------------------------------
    fig_1_name = "Rotor_Twist_Comparson_" + str(int(design_thrust))  + '_N'
    fig_1 = plt.figure(fig_1_name)
    fig_1.set_size_inches(PP.figure_width,PP.figure_height)
    axis_1 = fig_1.add_subplot(1,1,1)
    axis_1.set_ylabel(r'$\theta$ ($\degree$)') 
    axis_1.set_xlabel('r')    
    axis_1.minorticks_on()    
    fig_1.tight_layout()    
    
    # ------------------------------------------------------------------
    #   Chord Distribution
    # ------------------------------------------------------------------ 
    fig_2_name = "Rotor_Chord_Comparson_" + str(int(design_thrust))  + '_N'
    fig_2 = plt.figure(fig_2_name)     
    fig_2.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.set_ylabel('c/b') 
    axis_1.set_xlabel('r')    
    axis_2.minorticks_on()    
    fig_2.tight_layout()     

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_3_name = "Rotor_Thickness_Comparson_" + str(int(design_thrust))  + '_N'
    fig_3 = plt.figure(fig_3_name)     
    fig_3.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_3 = fig_3.add_subplot(1,1,1)  
    axis_3.set_ylabel('t/b') 
    axis_1.set_xlabel('r')    
    axis_3.minorticks_on()    
    fig_3.tight_layout()     
    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_4_name = "Rotor_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_4 = plt.figure(fig_4_name)     
    fig_4.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_4 = fig_4.add_subplot(1,1,1)  
    axis_4.set_xlabel('Power (MW)') 
    axis_4.set_ylabel('SPL (dBA)')    
    axis_4.minorticks_on()    
    fig_4.tight_layout()     
     
    return axis_1,axis_2,axis_3,axis_4,fig_1,fig_2,fig_3,fig_4 


# ------------------------------------------------------------------
#   Save Blade Geometry
# ------------------------------------------------------------------   
def save_blade_geometry(rotor,filename):
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(rotor, file) 
    return     

# ------------------------------------------------------------------
#   Load Blade Geometry
# ------------------------------------------------------------------   
def load_blade_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        rotor = pickle.load(file) 
    return rotor


if __name__ == '__main__': 
    main() 
    plt.show()