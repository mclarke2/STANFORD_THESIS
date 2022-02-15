import SUAVE 
from SUAVE.Core import Units , Data 
from SUAVE.Methods.Propulsion                                          import lift_rotor_design , prop_rotor_design , propeller_design 
from SUAVE.Analyses.Mission.Segments.Segment                           import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics           import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller 
from SUAVE.Plots.Geometry.plot_vehicle                                 import plot_propeller_geometry 
from SUAVE.Components.Energy.Converters                                import Lift_Rotor, Rotor, Prop_Rotor, Propeller

# Package Imports 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker 
from matplotlib.cm import ScalarMappable
import os
import pickle

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    

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
    plot_parameters.line_styles      = ['--',':','-',':','--']
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 7
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.plot_grid        = True  
    plot_parameters.colors           = ['deepskyblue','blue','black','red','firebrick']  
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
        
    #test_rotor_planform_function(plot_parameters) 
    
    alpha_weights     = np.array([0.5])  # np.linspace(0.0,1.0,11) 
    beta_weights      = np.array([0.25]) # np.linspace(0.0,1.0,11) 
    use_pyoptsparse   = False 
    plot_rotor_geomery_and_performance = True 
    

    #SR_lift_rotor_Adkins_Leibeck()  
    #SR_lift_rotor_single_design_point(alpha_weights,use_pyoptsparse,plot_rotor_geomery_and_performance,plot_parameters)
    #SR_lift_rotor_designs_and_pareto_fronteir(alpha_weights,plot_parameters)
    
    #TW_prop_rotor_Adkins_Leibeck()
    TW_prop_rotor_single_design_point(alpha_weights,beta_weights,use_pyoptsparse, plot_rotor_geomery_and_performance,plot_parameters)  
    #TW_prop_rotor_designs_and_pareto_fronteir(alpha_weights,np.ones_like(alpha_weights)*0.5,'Alpha_Sweep',plot_parameters)
    #TW_prop_rotor_designs_and_pareto_fronteir(beta_weights,np.ones_like(beta_weights)*0.5,'Beta_Sweep',plot_parameters)
    
    #examine_broadband_validation(plot_parameters)
    return 

# ------------------------------------------------------------------ 
# Test Rotor Planform Function
# ------------------------------------------------------------------ 
def test_rotor_planform_function(PP): 
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
    fig.set_size_inches(PP.figure_width,PP.figure_height)    
    axis = fig.add_subplot(1,1,1) 
    axis.set_ylabel(r'$c_n$')
    axis.set_xlabel(r'$\eta_n$')
    for i in range(len(p)):
        for j in range(len(q)):
            c_n = c_r*(1 - eta_n**p[i])**q[j] + c_t*eta_n
            line_label = 'p = ' + str(p[i]) +  ', q = ' + str(q[j]) 
            axis.plot(y_n,c_n,linestyle = '-', linewidth = PP.line_width, marker = markers[i],color = colors[j], label  = line_label) 
            
    
    fig.tight_layout()    
    fig_name = 'Rotor_Planform_Shape_Function'          
    fig.savefig(fig_name  + '.png')              
    return 


# ------------------------------------------------------------------ 
# Stopped-Rotor Lift Rotor Singe Point Design Point Analysis
# ------------------------------------------------------------------ 
def SR_lift_rotor_single_design_point(alpha_weights,use_pyoptsparse_flag, plot_rotor_geomery_and_performance,plot_parameters): 
     
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    
    for i in range(len(alpha_weights)):
 
        # DEFINE ROTOR OPERATING CONDITIONS 
        rotor                            = Lift_Rotor()
        rotor.tag                        = 'lift_rotor'
        rotor.tip_radius                 = 1.15
        rotor.hub_radius                 = 0.15 * rotor.tip_radius
        rotor.number_of_blades           = 3
        rotor.design_tip_mach            = 0.65     
        rotor.inflow_ratio               = 0.06 
        rotor.angular_velocity           = rotor.design_tip_mach* 343 /rotor.tip_radius   
        rotor.freestream_velocity        = rotor.inflow_ratio*rotor.angular_velocity*rotor.tip_radius  
        rotor.design_altitude            = 20 * Units.feet   
        Hover_Load                       = 2700*9.81      # hover load   
        rotor.design_thrust              = Hover_Load/(12-2) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
        rotor.variable_pitch             = True   
        rotor.airfoil_geometry           = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
        rotor.airfoil_polars             = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                           '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                           '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                           '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                           '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
        rotor.airfoil_polar_stations     = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
        
        
         # OPTIMIZATION PARAMETERS  
        opt_params                       = rotor.optimization_parameters 
        opt_params.aeroacoustic_weight   = alpha_weights[i]   # 1 means only perfomrance optimization 0.5 to weight noise equally
        
        # DESING ROTOR 
        rotor                            = lift_rotor_design(rotor,number_of_airfoil_section_points=100,use_pyoptsparse=use_pyoptsparse_flag)  
      
        # save rotor geomtry
        opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
        opt_weight = opt_weight.replace('.','_')    
        name       = 'Rotor_T_' + str(int(rotor.design_thrust))  + '_Alpha_' + opt_weight + '_Opt_' + optimizer
        save_blade_geometry(rotor,name)
        
        if  plot_rotor_geomery_and_performance: 
            plot_geoemtry_and_performance(rotor,name,plot_parameters) 
    return

# ------------------------------------------------------------------ 
# Stopped-Rotors Adkins and Liebeck
# ------------------------------------------------------------------ 
def SR_lift_rotor_Adkins_Leibeck():
    
    rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
    rotor.tag                             = 'lift_rotor'
    rotor.tip_radius                      = 1.15
    rotor.hub_radius                      = 0.15 * rotor.tip_radius
    rotor.number_of_blades                = 3
    rotor.design_tip_mach                 = 0.65     
    rotor.inflow_ratio                    = 0.06 
    rotor.angular_velocity                = rotor.design_tip_mach* 343 /rotor.tip_radius   
    rotor.freestream_velocity             = rotor.inflow_ratio*rotor.angular_velocity*rotor.tip_radius  
    rotor.design_altitude                 = 20 * Units.feet   
    Hover_Load                            = 2700*9.81      # hover load   
    rotor.design_thrust                   = Hover_Load/(12-2) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    rotor.variable_pitch                  = True       
    rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       
    rotor                                  = propeller_design(rotor,number_of_airfoil_section_points=100)   
 
    net                          = Battery_Propeller()
    net.number_of_rotor_engines  = 1                              
    net.identical_lift_rotors    = True  
    net.lift_rotors.append(rotor)  

    omega          = rotor.angular_velocity
    V              = rotor.freestream_velocity  
    alt            = rotor.design_altitude        

    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 

    # Run Conditions     
     
    theta  = np.array([90,112.5,135,157.5])*Units.degrees + 1E-2
    S      = np.maximum(alt , 20*Units.feet)

    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions[i][:] = [0.0 , S*np.sin(theta[i])  ,S*np.cos(theta[i])] 
            
    # Set up for Propeller Model
    rotor.inputs.omega                                     = np.atleast_2d(omega).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T  
    conditions.frames.inertial.velocity_vector             = np.array([[0, 0. ,V]]) 
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]])   
    # Run Propeller model 
    thrust , torque, power, Cp  , noise_data , etap        = rotor.spin(conditions)

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 

    # Store Noise Data 
    noise                                   = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                = noise.settings   
    num_mic                                 = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones  = num_mic   

    propeller_noise   = propeller_mid_fidelity(net.lift_rotors,noise_data,segment,settings)   
    mean_SPL          =  np.mean(propeller_noise.SPL_dBA)  
 
    rotor.design_SPL_dBA     = mean_SPL
    rotor.design_performance = noise_data
    rotor.design_acoustics   = propeller_noise 
    rotor.airfoil_flag       = True  
    rotor_tag                ='T_' + str(int(rotor.design_thrust)) + '_AL' 
    
    save_blade_geometry(rotor,rotor_tag) 
    return  


# ------------------------------------------------------------------ 
# Plot Results and Pareto Fronteir
# ------------------------------------------------------------------ 
def SR_lift_rotor_designs_and_pareto_fronteir(alpha_weights,PP):    
     
    PP.colors         = cm.viridis(np.linspace(0,1,len(alpha_weights)))    
    design_thrust     = (2300*9.81/(8-1))     
    optimizer         = 'SLSQP'

    AXES , FIGURES = set_up_axes(PP,design_thrust)
    axis_1  = AXES[0] 
    axis_2  = AXES[1] 
    axis_3  = AXES[2] 
    axis_4  = AXES[3] 
    axis_5  = AXES[4] 
    axis_6  = AXES[5] 
    axis_7  = AXES[6] 
    axis_8  = AXES[7] 
    axis_9  = AXES[8] 
    fig_1   = FIGURES[0] 
    fig_2   = FIGURES[1] 
    fig_3   = FIGURES[2] 
    fig_4   = FIGURES[3] 
    fig_5   = FIGURES[4] 
    fig_6   = FIGURES[5] 
    fig_7   = FIGURES[6] 
    fig_8   = FIGURES[7] 
    fig_9   = FIGURES[8]
     
    for idx in range(len(alpha_weights) + 1):   
        rotor_flag = True
        if idx == 0: 
            rotor_file_name  = 'T_' + str(int(design_thrust)) + '_AL' 
            rotor_tag        = 'T:' + str(int(design_thrust)) + 'A.& L.'
            rotor_name       = 'Adkins & Liebeck'
            rotor            = load_blade_geometry(rotor_file_name)            
        else:
            # save rotor geomtry
            alpha_opt_weight = str(alpha_weights[idx-1])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')     
            rotor_file_name  = 'Rotor_T_' + str(int(rotor.design_thrust))  + '_Alpha_' + alpha_opt_weight + '_Opt_' + optimizer
            try: 
                rotor       = load_blade_geometry(rotor_file_name)
                rotor_flag  = True 
                rotor_tag   = 'T:' + str(int(design_thrust)) + r', $\alpha$' + str(alpha_weights[idx-1])
                rotor_name  = r'$\alpha$ = ' + str(alpha_weights[idx-1]) 
            except: 
                rotor_flag  = False  
        
        if rotor_flag:
            noise_data         = rotor.design_performance
            propeller_noise    = rotor.design_acoustics 
            Total_SPL_1_3      = propeller_noise.SPL_1_3_spectrum_dBA
            Harmonic_1_3       = propeller_noise.SPL_harmonic_1_3_spectrum_dBA
            Broadband_1_3      = propeller_noise.SPL_broadband_1_3_spectrum_dBA 
            One_Third_Spectrum = propeller_noise.one_third_frequency_spectrum 
            
            if idx == 0:
                rotor.design_SPL_dBA = np.mean(propeller_noise.SPL_dBA)  
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                propeller_geoemtry_comparison_plots(rotor,noise_data,AXES,'black',PP,idx, rotor_name)  
            
            else: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                   
                propeller_geoemtry_comparison_plots(rotor,noise_data,AXES,PP.colors[idx-1],PP,idx-1, rotor_name)  
                
    
    cmap     = plt.get_cmap("viridis")
    norm     = plt.Normalize(0,1) 
    sm       =  ScalarMappable(norm=norm, cmap=cmap)
    ax_ticks = np.linspace(0,1,11)
    sm.set_array([]) 
            
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.1f')      
    cbar_1 = fig_1.colorbar(sm, ax = axis_1, ticks = list(ax_ticks),  format= sfmt)
    cbar_2 = fig_2.colorbar(sm, ax = axis_2, ticks = list(ax_ticks),  format= sfmt)
    cbar_3 = fig_3.colorbar(sm, ax = axis_3, ticks = list(ax_ticks),  format= sfmt)
    cbar_4 = fig_4.colorbar(sm, ax = axis_4, ticks = list(ax_ticks),  format= sfmt)
    cbar_5 = fig_5.colorbar(sm, ax = axis_5, ticks = list(ax_ticks),  format= sfmt)
    cbar_6 = fig_6.colorbar(sm, ax = axis_6, ticks = list(ax_ticks),  format= sfmt)
    cbar_7 = fig_7.colorbar(sm, ax = axis_7, ticks = list(ax_ticks),  format= sfmt)
    cbar_8 = fig_8.colorbar(sm, ax = axis_8, ticks = list(ax_ticks),  format= sfmt)
    cbar_9 = fig_9.colorbar(sm, ax = axis_9, ticks = list(ax_ticks),  format= sfmt)
    
    
    cbar_1.set_label(r'$\alpha$')
    cbar_2.set_label(r'$\alpha$')
    cbar_3.set_label(r'$\alpha$')
    cbar_4.set_label(r'$\alpha$') 
    cbar_5.set_label(r'$\alpha$')
    cbar_6.set_label(r'$\alpha$')
    cbar_7.set_label(r'$\alpha$') 
    cbar_8.set_label(r'$\alpha$')
    cbar_9.set_label(r'$\alpha$') 
    
    
    fig_1_name = "Rotor_Twist_Compairson_" + str(int(design_thrust))  + '_N' 
    fig_2_name = "Rotor_Chord_Compairson_" + str(int(design_thrust))  + '_N' 
    fig_3_name = "Rotor_Thickness_Comparison_" + str(int(design_thrust))  + '_N' 
    fig_4_name = "Rotor_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_5_name = "Rotor_Blade_Re_" + str(int(design_thrust))  + '_N' 
    fig_6_name = "Rotor_Blade_AoA_" + str(int(design_thrust))  + '_N'         
    fig_7_name = 'Rotor_Total_SPL_Comparison'
    fig_8_name = 'Rotor_Harmonic_Noise_Comparison'
    fig_9_name = 'Rotor_Broadband_Noise_Comparison'  
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout()
    fig_8.tight_layout()
    fig_9.tight_layout()   
    
    fig_1.savefig(fig_1_name  + '.png')               
    fig_2.savefig(fig_2_name  + '.png')               
    fig_3.savefig(fig_3_name  + '.png')               
    fig_4.savefig(fig_4_name  + '.png')              
    fig_5.savefig(fig_5_name  + '.png')       
    fig_6.savefig(fig_6_name  + '.png')        
    fig_7.savefig(fig_7_name  + '.png')               
    fig_8.savefig(fig_8_name  + '.png')               
    fig_9.savefig(fig_9_name  + '.png')     
    
    
    return   

# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Design Point Analysis
# ------------------------------------------------------------------ 
def TW_prop_rotor_single_design_point(alpha_weights,beta_weights,use_pyoptsparse_flag, plot_rotor_geomery_and_performance,plot_parameters): 
     
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    
    for i in range(len(alpha_weights)):
        for j in range(len(beta_weights)):
            
            # DEFINE ROTOR OPERATING CONDITIONS 
            prop_rotor                                 = Prop_Rotor() 
            prop_rotor.tag                             = 'prop_rotor'     
            prop_rotor.tip_radius                      = 1.25
            prop_rotor.hub_radius                      = 0.15 * prop_rotor.tip_radius
            prop_rotor.design_tip_mach                 = 0.6   
            prop_rotor.number_of_blades                = 3  
            inflow_ratio_hover                         = 0.06 
            prop_rotor.angular_velocity_hover          = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius 
            prop_rotor.design_altitude_hover           = 0 * Units.feet                  
            prop_rotor.design_thrust_hover             = (2300*9.81/(8-2)) # 23000/8
            prop_rotor.freestream_velocity_hover       = inflow_ratio_hover*prop_rotor.angular_velocity_hover*prop_rotor.tip_radius
            prop_rotor.angular_velocity_cruise         = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius   
            prop_rotor.design_altitude_cruise          = 2500 * Units.feet                      
            prop_rotor.design_thrust_cruise            = 1410/8     
            prop_rotor.freestream_velocity_cruise      = 175*Units.mph  
            prop_rotor.airfoil_geometry                = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
            prop_rotor.airfoil_polars                  = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                    '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                    '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                    '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                    '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
            prop_rotor.airfoil_polar_stations          = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
            
             # OPTIMIZATION PARAMETERS  
            opt_params                                    = prop_rotor.optimization_parameters 
            opt_params.multiobjective_performance_weight  = beta_weights[j]
            opt_params.multiobjective_acoustic_weight     = 1 # Do not consider cruise noise 
            opt_params.aeroacoustic_weight                = alpha_weights[i]   # 1 means only perfomrance optimization 0.5 to weight noise equally
                  
            # DESING ROTOR       
            prop_rotor                                    = prop_rotor_design(prop_rotor,number_of_airfoil_section_points=100,use_pyoptsparse=use_pyoptsparse_flag)  
          
            # save rotor geomtry
            alpha_opt_weight = str(alpha_weights[i])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')    
            beta_opt_weight  = str(beta_weights[i])
            beta_opt_weight  = beta_opt_weight.replace('.','_')    
            name       = 'Rotor_TH_' + str(int(prop_rotor.design_thrust_hover)) + '_TC_' + str(int(prop_rotor.design_thrust_cruise)) +\
                          '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer
            save_blade_geometry(prop_rotor,name)
            
            if plot_rotor_geomery_and_performance: 
                plot_geoemtry_and_performance(prop_rotor,name,plot_parameters)
    
    return  

# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Adkins and Liebeck
# ------------------------------------------------------------------ 
def TW_prop_rotor_Adkins_Leibeck(): 
    
    prop_rotor                                 = Prop_Rotor()  
    prop_rotor.tip_radius                      = 1.25
    prop_rotor.hub_radius                      = 0.15 * prop_rotor.tip_radius
    prop_rotor.design_tip_mach                 = 0.6   
    prop_rotor.number_of_blades                = 3  
    
    prop_rotor.freestream_velocity             = 130 * Units.mph  # 10  
    prop_rotor.angular_velocity                = prop_rotor.design_tip_mach*343/prop_rotor.tip_radius      
    prop_rotor.design_Cl                       = 0.7
    prop_rotor.design_altitude                 = 500 * Units.feet                   
    prop_rotor.design_thrust                   = (2300*9.81/(8-2)) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    
    
    inflow_ratio_hover                         = 0.06 
    prop_rotor.angular_velocity_hover          = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius 
    prop_rotor.design_altitude_hover           = 0 * Units.feet                  
    prop_rotor.design_thrust_hover             = (2300*9.81/(8-2)) # 23000/8
    prop_rotor.freestream_velocity_hover       = inflow_ratio_hover*prop_rotor.angular_velocity_hover*prop_rotor.tip_radius
    prop_rotor.angular_velocity_cruise         = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius   
    prop_rotor.design_altitude_cruise          = 2500 * Units.feet                      
    prop_rotor.design_thrust_cruise            = 1410/8     
    prop_rotor.freestream_velocity_cruise      = 175*Units.mph     
    
    prop_rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
    prop_rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                             '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
    prop_rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       
    prop_rotor                             = propeller_design(prop_rotor,number_of_airfoil_section_points=100)   
 
    net                                     = Battery_Propeller()
    net.number_of_propeller_engines         = 1                              
    net.identical_propellers                = True  
    net.propellers.append(prop_rotor)   
    # ------------------------------------------------------------------
    # HOVER PERFORMANCE
    # ------------------------------------------------------------------
    omega                           = prop_rotor.angular_velocity_hover
    prop_rotor.inputs.pitch_command =  -10.  * Units.degrees
    V                               = prop_rotor.freestream_velocity_hover  
    alt                             = prop_rotor.design_altitude_hover   
    
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 
    
    # Run Conditions      
    theta  = np.array([90,112.5,135,157.5])*Units.degrees + 1E-2
    S      = np.maximum(alt , 20*Units.feet)
    
    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions[i][:] = [0.0 , S*np.sin(theta[i])  ,S*np.cos(theta[i])]   
            
    # Set up for Propeller Model
    prop_rotor.inputs.omega                          = np.atleast_2d(omega).T
    conditions                                       = Aerodynamics()   
    conditions.freestream.density                    = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity          = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound             = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector       = np.array([[0, 0. ,V]]) 
    conditions.propulsion.throttle                   = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial     = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]]) 
    
    # Run Propeller model 
    thrust , torque, power, Cp_hover  , noise_data_hover , etap  = prop_rotor.spin(conditions)
    
    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 
    
    # Store Noise Data 
    noise                                                  = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                               = noise.settings   
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic   
    
    propeller_noise_hover   = propeller_mid_fidelity(net.propellers,noise_data_hover,segment,settings)   
    mean_SPL_hover          = np.mean(propeller_noise_hover.SPL_dBA) 
    
    if prop_rotor.design_power_hover == None: 
        prop_rotor.design_power_hover = power[0][0]
    if prop_rotor.design_thrust_hover == None: 
        prop_rotor.design_thrust_hover = thrust[0][0]
        
    design_torque_hover = power[0][0]/omega
    
    
    # ------------------------------------------------------------------
    # CRUISE PERFORMANCE
    # ------------------------------------------------------------------
    omega                           = prop_rotor.angular_velocity_cruise
    prop_rotor.inputs.pitch_command = 0.  * Units.degrees 
    V                               = prop_rotor.freestream_velocity_cruise  
    alt                             = prop_rotor.design_altitude_cruise    
    
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 
    
    # Run Conditions     
    theta     = np.array([45,90,135])*Units.degrees + 1E-2
    S         = np.maximum(alt , 20*Units.feet)
    ctrl_pts  = 1 
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions[i][:] = [S*np.cos(theta[i]) ,S*np.sin(theta[i]),-alt]
    
    # Set up for Propeller Model
    prop_rotor.inputs.omega                                = np.atleast_2d(omega).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector             = np.array([[V, 0. ,0.]])
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
    
    # Run Propeller model 
    thrust , torque, power, Cp_cruise  , noise_data_cruise , etap  = prop_rotor.spin(conditions)
    
    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 
    
    # Store Noise Data 
    noise                                                  = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                               = noise.settings   
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic   
    
    propeller_noise_cruise   = propeller_mid_fidelity(net.propellers,noise_data_cruise,segment,settings)   
    mean_SPL_cruise          = np.mean(propeller_noise_cruise.SPL_dBA) 
    
    if prop_rotor.design_power_cruise == None: 
        prop_rotor.design_power_cruise = power[0][0]
    if prop_rotor.design_thrust_cruise == None: 
        prop_rotor.design_thrust_cruise = -thrust[0][2] 
        
    design_torque_cruise = power[0][0]/omega
    
    prop_rotor.design_torque_hover               = design_torque_hover
    prop_rotor.design_torque_cruise              = design_torque_cruise  
    prop_rotor.design_power_coefficient_hover    = Cp_hover[0][0] 
    prop_rotor.design_power_coefficient_cruise   = Cp_cruise[0][0] 
    prop_rotor.design_thrust_coefficient_hover   = noise_data_hover.thrust_coefficient[0][0] 
    prop_rotor.design_thrust_coefficient_cruise  = noise_data_cruise.thrust_coefficient[0][0] 
    prop_rotor.design_SPL_dBA_hover              = mean_SPL_hover
    prop_rotor.design_SPL_dBA_cruise             = mean_SPL_cruise
    prop_rotor.design_performance_hover          = noise_data_hover
    prop_rotor.design_performance_cruise         = noise_data_cruise
    prop_rotor.design_acoustics_hover            = propeller_noise_hover
    prop_rotor.design_acoustics_cruise           = propeller_noise_cruise  
    prop_rotor.airfoil_flag                      = True     
    
    
    rotor_tag                        = 'T_' + str(int(prop_rotor.design_thrust)) + '_AL' 
    
    save_blade_geometry(prop_rotor,rotor_tag) 
    
    return  

# ------------------------------------------------------------------ 
# Plot Results and Pareto Fronteir
# ------------------------------------------------------------------ 
def TW_lift_rotor_designs_and_pareto_fronteir(weight_1,weight_2,sweep_name,PP):    
     
    PP.colors            = cm.viridis(np.linspace(0,1,len(weight_1)))    
    design_thrust_hover  = (2300*9.81/(8-2))    
    design_thrust_cruise = 1410/8 
    optimizer            = 'SLSQP'

    AXES , FIGURES = set_up_axes(PP,design_thrust_hover)
    axis_1  = AXES[0] 
    axis_2  = AXES[1] 
    axis_3  = AXES[2] 
    axis_4  = AXES[3] 
    axis_5  = AXES[4] 
    axis_6  = AXES[5] 
    axis_7  = AXES[6] 
    axis_8  = AXES[7] 
    axis_9  = AXES[8] 
    fig_1   = FIGURES[0] 
    fig_2   = FIGURES[1] 
    fig_3   = FIGURES[2] 
    fig_4   = FIGURES[3] 
    fig_5   = FIGURES[4] 
    fig_6   = FIGURES[5] 
    fig_7   = FIGURES[6] 
    fig_8   = FIGURES[7] 
    fig_9   = FIGURES[8]
     
    for idx in range(len(weight_1) + 1):           
        rotor_flag = True
        if idx == 0: 
            rotor_file_name  = 'T_' + str(int(design_thrust_hover)) + '_AL'   
            rotor_tag        = 'T:' + str(int(design_thrust_hover)) + 'A.& L.'
            rotor_name       = 'Adkins & Liebeck'            
            rotor            = load_blade_geometry(rotor_file_name)      
            
        else:
            # save rotor geomtry
            alpha_opt_weight = str(weight_1[idx-1])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')    
            beta_opt_weight  = str(weight_2[idx-1])
            beta_opt_weight  = beta_opt_weight.replace('.','_')    
            rotor_file_name = 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                          '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer 
            try: 
                rotor       = load_blade_geometry(rotor_file_name)
                rotor_flag  = True 
                rotor_tag   = 'T:' + str(int(design_thrust_hover)) + r', $\alpha$' + str(weight_1[idx-1]) + r', $\beta$' + str(weight_2[idx-1])
                rotor_name  = r'$\alpha$ = ' + str(weight_1[idx-1]) + r' $\beta$ = ' + str(weight_2[idx-1])  
            except: 
                rotor_flag  = False  
        
        if rotor_flag:
            noise_data         = rotor.design_performance
            propeller_noise    = rotor.design_acoustics 
            Total_SPL_1_3      = propeller_noise.SPL_1_3_spectrum_dBA
            Harmonic_1_3       = propeller_noise.SPL_harmonic_1_3_spectrum_dBA
            Broadband_1_3      = propeller_noise.SPL_broadband_1_3_spectrum_dBA 
            One_Third_Spectrum = propeller_noise.one_third_frequency_spectrum
            
            if idx == 0:
                rotor.design_SPL_dBA = np.mean(propeller_noise.SPL_dBA)  
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                propeller_geoemtry_comparison_plots(rotor,noise_data,AXES,'black',PP,idx, rotor_name)  
            
            else: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                   
                propeller_geoemtry_comparison_plots(rotor,noise_data,AXES,PP.colors[idx-1],PP,idx-1, rotor_name)  
                
    
    cmap     = plt.get_cmap("viridis")
    norm     = plt.Normalize(0,1) 
    sm       =  ScalarMappable(norm=norm, cmap=cmap)
    ax_ticks = np.linspace(0,1,11)
    sm.set_array([]) 
            
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.1f')      
    cbar_1 = fig_1.colorbar(sm, ax = axis_1, ticks = list(ax_ticks),  format= sfmt)
    cbar_2 = fig_2.colorbar(sm, ax = axis_2, ticks = list(ax_ticks),  format= sfmt)
    cbar_3 = fig_3.colorbar(sm, ax = axis_3, ticks = list(ax_ticks),  format= sfmt)
    cbar_4 = fig_4.colorbar(sm, ax = axis_4, ticks = list(ax_ticks),  format= sfmt)
    cbar_5 = fig_5.colorbar(sm, ax = axis_5, ticks = list(ax_ticks),  format= sfmt)
    cbar_6 = fig_6.colorbar(sm, ax = axis_6, ticks = list(ax_ticks),  format= sfmt)
    cbar_7 = fig_7.colorbar(sm, ax = axis_7, ticks = list(ax_ticks),  format= sfmt)
    cbar_8 = fig_8.colorbar(sm, ax = axis_8, ticks = list(ax_ticks),  format= sfmt)
    cbar_9 = fig_9.colorbar(sm, ax = axis_9, ticks = list(ax_ticks),  format= sfmt)
    
    
    cbar_1.set_label(r'$\alpha$')
    cbar_2.set_label(r'$\alpha$')
    cbar_3.set_label(r'$\alpha$')
    cbar_4.set_label(r'$\alpha$') 
    cbar_5.set_label(r'$\alpha$')
    cbar_6.set_label(r'$\alpha$')
    cbar_7.set_label(r'$\alpha$') 
    cbar_8.set_label(r'$\alpha$')
    cbar_9.set_label(r'$\alpha$') 
    
    
    fig_1_name = "Rotor_Twist_Compairson_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_2_name = "Rotor_Chord_Compairson_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_3_name = "Rotor_Thickness_Comparison_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_4_name = "Rotor_Power_Noise_Pareto_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_5_name = "Rotor_Blade_Re_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_6_name = "Rotor_Blade_AoA_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name    
    fig_7_name = 'Rotor_Total_SPL_Comparison' + sweep_name
    fig_8_name = 'Rotor_Harmonic_Noise_Comparison' + sweep_name
    fig_9_name = 'Rotor_Broadband_Noise_Comparison' + sweep_name 
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout()
    fig_8.tight_layout()
    fig_9.tight_layout()   
    
    fig_1.savefig(fig_1_name  + '.png')               
    fig_2.savefig(fig_2_name  + '.png')               
    fig_3.savefig(fig_3_name  + '.png')               
    fig_4.savefig(fig_4_name  + '.png')              
    fig_5.savefig(fig_5_name  + '.png')       
    fig_6.savefig(fig_6_name  + '.png')        
    fig_7.savefig(fig_7_name  + '.png')               
    fig_8.savefig(fig_8_name  + '.png')               
    fig_9.savefig(fig_9_name  + '.png')     
    
    
    return  

# ------------------------------------------------------------------ 
# Plot Single Propeller Geometry and Perfomrance Results
# ------------------------------------------------------------------ 
def plot_geoemtry_and_performance(rotor,rotor_name,PP):
    
    prop_rotor_flag  = False
    c    = rotor.chord_distribution
    beta = rotor.twist_distribution
    MCA  = rotor.mid_chord_alignment
    r    = rotor.radius_distribution/rotor.tip_radius
    t    = rotor.max_thickness_distribution

    if type(rotor) == Prop_Rotor: 
        prop_rotor_flag   = True 
        T_hover           = rotor.design_performance_hover.blade_thrust_distribution[0] 
        Q_hover           = rotor.design_performance_hover.blade_torque_distribution[0] 
        Re_hover          = rotor.design_performance_hover.blade_reynolds_number_distribution[0]  
        AoA_hover         = rotor.design_performance_hover.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_hover = rotor.design_acoustics_hover.SPL_1_3_spectrum_dBA[0,1] 
        frequency         = rotor.design_acoustics_hover.one_third_frequency_spectrum
        RPM_hover         = rotor.angular_velocity_hover/Units.rpm
        PM_hover          = rotor.inputs.pitch_command_hover

        T_cruise           = rotor.design_performance_cruise.blade_thrust_distribution[0] 
        Q_cruise           = rotor.design_performance_cruise.blade_torque_distribution[0] 
        Re_cruise          = rotor.design_performance_cruise.blade_reynolds_number_distribution[0]  
        AoA_cruise         = rotor.design_performance_cruise.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_cruise = rotor.design_acoustics_cruise.SPL_1_3_spectrum_dBA[0,1] 
        RPM_cruise         = rotor.angular_velocity_cruise/Units.rpm
        PM_cruise          = rotor.inputs.pitch_command_cruise
        
    elif type(rotor) == Lift_Rotor: 
        T_hover           = rotor.design_performance.blade_thrust_distribution[0] 
        Q_hover           = rotor.design_performance.blade_torque_distribution[0] 
        Re_hover          = rotor.design_performance.blade_reynolds_number_distribution[0]  
        AoA_hover         = rotor.design_performance.blade_effective_angle_of_attack[0]/Units.degrees
        SPL_dBA_1_3_hover = rotor.design_acoustics.SPL_1_3_spectrum_dBA[0,1]  
        frequency         = rotor.design_acoustics.one_third_frequency_spectrum  
        RPM_hover         = rotor.angular_velocity/Units.rpm
        PM_hover          = rotor.inputs.pitch_command
    
        
    # ----------------------------------------------------------------------------
    # 2D - Plots     
    # ----------------------------------------------------------------------------  
    hover_label  = r'Hover: RPM = ' + str(int(RPM_hover)) + r', $\beta_c$ = ' + str(int(PM_hover/Units.degrees)) + r' $\degree$'
    cruise_label = r'Cruise: RPM = ' + str(int(RPM_cruise)) + r', $\beta_c$ = ' + str(int(PM_cruise/Units.degrees)) + r' $\degree$'
    
    fig_1 = plt.figure('Blade_Thurst_Distribution')
    fig_1.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_1 = fig_1.add_subplot(1,1,1)    
    axis_1.plot(r , T_hover ,color = PP.colors[2]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width, label =  hover_label )    
    if prop_rotor_flag: 
        axis_1.plot(r , T_cruise ,color = PP.colors[4] , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label  )            
    axis_1.set_xlabel('r')
    axis_1.set_ylabel('T (N)')   
    axis_1.set_ylim([1.3*min(T_cruise),1.5*max(T_hover)])  
    axis_1.legend(loc='upper left')
    
    fig_2 = plt.figure('Blade_Torque_Distribution') 
    fig_2.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.plot(r , Q_hover ,color = PP.colors[2]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)  
    if prop_rotor_flag: 
        axis_2.plot(r , Q_cruise ,color =  PP.colors[4] , markersize = PP.marker_size ,marker =  PP.markers[0], linestyle = PP.line_styles[0], linewidth = PP.line_width,label =  cruise_label  )            
    axis_2.set_xlabel('r')
    axis_2.set_ylabel('Q (N-m)')
    axis_2.set_ylim([min(Q_cruise) - 10,1.5*max(Q_hover)])  
    axis_2.legend(loc='upper left')
     

    fig_3 = plt.figure('Blade_Re_Distribution')      
    fig_3.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_3 = fig_3.add_subplot(1,1,1)
    axis_3.plot(r , Re_hover ,color = PP.colors[2]  , markersize = PP.marker_size ,marker =PP.markers[2],linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)    
    if prop_rotor_flag: 
        axis_3.plot(r , Re_cruise ,color =  PP.colors[4]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =   cruise_label  )       
    axis_3.set_xlabel('r')
    axis_3.set_ylabel(r'Re.') 
    axis_3.set_ylim([0.7*min(Re_hover),1.5*max(Re_hover)])  
    axis_3.legend(loc='upper left')  
    

    fig_4 = plt.figure('Blade_AoA_Distribution')   
    fig_4.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_4 = fig_4.add_subplot(1,1,1)   
    axis_4.plot(r , AoA_hover ,color = PP.colors[2]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label )  
    if prop_rotor_flag: 
        axis_4.plot(r , AoA_cruise ,color = PP.colors[4]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label   )         
    axis_4.set_xlabel('r')
    axis_4.set_ylabel(r'AoA$_{eff}$ ($\degree$)') 
    axis_4.set_ylim([-5,35])
    axis_4.set_ylim([1.3*min(AoA_cruise),1.5*max(AoA_hover)])  
    axis_4.legend(loc='lower right')      

    fig_5 = plt.figure('Blade_SPL_dBA')    
    fig_5.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.semilogx(frequency , SPL_dBA_1_3_hover ,color = PP.colors[2]   , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label ) 
    if prop_rotor_flag: 
        axis_5.semilogx(frequency , SPL_dBA_1_3_cruise,color = PP.colors[4] , markersize = PP.marker_size  ,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label = cruise_label )          
    axis_5.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_5.set_xlabel('Frequency (Hz)') 
    axis_5.set_ylim([0,1.5*max(SPL_dBA_1_3_hover)])  
    axis_5.legend(loc='upper right')  
        
    fig_6 = plt.figure('Twist_Distribution')
    fig_6.set_size_inches(PP.figure_width, PP.figure_height) 
    fig_6.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_6 = fig_6.add_subplot(1,1,1)
    axis_6.plot(r, beta/Units.degrees,color = PP.colors[2]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width)  
    axis_6.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_6.set_xlabel('r')    
    
    fig_7 = plt.figure('Chord_Distribution')
    fig_7.set_size_inches(PP.figure_width, PP.figure_height)   
    axis_7 = fig_7.add_subplot(1,1,1)
    axis_7.plot(r, c ,color = PP.colors[2]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)    
    axis_7.set_ylabel('c (m)') 
    axis_7.set_xlabel('r')    

    fig_8 = plt.figure('Thickness_Distribution')
    fig_8.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_8 = fig_8.add_subplot(1,1,1)
    axis_8.plot(r , t,color = PP.colors[2]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)     
    axis_8.set_ylabel('t (m)')  
    axis_8.set_xlabel('r')    

    fig_9 = plt.figure('MCA_Distribution')
    fig_9.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_9 = fig_9.add_subplot(1,1,1)
    axis_9.plot(r, MCA,color = PP.colors[2]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)  
    axis_9.set_ylabel('M.C.A (m)')  
    axis_9.set_xlabel('r')    

    fig_1_name = "Thrust_" + rotor_name
    fig_2_name = "Torque_"+ rotor_name 
    fig_3_name = "Blade_Re_" + rotor_name
    fig_4_name = "Blade_AoA_"+ rotor_name 
    fig_5_name = 'SPL_'  + rotor_name 
    fig_6_name = "Twist_" + rotor_name
    fig_7_name = "Chord_" + rotor_name
    fig_8_name = "Thickness_" + rotor_name
    fig_9_name = "MCA_" + rotor_name 
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout(rect= (0.05,0,1,1))
    fig_8.tight_layout(rect= (0.05,0,1,1))
    fig_9.tight_layout(rect= (0.05,0,1,1))   
     
    fig_1.savefig(fig_1_name  + '.png')               
    fig_2.savefig(fig_2_name  + '.png')               
    fig_3.savefig(fig_3_name  + '.png')               
    fig_4.savefig(fig_4_name  + '.png')              
    fig_5.savefig(fig_5_name  + '.png')       
    fig_6.savefig(fig_6_name  + '.png')        
    fig_7.savefig(fig_7_name  + '.png')               
    fig_8.savefig(fig_8_name  + '.png')               
    fig_9.savefig(fig_9_name  + '.png')    
    

    # ----------------------------------------------------------------------------
    # 3D - Plots     
    # ----------------------------------------------------------------------------        
    
    fig_0 = plt.figure('Rotor_3D') 
    fig_0.set_size_inches(12,12) 
    axis_0 = plt.axes(projection='3d')
    if prop_rotor_flag: 
        axis_0.view_init(elev= -50, azim= -2)
    else: 
        axis_0.view_init(elev= 45, azim= 30)   
    axis_0.set_xlim(-1,1)
    axis_0.set_ylim(-1,1)
    axis_0.set_zlim(-1,1) 
    axis_0.grid(False)  
    plt.axis('off')
    
    # append a network for origin and thrust angle default values
    network = Battery_Propeller() 
    
    # plot propeller geometry
    plot_propeller_geometry(axis_0,rotor,network,rotor.tag)    
    

    fig_0_name = "3D_" + rotor_name
    fig_0.savefig(fig_0_name  + '.png') 
    
    return 



# ------------------------------------------------------------------ 
# Plot Propeller Comparison Results
# ------------------------------------------------------------------ 
def propeller_geoemtry_comparison_plots(rotor,outputs,AXES,color,PP,idx,label_name = 'prop'):  
    axis_1  = AXES[0] 
    axis_2  = AXES[1] 
    axis_3  = AXES[2] 
    axis_4  = AXES[3] 
    axis_5  = AXES[4] 
    axis_6  = AXES[5]          
    axis_1.plot(rotor.radius_distribution/rotor.tip_radius, rotor.twist_distribution/Units.degrees,
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_2.plot(rotor.radius_distribution/rotor.tip_radius, rotor.chord_distribution,
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_3.plot(rotor.radius_distribution/rotor.tip_radius  , rotor.max_thickness_distribution,
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_4.scatter(rotor.design_power/1E6, rotor.design_SPL_dBA,
                   color  = color,
                   marker = 'o',
                   s      = 150,
                   label  = label_name )     
    
    axis_5.plot(rotor.radius_distribution/rotor.tip_radius, outputs.blade_reynolds_number_distribution[0],
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_6.plot(rotor.radius_distribution/rotor.tip_radius, outputs.blade_effective_angle_of_attack[0]/Units.degrees,
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 

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
    axis_1.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_1.set_xlabel('r')    
    axis_1.minorticks_on()   
    
    # ------------------------------------------------------------------
    #   Chord Distribution
    # ------------------------------------------------------------------ 
    fig_2_name = "Rotor_Chord_Comparson_" + str(int(design_thrust))  + '_N'
    fig_2 = plt.figure(fig_2_name)     
    fig_2.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.set_ylabel('c (m)') 
    axis_1.set_xlabel('r')    
    axis_2.minorticks_on()    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_3_name = "Rotor_Thickness_Comparson_" + str(int(design_thrust))  + '_N'
    fig_3 = plt.figure(fig_3_name)     
    fig_3.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_3 = fig_3.add_subplot(1,1,1)  
    axis_3.set_ylabel('t (m)') 
    axis_1.set_xlabel('r')    
    axis_3.minorticks_on()  

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
    
    # ------------------------------------------------------------------
    #  Spanwise Re Distribution
    # ------------------------------------------------------------------ 
    fig_5_name = "Rotor_Spanwise_Re_" + str(int(design_thrust))  + '_N'
    fig_5 = plt.figure(fig_5_name)     
    fig_5.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.set_ylabel(r'Sectional Re.') 
    axis_5.set_xlabel('r')    
    axis_5.minorticks_on()   
     

    # ------------------------------------------------------------------
    # Spanwise AoA
    # ------------------------------------------------------------------ 
    fig_6_name = "Rotor_Spanwise_AoA_" + str(int(design_thrust))  + '_N'
    fig_6 = plt.figure(fig_6_name)     
    fig_6.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_6 = fig_6.add_subplot(1,1,1)  
    axis_6.set_ylabel(r'Sectional AoA ($\degree$)') 
    axis_6.set_xlabel('r')      
    axis_6.minorticks_on()     
      

    # ------------------------------------------------------------------
    # Total SPL Spectrum Comparison
    # ------------------------------------------------------------------      
    fig_7 = plt.figure('Rotor_Total_SPL_Comparison')    
    fig_7.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_7 = fig_7.add_subplot(1,1,1)    
    axis_7.set_xscale('log') 
    axis_7.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_7.set_xlabel('Frequency (Hz)') 
    axis_7.set_ylim([0,100])


    # ------------------------------------------------------------------
    # Harmonic Noise Spectrum Comparison
    # ------------------------------------------------------------------  
    fig_8 = plt.figure('Rotor_Harmonic_Noise_Comparison') 
    fig_8.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_8 = fig_8.add_subplot(1,1,1)      
    axis_8.set_xscale('log')
    axis_8.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_8.set_xlabel('Frequency (Hz)')  
    axis_8.set_ylim([0,100]) 
    

    # ------------------------------------------------------------------
    # Broadband Noise Spectrum Comparison
    # ------------------------------------------------------------------      
    fig_9 = plt.figure('Rotor_Broadband_Noise_Comparison')    
    fig_9.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_9 = fig_9.add_subplot(1,1,1)    
    axis_9.set_xscale('log')
    axis_9.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_9.set_xlabel('Frequency (Hz)') 
    axis_9.set_ylim([0,100])
    
    
    AXES    = [axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,axis_7,axis_8,axis_9]
    FIGURES = [fig_1,fig_2,fig_3,fig_4,fig_5,fig_6,fig_7,fig_8,fig_9]
    return AXES , FIGURES


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