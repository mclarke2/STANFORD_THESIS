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
    plot_parameters = define_plot_parameters() 
 
    
    #TW_prop_rotor_Adkins_Leibeck()
    
    alpha_weights                      = np.linspace(0,1.0,9) 
    beta_weights                       = np.array([0.1,0.9]) # 
    use_pyoptsparse                    = False 
    plot_rotor_geomery_and_performance = False 
    save_figures                       = False 
    #TW_prop_rotor_single_design_point(alpha_weights,beta_weights,use_pyoptsparse, plot_rotor_geomery_and_performance,plot_parameters,save_figures)  
    
    # 2D PARETO
    #TW_prop_rotor_designs_and_pareto_fronteir(alpha_weights,np.ones_like(alpha_weights)*0.5,'Alpha_Sweep',use_pyoptsparse,plot_parameters,save_figures)
    #TW_prop_rotor_designs_and_pareto_fronteir(np.ones_like(beta_weights)*0.625,beta_weights,'Beta_Sweep',use_pyoptsparse,plot_parameters,save_figures)
     
    # COMPARE TWO ROTORS 
    alpha_weights     = np.array([1.0,1.0])  # low blade solidity reduces thickness noise 
    beta_weights      = np.array([0.1,0.9])
    
    

    #alpha_weights     = np.array([1.0,0.5])  
    #beta_weights      =  np.array([0.3,0.3])    
    TW_prop_rotor_design_comparisons(alpha_weights,beta_weights,use_pyoptsparse,plot_parameters, save_figures)
    
    #examine_broadband_validation(plot_parameters)
    return 
 

# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Design Point Analysis
# ------------------------------------------------------------------ 
def TW_prop_rotor_single_design_point(alpha_weights,beta_weights,use_pyoptsparse_flag, plot_rotor_geomery_and_performance,plot_parameters,save_figures): 
     
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
            prop_rotor.design_tip_mach_range_hover     = [0.3,0.65]  
            prop_rotor.design_tip_mach_range_cruise    = [0.3,0.65]  
            inflow_ratio_hover                         = 0.06 
            prop_rotor.angular_velocity_hover          = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius 
            prop_rotor.design_altitude_hover           = 0 * Units.feet                  
            prop_rotor.design_thrust_hover             = (2300*9.81/(8))  
            prop_rotor.freestream_velocity_hover       = inflow_ratio_hover*prop_rotor.angular_velocity_hover*prop_rotor.tip_radius  
         

            prop_rotor.angular_velocity_cruise         = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius                     
            prop_rotor.design_altitude_cruise          = 2500 * Units.feet                      
            prop_rotor.design_thrust_cruise            = 1408.25/8     
            prop_rotor.freestream_velocity_cruise      = 175*Units.mph   
            
            prop_rotor.airfoil_geometry                = [ '../../../XX_Supplementary/Airfoils/NACA_4412.txt']
            prop_rotor.airfoil_polars                  = [['../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                           '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                           '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                           '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                           '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
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
            beta_opt_weight  = str(beta_weights[j])
            beta_opt_weight  = beta_opt_weight.replace('.','_')    
            name       = 'Rotor_TH_' + str(int(prop_rotor.design_thrust_hover)) + '_TC_' + str(int(prop_rotor.design_thrust_cruise)) +\
                          '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer
            save_blade_geometry(prop_rotor,name)
            
            if plot_rotor_geomery_and_performance: 
                plot_geoemtry_and_performance(prop_rotor,name,plot_parameters,save_figures)
    
    return  

# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Design Comparisons
# ------------------------------------------------------------------ 
def TW_prop_rotor_design_comparisons(alpha_weights,beta_weights,use_pyoptsparse_flag,PP,save_figures): 

    design_thrust_hover  = (2300*9.81/(8))    
    design_thrust_cruise = 1410/8

    alpha_0 = str(alpha_weights[0])
    alpha_0 = alpha_0.replace('.','_')    
    beta_0  = str(beta_weights[0])
    beta_0  = beta_0.replace('.','_')     
    alpha_1 = str(alpha_weights[1])
    alpha_1 = alpha_1.replace('.','_')    
    beta_1  = str(beta_weights[1])
    beta_1  = beta_1.replace('.','_')       

    rotor_comparison = '_A' + alpha_0 + '_B' +  beta_0 + '_A' + alpha_1 + '_B'+  beta_1
    
    fig_1_name = "Thrust_TW_Rotor_Comparison" + rotor_comparison
    fig_2_name = "Torque_TW_Rotor_Comparison" + rotor_comparison
    fig_3_name = "Blade_Re_TW_Rotor_Comparison" + rotor_comparison 
    fig_4_name = "Blade_AoA_TW_Rotor_Comparison" + rotor_comparison
    fig_5_name = 'SPL_TW_Rotor_Comparison' + rotor_comparison 
    fig_6_name = "Twist_TW_Rotor_Comparison" + rotor_comparison 
    fig_7_name = "Chord_TW_Rotor_Comparison"  + rotor_comparison
    fig_8_name = "Thickness_TW_Rotor_Comparison" + rotor_comparison 
    fig_9_name = "MCA_TW_Rotor_Comparison" + rotor_comparison 
      
    fig_1 = plt.figure(fig_1_name)
    fig_1.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_1 = fig_1.add_subplot(1,1,1)   
    axis_1.set_xlabel('r')
    axis_1.set_ylabel('T (N)')    
    
    fig_2 = plt.figure(fig_2_name) 
    fig_2.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_2 = fig_2.add_subplot(1,1,1)    
    axis_2.set_xlabel('r')
    axis_2.set_ylabel('Q (N-m)') 
    
    fig_3 = plt.figure(fig_3_name)      
    fig_3.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_3 = fig_3.add_subplot(1,1,1) 
    axis_3.set_xlabel('r')
    axis_3.set_ylabel(r'Re.')  

    fig_4 = plt.figure(fig_4_name)   
    fig_4.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_4 = fig_4.add_subplot(1,1,1)   
    axis_4.set_ylabel(r'AoA$_{eff}$ ($\degree$)') 
    axis_4.set_ylim([-5,35])
    axis_4.set_xlabel('r')
        
    fig_5 = plt.figure(fig_5_name)    
    fig_5.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_5.set_xlabel('Frequency (Hz)') 
        
    fig_6 = plt.figure(fig_6_name)
    fig_6.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_6 = fig_6.add_subplot(1,1,1)  
    axis_6.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_6.set_xlabel('r')      
    
    fig_7 = plt.figure(fig_7_name)
    fig_7.set_size_inches(PP.figure_width, PP.figure_height)   
    axis_7 = fig_7.add_subplot(1,1,1)
    axis_7.set_ylabel('c (m)') 
    axis_7.set_xlabel('r')    
    
    fig_8 = plt.figure(fig_8_name)
    fig_8.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_8 = fig_8.add_subplot(1,1,1)
    axis_8.set_ylabel('t (m)')  
    axis_8.set_xlabel('r')      
    
    
    fig_9 = plt.figure(fig_9_name)
    fig_9.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_9 = fig_9.add_subplot(1,1,1)  
    axis_9.set_ylabel('M.C.A (m)')  
    axis_9.set_xlabel('r')   
    
    
    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator  
    angles               = np.array([135]) 
    folder               = 'Rotor_Designs' 
    
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    
    for i in range(len(alpha_weights)): 
                
        # save rotor geomtry
        alpha_opt_weight = str(alpha_weights[i])
        alpha_opt_weight = alpha_opt_weight.replace('.','_')    
        beta_opt_weight  = str(beta_weights[i])
        beta_opt_weight  = beta_opt_weight.replace('.','_')    
        file_name        =  rel_path +  folder + separator + 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                         '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer  
        rotor            = load_blade_geometry(file_name)
        rotor_name       =  'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                           '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer 
 
        c    = rotor.chord_distribution
        beta = rotor.twist_distribution
        MCA  = rotor.mid_chord_alignment
        r    = rotor.radius_distribution/rotor.tip_radius
        t    = rotor.max_thickness_distribution
       
        T_hover           = rotor.design_performance_hover.blade_thrust_distribution[0] 
        Q_hover           = rotor.design_performance_hover.blade_torque_distribution[0] 
        Re_hover          = rotor.design_performance_hover.blade_reynolds_number_distribution[0]  
        AoA_hover         = rotor.design_performance_hover.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_hover = rotor.design_acoustics_hover.SPL_1_3_spectrum_dBA 
        frequency         = rotor.design_acoustics_hover.one_third_frequency_spectrum 
        RPM_hover         = rotor.angular_velocity_hover/Units.rpm
        PM_hover          = rotor.inputs.pitch_command_hover

        T_cruise           = rotor.design_performance_cruise.blade_thrust_distribution[0] 
        Q_cruise           = rotor.design_performance_cruise.blade_torque_distribution[0] 
        Re_cruise          = rotor.design_performance_cruise.blade_reynolds_number_distribution[0]  
        AoA_cruise         = rotor.design_performance_cruise.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_cruise = rotor.design_acoustics_cruise.SPL_1_3_spectrum_dBA  
        RPM_cruise         = rotor.angular_velocity_cruise/Units.rpm
        PM_cruise          = rotor.inputs.pitch_command_cruise 
        
        
        TM_cruise = rotor.angular_velocity_cruise*rotor.tip_radius/343    
        TM_hover = rotor.angular_velocity_hover*rotor.tip_radius/343    
        # ----------------------------------------------------------------------------
        # 2D - Plots     
        # ----------------------------------------------------------------------------  
        rotor_label         = r'$\alpha$ = ' +  str(alpha_weights[i]) + r', $\beta$ = ' + str(beta_weights[i])
        hover_text          = r'Hover        : RPM = ' + str(int(RPM_hover)) + ' TM = ' + str(TM_hover) +  r', Pitch Command = ' + str(int(PM_hover/Units.degrees)) + r' degrees '    
        cruise_text         = r'Cruise       : RPM = ' + str(int(RPM_cruise)) + ' TM = ' + str(TM_cruise)  + r', Pitch Command = ' + str(int(PM_cruise/Units.degrees)) + r' degrees ' 
        SPL_text            =  'SPL          : ' + str(rotor.design_SPL_dBA_hover)
        cruise_Pow_text     =  'Cruise Power : ' + str(rotor.design_performance_cruise.power[0][0])
        hover_Pow_text      =  'Hover Power  : ' + str(rotor.design_performance_hover.power[0][0])
        print('Alpha = ' +  str(alpha_weights[i]) + ', Beta = ' + str(beta_weights[i]))
        print(hover_text)   
        print(hover_Pow_text)
        print(cruise_text) 
        print(cruise_Pow_text)
        print(SPL_text)
        
        hover_label  = r'Hover: ' + rotor_label
        cruise_label = r'Cruise: ' + rotor_label
  
        axis_1.plot(r , T_hover ,color = PP.colors[0][i]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width, label =  hover_label )   
        axis_1.plot(r , T_cruise ,color = PP.colors[1][i] , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label  )            
  
        axis_2.plot(r , Q_hover , color = PP.colors[0][i]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)  
        axis_2.plot(r , Q_cruise ,color =  PP.colors[1][i] , markersize = PP.marker_size ,marker =  PP.markers[0], linestyle = PP.line_styles[0], linewidth = PP.line_width,label =  cruise_label  )            
  
        axis_3.plot(r , Re_hover ,color = PP.colors[0][i] , markersize = PP.marker_size ,marker =PP.markers[2],linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)    
        axis_3.plot(r , Re_cruise ,color =  PP.colors[1][i]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =   cruise_label  )       
 
        axis_4.plot(r , AoA_hover ,color = PP.colors[0][i] , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label )  
        axis_4.plot(r , AoA_cruise ,color = PP.colors[1][i]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label   )         
       
        mic = 0
        hover_noise_label  = r'Hover: $\theta_{mic}$ = ' + str(int(angles[mic])) + r'$\degree$, ' + rotor_label
        #cruise_noise_label = r'Cruise: $\theta_{mic}$ = ' + str(int(angles[mic])) + r' $\degree$' + rotor_label
        axis_5.semilogx(frequency , SPL_dBA_1_3_hover[0,mic] ,color = PP.colors[0][i]   , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_noise_label )  
        #axis_5.semilogx(frequency , SPL_dBA_1_3_cruise[0,mic],color = PP.colors[1][i] , markersize = PP.marker_size  ,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label = cruise_noise_label )        
  
  
        axis_6.plot(r, beta/Units.degrees,color = PP.colors[0][i] , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)  
  
        axis_7.plot(r, c ,color = PP.colors[0][i]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)    
    
        axis_8.plot(r , t,color = PP.colors[0][i]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)     
        
        axis_9.plot(r, MCA,color = PP.colors[0][i]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width, label = rotor_label)  
   

        # ----------------------------------------------------------------------------
        # 3D - Plots     
        # ----------------------------------------------------------------------------        
        
        fig_0 = plt.figure(rotor_name) 
        fig_0.set_size_inches(12,12) 
        axis_0 = plt.axes(projection='3d') 
        axis_0.view_init(elev= -50, azim= -2) 
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
        if save_figures:
            fig_0.savefig(fig_0_name  + '.pdf') 
        
    
    axis_1.set_ylim([-40,250]) 
    axis_2.set_ylim([-20,40]) 
    axis_3.set_ylim([35E3,35E5])   
    axis_4.set_ylim([-20,10])  
    axis_5.set_ylim([0,120]) 
    axis_6.set_ylim([-10,60]) 
    axis_7.set_ylim([0.0,0.40]) 
    axis_8.set_ylim([0.0,0.05]) 
    axis_9.set_ylim([-0.06,0.03]) 
    
    axis_1.legend(loc='upper left')    
    axis_2.legend(loc='upper left')    
    axis_3.legend(loc='upper left')  
    axis_4.legend(loc='lower right')     
    axis_5.legend(loc='upper right') 
    axis_6.legend(loc='upper right')     
    axis_7.legend(loc='upper right') 
    axis_8.legend(loc='upper right')     
    axis_9.legend(loc='upper right')   
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout(rect= (0.05,0,1,1))
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout(rect= (0.05,0,1,1))
    fig_8.tight_layout(rect= (0.05,0,1,1))
    fig_9.tight_layout(rect= (0.05,0,1,1))   
      
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')   
    
    return  



# ------------------------------------------------------------------ 
# Tiltwing Prop-Rotor Adkins and Liebeck
# ------------------------------------------------------------------ 
def TW_prop_rotor_Adkins_Leibeck(): 
    
    prop_rotor                                 = Prop_Rotor()  
    prop_rotor.tip_radius                      = 1.25
    prop_rotor.hub_radius                      = 0.15 * prop_rotor.tip_radius
    prop_rotor.design_tip_mach                 = 0.65 # 0.6   
    prop_rotor.number_of_blades                = 3  
    
    prop_rotor.freestream_velocity             = 130 * Units.mph  # 10  
    prop_rotor.angular_velocity                = prop_rotor.design_tip_mach*343/prop_rotor.tip_radius      
    prop_rotor.design_Cl                       = 0.7
    prop_rotor.design_altitude                 = 500 * Units.feet                   
    prop_rotor.design_thrust                   = (2300*9.81/(8)) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    
    # HOVER 
    inflow_ratio_hover                         = 0.1  # 0.06 
    prop_rotor.angular_velocity_hover          = prop_rotor.design_tip_mach*343 /prop_rotor.tip_radius 
    prop_rotor.design_altitude_hover           = 0 * Units.feet                  
    prop_rotor.design_thrust_hover             = (2300*9.81/(8))  
    prop_rotor.freestream_velocity_hover       = inflow_ratio_hover*prop_rotor.angular_velocity_hover*prop_rotor.tip_radius 
    
    # CRUISE
    prop_rotor.angular_velocity_cruise          = prop_rotor.design_tip_mach*343/prop_rotor.tip_radius     
    prop_rotor.design_altitude_cruise          = 2500 * Units.feet                      
    prop_rotor.design_thrust_cruise            = 1410/8     
    prop_rotor.freestream_velocity_cruise      = 175*Units.mph     
    
    prop_rotor.airfoil_geometry                 = [ '../../../XX_Supplementary/Airfoils/NACA_4412.txt']
    prop_rotor.airfoil_polars                   = [['../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                    '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                    '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                    '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                    '../../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
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
    theta  = np.array([135])*Units.degrees + 1E-4
    S      = np.maximum(alt , 20*Units.feet) 
    
    # microphone locations
    positions  = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        positions [i][:] = [0.0 , S*np.sin(theta[i])  ,S*np.cos(theta[i])]   
            
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
    thrust , torque, power, Cp_hover  , rotor_aero_data_hover , etap  = prop_rotor.spin(conditions)
    
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
    
    rotor_noise_hover   = propeller_mid_fidelity(net.propellers,rotor_aero_data_hover,segment,settings)   
    mean_SPL_hover          = np.mean(rotor_noise_hover.SPL_dBA) 
    
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
    theta     = np.array([135])*Units.degrees + 1E-4
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
    thrust , torque, power, Cp_cruise  , rotor_aero_data_cruise , etap  = prop_rotor.spin(conditions)
    
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
    
    rotor_noise_cruise   = propeller_mid_fidelity(net.propellers,rotor_aero_data_cruise,segment,settings)   
    mean_SPL_cruise          = np.mean(rotor_noise_cruise.SPL_dBA) 
    
    if prop_rotor.design_power_cruise == None: 
        prop_rotor.design_power_cruise = power[0][0]
    if prop_rotor.design_thrust_cruise == None: 
        prop_rotor.design_thrust_cruise = -thrust[0][2] 
        
    design_torque_cruise = power[0][0]/omega
    
    prop_rotor.design_torque_hover               = design_torque_hover
    prop_rotor.design_torque_cruise              = design_torque_cruise  
    prop_rotor.design_power_coefficient_hover    = Cp_hover[0][0] 
    prop_rotor.design_power_coefficient_cruise   = Cp_cruise[0][0] 
    prop_rotor.design_thrust_coefficient_hover   = rotor_aero_data_hover.thrust_coefficient[0][0] 
    prop_rotor.design_thrust_coefficient_cruise  = rotor_aero_data_cruise.thrust_coefficient[0][0] 
    prop_rotor.design_SPL_dBA_hover              = mean_SPL_hover
    prop_rotor.design_SPL_dBA_cruise             = mean_SPL_cruise
    prop_rotor.design_SPL_dBA                    = mean_SPL_cruise
    prop_rotor.design_performance_hover          = rotor_aero_data_hover
    prop_rotor.design_performance_cruise         = rotor_aero_data_cruise
    prop_rotor.design_acoustics_hover            = rotor_noise_hover
    prop_rotor.design_acoustics_cruise           = rotor_noise_cruise 
    prop_rotor.design_performance                = rotor_aero_data_hover        
    prop_rotor.design_acoustics                  = rotor_noise_hover
    prop_rotor.airfoil_flag                      = True 
    rotor_tag                                    = 'Rotor_T_' + str(int(prop_rotor.design_thrust)) + '_AL' 
    
    save_blade_geometry(prop_rotor,rotor_tag) 
    
    return  

# ------------------------------------------------------------------ 
# Plot Results and Pareto Fronteir
# ------------------------------------------------------------------ 
def TW_prop_rotor_designs_and_pareto_fronteir(weight_1,weight_2,sweep_name,use_pyoptsparse_flag,PP,save_figures):    
     
    PP.colors            = cm.viridis(np.linspace(0,1,len(weight_2)))    
    design_thrust_hover  = (2300*9.81/(8))    
    design_thrust_cruise = 1410/8 
    folder               = 'Rotor_Designs'
    if use_pyoptsparse_flag:
        optimizer = 'SNOPT'
    else: 
        optimizer = 'SLSQP'
    ospath               = os.path.abspath(__file__)
    separator            = os.path.sep
    rel_path             = os.path.dirname(ospath) + separator   

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
    axis_10  = AXES[9] 
    fig_1   = FIGURES[0] 
    fig_2   = FIGURES[1] 
    fig_3   = FIGURES[2] 
    fig_4   = FIGURES[3] 
    fig_5   = FIGURES[4] 
    fig_6   = FIGURES[5] 
    fig_7   = FIGURES[6] 
    fig_8   = FIGURES[7] 
    fig_9   = FIGURES[8]
    fig_10  = FIGURES[9]
     
    for idx in range(len(weight_1) + 1):           
        rotor_flag = True
        if idx == 0: 
            rotor_file_name  = rel_path +  folder + separator + 'Rotor_T_' + str(int(design_thrust_hover)) + '_AL'   
            rotor_tag        = 'T:' + str(int(design_thrust_hover)) + 'A.& L.'
            rotor_name       =  'Adkins & Liebeck'            
            rotor            = load_blade_geometry(rotor_file_name)      
            
        else:
            # save rotor geomtry
            alpha_opt_weight = str(weight_1[idx-1])
            alpha_opt_weight = alpha_opt_weight.replace('.','_')    
            beta_opt_weight  = str(weight_2[idx-1])
            beta_opt_weight  = beta_opt_weight.replace('.','_')    
            rotor_file_name = rel_path +  folder + separator + 'Rotor_TH_' + str(int(design_thrust_hover)) + '_TC_' + str(int(design_thrust_cruise)) +\
                         '_Alpha_' + alpha_opt_weight + '_Beta_' + beta_opt_weight + '_Opt_' + optimizer  
            try: 
                rotor       = load_blade_geometry(rotor_file_name)
                rotor_flag  = True 
                rotor_tag   = 'T:' + str(int(design_thrust_hover)) + r', $\alpha$' + str(weight_1[idx-1]) + r', $\beta$' + str(weight_2[idx-1])
                rotor_name  = r'$\alpha$ = ' + str(weight_1[idx-1]) + r' $\beta$ = ' + str(weight_2[idx-1])  
            except: 
                rotor_flag  = False  
        
        if rotor_flag:
            rotor_aero_data_hover    = rotor.design_performance_hover
            rotor_noise_data_hover   = rotor.design_acoustics_hover 
            Total_SPL_1_3            = rotor_noise_data_hover.SPL_1_3_spectrum_dBA
            Harmonic_1_3             = rotor_noise_data_hover.SPL_harmonic_1_3_spectrum_dBA
            Broadband_1_3            = rotor_noise_data_hover.SPL_broadband_1_3_spectrum_dBA 
            One_Third_Spectrum       = rotor_noise_data_hover.one_third_frequency_spectrum 
            if idx == 0: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = 'black' , linestyle = PP.line_styles[2], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                propeller_geoemtry_comparison_plots(rotor,rotor_aero_data_hover,AXES,'black',PP,idx, rotor_name)  
            
            else: 
                axis_7.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_8.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
                axis_9.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = PP.colors[idx-1] , linestyle = PP.line_styles[2], marker = PP.markers[(idx-1)%9] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
                propeller_geoemtry_comparison_plots(rotor,rotor_aero_data_hover,AXES,PP.colors[idx-1],PP,idx-1, rotor_name)  
                
    
    cmap     = plt.get_cmap("viridis")
    norm     = plt.Normalize(0,1) 
    sm       =  ScalarMappable(norm=norm, cmap=cmap)
    ax_ticks = np.linspace(0,1,11)
    sm.set_array([]) 
            
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.1f')      
    cbar_1  = fig_1.colorbar(sm, ax = axis_1, ticks = list(ax_ticks),  format= sfmt)
    cbar_2  = fig_2.colorbar(sm, ax = axis_2, ticks = list(ax_ticks),  format= sfmt)
    cbar_3  = fig_3.colorbar(sm, ax = axis_3, ticks = list(ax_ticks),  format= sfmt)
    cbar_4  = fig_4.colorbar(sm, ax = axis_4, ticks = list(ax_ticks),  format= sfmt)
    cbar_5  = fig_5.colorbar(sm, ax = axis_5, ticks = list(ax_ticks),  format= sfmt)
    cbar_6  = fig_6.colorbar(sm, ax = axis_6, ticks = list(ax_ticks),  format= sfmt)
    cbar_7  = fig_7.colorbar(sm, ax = axis_7, ticks = list(ax_ticks),  format= sfmt)
    cbar_8  = fig_8.colorbar(sm, ax = axis_8, ticks = list(ax_ticks),  format= sfmt)
    cbar_9  = fig_9.colorbar(sm, ax = axis_9, ticks = list(ax_ticks),  format= sfmt)
    cbar_10 = fig_10.colorbar(sm, ax = axis_10, ticks = list(ax_ticks),  format= sfmt)
    
    
    cbar_1.set_label(r'$\alpha$')
    cbar_2.set_label(r'$\alpha$')
    cbar_3.set_label(r'$\alpha$')
    cbar_4.set_label(r'$\alpha$') 
    cbar_5.set_label(r'$\alpha$')
    cbar_6.set_label(r'$\alpha$')
    cbar_7.set_label(r'$\alpha$') 
    cbar_8.set_label(r'$\alpha$')
    cbar_9.set_label(r'$\alpha$') 
    cbar_10.set_label(r'$\alpha$')
    
    
    fig_1_name = "Rotor_Twist_Compairson_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_2_name = "Rotor_Chord_Compairson_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_3_name = "Rotor_Thickness_Comparison_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_4_name = "Rotor_Hover_Power_Noise_Pareto_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_5_name = "Rotor_Blade_Re_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    fig_6_name = "Rotor_Blade_AoA_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name    
    fig_7_name = 'Rotor_Total_SPL_Comparison' + sweep_name
    fig_8_name = 'Rotor_Harmonic_Noise_Comparison' + sweep_name
    fig_9_name = 'Rotor_Broadband_Noise_Comparison' + sweep_name 
    fig_10_name = "Rotor_Cruise_Power_Noise_Pareto_TH" + str(int(design_thrust_hover))  + '_TC' +   str(int(design_thrust_cruise)) + sweep_name
    
    fig_1.tight_layout()
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout()
    fig_7.tight_layout()
    fig_8.tight_layout()
    fig_9.tight_layout()  
    fig_10.tight_layout()   
    
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')                
        fig_10.savefig(fig_10_name  + '.pdf')    
    
    
    return  

# ------------------------------------------------------------------ 
# Plot Single Propeller Geometry and Perfomrance Results
# ------------------------------------------------------------------ 
def plot_geoemtry_and_performance(rotor,rotor_name,PP,save_figures):
    
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
        SPL_dBA_1_3_hover = rotor.design_acoustics_hover.SPL_1_3_spectrum_dBA 
        frequency         = rotor.design_acoustics_hover.one_third_frequency_spectrum
        PM_hover          = rotor.inputs.pitch_command_hover

        T_cruise           = rotor.design_performance_cruise.blade_thrust_distribution[0] 
        Q_cruise           = rotor.design_performance_cruise.blade_torque_distribution[0] 
        Re_cruise          = rotor.design_performance_cruise.blade_reynolds_number_distribution[0]  
        AoA_cruise         = rotor.design_performance_cruise.blade_effective_angle_of_attack[0]/Units.degrees 
        SPL_dBA_1_3_cruise = rotor.design_acoustics_cruise.SPL_1_3_spectrum_dBA  
        PM_cruise          = rotor.inputs.pitch_command_cruise
        
    elif type(rotor) == Lift_Rotor: 
        T_hover           = rotor.design_performance.blade_thrust_distribution[0] 
        Q_hover           = rotor.design_performance.blade_torque_distribution[0] 
        Re_hover          = rotor.design_performance.blade_reynolds_number_distribution[0]  
        AoA_hover         = rotor.design_performance.blade_effective_angle_of_attack[0]/Units.degrees
        SPL_dBA_1_3_hover = rotor.design_acoustics.SPL_1_3_spectrum_dBA 
        frequency         = rotor.design_acoustics.one_third_frequency_spectrum  
        PM_hover          = rotor.inputs.pitch_command
    
        
    # ----------------------------------------------------------------------------
    # 2D - Plots     
    # ----------------------------------------------------------------------------  
    hover_label  = r'Hov.: $\beta_c$ = ' + str(int(PM_hover/Units.degrees)) + r'$\degree$'
    if prop_rotor_flag:
        cruise_label = r'Cr.: $\beta_c$ = ' + str(int(PM_cruise/Units.degrees)) + r'$\degree$'
    
    fig_1 = plt.figure('Blade_Thurst_Distribution')
    fig_1.set_size_inches(PP.figure_width, PP.figure_height)     
    axis_1 = fig_1.add_subplot(1,1,1)    
    axis_1.plot(r , T_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2], linewidth = PP.line_width, label =  hover_label )    
    if prop_rotor_flag: 
        axis_1.plot(r , T_cruise ,color = PP.colors[0][1] , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label  )            
    axis_1.set_xlabel('r')
    axis_1.set_ylabel('T (N)')   
    axis_1.set_ylim([1.3*min(T_cruise),1.5*max(T_hover)])  
    axis_1.legend(loc='upper left')
    
    fig_2 = plt.figure('Blade_Torque_Distribution') 
    fig_2.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_2 = fig_2.add_subplot(1,1,1)  
    axis_2.plot(r , Q_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)  
    if prop_rotor_flag: 
        axis_2.plot(r , Q_cruise ,color =  PP.colors[0][1] , markersize = PP.marker_size ,marker =  PP.markers[0], linestyle = PP.line_styles[0], linewidth = PP.line_width,label =  cruise_label  )            
    axis_2.set_xlabel('r')
    axis_2.set_ylabel('Q (N-m)')
    axis_2.set_ylim([min(Q_cruise) - 10,1.5*max( np.maximum(Q_hover,Q_cruise))])  
    axis_2.legend(loc='upper left')
     

    fig_3 = plt.figure('Blade_Re_Distribution')      
    fig_3.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_3 = fig_3.add_subplot(1,1,1)
    axis_3.plot(r , Re_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size ,marker =PP.markers[2],linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label)    
    if prop_rotor_flag: 
        axis_3.plot(r , Re_cruise ,color =  PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =   cruise_label  )       
    axis_3.set_xlabel('r')
    axis_3.set_ylabel(r'Re.') 
    axis_3.set_ylim([0.7*min(Re_hover),1.5*max(Re_hover)])  
    axis_3.legend(loc='upper left')  
    

    fig_4 = plt.figure('Blade_AoA_Distribution')   
    fig_4.set_size_inches(PP.figure_width, PP.figure_height)    
    axis_4 = fig_4.add_subplot(1,1,1)   
    axis_4.plot(r , AoA_hover ,color = PP.colors[0][0]  , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label )  
    if prop_rotor_flag: 
        axis_4.plot(r , AoA_cruise ,color = PP.colors[0][1]  , markersize = PP.marker_size,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label =  cruise_label   )         
    axis_4.set_xlabel('r')
    axis_4.set_ylabel(r'AoA$_{eff}$ ($\degree$)') 
    axis_4.set_ylim([-5,35])
    axis_4.set_ylim([1.3*min(AoA_cruise),1.5*max(AoA_hover)])  
    axis_4.legend(loc='lower right')      

    fig_5 = plt.figure('Blade_SPL_dBA')    
    fig_5.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_5 = fig_5.add_subplot(1,1,1)  
    theta   = [135]
    axis_5.semilogx(frequency , SPL_dBA_1_3_hover[0,0] ,color = PP.colors[0][0]   , markersize = PP.marker_size,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width, label = hover_label + r',$\theta$ = ' + str(theta[0])+ r'$\degree$')  
    if prop_rotor_flag: 
        axis_5.semilogx(frequency , SPL_dBA_1_3_cruise[0,0],color = PP.colors[0][1] , markersize = PP.marker_size  ,marker = PP.markers[0], linestyle = PP.line_styles[0],linewidth = PP.line_width, label = cruise_label  + r',$\theta$ = ' + str(theta[0])+ r'$\degree$')        
    axis_5.set_ylabel(r'SPL$_{1/3}$ (dBA)')
    axis_5.set_xlabel('Frequency (Hz)') 
    axis_5.set_ylim([0,120])  
    axis_5.legend(loc='upper right', ncol = 2)  
        
    fig_6 = plt.figure('Twist_Distribution')
    fig_6.set_size_inches(PP.figure_width, PP.figure_height)  
    axis_6 = fig_6.add_subplot(1,1,1)
    axis_6.plot(r, beta/Units.degrees,color = PP.colors[0][0]  , markersize = PP.marker_size ,marker = PP.markers[2], linestyle = PP.line_styles[2],linewidth = PP.line_width)  
    axis_6.set_ylabel(r'$\beta$ ($\degree$)') 
    axis_6.set_xlabel('r')    
    
    fig_7 = plt.figure('Chord_Distribution')
    fig_7.set_size_inches(PP.figure_width, PP.figure_height)   
    axis_7 = fig_7.add_subplot(1,1,1)
    axis_7.plot(r, c ,color = PP.colors[0][0]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)    
    axis_7.set_ylabel('c (m)') 
    axis_7.set_xlabel('r')    

    fig_8 = plt.figure('Thickness_Distribution')
    fig_8.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_8 = fig_8.add_subplot(1,1,1)
    axis_8.plot(r , t,color = PP.colors[0][0]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)     
    axis_8.set_ylabel('t (m)')  
    axis_8.set_xlabel('r')    

    fig_9 = plt.figure('MCA_Distribution')
    fig_9.set_size_inches(PP.figure_width, PP.figure_height)       
    axis_9 = fig_9.add_subplot(1,1,1)
    axis_9.plot(r, MCA,color = PP.colors[0][0]  ,marker = PP.markers[2] , markersize = PP.marker_size, linestyle = PP.line_styles[2],linewidth = PP.line_width)  
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
     
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')               
        fig_2.savefig(fig_2_name  + '.pdf')               
        fig_3.savefig(fig_3_name  + '.pdf')               
        fig_4.savefig(fig_4_name  + '.pdf')              
        fig_5.savefig(fig_5_name  + '.pdf')       
        fig_6.savefig(fig_6_name  + '.pdf')        
        fig_7.savefig(fig_7_name  + '.pdf')               
        fig_8.savefig(fig_8_name  + '.pdf')               
        fig_9.savefig(fig_9_name  + '.pdf')    
    

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
    fig_0.savefig(fig_0_name  + '.pdf') 
    
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
    axis_10 = AXES[9]          
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
    axis_4.scatter(rotor.design_power_hover/1E3, rotor.design_SPL_dBA_hover,
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
    axis_10.scatter(rotor.design_power_cruise/1E3, rotor.design_SPL_dBA_cruise,
                   color  = color,
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
    fig_4_name = "Rotor_Hover_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_4 = plt.figure(fig_4_name)     
    fig_4.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_4 = fig_4.add_subplot(1,1,1)  
    axis_4.set_xlabel('Power (kW)') 
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
    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_10_name = "Rotor_Cruise_Power_Noise_Pareto_" + str(int(design_thrust))  + '_N'
    fig_10 = plt.figure(fig_10_name)     
    fig_10.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_10 = fig_10.add_subplot(1,1,1)  
    axis_10.set_xlabel('Power (kW)') 
    axis_10.set_ylabel('SPL (dBA)')    
    axis_10.minorticks_on()   
    
    AXES    = [axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,axis_7,axis_8,axis_9,axis_10]
    FIGURES = [fig_1,fig_2,fig_3,fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,fig_10]
    return AXES , FIGURES

  

def define_plot_parameters(): 

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

    plot_parameters.colors           = [['black','firebrick','darkblue'],
                                        ['dimgray','red','blue'], 
                                        ['darkgray','salmon','deepskyblue']]  
     
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
    return plot_parameters

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