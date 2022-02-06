import SUAVE 
from SUAVE.Core import Units , Data 
from SUAVE.Methods.Propulsion                                          import  rotor_design , propeller_design
from SUAVE.Plots.Geometry                                              import plot_propeller
from SUAVE.Analyses.Mission.Segments.Segment                           import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics           import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                import Battery_Propeller  

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
    
        
    #test_rotor_planform_function()
    single_design_point()
    #plot_rotor_designs_and_pareto_fronteir(plot_parameters)
    #examine_broadband_validation(plot_parameters)
    return 

# ------------------------------------------------------------------ 
# Test Rotor Planform Function
# ------------------------------------------------------------------ 
def test_rotor_planform_function(): 
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


# ------------------------------------------------------------------ 
# Single Rotor Design Point Analysis
# ------------------------------------------------------------------ 
def single_design_point():
    
    objective_weights = np.array([1.0])# np.linspace(0.0,1.0,51) 
    for i in range(len(objective_weights)):
 
        # DEFINE ROTOR OPERATING CONDITIONS 
        rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
        rotor.tag                             = 'rotor'     
        rotor.tip_radius                      = 1.25
        rotor.hub_radius                      = 0.15 * rotor.tip_radius
        rotor.design_tip_mach                 = 0.65 # gives better noise results and more realistic blade 
        rotor.number_of_blades                = 3  
        inflow_ratio                          = 0.1
        rotor.angular_velocity                = rotor.design_tip_mach*343 /rotor.tip_radius 
        rotor.freestream_velocity             = inflow_ratio*rotor.angular_velocity*rotor.tip_radius #  0.1 # 500* Units['ft/min'] #130 * Units.mph     
        Hover_Load                            = 2300*9.81      # hover load   
        rotor.design_altitude                 = 0 * Units.feet                             
        rotor.design_thrust                   = (Hover_Load/(8-1))    
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
        rotor                                  = rotor_design(rotor,number_of_airfoil_section_points=100,use_pyoptsparse=True)  
      
        # save rotor geomtry
        opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
        opt_weight = opt_weight.replace('.','_')    
        name       = 'Rotor_T_' + str(int(rotor.design_thrust)) +  '_V_' + str(int(rotor.freestream_velocity)) + '_Alpha_' + opt_weight
        save_blade_geometry(rotor,name)
        
        plot_propeller(rotor)  
    
    return  


# ------------------------------------------------------------------ 
# Plot Results and Pareto Fronteir
# ------------------------------------------------------------------ 
def plot_rotor_designs_and_pareto_fronteir(PP):    
    
    objective_weights = [0.0,0.1,0.2,0.25,0.3,0.4,0.5,0.6,0.7,0.75,0.8,0.9,1.0]  
    PP.colors         = cm.viridis(np.linspace(0,1,len(objective_weights)))    
    design_thrust     = (2300*9.81/(8-1))    # (2300*9.81/(8))     
    
    


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
 
    
    for idx in range(len(objective_weights) + 1):  
        if idx == 0:
            rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
            rotor.tag                             = 'rotor'     
            rotor.tip_radius                      = 1.25
            rotor.hub_radius                      = 0.15 * rotor.tip_radius
            rotor.design_tip_mach                 = 0.65 # gives better noise results and more realistic blade 
            rotor.number_of_blades                = 3  
            inflow_ratio                          = 0.1
            rotor.angular_velocity                = rotor.design_tip_mach*343 /rotor.tip_radius  
            freestream_V                          = inflow_ratio*rotor.angular_velocity*rotor.tip_radius 
            rotor.freestream_velocity             = freestream_V
            Hover_Load                            = 2300*9.81      # hover load   
            rotor.design_altitude                 = 0 * Units.feet   
            rotor.design_Cl                       = 0.7
            rotor.design_thrust                   = (Hover_Load/(8-1))    
            rotor.airfoil_geometry                 = [ '../../XX_Supplementary/Airfoils/NACA_4412.txt']
            rotor.airfoil_polars                   = [['../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                                     '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                                     '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                                     '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                                     '../../XX_Supplementary/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt']]   
            rotor.airfoil_polar_stations           = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]       
            rotor                                  = propeller_design(rotor,number_of_airfoil_section_points=100)  
            rotor_tag                              = 'T:' + str(int(design_thrust)) + 'A.& L.'
            rotor_name                             = 'Adkins & Liebeck'
        else:
            # save rotor geomtry
            opt_weight      = str(objective_weights[idx-1])
            opt_weight      = opt_weight.replace('.','_')  
            separator       = os.path.sep
            rotor_file_name = 'Rotor_T_' + str(int(design_thrust)) + '_V_' + str(int(freestream_V)) + separator\
                              +'Rotor_T_' + str(int(design_thrust)) +  '_V_' + str(int(freestream_V)) + '_Alpha_' + opt_weight
            rotor           = load_blade_geometry(rotor_file_name)
            rotor_tag       = 'T:' + str(int(design_thrust)) + r', $\alpha$' + str(objective_weights[idx-1])
            rotor_name      = r'$\alpha$ = ' + str(objective_weights[idx-1]) 
    
    
        net                          = Battery_Propeller()
        net.number_of_rotor_engines  = 1                              
        net.identical_rotors         = True  
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
        theta          = np.array([90,112.5,135])*Units.degrees + 1E-1
        S              = 10.  
    
        # microphone locations
        positions = np.zeros(( len(theta),3))
        for i in range(len(theta)):
            if theta[i]*Units.degrees < np.pi/2:
                positions[i][:] = [-S*np.cos(theta[i]*Units.degrees)  ,S*np.sin(theta[i]*Units.degrees), 0.0]
            else: 
                positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2)  ,S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 
    
        # Set up for Propeller Model
        rotor.inputs.omega                                     = np.atleast_2d(omega).T
        conditions                                             = Aerodynamics()   
        conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
        conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
        conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
        conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T 
        conditions.frames.inertial.velocity_vector             = np.array([[V, 0. ,0.]])
        conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
        conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
    
        # Run Propeller model 
        thrust , torque, power, Cp  , noise_data , etap        = rotor.spin(conditions)
    
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
    
        propeller_noise    = propeller_mid_fidelity(net.lift_rotors,noise_data,segment,settings)   
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
# Plot Results
# ------------------------------------------------------------------ 
def propeller_geoemtry_comparison_plots(rotor,outputs,AXES,color,PP,idx,label_name = 'prop'):  
    axis_1  = AXES[0] 
    axis_2  = AXES[1] 
    axis_3  = AXES[2] 
    axis_4  = AXES[3] 
    axis_5  = AXES[4] 
    axis_6  = AXES[5]          
    axis_1.plot(rotor.radius_distribution, rotor.twist_distribution/Units.degrees,
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_2.plot(rotor.radius_distribution, rotor.chord_distribution/rotor.tip_radius,
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_3.plot(rotor.radius_distribution  , rotor.max_thickness_distribution/rotor.chord_distribution,
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
    
    axis_5.plot(rotor.radius_distribution, outputs.blade_reynolds_number_distribution[0],
                color      = color,
                marker     = PP.markers[idx%9],
                linestyle  = PP.line_styles[2],
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_6.plot(rotor.radius_distribution, outputs.blade_effective_angle_of_attack[0]/Units.degrees,
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
    axis_1.set_ylabel(r'$\theta$ ($\degree$)') 
    axis_1.set_xlabel('r')    
    axis_1.minorticks_on()   
    
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