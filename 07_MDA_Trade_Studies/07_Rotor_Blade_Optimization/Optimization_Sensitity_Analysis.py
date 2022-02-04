# SUAVE Imports 
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions                                           import Aerodynamics
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis      import airfoil_analysis 
import matplotlib.pyplot as plt   
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import  compute_naca_4series
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry\
     import import_airfoil_geometry

# Python Imports 
import time 
import numpy as np  
from scipy.special import fresnel
import matplotlib.pyplot as plt  

# import propeller/rotors geometries  
import sys
sys.path.append('../../XX_Supplementary')

from Propellers_Rotors.design_SR2_4_blade_prop import design_SR2_4_blade_prop 
from Propellers_Rotors.design_SR2_8_blade_prop import design_SR2_8_blade_prop
from Propellers_Rotors.design_SR7_8_blade_prop import design_SR7_8_blade_prop 
from Propellers_Rotors.design_BO_105_prop      import design_BO_105_prop 
from Propellers_Rotors.design_Hubbard_prop     import design_Hubbard_prop     
from Propellers_Rotors.design_F8745D4_prop     import design_F8745D4_prop     
from Propellers_Rotors.design_DJI_9_4x5_prop   import design_DJI_9_4x5_prop   
from Propellers_Rotors.design_APC_11x_4_7_prop import design_APC_11x_4_7_prop   
from Propellers_Rotors.design_APC_11x45_prop   import design_APC_11x45_prop   
from Propellers_Rotors.design_APC_10x7_prop    import design_APC_10x7_prop 
from Propellers_Rotors.design_APC_11x8_prop    import design_APC_11x8_prop
from Propellers_Rotors.design_SR1_prop         import design_SR1_prop

from plot_propeller_performance import plot_propeller_performance 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main(): 
    '''contains function calls for all of the noise validations listed below
    TEST CASE     |                                SOURCE                                |           AUTHOR
    ------------------------------------------------------------------------------------------------------------------------------
    Test Case 1.1 | Applicability of Early Acoustic Theory for Modern Propeller Design   | Kotwicz Herniczek, Mark : Feszty, Daniel
    Test Case 1.11| Applicability of Early Acoustic Theory for Modern Propeller Design   | Kotwicz Herniczek, Mark : Feszty, Daniel
    Test Case 1.14| Applicability of Early Acoustic Theory for Modern Propeller Design   | Kotwicz Herniczek, Mark : Feszty, Daniel

    Test Case 2.1 | Auralization of tonal rotor noise components of a quadcopter flyover | Christian, Andrew: Boyd, D Douglas 
    Test Case 2.2 | Auralization of tonal rotor noise components of a quadcopter flyover | Christian, Andrew: Boyd, D Douglas  
    Test Case 2.3 | Acoustic Characterization and Prediction of Representative ...       | Zawodny, Nikolas S. Boyd, D. Douglas, Jr. Burley, Casey L.
    Test Case 2.4 | Acoustic Characterization and Prediction of Representative ...       | Zawodny, Nikolas S. Boyd, D. Douglas, Jr. Burley, Casey L.

    Test Case 3.1 | Comparisons of predicted propeller noise with windtunnel ...         | Weir, D Powers, J.

    Test Case 4.1 | Cruise Noise of the SR-2 Propeller Model in a Wind Tunnel            | Dittmar, James H 

    Test Case 5.1 | Prediction of rotorcraft broadband trailing-edge noise and parameter | Sicheng Li & Seongkyu Lee
    ''' 


    # Universal Plot Settings
    plt.rcParams["font.family"]    = "Times New Roman"
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['axes.linewidth'] = 1. 

    plot_parmeters = Data( 
        lw  = 2,                             # line_width               
        m = 10,                              # markersize               
        lf = 10,                             # legend_font_size         
        Slc = ['black','dimgray','silver' ], # SUAVE_line_colors        
        Slm = '^',                           # SUAVE_line_markers       
        Sls = '-',                           # SUAVE_line_styles        
        Elc = ['darkred','red','tomato'],    # Experimental_line_colors 
        Elm = 's',                           # Experimental_line_markers
        Els = '',                            # Experimental_line_styles 
        Rlc = ['darkblue','blue','cyan'],    # Ref_Code_line_colors     
        Rlm = 'o',                           # Ref_Code_line_markers    
        Rls = ':',                           # Ref_Code_line_styles     
    )

    ti = time.time()      

    Test_Cases_Group_1(plot_parmeters)
    Test_Cases_Group_2(plot_parmeters)
    Test_Cases_Group_3(plot_parmeters)    
    Test_Cases_Group_4(plot_parmeters)
    Test_Cases_Group_5(plot_parmeters) 


    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Elapsed Time: ' + str(elapsed_time) + ' mins')        

    return
# ----------------------------------------------------------------------
#   Test Cases 
# ----------------------------------------------------------------------
def Test_Cases_Group_1(plot_parmeters):
    '''Harmonic Noise Validation'''

    ''' Acoustic and Aerodynamic Study of a Pusher-Propeller Aircraft Model by Soderman and Horne '''   
    Test_Case_1_1(plot_parmeters)    # OK 

    ''' Sound Measurements for Five Shrouded Propellers at Static Conditions by Hubbard '''      
    Test_Case_1_11(plot_parmeters)   # OK 

    ''' Propeller Noise Tests in the German-Dutch Wind Tunnel by Dobrzynski '''    
    Test_Case_1_14(plot_parmeters)   # BAD  
    return 

def Test_Cases_Group_2(plot_parmeters):
    DJI_CF = design_DJI_9_4x5_prop() 
    APC_SF = design_APC_11x_4_7_prop()  

    '''Acoustic Characterization and Prediction of Representative ...   '''

    APC_SF_inflow_ratio = 0.08
    DJI_CF_inflow_ratio = 0.09
    Test_Case_2_3(DJI_CF,APC_SF,APC_SF_inflow_ratio,DJI_CF_inflow_ratio,plot_parmeters)    
    Test_Case_2_4(DJI_CF,APC_SF,APC_SF_inflow_ratio,DJI_CF_inflow_ratio,plot_parmeters)   
    return  

def Test_Cases_Group_3(plot_parmeters):  
    '''Harmonic Noise Validation'''
    Test_Case_3_1(plot_parmeters)    # GREAT
    return   

def Test_Cases_Group_4(plot_parmeters):  
    Test_Case_4_1(plot_parmeters)   # OK 
    return  

def Test_Cases_Group_5(plot_parmeters):
    '''These tests are to validate the broadband noise '''
    Test_Case_5_1(plot_parmeters)  


    '''Test for Figure 4, Prediction of rotorcraft broadband trailing-edge
    noise and parameter sensitivity study by  Sicheng Li & Seongkyu Lee''' 

    Test_Case_5_2(plot_parmeters)


    return  

def Test_Cases_Group_6(plot_parmeters):
    '''Helicoidal surface theory for harmonic noise of propellers in the far field''' 
    Test_Case_6_1(plot_parmeters)  

    return 

def Test_Case_1_1(PP): 
    # Define Network
    net                              = Battery_Propeller()
    net.number_of_propeller_engines  = 1       
    net.identical_propellers         = True                             
    prop                             = design_SR2_4_blade_prop()   
    prop.origin                      = [[0.,1.,0.],[0.,-1.,0.]]
    net.propellers.append(prop)

    # Atmosheric & Run Conditions                          
    a                                                      = 340.294
    T                                                      = 288.150
    Helical_Tip_Mach                                       = 0.77
    Mach                                                   = 0.2
    Tip_Mach                                               = np.sqrt(Helical_Tip_Mach**2 - Mach**2)
    density                                                = 1.2250	
    dynamic_viscosity                                      = 1.81E-5
    theta                                                  = np.array([15,90,105,140]) # Tranducer 9, 12 , 7 , 8 
    ctrl_pts                                               = 1

    # Set up for Propeller Model                           
    test_omega                                             = a*Tip_Mach/prop.tip_radius  
    prop.inputs.omega                                      = np.ones((ctrl_pts,1)) *  test_omega
    conditions                                             = Aerodynamics() 
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    velocity_vector                                        = np.array([[Mach*a, 0. ,0.]])
    conditions.frames.inertial.velocity_vector             = np.tile(velocity_vector,(ctrl_pts,1))   
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    F, Q, P, Cp , noise_data , etap = prop.spin(conditions) 
    #plot_propeller_performance(F, Q, P, Cp , noise_data,prop, col1 = 'bo-', col2 = 'bs-')

    # Prepare Inputs for Noise Model   
    conditions.noise.total_microphone_locations             = np.array([[[-2.318, -0.621165, 0.0],[0.0, -3.72, 2.14],[0.36, -1.35,0.0 ],[1.072,-0.899,0.0]]])
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment)  
    num_mic                                                = len(conditions.noise.total_microphone_locations[0] ) 
    conditions.noise.number_of_microphones                 = num_mic 
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data

    # Run Noise Model  
    propeller_noise  = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings)  
    SPL_harmonic_1   = propeller_noise.SPL_harmonic_1_3_spectrum[0,:,0]         
    SPL_harmonic_2   = propeller_noise.SPL_harmonic_1_3_spectrum[0,:,1]     

    # experimental data 
    exp_harmonic_1       = np.array([91.3636, 101.2121, 100.8333, 87.95454])
    exp_harmonic_2       = np.array([85.30303,95.530303, 93.63636, 87.95454])    
    SPL_harmonic_1_paper = np.array([72.79279279279277, 103.64864864864862,111.0810810810810 ,101.6216216])
    SPL_harmonic_2_paper = np.array([34.7511312, 99.23076 ,105.7918552, 84.751131]) 

    # plot results
    fig = plt.figure('Test_Case_1_1')
    fig.set_size_inches(16, 6)     
    axes = fig.add_subplot(1,2,1) 
    axes.plot(theta, SPL_harmonic_1_paper, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label='Hanson Paper')
    axes.plot(theta, SPL_harmonic_1      , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw, label='Hanson')
    axes.plot(theta, exp_harmonic_1      , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw, label='Experimental')     
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('theta (deg)')
    axes.legend(loc='lower right',
                prop={'size': PP.lf} ) 
    plt.ylim((20,130)) 

    axes = fig.add_subplot(1,2,2) 
    axes.plot(theta, SPL_harmonic_2_paper, color = PP.Rlc[1] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label='Hanson Paper')
    axes.plot(theta, SPL_harmonic_2      , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw, label='Hanson')
    axes.plot(theta, exp_harmonic_2      , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw, label='Experimental')       
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('theta (deg)') 
    axes.legend(loc='lower right',
                prop={'size': PP.lf} ) 
    plt.ylim((20,130)) 

    return 
def Test_Case_1_11(PP):

    # Define Network
    net                              = Battery_Propeller()
    net.number_of_propeller_engines  = 1
    net.identical_propellers         = True                    
    prop                             = design_Hubbard_prop() 
    net.propellers.append(prop)

    # Atmosheric & Run Conditions                  
    a                 = 340.294
    T                 = 288.150
    Helical_Tip_Mach  = 0.62
    Mach              = 0.0 
    Tip_Mach          = np.sqrt(Helical_Tip_Mach**2 - Mach**2)
    density           = 1.2250	
    dynamic_viscosity = 1.81E-5 

    # First Harmonic  
    theta_1h          = np.array([1.  ,10.178, 19.821, 27.857, 38.571,45.000, 55.714, 64.285, 80.892, 95.357, 109.28, 119.46,130.17,148.92,165 ])
    S                 = 30.*Units.feet
    ctrl_pts          = 1

    # Microphone Locations 
    positions_h1 = np.zeros((len(theta_1h),3))
    for i in range(len(theta_1h)):
        if theta_1h[i]*Units.degrees < np.pi/2:
            positions_h1[i][:] = [-S*np.cos(theta_1h[i]*Units.degrees),-S*np.sin(theta_1h[i]*Units.degrees), 0.0]
        else: 
            positions_h1[i][:] = [S*np.sin(theta_1h[i]*Units.degrees- np.pi/2),-S*np.cos(theta_1h[i]*Units.degrees - np.pi/2), 0.0] 

    # Set up Propeller Model 
    test_omega                                             = a*Tip_Mach/prop.tip_radius
    prop.inputs.omega                                      = np.ones((ctrl_pts,1)) *  test_omega
    conditions                                             = Aerodynamics() 
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    velocity_vector                                        = np.array([[Mach*a, 0. ,0.]])
    conditions.frames.inertial.velocity_vector             = np.tile(velocity_vector,(ctrl_pts,1))     
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    F, Q, P, Cp , noise_data  , etap = prop.spin(conditions) 
    #plot_propeller_performance(F, Q, P, Cp , noise_data,prop, col1 = 'ro-', col2 = 'rs-')

    # Prepare Inputs for Noise Model
    conditions.noise.total_microphone_locations            = np.array([positions_h1])
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    #conditions.noise.sources.propeller                     = noise_data   
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data

    # Run Noise Model   
    propeller_noise  = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings )   
    SPL_harmonic_1   = propeller_noise.SPL_harmonic_bpf_spectrum[0,:,0]   


    # Harmonic 2
    theta_2h                                                = np.array([1    ,15.45,27.19,38.04,43.94,53.88,65.40,73.48,90.61,113.3,123.7,133.4 ,141.8,149.7,158.2,169.9 ])
    S                                                      = 30.*Units.feet
    ctrl_pts                                               = 1

    # Microphone Locations 
    positions_h2 = np.zeros((len(theta_2h),3))
    for i in range(len(theta_2h)):
        if theta_2h[i]*Units.degrees < np.pi/2:
            positions_h2[i][:] = [S*np.cos(theta_2h[i]*Units.degrees),S*np.sin(theta_2h[i]*Units.degrees), 0.0]
        else: 
            positions_h2[i][:] = [-S*np.sin(theta_2h[i]*Units.degrees- np.pi/2),S*np.cos(theta_2h[i]*Units.degrees - np.pi/2), 0.0] 

    # Set up Propeller Model 
    test_omega                                             = a*Tip_Mach/prop.tip_radius
    prop.inputs.omega                                      = np.ones((ctrl_pts,1)) *  test_omega
    conditions                                             = Aerodynamics() 
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    velocity_vector                                        = np.array([[Mach*a, 0. ,0.]])
    conditions.frames.inertial.velocity_vector             = np.tile(velocity_vector,(ctrl_pts,1))     
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    F, Q, P, Cp , noise_data  , etap = prop.spin(conditions) 
    #plot_propeller_performance(F, Q, P, Cp , noise_data,prop, col1 = 'ro-', col2 = 'rs-')

    # Prepare Inputs for Noise Model
    conditions.noise.total_microphone_locations            = np.array([positions_h2])
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic 
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data

    # Run Noise Model   
    propeller_noise  = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings)    
    SPL_harmonic_2   = propeller_noise.SPL_harmonic_bpf_spectrum[0,:,1] 

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------      

    exp_theta_1 = np.array([0.0 ,10.178, 19.821, 27.857, 38.571,45.000, 55.714, 64.285, 80.892, 95.357, 109.28, 119.46,130.17,148.92,165 ])
    exp_harmonic_1 = np.array([ 79.8643, 84.4661,91.3567,94.4272, 97.1217,97.5170, 97.1580, 97.1762, 101.028,102.203,101.851,99.2016, 96.1709, 94.3022,93.5730,])
    exp_theta_2 = np.array([1.0650,15.443,27.692,38.875,44.201,55.384,65.502,74.023,91.597, 113.96, 124.08,134.20,142.72, 150.17, 158.16, 169.88  ])
    exp_harmonic_2 = np.array([ 85.9541,85.5725,83.6641, 85.9541, 93.2061,95.1145, 96.2595,96.6412,97.0229, 97.4045,96.2595, 92.0610, 88.6259, 87.8625, 88.6259, 91.2977,])    
    SPL_harmonic_1_paper = np.array([62.018, 75.484, 81.325 , 84.250 , 87.851 , 89.428 , 92.132 , 94.834 , 98.217 , 100.03 , 100.71 , 100.28 , 99.175 , 94.939 , 86.215 ])
    SPL_harmonic_2_paper = np.array([18.76,58.33,71.15,78.58,81.29,85.57,88.95,91.66,94.60,94.63,92.85,89.49,85.45,80.52,72.22,53.35])



    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots
    # ----------------------------------------------------------------------------------------------------------------------------------------      

    fig = plt.figure('Test_Case_1_11_p1')
    fig.set_size_inches(16, 6)           

    axes = fig.add_subplot(1,2,1)  
    axes.plot(theta_1h, SPL_harmonic_1_paper  , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label='Hanson Paper')
    axes.plot(theta_1h, SPL_harmonic_1        , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw, label='Hanson')
    axes.plot(exp_theta_1, exp_harmonic_1     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw, label='Experimental') 
    axes.set_ylabel('SPL (dB)')  
    axes.set_xlabel('theta (deg)')
    axes.legend(loc='lower right') 
    axes.set_title('$1^{st}$ Harmonic')    
    plt.ylim((10,110))   

    axes = fig.add_subplot(1,2,2)  
    axes.plot(theta_2h, SPL_harmonic_2_paper  , color = PP.Rlc[1] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label='Hanson Paper')
    axes.plot(theta_2h, SPL_harmonic_2        , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw, label='Hanson')
    axes.plot(exp_theta_2, exp_harmonic_2     , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw, label='Experimental') 
    axes.set_title('$2^{nd}$ Harmonic')    
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('theta (deg)') 
    axes.legend(loc='lower right') 
    plt.ylim((10,110)) 


    # Polar plot of noise    
    fig2 = plt.figure('Test_Case_1_11_p2')
    fig2.set_size_inches(12, 5)   
    axis1 = fig2.add_subplot(121, projection='polar')    
    l1, = axis1.plot(-theta_1h*Units.degrees, SPL_harmonic_1, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)     
    axis1.plot(theta_1h*Units.degrees, SPL_harmonic_1, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)   
    l2, = axis1.plot(-theta_1h*Units.degrees, SPL_harmonic_1_paper  , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw) 
    axis1.plot(theta_1h*Units.degrees, SPL_harmonic_1_paper  , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw )
    l3, = axis1.plot(-exp_theta_1, exp_harmonic_1   , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw )    
    axis1.plot(exp_theta_1, exp_harmonic_1     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw )       
    axis1.set_ylim([0,120])
    axis1.set_yticks(np.arange(50,120,25) )    
    axis1.set_title('$1^{st}$ Harmonic')      
    axis1.grid(True)     
    realign_polar_xticks(axis1)

    axis2 = fig2.add_subplot(122, projection='polar')     
    axis2.plot(-theta_2h*Units.degrees, SPL_harmonic_2, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)   
    axis2.plot(theta_2h*Units.degrees, SPL_harmonic_2, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw) 
    axis2.plot(-theta_2h*Units.degrees, SPL_harmonic_2_paper , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw) 
    axis2.plot(theta_2h*Units.degrees, SPL_harmonic_2_paper  , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw )
    axis2.plot(-exp_theta_2*Units.degrees, exp_harmonic_2    , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw)     
    axis2.plot(exp_theta_2*Units.degrees, exp_harmonic_2   , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw )    
    axis2.set_ylim([0,120])
    axis2.set_title('$2^{nd}$ Harmonic')    
    axis2.set_yticks(np.arange(50,120,25) )     
    axis2.grid(True)      
    realign_polar_xticks(axis2)    
    plt.figlegend((l1,l2,l3),
                  ('SUAVE', 'Hanson','Exp.'),
                  loc=(0.3, 0.), 
                  prop={'size': PP.lf} , 
                  ncol= 3 )  
    plt.tight_layout()    
    return 



def Test_Case_1_14(PP):
    # Define Network
    net                              = Battery_Propeller()
    net.number_of_propeller_engines  = 1
    net.identical_propellers         = True                    
    prop                             = design_F8745D4_prop()  
    net.propellers.append(prop) 

    # Atmosheric & Run Conditions                                               
    a                     = 343.376
    T                     = 286.16889478 
    Helical_Tip_Mach      = 0.86
    Mach                  = 0.203
    Tip_Mach              = np.sqrt(Helical_Tip_Mach**2 - Mach**2)
    density               = 1.2250	
    dynamic_viscosity     = 1.81E-5    
    theta                 = np.linspace(90,105,2)  
    harmonics             = np.arange(1,21)
    S                     = 4.
    ctrl_pts              = 1

    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees),-S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2),-S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Set up for Propeller Model 
    test_omega                                             = a*Tip_Mach/prop.tip_radius  
    prop.inputs.omega                                      = np.ones((ctrl_pts,1)) *test_omega
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    velocity_vector                                        = np.array([[Mach*a, 0. ,0.]])
    conditions.frames.inertial.velocity_vector             = np.tile(velocity_vector,(ctrl_pts,1)) 
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    F, Q, P, Cp , noise_data , etap   = prop.spin(conditions) 
    #plot_propeller_performance(F, Q, P, Cp , noise_data,prop, col1 = 'go-', col2 = 'gs-')

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.array([positions])
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
    conditions.noise.number_of_microphones                 = num_mic 
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data

    # Run Noise Model  
    propeller_noise  = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings)   
    SPL_Hanson       = propeller_noise.SPL_harmonic_bpf_spectrum[0,:,:][:,:len(harmonics)]        

    # PAPER RESULTS 
    # experimental data 
    exp_transducer_4 = np.array([  116.90, 114.813, 116.772,  114.81, 114.029, 112.332, 113.376, 110.89,110.895, 109.850,108.15,107.238,106.97, 103.32, 104.23, 102.40, 99.7947,98.488, 97.835,96.529 ])
    exp_transducer_5 = np.array([  118.4762, 114.4346,116.2694, 116.7982, 109.7525,112.3709,113.8139, 108.9887, 108.9955,107.5651, 107.3103, 104.9664, 103.7975, 103.2816, 100.8071, 99.24647, 97.81637, 95.73291, 93.12784, 93.39586 ])    
    SPL_Hanson_paper_T4  = np.array([116.11960,116.27682,115.89781,115.82501,115.44600,114.76051,113.76867,112.77683,111.55537,110.33377,108.88241,107.12471,105.52031,103.83919,102.15807,100.47722,98.642798,96.732051,94.897759,93.140186])
    SPL_Hanson_paper_T5  = np.array([117.90,117.75,116.91,115.92,114.85,113.55,112.25,110.95,109.66,108.20,106.75,105.30,103.70,102.17,100.49,98.889,97.284,95.450,93.769,92.011])


    # plot results
    fig = plt.figure('Test_Case_1_14')
    fig.set_size_inches(16, 6)   
    axes = fig.add_subplot(1,2,1)  
    axes.plot(harmonics , SPL_Hanson_paper_T4   , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label='Hanson Paper')
    axes.plot(harmonics , SPL_Hanson[0,:]       , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw, label='Hanson')
    axes.plot(harmonics , exp_transducer_4      , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw, label='Experimental')       
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('harmonic ')
    axes.legend(loc='upper right',
                prop={'size': PP.lf}  )

    plt.ylim((80,125)) 
    axes = fig.add_subplot(1,2,2)  
    axes.plot(harmonics , SPL_Hanson_paper_T5  , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label='Hanson Paper')
    axes.plot(harmonics , SPL_Hanson[1,:]      , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw, label='Hanson')
    axes.plot(harmonics , exp_transducer_5     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw, label='Experimental')   
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('harmonic ') 
    axes.legend(loc='upper right',
                prop={'size': PP.lf}  ) 
    plt.ylim((80,125)) 

    return 

def Test_Case_2_3(DJI_CF,APC_SF,APC_SF_inflow_ratio,DJI_CF_inflow_ratio,PP):
    # Define Network
    net_APC_SF                                          = Battery_Propeller()
    net_APC_SF.number_of_propeller_engines              = 1   
    net_APC_SF.identical_propellers                     = True      
    net_APC_SF.propellers.append(APC_SF)               

    # Define Network            
    net_DJI_CF                                         = Battery_Propeller()
    net_DJI_CF.number_of_propeller_engines             = 1  
    net_DJI_CF.identical_propellers                    = True       
    net_DJI_CF.propellers.append(DJI_CF)               

    # Run conditions                                   
    RPM                                                = np.linspace(2000,6000,10)
    omega_vector                                       = RPM * Units.rpm
    ctrl_pts                                           = len(omega_vector)  

    APC_SF_velocity                                    = APC_SF_inflow_ratio*omega_vector*APC_SF.tip_radius  
    DJI_CF_velocity                                    = DJI_CF_inflow_ratio*omega_vector*DJI_CF.tip_radius  
    APC_SF.induced_hover_velocity                      = np.atleast_2d(APC_SF_inflow_ratio*omega_vector*APC_SF.tip_radius).T 
    DJI_CF.induced_hover_velocity                      = np.atleast_2d(DJI_CF_inflow_ratio*omega_vector*DJI_CF.tip_radius).T

    # Atmosheric conditions                            
    a                                                  = 343   
    density                                            = 1.225
    dynamic_viscosity                                  = 1.78899787e-05   
    T                                                  = 286.16889478    

    # Define conditions 
    APC_SF.thrust_angle                                = 90. * Units.degrees
    APC_SF.inputs.omega                                = np.atleast_2d(omega_vector).T
    APC_SF_conditions                                  = Aerodynamics() 
    APC_SF_conditions.freestream.density               = np.ones((ctrl_pts,1)) * density
    APC_SF_conditions.freestream.dynamic_viscosity     = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    APC_SF_conditions.freestream.speed_of_sound        = np.ones((ctrl_pts,1)) * a 
    APC_SF_conditions.freestream.temperature           = np.ones((ctrl_pts,1)) * T
    v_mat                                              = np.zeros((ctrl_pts,3))
    v_mat[:,0]                                         = APC_SF_velocity 
    APC_SF_conditions.frames.inertial.velocity_vector  = v_mat 
    APC_SF_conditions.propulsion.throttle              = np.ones((ctrl_pts,1)) * 1.0 
    APC_SF_conditions.frames.body.transform_to_inertial= np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]]) 
    APC_SF_thrust, APC_SF_torque, APC_SF_power, APC_SF_Cp,  APC_SF_res  , APC_SF_etap  = APC_SF.spin(APC_SF_conditions)  


    DJI_CF.thrust_angle                                = 90. * Units.degrees
    DJI_CF.inputs.omega                                = np.atleast_2d(omega_vector).T
    DJI_CF_conditions                                  = Aerodynamics() 
    DJI_CF_conditions.freestream.density               = np.ones((ctrl_pts,1)) * density
    DJI_CF_conditions.freestream.dynamic_viscosity     = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    DJI_CF_conditions.freestream.speed_of_sound        = np.ones((ctrl_pts,1)) * a 
    DJI_CF_conditions.freestream.temperature           = np.ones((ctrl_pts,1)) * T
    v_mat                                              = np.zeros((ctrl_pts,3))
    v_mat[:,0]                                         = DJI_CF_velocity 
    DJI_CF_conditions.frames.inertial.velocity_vector  = v_mat 
    DJI_CF_conditions.propulsion.throttle              = np.ones((ctrl_pts,1)) * 1.0 
    DJI_CF_conditions.frames.body.transform_to_inertial= np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]]) 
    DJI_CF_thrust, DJI_CF_torque, DJI_CF_power, DJI_CF_Cp,  DJI_CF_res  , DJI_CF_etap  = DJI_CF.spin(DJI_CF_conditions)


    # ----------------------------------------------------------------------------
    #  Experimental Data
    # ---------------------------------------------------------------------------- 
    APC_SF_raw_data_1 = np.array([[1688.3116883116882, 0.012751391465677182 ], [1948.0519480519479, 0.012967532467532468 ],[2272.7272727272725, 0.013219851576994436 ],[2584.415584415585, 0.013507792207792208  ],
                                  [3000, 0.013796474953617813               ],[3337.6623376623374, 0.014048886827458258 ],[3623.3766233766237, 0.014300927643784787 ],[3961.0389610389607, 0.014553339517625233 ],
                           [4324.675324675325, 0.014913079777365494  ],[4740.25974025974, 0.015308905380333953   ],[5142.857142857143, 0.015633209647495365  ],[5441.558441558442, 0.01592105751391466   ]])

    APC_SF_raw_data_2 = np.array([[5.277831101135543  , 0.1494661921708187 ],[7.785177419116668  , 0.20284697508896832],[11.044727632492137 , 0.2722419928825621 ],
                                  [15.3019004444707   , 0.3469750889679717 ],[20.5584678312488   , 0.432384341637011  ],[27.065164424624573 , 0.5338078291814945 ],[35.06740892780674  , 0.6405693950177938 ],
                           [45.068442580588    , 0.7686832740213525 ],[59.305385330990404 , 0.9074733096085414 ],[72.05386807637217  , 1.0622775800711746 ],[91.28601172457583  , 1.2491103202846978 ],
                           [112.50985661759279 , 1.4359430604982206 ]]) 
    DJI_CF_raw_data_1 = np.array([[2979.020979020979, 0.00846376811594203   ],[3293.7062937062938, 0.008753623188405798 ],[3590.909090909091, 0.008898550724637681  ],[3888.111888111888, 0.009091787439613528  ],[4185.3146853146845, 0.009236714975845411 ],
                                  [4482.517482517483, 0.009091787439613528  ],[4779.72027972028, 0.009236714975845411   ],[5094.405594405594, 0.009333333333333334  ],[5391.608391608392, 0.009285024154589374  ],[5706.293706293706, 0.009285024154589374  ],
                                  [6020.979020979021, 0.009429951690821257  ],[6283.216783216783, 0.009333333333333334  ],[6580.41958041958, 0.009429951690821257   ],[6877.622377622378, 0.009333333333333334  ],[7209.79020979021, 0.009333333333333334   ]])

    DJI_CF_raw_data_2 = np.array([[6.418720598287166, 0.1435406698564594 ],[7.77129990752282, 0.17942583732057393 ],[9.460013670539983, 0.2153110047846889 ],[11.150335732379077, 0.2583732057416266],[13.849061155562701, 0.3014354066985647],[16.883921032527837, 0.3444976076555024],
                                  [19.582646455711476, 0.3875598086124401],[22.620722930320454, 0.4449760765550237],[25.994933858710944, 0.5023923444976077],[29.705279240882952, 0.5598086124401913],[34.08950182943991, 0.6244019138755981 ],
                                  [38.13598166539343, 0.6818181818181817 ],[43.86635036789835, 0.7535885167464116 ],[49.59511077158136, 0.8181818181818183 ],[55.9977483816493, 0.8899521531100478  ]])
    # pack data 
    APC_SF_exp_CT    = APC_SF_raw_data_1[:,1]  
    APC_SF_exp_RPM   = APC_SF_raw_data_1[:,0] 
    APC_SF_exp_T     = APC_SF_raw_data_2[:,1]  
    APC_SF_exp_P     = APC_SF_raw_data_2[:,0]  

    DJI_CF_exp_CT    = DJI_CF_raw_data_1[:,1]  
    DJI_CF_exp_RPM   = DJI_CF_raw_data_1[:,0] 
    DJI_CF_exp_T     = DJI_CF_raw_data_2[:,1]  
    DJI_CF_exp_P     = DJI_CF_raw_data_2[:,0]      

    # ----------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------  

    fig1 = plt.figure('Performance')    
    fig1.set_size_inches(12, 6) 
    axes1 = fig1.add_subplot(1,2,1)
    axes1.plot( APC_SF_exp_P , APC_SF_exp_T                , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'APC SF Exp')  
    axes1.plot( DJI_CF_exp_P , DJI_CF_exp_T                , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'DJI SF Exp')  
    axes1.plot(APC_SF_power[:,0],APC_SF_thrust[:,0]* 0.2248, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'APC SF')   
    axes1.plot(DJI_CF_power[:,0],DJI_CF_thrust[:,0]* 0.2248, color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'DJI SF')      
    axes1.set_ylabel('Thrust, lbs.')
    axes1.set_xlabel('Power, W') 
    axes1.legend(loc='upper left',
                 prop={'size': PP.lf}  )

    axes2 = fig1.add_subplot(1,2,2) 
    axes2.plot( APC_SF_exp_RPM , APC_SF_exp_CT                        , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'APC SF Exp')   
    axes2.plot( DJI_CF_exp_RPM , DJI_CF_exp_CT                        , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'DJI SF Exp')     
    axes2.plot(RPM, APC_SF_res.thrust_coefficient[:,0]*(8/(np.pi**3)) , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'APC SF')   
    axes2.plot(RPM, DJI_CF_res.thrust_coefficient[:,0]*(8/(np.pi**3)) , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'DJI SF')     
    axes2.set_xlabel('RPM')
    axes2.set_ylabel('$C_T$') 
    axes2.legend(loc='upper left',
                 prop={'size': PP.lf}  )


    return 

def Test_Case_2_4(DJI_CF,APC_SF,APC_SF_inflow_ratio,DJI_CF_inflow_ratio,PP): 

    # Atmosheric conditions 
    a                     = 343   
    density               = 1.225
    dynamic_viscosity     = 1.78899787e-05   
    T                     = 286.16889478 

    # ---------------------------------------------------------------------------------------------------------------------------
    # APC SF Rotor
    # ---------------------------------------------------------------------------------------------------------------------------
    # Define Network
    net_APC_SF                                  = Battery_Propeller()
    net_APC_SF.number_of_propeller_engines      = 1        
    net_APC_SF.identical_propellers             = True  
    net_APC_SF.propellers.append(APC_SF)    

    # Run conditions                            
    APC_SF_RPM                                  = np.array([3600,4200,4800])
    APC_SF_omega_vector                         = APC_SF_RPM * Units.rpm 
    ctrl_pts                                    = len(APC_SF_omega_vector)   
    velocity                                    = APC_SF_inflow_ratio*APC_SF_omega_vector*APC_SF.tip_radius 
    theta                                       = np.array([45., 67.5, 90.001, 112.5 , 135.]) # np.array([-45., -22.5, 0.001, 22.5 , 45.])*Units.degrees    
    S                                           = 1.905

    # Microphone Locations 
    positions = np.zeros((len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees),-S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2),-S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Define conditions 
    APC_SF.thrust_angle                                            = 0. * Units.degrees
    APC_SF.inputs.omega                                            = np.atleast_2d(APC_SF_omega_vector).T
    APC_SF_conditions                                              = Aerodynamics() 
    APC_SF_conditions.freestream.density                           = np.ones((ctrl_pts,1)) * density
    APC_SF_conditions.freestream.dynamic_viscosity                 = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    APC_SF_conditions.freestream.speed_of_sound                    = np.ones((ctrl_pts,1)) * a 
    APC_SF_conditions.freestream.temperature                       = np.ones((ctrl_pts,1)) * T
    v_mat                                                          = np.zeros((ctrl_pts,3))
    v_mat[:,0]                                                     = velocity 
    APC_SF_conditions.frames.inertial.velocity_vector              = v_mat 
    APC_SF_conditions.propulsion.throttle                          = np.ones((ctrl_pts,1)) * 1.0 
    APC_SF_conditions.frames.body.transform_to_inertial            = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller BEMT new model  
    APC_SF_thrust, APC_SF_torque, APC_SF_power, APC_SF_Cp,  APC_SF_res  , APC_SF_etap  =  APC_SF.spin(APC_SF_conditions)  

    # Prepare Inputs for Noise Model  
    APC_SF_conditions.noise.total_microphone_locations             = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    APC_SF_conditions.aerodynamics.angle_of_attack                 = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    APC_SF_segment                                                 = Segment() 
    APC_SF_segment.state.conditions                                = APC_SF_conditions
    APC_SF_segment.state.conditions.expand_rows(ctrl_pts)
    APC_SF_settings                                                = Data()
    APC_SF_settings                                                = setup_noise_settings(APC_SF_segment)   
    acoustic_outputs                                               = Data()
    acoustic_outputs.propeller                                     = APC_SF_res

    # Run Noise Model    
    APC_SF_propeller_noise             = propeller_mid_fidelity(net_APC_SF.propellers,acoustic_outputs,APC_SF_segment,APC_SF_settings )    
    APC_SF_1_3_Spectrum                = APC_SF_propeller_noise.SPL_1_3_spectrum 
    APC_SF_SPL_broadband_1_3_spectrum  = APC_SF_propeller_noise.SPL_broadband_1_3_spectrum 
    APC_SF_SPL_tonal                   = APC_SF_propeller_noise.SPL_harmonic_bpf_spectrum_dBA

    # ---------------------------------------------------------------------------------------------------------------------------
    # DJI CF Rotor
    # ---------------------------------------------------------------------------------------------------------------------------   
    # Define Network
    net_DJI_CF                              = Battery_Propeller()
    net_DJI_CF.number_of_propeller_engines  = 1    
    net_DJI_CF.identical_propellers         = True       
    net_DJI_CF.propellers.append(DJI_CF)  

    DJI_CF_RPM                                  = np.array([4800,5400,6000])
    DJI_CF_omega_vector                         = DJI_CF_RPM * Units.rpm    
    ctrl_pts                                    = len(DJI_CF_omega_vector) 
    velocity                                    = DJI_CF_inflow_ratio*DJI_CF_omega_vector*DJI_CF.tip_radius  
    DJI_CF.induced_hover_velocity               = np.atleast_2d(DJI_CF_inflow_ratio*DJI_CF_omega_vector*DJI_CF.tip_radius ).T 
    ctrl_pts                                    = len(DJI_CF_RPM) 

    DJI_CF.thrust_angle                                            = 0. * Units.degrees
    DJI_CF.inputs.omega                                            = np.atleast_2d(DJI_CF_omega_vector).T
    DJI_CF_conditions                                              = Aerodynamics() 
    DJI_CF_conditions.freestream.density                           = np.ones((ctrl_pts,1)) * density
    DJI_CF_conditions.freestream.dynamic_viscosity                 = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    DJI_CF_conditions.freestream.speed_of_sound                    = np.ones((ctrl_pts,1)) * a 
    DJI_CF_conditions.freestream.temperature                       = np.ones((ctrl_pts,1)) * T
    v_mat                                                          = np.zeros((ctrl_pts,3))
    v_mat[:,0]                                                     = velocity 
    DJI_CF_conditions.frames.inertial.velocity_vector              = v_mat 
    DJI_CF_conditions.propulsion.throttle                          = np.ones((ctrl_pts,1)) * 1.0 
    DJI_CF_conditions.frames.body.transform_to_inertial            = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]]) 

    # Run Propeller BEMT new model
    DJI_CF_thrust, DJI_CF_torque, DJI_CF_power, DJI_CF_Cp,  DJI_CF_res  , DJI_CF_etap  = DJI_CF.spin(DJI_CF_conditions)

    # Prepare Inputs for Noise Model  
    DJI_CF_conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    DJI_CF_conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    DJI_CF_segment                                                = Segment() 
    DJI_CF_segment.state.conditions                               = DJI_CF_conditions
    DJI_CF_segment.state.conditions.expand_rows(ctrl_pts)
    DJI_CF_settings                                               = Data()
    DJI_CF_settings                                               = setup_noise_settings(DJI_CF_segment)
    num_mic                                                       = len(DJI_CF_conditions.noise.total_microphone_locations[0] )  
    acoustic_outputs                                              = Data()
    acoustic_outputs.propeller                                    = DJI_CF_res    


    ## Run Noise Model   
    #DJI_CF_propeller_noise             = propeller_mid_fidelity(net_DJI_CF.propellers,acoustic_outputs,DJI_CF_segment,DJI_CF_settings )  
    #DJI_CF_1_3_Spectrum                = DJI_CF_propeller_noise.SPL_1_3_spectrum
    #DJI_CF_SPL_tonal                   = DJI_CF_propeller_noise.SPL_harmonic_bpf_spectrum_dBA
    #DJI_CF_SPL_broadband_1_3_spectrum  = DJI_CF_propeller_noise.SPL_broadband_1_3_spectrum 

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------      
    Exp_APC_SF_freqency_spectrum =  np.array([100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 
                                              2000, 2500, 3150,4000, 5000, 6300, 8000, 10000])  


    Exp_DJI_CF_freqency_spectrum = np.array([100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 
                                             2000, 2500, 3150,4000, 5000, 6300, 8000, 10000])  

    Exp_APC_SF_1_3_Spectrum      = np.array([[22.149, 42.242, 24.252, 22.616, 26.121, 24.953, 28.925, 29.158, 39.205
                                              , 42.943, 39.205, 38.971, 47.149, 45.280, 40.373, 38.738, 38.037, 39.906
                                              , 41.308, 45.981, 42.710, 39.205, 41.775, 37.570, 34.065, 33.598 ],
                                             [17.943,46.214,46.214,22.850,27.056,27.990,31.495,31.261,37.336,
                                             42.242,50.186,40.373,45.280,42.476,45.747,43.878,43.878,48.084,
                                             48.317,49.252,49.018,49.018,46.214,42.242,40.140,39.205 ],
                                            [ 19.345, 18.411, 54.859, 24.018, 26.355, 34.065, 33.130, 33.130, 36.635
                                              , 45.981, 45.046, 40.841, 42.710, 43.411, 44.813, 51.588, 45.981, 46.915
                                              , 52.289, 48.551, 50.186, 48.551, 48.551, 48.317, 43.177, 41.308]])  
    Exp_DJI_CF_1_3_Spectrum       = np.array([[25.887,26.822,43.177,17.242,17.710,23.785,18.411,27.990,30.327,
                                               33.130,44.579,36.635,38.037,39.439,40.841,43.878,49.953,45.747,
                                              49.953,51.588,53.457,55.093,58.130,54.626,47.616,40.841 ],
                                              [27.906,16.034,44.163,44.616,24.605,16.452,27.603,28.987,25.953,
                                                  32.685,36.396,45.453,39.395,45.663,40.999,42.849,44.464,45.616,
                                                  47.699,50.709,52.559,54.640,56.256,58.802,58.558,47.151  ],
                                                 [ 37.102,21.214,19.579,50.887,17.710,27.523,24.953,22.850,31.028,34.766,
                                                   46.682,40.373,40.607,41.074,42.943,44.112,45.046,45.514,47.616,
                                              49.953,50.887,53.224,54.859,56.028,56.962,56.028]])  

    Exp_broadband_APC = np.array([[636.3737383699421, 37.72616136919316],[804.042689130632, 39.975550122249395],[1002.588290790458, 40.95354523227385],[1249.705673057358, 39.68215158924206],[1609.6149406056857, 40.4645476772616],[2019.439730751808, 39.29095354523228],
                                  [2501.431663210214, 39.29095354523228],[3178.441800310014, 36.45476772616137],[4041.1827320586362, 37.4327628361858],[5039.568272636746, 38.99755501222494],
                                  [6324.901337187314, 39.97555012224939],[7988.190834237982, 39.7799511002445],[10086.004006369305, 37.8239608801956],[12653.002728989723, 36.1613691931540],
                                  [15992.845989269103, 40.7579462102689],[20052.067840706222, 35.6723716381418],[25309.998789335245, 31.7603911980440],[31760.73622497443, 31.85819070904646]])

    Exp_angles                   = np.array([45., 22.5, 0.001, -22.5 , -45.]) 

    Exp_APC_SF_SPL               = np.array([[40.93,45.10,47.29,46.77,41.87 ],
                                             [43.12,48.43,50.93,51.87,48.95 ],
                                            [ 49.37,53.75,56.56,57.08,54.37]]) 

    Exp_DJI_CF_SPL               = np.array([[36.9568,42.7628,45.5689,45.7886,43.1123 ],
                                             [ 41.3019,46.2802,49.4996,50.0299,47.7672],
                                            [44.4052,50.3149,52.9134,53.1333,50.8706 ]]) 
    test_angles = np.array([45, 22.5,0,-22.5,-45])
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------   

    # Figures 8 and 9 comparison 
    fig1 = plt.figure('APC_SF_Rotor')    
    fig1.set_size_inches(8, 7) 
    axes1 = fig1.add_subplot(1,1,1)
    axes1.plot(Exp_APC_SF_freqency_spectrum , Exp_APC_SF_1_3_Spectrum[0,:-5]  , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'Exp. 3600 RPM')        
    axes1.plot(Exp_APC_SF_freqency_spectrum , Exp_APC_SF_1_3_Spectrum[1,:-5]  , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp. 4200 RPM')       
    #axes1.plot(Exp_APC_SF_freqency_spectrum , Exp_APC_SF_1_3_Spectrum[2,:-5]  , color = PP.Elc[2] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp. 4800 RPM')          
    axes1.plot(Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[0,0,8:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label =' SUAVE 3600 RPM')   
    axes1.plot(Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[1,0,8:]  , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,label = ' SUAVE 4200 RPM')   
    #axes1.plot(Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[2,0,8:]  , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,label = ' SUAVE 4800 RPM')   
    axes1.set_xscale('log')
    #axes1.set_title('APC SF')
    axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes1.set_xlabel('Frequency (Hz)') 
    axes1.legend(loc='lower right') 
    plt.ylim((15,55)) 

    # Figures 10 and 11 comparison 
    fig2 = plt.figure('APC_SF_Rotor_Broadband')    
    fig2.set_size_inches(8, 7) 
    axes1 = fig2.add_subplot(1,1,1)            
    axes1.plot(Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[0,4,8:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE -45 deg')      
    axes1.plot(Exp_broadband_APC[:,0], Exp_broadband_APC[:,1], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,    label = 'Exp -45 deg')     
    axes1.set_xscale('log')
    axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes1.set_xlabel('Frequency (Hz)') 
    axes1.legend(loc='lower right') 
    plt.ylim((20,45)) 

    # Figures 16 a Comparison  
    fig3 = plt.figure('Test_Case_2_4_p3')    
    fig3.set_size_inches(12, 6) 
    axes1 = fig3.add_subplot(1,2,1)    
    axes1.plot( Exp_APC_SF_SPL[0,:]     ,Exp_angles    , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,   label = 'Exp. 3600 RPM')  
    axes1.plot( Exp_APC_SF_SPL[1,:]     ,Exp_angles    , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,   label = 'Exp. 4200 RPM')  
    axes1.plot( Exp_APC_SF_SPL[2,:]     ,Exp_angles    , color = PP.Elc[2] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,   label = 'Exp. 4800 RPM')     
    axes1.plot( APC_SF_SPL_tonal[0,:,0]   ,test_angles  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE 3600 RPM')  
    axes1.plot( APC_SF_SPL_tonal[1,:,0]   ,test_angles  , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE 4200 RPM')  
    axes1.plot( APC_SF_SPL_tonal[2,:,0]   ,test_angles  , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE 4800 RPM')        
    axes1.set_xlabel('B.P.F. SPL (dB)')
    axes1.set_ylabel('Microphone Angle (.deg)') 
    axes1.legend(loc='upper left') 
    plt.ylim((-50,90)) 
    plt.xlim((30,60)) 

    axes2 = fig3.add_subplot(1,2,2)             
    axes2.plot(Exp_APC_SF_freqency_spectrum , Exp_APC_SF_1_3_Spectrum[1,:-5]  , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp. 4200 RPM')    
    axes2.plot(Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[1,0,8:]  , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,label = ' SUAVE 4200 RPM')    
    axes2.set_xscale('log') 
    axes2.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes2.set_xlabel('Frequency (Hz)') 
    axes2.legend(loc='lower right') 
    plt.ylim((10,60))   

    fig4 = plt.figure('Test_Case_2_4_p4')    
    fig4.set_size_inches(12, 6)  
    axes1 = fig4.add_subplot(1,2,1)            
    axes1.plot(Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[1,4,8:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE -45 deg')   
    axes1.plot(Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[1,3,8:] , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE -22.5 deg')    
    axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes1.set_xlabel('Frequency (Hz)') 
    axes1.legend(loc='upper left') 
    plt.ylim((20,50)) 

    axes1 = fig4.add_subplot(1,2,2)      
    axes1.plot( Exp_APC_SF_SPL[1,:]     ,Exp_angles    , color = PP.Elc[1] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,   label = 'Exp. 4200 RPM')   
    axes1.plot( APC_SF_SPL_tonal[1,:,0]   ,test_angles  , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE 4200 RPM')         
    axes1.set_xlabel('B.P.F. SPL (dB)')
    axes1.set_ylabel('Microphone Angle (.deg)') 
    axes1.legend(loc='upper left') 
    plt.ylim((-50,90)) 
    plt.xlim((30,60))   


    return 

def Test_Case_3_1(PP):    
    # Define Network
    net                                = Battery_Propeller()
    net.number_of_propeller_engines    = 1                                      
    prop                               = design_F8745D4_prop()  
    net.identical_propellers           = True  
    net.propellers.append(prop)  

    # Atmosheric & Run Conditions                                               
    a                       = 343.376
    T                       = 288.16889478  
    density                 = 1.2250	
    dynamic_viscosity       = 1.81E-5 

    theta                   = np.array([1,10,20,30.1,40,50,59.9,70,80,89.9,100,110,120.1,130,140,150.1,160,170,179])  
    S                       = 4.
    test_omega              = np.array([2390,2710,2630]) * Units.rpm    
    ctrl_pts                = len(test_omega)

    # Set twist target
    three_quarter_twist     = 21 * Units.degrees 
    n                       = len(prop.twist_distribution)
    beta                    = prop.twist_distribution
    beta_75                 = beta[round(n*0.75)] 
    delta_beta              = three_quarter_twist-beta_75
    prop.twist_distribution = beta + delta_beta 

    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees)  ,S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2)  ,S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Set up for Propeller Model
    prop.inputs.omega                                      = np.atleast_2d(test_omega).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector             = np.array([[77.2, 0. ,0.],[ 77.0,0.,0.], [ 77.2, 0. ,0.]])
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    F, Q, P, Cp , noise_data , etap                        = prop.spin(conditions) 

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts)
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment)

    num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
    conditions.noise.number_of_microphones                 = num_mic  
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data

    # Run Noise Model  
    propeller_noise                       = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings )     
    F8745D4_SPL                           = propeller_noise.SPL     
    F8745D4_SPL_harmonic                  = propeller_noise.SPL_harmonic 
    F8745D4_SPL_broadband                 = propeller_noise.SPL_broadband   
    F8745D4_SPL_harmonic_bpf_spectrum     = propeller_noise.SPL_harmonic_bpf_spectrum      
    F8745D4_SPL_1_3_spectrum              = propeller_noise.SPL_1_3_spectrum         
    F8745D4_SPL_harmonic_1_3_spectrum     = propeller_noise.SPL_harmonic_1_3_spectrum      
    F8745D4_SPL_broadband_1_3_spectrum    = propeller_noise.SPL_broadband_1_3_spectrum        
    F8745D4_one_third_frequency_spectrum  = propeller_noise.one_third_frequency_spectrum     

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------      

    harmonics              = np.arange(1,19)
    ANOPP_PAS_Case_1_60deg = np.array([105.82,101.08,100.13,97.581,94.035,89.095,82.957,
                                       80.609,81.052,72.718,70.772,68.023,67.072,53.949,
                                       np.inf,np.inf,np.inf,np.inf]) 

    Exp_Test_Case_1_60deg  = np.array([103.23,100.08,98.733,94.990,91.045,87.500,82.161,79.012,
                                       74.469,70.128,65.784,61.241,56.699,51.958,47.013,42.673,
                                       37.927,32.989,]) 

    ANOPP_PAS_Case_1_90deg = np.array([107.671,104.755,105.829,103.307,101.385,100.465,99.1399,
                                       96.8208,93.6988,91.7765,89.6573,86.5323,85.2098,83.4874,
                                       78.1692,75.4503,73.7248,72.0024]) 

    Exp_Test_Case_1_90deg  = np.array([108.077,107.554,105.626,103.307,100.988,100.068,98.9430,
                                       96.8208,93.6988,91.7796,89.6542,85.5295,85.0099,81.8879,
                                       77.9724,74.8566,73.1250,71.2057  ])    

    Exp_Test_Case_2_60deg = np.array([111.951,108.175,108.789,106.352,105.059,100.140,100.945,
                                      99.8430,93.9683,93.8203,91.1914,85.3167,85.3626,82.1580,
                                       78.1933,75.7552,80.1887,72.2133])

    ANOPP_PAS_Case_2_60deg = np.array([111.760,111.421,108.984,106.352,104.487,101.856,99.0369,
                                       95.4522,93.2050,89.4327,86.2313,82.6498,79.2559,75.4804,
                                       71.1356,68.1219,63.9663,60.0000]) 


    Exp_Test_Case_2_90deg  = np.array([115.587,113.363,115.520,113.868,113.365,111.331,
                                       113.491,110.507,109.999,109.873,107.649,106.949,
                                       106.822,103.079,103.715,102.633,99.6502,97.8095])

    ANOPP_PAS_Case_2_90deg = np.array([115.397,115.273,114.377,113.870,113.362,111.143,
                                       110.631,109.752,108.859,107.585,109.175,105.234,
                                       103.782,102.127,101.236,99.7790,98.7002,98.9523  ])  

    Exp_Test_Case_3_60deg  = np.array([110.93,107.28,108.60,106.28,104.17,99.377,
                                       100.69,100.28,95.688,95.094,92.975,84.365,
                                       84.533,82.224,77.622,77.411,78.152,74.312])
    ANOPP_PAS_Case_3_60deg = np.array([110.93,110.53,108.41,107.43,104.55,101.47,
                                       98.592,95.328,92.635,88.987,86.103,83.028,
                                       79.573,76.114,73.040,69.775,65.554,61.908 ])

    Exp_Test_Case_3_90deg  = np.array([114.499,112.135,114.674,112.898,112.299,111.308,
                                       112.473,110.894,109.510,109.303,107.724,107.124,
                                       106.133,102.790,103.758,101.983,99.2279,98.0404])

    ANOPP_PAS_Case_3_90deg = np.array([114.499,114.291,113.889,113.879,111.122,110.523,
                                       109.924,109.129,108.725,107.342,106.743,105.164,
                                       104.369,102.593,101.210,100.021,98.6401,96.6674])


    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots
    # ----------------------------------------------------------------------------------------------------------------------------------------    
    # plot results
    fig31 = plt.figure('Test_Case_3_p1')
    fig31.set_size_inches(16, 5)   
    axes = fig31.add_subplot(1,3,1) 
    axes.plot(harmonics, F8745D4_SPL_harmonic_bpf_spectrum[0,6,:][:len(harmonics)]   , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'SUAVE')    
    axes.plot(harmonics, ANOPP_PAS_Case_1_60deg                                      , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label = 'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_1_60deg                                       , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')    
    axes.set_title('Case 1, $C_P$ = ' + str(round(Cp[0,0],3)))
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('Harmonic #') 
    axes.minorticks_on() 
    #plt.ylim((80,125))      

    # Test Case 2
    axes = fig31.add_subplot(1,3,2) 
    axes.plot(harmonics, F8745D4_SPL_harmonic_bpf_spectrum[1,6,:][:len(harmonics)] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,   label = 'SUAVE')    
    axes.plot(harmonics, ANOPP_PAS_Case_2_60deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_2_60deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,   label = 'Exp.')  
    #axes.set_ylabel('SPL (dB)') 
    axes.set_title('Case 2, $C_P$ = ' +  str(round(Cp[1,0],3)))  
    axes.set_xlabel('Harmonic #') 
    axes.minorticks_on()  
    axes.legend(loc='upper center', prop={'size':  PP.lf} , bbox_to_anchor=(0.5, -0.2), ncol= 3 )  

    # Test Case 3
    axes = fig31.add_subplot(1,3,3) 
    axes.plot(harmonics, F8745D4_SPL_harmonic_bpf_spectrum[2,6,:][:len(harmonics)] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,   label = 'SUAVE')    
    axes.plot(harmonics, ANOPP_PAS_Case_3_60deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_3_60deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')        
    axes.set_title('Case 3, $C_P$ = ' +  str(round(Cp[2,0],3)))  
    axes.set_xlabel('Harmonic #')  
    axes.minorticks_on() 
    plt.tight_layout()

    fig32 = plt.figure('Test_Case_3_p2') 
    fig32.set_size_inches(16, 5)       
    axes = fig32.add_subplot(1,3,1)    
    axes.plot(harmonics, F8745D4_SPL_harmonic_bpf_spectrum[0,9,:][:len(harmonics)] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE')    
    axes.plot(harmonics, ANOPP_PAS_Case_1_90deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_1_90deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')       
    axes.set_title('Case 1, $C_P$ = ' + str(round(Cp[0,0],3)))
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('Harmonic #')   
    axes.minorticks_on() 

    axes = fig32.add_subplot(1,3,2)              
    axes.plot(harmonics, F8745D4_SPL_harmonic_bpf_spectrum[1,9,:][:len(harmonics)]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'SUAVE')    
    axes.plot(harmonics, ANOPP_PAS_Case_2_90deg                                     , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw, label = 'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_2_90deg                                      , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'Exp.')   
    axes.set_title('Case 2, $C_P$ = ' +  str(round(Cp[1,0],3)))   
    axes.set_xlabel('Harmonic #')  
    axes.legend(loc='upper center', prop={'size': PP.lf} , bbox_to_anchor=(0.5, -0.2), ncol= 3 )  
    axes.minorticks_on() 

    axes = fig32.add_subplot(1,3,3)    
    axes.plot(harmonics, F8745D4_SPL_harmonic_bpf_spectrum[2,9,:][:len(harmonics)] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,   label = 'SUAVE')    
    axes.plot(harmonics, ANOPP_PAS_Case_3_90deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_3_90deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')     
    axes.set_title('Case 3, $C_P$ = ' +  str(round(Cp[2,0],3)))     
    axes.set_xlabel('Harmonic #')  
    axes.minorticks_on() 
    plt.tight_layout()

    # Polar plot of noise   
    fig33 = plt.figure('Test_Case_3_p3')
    axis = fig33.add_subplot(111, projection='polar')    
    axis.plot(theta*Units.degrees,F8745D4_SPL[0,:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'Total'  )  
    axis.plot(-theta*Units.degrees,F8745D4_SPL[0,:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw  )  
    axis.plot(theta*Units.degrees,F8745D4_SPL_harmonic[0,:] , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw   )  
    axis.plot(-theta*Units.degrees,F8745D4_SPL_harmonic[0,:] , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'Harmonic'  )  
    axis.plot(theta*Units.degrees,F8745D4_SPL_broadband[0,:] , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw   )  
    axis.plot(-theta*Units.degrees,F8745D4_SPL_broadband[0,:] , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'Broadband' )     
    axis.set_yticks(np.arange(50,150,25))     
    axis.grid(True)     


    fig34 = plt.figure('Test_Case_3_p4') 
    fig34.set_size_inches(10, 8)       
    axes = fig34.add_subplot(1,1,1)    
    axes.semilogx(F8745D4_one_third_frequency_spectrum,F8745D4_SPL_1_3_spectrum[0,6,:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'Total SPL @ 60 deg')    
    axes.semilogx(F8745D4_one_third_frequency_spectrum,F8745D4_SPL_harmonic_1_3_spectrum[0,6,:], color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label = 'Harmonic SPL @ 60 deg' )       
    axes.semilogx(F8745D4_one_third_frequency_spectrum,F8745D4_SPL_broadband_1_3_spectrum[0,6,:]  , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'Broadband SPL @ 60 deg')    
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('Frequency') 
    plt.ylim((65,105)) 
    axes.legend(loc='lower center', prop={'size': PP.lf}) 
    axes.minorticks_on()     

    return    

def Test_Case_4_1(PP): 
    # Define Network
    net                              = Battery_Propeller()
    net.number_of_propeller_engines  = 1                                      
    prop                             = design_SR2_8_blade_prop()     
    net.identical_propellers         = True  
    net.propellers.append(prop) 

    # Atmosheric & Run Conditions                                               
    a                                                      = 340.376
    T                                                      = 280.16889478  
    density                                                = 1.2	
    dynamic_viscosity                                      = 1.81E-5  
    theta                                                  = np.array([46.8,50,58.5,72.2,80,90.9,100,104,110,116.8,120,130.4])  
    harmonics                                              = np.arange(1,13) 
    Mach                                                   = 0.6
    velocity                                               = Mach * a  
    J                                                      = 2.7 # 3.06
    n                                                      = velocity /(2*prop.tip_radius  *J)
    X                                                      = np.array([-46.7,-41.7, -30.5, - 16., -8.9 , 0.8, 8.9, 12.4, 18.0, 25.0, 28.7, 42.4]) /100
    test_omega                                             = n*2*np.pi
    ctrl_pts                                               = 1

    # Set twist target
    three_quarter_twist     = 59 * Units.degrees 
    n                       = len(prop.twist_distribution)
    beta                    = prop.twist_distribution
    beta_75                 = beta[round(n*0.75)] 
    delta_beta              = three_quarter_twist-beta_75
    prop.twist_distribution = beta +  delta_beta  

    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if X[i] < 0:
            positions[i][:] = [-X[i], X[i]*np.tan(theta[i]*Units.degrees)  , 0.0]
        else: 
            positions[i][:] = [-X[i], -X[i]/np.tan(theta[i]*Units.degrees - np.pi/2)  , 0.0] 


    # Set up for Propeller Model
    prop.inputs.omega                                      = np.ones((ctrl_pts,1)) *test_omega
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    velocity_vector                                        = np.array([[velocity, 0. ,0.]])
    conditions.frames.inertial.velocity_vector             = np.tile(velocity_vector,(ctrl_pts,1)) 
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    F, Q, P, Cp , noise_data , etap                        = prop.spin(conditions) 
    plot_propeller_performance(F, Q, P, Cp , noise_data,prop, col1 = 'go-', col2 = 'gs-') 

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations                  = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    #conditions.noise.sources.propeller                     = noise_data    
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts)
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
    conditions.noise.number_of_microphones                 = num_mic
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data

    # Run Noise Model  
    propeller_noise  = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings )   
    SPL_harmonic_bpf_spectrum  = propeller_noise.SPL_harmonic_bpf_spectrum[0,:][:len(harmonics)] 


    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------      
    Exp_Harmonic = np.zeros((12,8)) 
    # J = 2.75, M = 0.65 
    #Exp_Harmonic[:,0] = np.array([np.inf,152.0, 151.5,158.5,163.0,163.0,157.5,154.5,152.5, 154.0, 153.5, 150.0])
    #Exp_Harmonic[:,1] = np.array([np.inf,139.0, 141.0,149.0,150.5,157.5,155.0,151.0,139.5,143.0 ,141.0 ,137.5,])
    #Exp_Harmonic[:,2] = np.array([np.inf,138.5, 137.5,145.5,153.0,147.5,136.0,139.0,139.5,138.5 ,130.0 ,143.5 ])

    # J = 3.06, M -= 0.6 
    Exp_Harmonic[:,0] = np.array([np.inf,131.5,136.5,140.5,144.0,145.0, 143.5,142.0, 139.5, 136.0, 133.5,134.0 ])
    Exp_Harmonic[:,1] = np.array([np.inf,np.inf,np.inf,127.5,132.0,134.0, 129.5,125.0, 122.0,124.5, 125.0, 127.5 ])    

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots
    # ----------------------------------------------------------------------------------------------------------------------------------------      

    # plot results
    fig = plt.figure('Test_Case_4')
    fig.set_size_inches(8, 6)   
    axes = fig.add_subplot(1,1,1) 
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel(r'harmonics')  
    l1 = ['k-','b-','r-','g-','m-']
    l2 = ['ko','bs','r^','gP','mY']
    for i in range(2):
        axes.plot(harmonics, SPL_harmonic_bpf_spectrum[:,i]  , color = PP.Slc[i] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'Harminic' + str(i+1) + ' Pred')    
        axes.plot(harmonics, Exp_Harmonic[:,i]    , color = PP.Elc[i] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Harminic' + str(i+1) + ' Exp')  
        axes.set_ylim(100,150)         
    axes.legend(loc='lower center', ncol = 2, prop={'size': PP.lf})        
    return 



def Test_Case_5_1(PP):

    '''Empirical wall-pressure spectral modeling for zero and adverse pressure gradient flows''' 

    # ****** DEFINE INPUTS ****** 

    pi           = np.pi 
    rho          = 1.2                                               #  CHECKED 
    c_0          = np.array([[343.]])  # speed of sound             #  CHECKED 
    kine_visc    = np.array([[0.0000150]])                       #  CHECKED  
    num_sec      = 1 # number of sections  
    N_r          = 1
    ctrl_pts     = 1
    BSR          = 100 # broadband spectrum resolution
    frequency    = np.linspace(1E2,1E4,BSR) 
    w            = 2*pi*frequency
    Re           = np.array([[1.5E6]])                                              #  CHECKED 
    V_inf        = 69.5
    chord        = Re*kine_visc/V_inf 
    alpha        = np.array([[6.]]) *Units.degrees                                            #  CHECKED     

    kine_visc    =  vectorize(kine_visc,ctrl_pts,num_sec,N_r,BSR,method = 1)

    delta        = np.zeros((ctrl_pts,N_r,num_sec,BSR,2)) #  control points ,  number rotors, number blades , number sections , sides of airfoil   
    delta_star   = np.zeros_like(delta)
    dp_dx        = np.zeros_like(delta)
    C_f          = np.zeros_like(delta)
    tau_w        = np.zeros_like(delta)
    Ue           = np.zeros_like(delta)
    Theta        = np.zeros_like(delta)

    # ------------------------------------------------------------
    # ****** TRAILING EDGE BOUNDARY LAYER PROPERTY CALCULATIONS  ******  
    npanel                  = 50
    Re_batch                = np.atleast_2d(np.ones(num_sec)*Re[0,0]).T
    AoA_batch               = np.atleast_2d(np.ones(num_sec)*alpha[0,0]).T       
    airfoil_geometry        = compute_naca_4series(0.0,0.0,0.12,npoints=npanel) 
    airfoil_stations        = [0] * num_sec
    AP                      = airfoil_analysis(airfoil_geometry,AoA_batch,Re_batch, npanel, batch_analysis = False, airfoil_stations = airfoil_stations)     

    TE_idx                  = -4 
    delta[:,:,:,:,0]        = vectorize(AP.delta[:,TE_idx]                         ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED                
    delta[:,:,:,:,1]        = vectorize(AP.delta[:,-TE_idx]                        ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED   
    #delta_star[:,:,:,:,0]   = vectorize(AP.delta_star[:,TE_idx]                    ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED                     
    #delta_star[:,:,:,:,1]   = vectorize(AP.delta_star[:,-TE_idx]                   ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED  
    #surface_dcp_dx          = (np.diff(AP.Cp*0.5*rho*(V_inf**2),axis = 1)/(np.diff(AP.x,axis = 1)*chord)) 
    #dp_dx[:,:,:,:,0]        = vectorize(abs(surface_dcp_dx[:,TE_idx]),ctrl_pts,num_sec,N_r,BSR,method = 2)                         
    #dp_dx[:,:,:,:,1]        = vectorize(abs(surface_dcp_dx[:,-TE_idx]),ctrl_pts,num_sec,N_r,BSR,method = 2)                    
    #C_f[:,:,:,:,0]          = vectorize(AP.Cf[:,TE_idx]                            ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED                        
    #C_f[:,:,:,:,1]          = vectorize(AP.Cf[:,-TE_idx]                           ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED       
    #Ue[:,:,:,:,0]           = vectorize(AP.Ue_Vinf[:,TE_idx]*V_inf                 ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED                     
    #Ue[:,:,:,:,1]           = vectorize(AP.Ue_Vinf[:,-TE_idx]*V_inf                ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED                                 
    #Theta[:,:,:,:,0]        = vectorize(AP.theta[:,TE_idx]                         ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED                    
    #Theta[:,:,:,:,1]        = vectorize(AP.theta[:,-TE_idx]                        ,ctrl_pts,num_sec,N_r,BSR,method = 2) # CHECKED 

    #delta      = Theta*(3.15 + (1.72/((delta_star/Theta)- 1))) + delta_star
    # lower surface is 0, upper surface is 1 
    delta[:,:,:,:,0]        = vectorize(np.array([0.0142]),ctrl_pts,num_sec,N_r,BSR,method = 2)             
    delta[:,:,:,:,1]        = vectorize(np.array([0.0142]),ctrl_pts,num_sec,N_r,BSR,method = 2)                       
    delta_star[:,:,:,:,0]   = vectorize(np.array([0.00236]),ctrl_pts,num_sec,N_r,BSR,method = 2)                      
    delta_star[:,:,:,:,1]   = vectorize(np.array([0.00236]),ctrl_pts,num_sec,N_r,BSR,method = 2)                          
    dp_dx[:,:,:,:,0]        = vectorize(np.array([12140]),ctrl_pts,num_sec,N_r,BSR,method = 2)                         
    dp_dx[:,:,:,:,1]        = vectorize(np.array([12140]),ctrl_pts,num_sec,N_r,BSR,method = 2)                    
    C_f[:,:,:,:,0]          = vectorize(np.array([0.00217]),ctrl_pts,num_sec,N_r,BSR,method = 2)                        
    C_f[:,:,:,:,1]          = vectorize(np.array([0.00217]),ctrl_pts,num_sec,N_r,BSR,method = 2)       
    Ue[:,:,:,:,0]           = vectorize(np.array([64.6]),ctrl_pts,num_sec,N_r,BSR,method = 2)                     
    Ue[:,:,:,:,1]           = vectorize(np.array([64.6]),ctrl_pts,num_sec,N_r,BSR,method = 2)                                 
    Theta[:,:,:,:,0]        = vectorize(np.array([0.00157]),ctrl_pts,num_sec,N_r,BSR,method = 2)                    
    Theta[:,:,:,:,1]        = vectorize(np.array([0.00157]),ctrl_pts,num_sec,N_r,BSR,method = 2)                    


    tau_w = C_f*(0.5*rho*(Ue**2))
    # ------------------------------------------------------------
    # ****** BLADE MOTION CALCULATIONS ******  
    omega   = vectorize(w,ctrl_pts,num_sec,N_r,BSR,method = 3)                                                

    # ------------------------------------------------------------
    # ****** EMPIRICAL WALL PRESSURE SPECTRUM ******  
    # equation 8 
    mu_tau              = (tau_w/rho)**0.5 
    ones                = np.ones_like(mu_tau)  
    R_T                 = (delta/Ue)/(kine_visc/(mu_tau**2))       
    beta_c              =  (Theta/tau_w)*dp_dx                                                    
    Delta               = delta/delta_star    
    e                   = 3.7 + 1.5*beta_c             
    d                   =  4.76*((1.4/Delta)**0.75)*(0.375*e - 1)                            
    PI                  = 0.8*((beta_c + 0.5)**3/4)                        
    a                   = (2.82*(Delta**2)*((6.13*(Delta**(-0.75)) + d)**e))*(4.2*(PI/Delta) + 1)   
    h_star              = np.minimum(3*ones,(0.139 + 3.1043*beta_c)) + 7  
    d_star              = d   
    d_star[beta_c<0.5]  = np.maximum(ones,1.5*d)[beta_c<0.5] 
    expression_F        = (omega*delta_star/Ue)     
    expression_C        = np.maximum(a, (0.25*beta_c - 0.52)*a)*(expression_F**2) 
    expression_D        = (4.76*(expression_F**0.75) + d_star)**e                                
    expression_E        = (8.8*(R_T**(-0.57))*expression_F)**h_star                              
    Phi_pp_expression   =  expression_C/( expression_D + expression_E)                           
    Phi_pp              = ((tau_w**2)*delta_star*Phi_pp_expression)/Ue      
    var = 10*np.log10((Phi_pp)/((2E-5)**2))  


    # Rosenberg

    delta = 0.0142
    delta_star = 0.00236
    Theta = 0.00157
    Cf = 0.00217
    dp_dx = 12140
    nu = 0.0000150
    rho = 1.2
    Ue  = 64.6
    beta_c = 3.51
    tau_w = Cf*(0.5*rho*(Ue**2))
    mu_tau    = (tau_w/rho)**0.5 
    R_T  = (delta/Ue)/(nu/(mu_tau**2)) 
    Delta = delta/delta_star


    PI = 0.8*((beta_c + 0.5)**3/4)
    SS = Ue/((tau_w**2)*delta_star)
    FS = delta_star/Ue
    b = 2
    c = 0.75
    e = 3.7 + 1.5*beta_c
    d = 4.76*((1.4/Delta)**0.75)*(0.375*e - 1)
    f = 8.8
    g = -0.57 
    a = (2.82*(Delta**2)*((6.13*(Delta**(-0.75)) + d)**e))*(4.2*(PI/Delta) + 1)
    criteria_b = (19/np.sqrt(R_T))
    h = np.minimum(3,criteria_b) + 7
    i = 4.76

    phi_ros = (a*(w*FS)**b)/(( i*(w*FS)**c  + d)**e + ((f*R_T**g)*(w*FS))**h  ) 
    var2 = 10*np.log10((phi_ros/SS)/((2E-5)**2)) 


    fig  = plt.figure() 
    axis = fig.add_subplot(1,1,1)      
    axis.set_ylim(50,90)
    axis.set_ylabel('10log10($\Phi$)') 
    axis.set_xlabel('Frequency (Hz)')  
    axis.semilogx(frequency,var[0,0,0,:,0],'ks-')  
    axis.semilogx(frequency,var2,'r-')  

    return 


def Test_Case_5_2(PP):

    ''' Validation of airfoil trailing-edge noise prediction against measurement Figure 4 Li and Lee (2019)
    Reference : Prediction of rotorcraft broadband trailing-edge noise and parameter sensitivity study 
    Experimental Data: Broadband trailing-edge noise predictions— overview of BANC-III results ''' 

    # Atmosheric conditions 
    a                     = 343   
    density               = 1.225
    dynamic_viscosity     = 1.78899787e-05   
    T                     = 286.16889478 


    BO_105_advance_ratios                       = np.array([0.15])  #  np.array([0.01,0.086,0.173]) 
    BO_105_RPM                                  = np.array([1040]) #  np.array([1050,1050,525])
    thrust_angle                                = np.array([84.7])*Units.degrees 


    BO_105_freqency_spectrum              = np.zeros((3,29))
    BO_105_SPL_broadband_1_3_spectrum     = np.zeros((3,29))
    C_T                                   = np.zeros(3)

    # ---------------------------------------------------------------------------------------------------------------------------
    # APC SF Rotor
    # ---------------------------------------------------------------------------------------------------------------------------
    # Define Network
    net_BO_105                                  = Battery_Propeller()
    net_BO_105.number_of_propeller_engines      = 1        
    net_BO_105.identical_propellers             = True  
    BO_105                                      = design_BO_105_prop()
    net_BO_105.propellers.append(BO_105)    

    for i in range(1):

        # Run conditions                            
        BO_105_omega_vector                         = BO_105_RPM[i]*Units.rpm
        ctrl_pts                                    = 1
        velocity                                    = BO_105_advance_ratios[i]*BO_105.tip_radius*BO_105_omega_vector
        theta                                       = np.array([179.9]) 
        S                                           = 7.14

        # Microphone Locations 
        positions      = np.zeros((len(theta),3))
        positions[:,0] = -2.0
        positions[:,1] = 1.6
        positions[:,1] = -2.3

        # Define conditions  
        BO_105.inputs.omega                                            = np.atleast_2d(BO_105_omega_vector).T
        BO_105.inputs.y_axis_rotation                                  = thrust_angle[i]
        BO_105_conditions                                              = Aerodynamics() 
        BO_105_conditions.freestream.density                           = np.ones((ctrl_pts,1)) * density
        BO_105_conditions.freestream.dynamic_viscosity                 = np.ones((ctrl_pts,1)) * dynamic_viscosity   
        BO_105_conditions.freestream.speed_of_sound                    = np.ones((ctrl_pts,1)) * a 
        BO_105_conditions.freestream.temperature                       = np.ones((ctrl_pts,1)) * T
        v_mat                                                          = np.zeros((ctrl_pts,3)) 
        v_mat[:,0]                                                     = velocity 
        BO_105_conditions.frames.inertial.velocity_vector              = v_mat 
        BO_105_conditions.propulsion.throttle                          = np.ones((ctrl_pts,1)) * 1.0 
        BO_105_conditions.frames.body.transform_to_inertial            = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

        # Run Propeller BEMT new model  
        BO_105_thrust, BO_105_torque, BO_105_power, BO_105_Cp,  BO_105_res  , BO_105_etap  =  BO_105.spin(BO_105_conditions)  

        # Prepare Inputs for Noise Model  
        BO_105_conditions.noise.total_microphone_locations             = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
        BO_105_conditions.aerodynamics.angle_of_attack                 = np.ones((ctrl_pts,1))* 0. * Units.degrees 
        BO_105_segment                                                 = Segment() 
        BO_105_segment.state.conditions                                = BO_105_conditions
        BO_105_segment.state.conditions.expand_rows(ctrl_pts)
        BO_105_settings                                                = Data()
        BO_105_settings                                                = setup_noise_settings(BO_105_segment)   
        acoustic_outputs                                               = Data()
        acoustic_outputs.propeller                                     = BO_105_res

        C_T[i] = BO_105_res.thrust_coefficient[:,0]*(8/(np.pi**3))

        # Run Noise Model    
        BO_105_propeller_noise                = propeller_mid_fidelity(net_BO_105.propellers,acoustic_outputs,BO_105_segment,BO_105_settings )     
        BO_105_freqency_spectrum[i]           = BO_105_propeller_noise.one_third_frequency_spectrum
        BO_105_SPL_broadband_1_3_spectrum[i]  = BO_105_propeller_noise.SPL_broadband_1_3_spectrum   

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------   


    '''DFigure 14 a'''
    fig1 = plt.figure('Case_1')    
    fig1.set_size_inches(8, 6) 
    axes1 = fig1.add_subplot(1,1,1)            
    axes1.plot(BO_105_freqency_spectrum[0]/1000 , BO_105_SPL_broadband_1_3_spectrum[0] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw)      
    #axes1.plot(Exp_broadband_BO_105[:,0], Exp_broadband_BO_105[:,1], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw )     
    axes1.set_title('BO-105 40% Scale Rotor Broadband')
    axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes1.set_xlabel('Frequency (kHz)') 
    axes1.legend(loc='upper left') 
    plt.ylim((0,90))     

    #fig2 = plt.figure('Case_2')    
    #fig2.set_size_inches(8, 6) 
    #axes2 = fig2.add_subplot(1,1,1)            
    #axes2.plot(BO_105_freqency_spectrum[0], BO_105_SPL_broadband_1_3_spectrum[0] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw)      
    ##axes2.plot(Exp_broadband_BO_105[:,0], Exp_broadband_BO_105[:,1], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw )     
    #axes2.set_title('BO-105 40% Scale Rotor Broadband')
    #axes2.set_ylabel(r'SPL$_{1/3}$ (dB)')
    #axes2.set_xlabel('Frequency (Hz)') 
    #axes2.legend(loc='upper left') 
    #plt.ylim((0,90))  

    #fig3 = plt.figure()    
    #fig3.set_size_inches(8, 6) 
    #axes3 = fig3.add_subplot(1,1,1)            
    #axes3.plot(BO_105_freqency_spectrum[0] , BO_105_SPL_broadband_1_3_spectrum[2] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw )      
    ##axes3.plot(Exp_broadband_BO_105[:,0], Exp_broadband_BO_105[:,1], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw )     
    #axes3.set_title('BO-105 40% Scale Rotor Broadband')
    #axes3.set_ylabel(r'SPL$_{1/3}$ (dB)')
    #axes3.set_xlabel('Frequency (Hz)') 
    #axes3.legend(loc='upper left') 
    #plt.ylim((0,90)) 


    return  
def vectorize(vec,ctrl_pts,num_sec,N_r,BSR,method):
    vec = np.atleast_2d(vec)
    if method == 1:
        res = np.repeat(np.repeat(np.repeat(np.repeat(vec,N_r,axis = 1)[:,:,np.newaxis],
                                            num_sec,axis = 2)[:,:,:,np.newaxis],BSR,axis = 3)[:,:,:,:,np.newaxis],2,axis = 4)

    elif method == 2:
        res = np.repeat(np.repeat(np.repeat(vec,N_r,axis = 1)[:,:,np.newaxis],num_sec,axis = 2)[:,:,:,np.newaxis],BSR,axis = 3) 


    elif method == 3:
        res = np.repeat(np.repeat(np.repeat(vec[:,np.newaxis,:],N_r,axis = 0)[:,:,np.newaxis,:],num_sec,axis = 0)[:,:,:,:,np.newaxis],2,axis = 4) 

    return res  


def realign_polar_xticks(ax):
    for theta, label in zip(ax.get_xticks(), ax.get_xticklabels()):
        theta = theta * ax.get_theta_direction() + ax.get_theta_offset()
        theta = np.pi/2 - theta
        y, x = np.cos(theta), np.sin(theta)
        if x >= 0.1:
            label.set_horizontalalignment('left')
        if x <= -0.1:
            label.set_horizontalalignment('right')
        if y >= 0.5:
            label.set_verticalalignment('bottom')
        if y <= -0.5:
            label.set_verticalalignment('top')

    return 

def setup_noise_settings(sts): 

    sts.ground_microphone_phi_angles   = np.array([30.,45.,60.,75.,89.9,90.1,105.,120.,135.,150.])*Units.degrees
    sts.ground_microphone_theta_angles = np.array([89.9,89.9,89.9,89.9,89.9,89.9,89.9,89.9, 89.9,89.9 ])*Units.degrees
    sts.center_frequencies             = np.array([16,20,25,31.5,40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, \
                                                   500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150,
                                                    4000, 5000, 6300, 8000, 10000])        
    sts.lower_frequencies              = np.array([14,18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,\
                                                   900,1120,1400,1800,2240,2800,3550,4500,5600,7100,9000 ])
    sts.upper_frequencies              = np.array([18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,900,1120,\
                                                   1400,1800,2240,2800,3550,4500,5600,7100,9000,11200 ])
    sts.harmonics                      = np.arange(1,30)


    sts.floating_point_precision             = np.float64
    sts.urban_canyon_microphone_z_resolution = 16 
    sts.mic_x_position                       = 0     
    sts.lateral_ground_distance              = 1000 * Units.feet  
    sts.level_ground_microphone_min_x        = -50
    sts.level_ground_microphone_max_x        = 1000
    sts.level_ground_microphone_min_y        = -1000 * Units.feet
    sts.number_of_multiprocessing_workers    = None
    sts.level_ground_microphone_max_y        = 1000 * Units.feet 
    sts.level_ground_microphone_x_resolution = 16 
    sts.level_ground_microphone_y_resolution = 4      
    return sts 

if __name__ == '__main__': 
    main()    
    plt.show() 