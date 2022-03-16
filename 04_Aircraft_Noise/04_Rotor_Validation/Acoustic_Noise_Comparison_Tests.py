# SUAVE Imports 
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions                                           import Aerodynamics
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis      import airfoil_analysis 
import matplotlib.pyplot as plt   
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import  compute_naca_4series 
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.dbA_noise                    import A_weighting  
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools                              import SPL_harmonic_to_third_octave
from SUAVE.Methods.Noise.Fidelity_One.Propeller.compute_source_coordinates     import compute_point_source_coordinates
from SUAVE.Plots.Geometry import plot_propeller

# Python Imports 
import time 
import numpy as np  
import scipy as sp
from scipy.special import jv 
from scipy.special import fresnel
import matplotlib.pyplot as plt  
import matplotlib.cm as cm 

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
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():  


    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
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
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']
    plot_parameters.colors           = cm.viridis(np.linspace(0,1,5))     
    plot_parameters.lw               = 2                              # line_width               
    plot_parameters.m                = 14                             # markersize               
    plot_parameters.legend_font      = 20                             # legend_font_size         
    plot_parameters.Slc              = ['black','dimgray','silver' ]  # SUAVE_line_colors        
    plot_parameters.Slm              = '^'                            # SUAVE_line_markers       
    plot_parameters.Sls              = '-'                            # SUAVE_line_styles        
    plot_parameters.Elc              = ['firebrick','red','tomato']  # Experimental_line_colors 
    plot_parameters.Elm              = 's'                            # Experimental_line_markers
    plot_parameters.Els              = '-'                            # Experimental_line_styles 
    plot_parameters.Rlc              = ['mediumblue','blue','cyan']   # Ref_Code_line_colors     
    plot_parameters.Rlm              = 'o'                            # Ref_Code_line_markers    
    plot_parameters.Rls              = '--'                           # Ref_Code_line_styles     

    ti                = time.time()      
    
    save_figures = False 
    #Harmonic_Spectrum_Shape_Sensitivity(plot_parameters,save_figures)
    #Harmonic_Stength_Validation(plot_parameters,save_figures)
    #Harmonic_Directivty_Validation(plot_parameters,save_figures) 
    
    #Broadband_Noise_Validation(plot_parameters,save_figures)
    #Broadband_Spectrum_Validation(plot_parameters,save_figures) 
    #Broadband_Spectrum_Shape_Sensitivity(plot_parameters,save_figures) 
    #High_Fidelity_Validation_1(plot_parameters,save_figures) 
    #High_Fidelity_Validation_2(plot_parameters,save_figures)  
    #Skew_function(plot_parameters,save_figures)  
    #Total_Rotor_Spenctrum_Sensitivity_Validation(plot_parameters,save_figures) 
    #ANOPP2_Validation(plot_parameters,save_figures) 

    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)),3)) + ' sec')        
    return 




# ------------------------------------------------------------------ 
# Harmonic Specturm Shape Sensitivity
# ------------------------------------------------------------------     
def Harmonic_Spectrum_Shape_Sensitivity(PP,save_figures):
    '''This regression script is for validation and verification of the mid-fidelity acoustics
    analysis routine. "Experimental Data is obtained from Comparisons of predicted propeller
    noise with windtunnel ..." by Weir, D and Powers, J.
    '''   
    
    M                     = [0.5,0.75,1,1.25,1.5]   
    sensitivity_variables = [r'$\Omega$',r'0.75 $\beta_c$','chord',r'$dT/dr$',r'$dQ/dr$',r'$t/c$','M.C.A']
    sensitivity_name      = ['omega','beta_c','chord','dT_dr','dQ_dr','t_c','MCA']
    M_name                = ['50 % ','75 % ','100 % ','125 % ','150 % ']   

 

    for var_idx in range(len(sensitivity_variables)):
        multipliers = np.ones(8) 
        fig_1  = plt.figure() 
        fig_1.set_size_inches(PP.figure_width,PP.figure_height)    
        axis_1 = fig_1.add_subplot(1,1,1) 
        axis_1.set_ylim(0,90)
        axis_1.set_ylabel('SPL (dBA)') 
        axis_1.set_xlabel('Blade Passing Frequency')                
        for idx in range(len(M)):
            multipliers[var_idx] = M[idx]  

            # Define Network 
            net                                = Battery_Propeller()     
            net.number_of_propeller_engines    = 1
            net.identical_propellers           = True
            prop                               = design_F8745D4_prop()
            net.propellers.append(prop)
        
        
            # Set-up Validation Conditions 
            a                       = 343.376
            T                       = 288.16889478  
            density                 = 1.2250	
            dynamic_viscosity       = 1.81E-5  
            theta                   = np.array([-30])*Units.degrees
            S                       = np.array([4]) # np.linspace(2,10,5) 
            omega                   = np.array([2390]) * Units.rpm * multipliers[0]      
        
            # Set twist target
            three_quarter_twist     = 21 * Units.degrees  * multipliers[1] 
            n                       = len(prop.twist_distribution)
            beta                    = prop.twist_distribution
            beta_75                 = beta[round(n*0.75)] 
            delta_beta              = three_quarter_twist-beta_75
            prop.twist_distribution = beta + delta_beta 
            prop.chord_distribution = prop.chord_distribution * multipliers[2] 
            prop.thickness_to_chord = prop.thickness_to_chord * multipliers[5]  
        
            # microphone locations
            ctrl_pts                = len(omega) 
            dim_theta               = len(theta)
            dim_S                   = len(S) 
            num_mic                 = dim_S*dim_theta
        
            theta                   = np.repeat(np.repeat(np.atleast_2d(theta).T ,dim_S , axis = 1)[np.newaxis,:,:],ctrl_pts, axis = 0)  
            S                       = np.repeat(np.repeat(np.atleast_2d(S)       ,dim_theta, axis = 0)[np.newaxis,:,:],ctrl_pts, axis = 0) 
            x_vals                  = S*np.sin(theta)
            y_vals                  = -S*np.cos(theta)
            z_vals                  = np.zeros_like(x_vals) 
        
            mic_locations           = np.zeros((ctrl_pts,num_mic,3))   
            mic_locations[:,:,0]    = x_vals.reshape(ctrl_pts,num_mic) 
            mic_locations[:,:,1]    = y_vals.reshape(ctrl_pts,num_mic) 
            mic_locations[:,:,2]    = z_vals.reshape(ctrl_pts,num_mic)     
        
            # Set up for Propeller Model
            prop.inputs.omega                            = np.atleast_2d(omega).T
            prop.inputs.pitch_command                    = 0.
            conditions                                   = Aerodynamics()
            conditions._size                             = 3
            conditions.freestream.density                = np.ones((ctrl_pts,1)) * density
            conditions.freestream.dynamic_viscosity      = np.ones((ctrl_pts,1)) * dynamic_viscosity   
            conditions.freestream.speed_of_sound         = np.ones((ctrl_pts,1)) * a 
            conditions.freestream.temperature            = np.ones((ctrl_pts,1)) * T 
            conditions.frames.inertial.velocity_vector   = np.array([[77.2, 0. ,0.]])
            conditions.propulsion.throttle               = np.ones((ctrl_pts,1))*1.0
            conditions.aerodynamics.angle_of_attack      = np.ones((ctrl_pts,1))* 0. * Units.degrees 
            conditions.frames.body.transform_to_inertial = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
            conditions.noise.total_microphone_locations  = mic_locations
         
        
            # Run Propeller model 
            F, Q, P, Cp , aeroacoustic_data , etap       = prop.spin(conditions)   
            
            # Prepare Inputs for Noise Model    
            segment                                                = Segment() 
            segment.state.conditions                               = conditions
            segment.state.conditions.expand_rows(ctrl_pts)
            settings                                               = Data()
            settings                                               = setup_noise_settings(segment)
        
            num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
            conditions.noise.number_of_microphones                 = num_mic   
        
            # unpack 
            conditions           = segment.state.conditions
            microphone_locations = conditions.noise.total_microphone_locations
            angle_of_attack      = conditions.aerodynamics.angle_of_attack 
            velocity_vector      = conditions.frames.inertial.velocity_vector
            freestream           = conditions.freestream  
            harmonics            = settings.harmonics    
         
        
            # compute position vector from point source at rotor hub to microphones
            position_vector = compute_point_source_coordinates(conditions,net.propellers,microphone_locations,settings)  
        
            num_h        = len(harmonics)     
            num_cpt      = len(angle_of_attack)
            num_mic      = len(position_vector[0,:,0,0])
            num_rot      = len(position_vector[0,0,:,0])  
            rotor        = net.propellers[list(net.propellers.keys())[0]] 
            num_r        = len(rotor.radius_distribution) 
            orientation  = np.array(rotor.orientation_euler_angles) * 1 
            body2thrust  = sp.spatial.transform.Rotation.from_rotvec(orientation).as_matrix()
        
            # ----------------------------------------------------------------------------------
            # Rotational Noise  Thickness and Loading Noise
            # ----------------------------------------------------------------------------------  
            # [control point ,microphones, rotors, radial distribution, harmonics]  
            m              = np.tile(harmonics[None,None,None,None,:],(num_cpt,num_mic,num_rot,num_r,1))                 # harmonic number 
            m_1d           = harmonics                                                                                         
            p_ref          = 2E-5                                                                                        # referece atmospheric pressure
            a              = np.tile(freestream.speed_of_sound[:,:,None,None,None],(1,num_mic,num_rot,num_r,num_h))      # speed of sound
            rho            = np.tile(freestream.density[:,:,None,None,None],(1,num_mic,num_rot,num_r,num_h))             # air density   
            alpha          = np.tile((angle_of_attack + np.arccos(body2thrust[0,0]))[:,:,None,None,None],(1,num_mic,num_rot,num_r,num_h))           
            x              = np.tile(position_vector[:,:,:,0][:,:,:,None,None],(1,1,1,num_r,num_h))                      # x component of position vector of rotor to microphone 
            y              = np.tile(position_vector[:,:,:,1][:,:,:,None,None],(1,1,1,num_r,num_h))                      # y component of position vector of rotor to microphone
            z              = np.tile(position_vector[:,:,:,2][:,:,:,None,None],(1,1,1,num_r,num_h))                      # z component of position vector of rotor to microphone
            Vx             = np.tile(velocity_vector[:,0][:,None,None,None,None],(1,num_mic,num_rot,num_r,num_h))        # x velocity of rotor  
            Vy             = np.tile(velocity_vector[:,1][:,None,None,None,None],(1,num_mic,num_rot,num_r,num_h))        # y velocity of rotor 
            Vz             = np.tile(velocity_vector[:,2][:,None,None,None,None],(1,num_mic,num_rot,num_r,num_h))        # z velocity of rotor 
            B              = rotor.number_of_blades                                                                      # number of rotor blades
            omega          = np.tile(aeroacoustic_data.omega[:,:,None,None,None],(1,num_mic,num_rot,num_r,num_h))        # angular velocity       
            dT_dr          = np.tile(aeroacoustic_data.blade_dT_dr[:,None,None,:,None],(1,num_mic,num_rot,1,num_h)) * multipliers[3]      # nondimensionalized differential thrust distribution 
            dQ_dr          = np.tile(aeroacoustic_data.blade_dQ_dr[:,None,None,:,None],(1,num_mic,num_rot,1,num_h))* multipliers[4]       # nondimensionalized differential torque distribution
            R              = np.tile(rotor.radius_distribution[None,None,None,:,None],(num_cpt,num_mic,num_rot,1,num_h)) # radial location     
            c              = np.tile(rotor.chord_distribution[None,None,None,:,None],(num_cpt,num_mic,num_rot,1,num_h))  # blade chord    
            R_tip          = rotor.tip_radius                                                     
            t_c            = np.tile(rotor.thickness_to_chord[None,None,None,:,None],(num_cpt,num_mic,num_rot,1,num_h)) # thickness to chord ratio
            MCA            = np.tile(rotor.mid_chord_alignment[None,None,None,:,None],(num_cpt,num_mic,num_rot,1,num_h)) # Mid Chord Alighment  
            f              = B*omega*m/(2*np.pi) 
            D              = 2*R[0,0,0,-1,:]                                                                             # rotor diameter    
            r              = R/R[0,0,0,-1,:]                                                                             # non dimensional radius distribution  
            S              = np.sqrt(x**2 + y**2 + z**2)                                                                 # distance between rotor and the observer    
            theta          = np.arccos(x/S)                                                            
            Y              = np.sqrt(y**2 + z**2)                                                                        # observer distance from rotor axis          
            V              = np.sqrt(Vx**2 + Vy**2 + Vz**2)                                                              # velocity magnitude
            M_x            = V/a                                                                                         
            V_tip          = R_tip*omega                                                                                 # blade_tip_speed 
            M_t            = V_tip/a                                                                                     # tip Mach number 
            M_r            = np.sqrt(M_x**2 + (r**2)*(M_t**2))                                                           # section relative Mach number     
            B_D            = c/D                                                                                         
            phi            = np.arctan(z/y)                                                                              # tangential angle   
        
            # retarted  theta angle in the retarded reference frame
            theta_r        = np.arccos(np.cos(theta)*np.sqrt(1 - (M_x**2)*(np.sin(theta))**2) + M_x*(np.sin(theta))**2 )   
            theta_r_prime  = np.arccos(np.cos(theta_r)*np.cos(alpha) + np.sin(theta_r)*np.sin(phi)*np.sin(alpha) )
        
            # initialize thickness and loading noise matrices
            psi_L          = np.zeros((num_cpt,num_mic,num_rot,num_r,num_h))
            psi_V          = np.zeros((num_cpt,num_mic,num_rot,num_r,num_h))
        
            # normalized thickness  and loading shape functions                
            k_x               = ((2*m*B*B_D*M_t)/(M_r*(1 - M_x*np.cos(theta_r))))      # wave number 
            psi_V[:,:,:,0,:]  = 2/3   
            psi_L[:,:,:,0,:]  = 1     
            psi_V[:,:,:,1:,:] = (8/(k_x[:,:,:,1:,:]**2))*((2/k_x[:,:,:,1:,:])*np.sin(0.5*k_x[:,:,:,1:,:]) - np.cos(0.5*k_x[:,:,:,1:,:]))    
            psi_L[:,:,:,1:,:] = (2/k_x[:,:,:,1:,:])*np.sin(0.5*k_x[:,:,:,1:,:])                  
        
            # sound pressure for thickness noise   
            Jmb               = jv(m*B,((m*B*r*M_t*np.sin(theta_r_prime))/(1 - M_x*np.cos(theta_r))))  
            phi_s             = ((2*m*B*M_t)/(M_r*(1 - M_x*np.cos(theta_r))))*(MCA/D)
            phi_prime         = np.arccos((np.sin(theta_r)/np.sin(theta_r_prime))*np.cos(phi))      
            S_r               = Y/(np.sin(theta_r))                                # distance in retarded reference frame                                                                             
            exponent_fraction = np.exp(1j*m_1d*B*((omega*S_r/a) +  phi_prime - np.pi/2))/(1 - M_x*np.cos(theta_r))
            p_mT_H_integral   = -((M_r**2)*(t_c)*np.exp(1j*phi_s)*Jmb*(k_x**2)*psi_V ) * ((rho*(a**2)*B*np.sin(theta_r))/(4*np.sqrt(2)*np.pi*(Y/D)))* exponent_fraction
            p_mT_H            = np.trapz(p_mT_H_integral,x = r[0,0,0,:,0], axis =3) 
        
            p_mT_H_abs        = abs(p_mT_H)             
            p_mL_H_integral   = (((np.cos(theta_r_prime)/(1 - M_x*np.cos(theta_r)))*dT_dr - (1/((r**2)*M_t*R_tip))*dQ_dr)
                                 * np.exp(1j*phi_s)*Jmb * psi_L)*(m_1d*B*M_t*np.sin(theta_r)/ (2*np.sqrt(2)*np.pi*Y*R_tip)) *exponent_fraction
            p_mL_H            = np.trapz(p_mL_H_integral,x = r[0,0,0,:,0], axis = 3 ) 
            p_mL_H_abs        =  abs(p_mL_H)  
        
            # sound pressure levels  
            SPL_prop_harmonic_bpf_spectrum     = 20*np.log10((abs(p_mL_H_abs + p_mT_H_abs))/p_ref) 
            SPL_prop_harmonic_bpf_spectrum_dBA = A_weighting(SPL_prop_harmonic_bpf_spectrum,f[:,:,:,0,:])        
             

            axis_1.plot(harmonics,SPL_prop_harmonic_bpf_spectrum_dBA[0,0,0],color = PP.colors[idx], 
                            linestyle = PP.line_style, markersize = PP.m, marker = PP.markers[idx],
                          label = M_name[idx] + sensitivity_variables[var_idx] )      
            axis_1.legend(loc='upper right', ncol= 3, prop={'size': PP.legend_font_size})     
            axis_1.set_ylim([0,135])
        fig_1.tight_layout()  
        fig_1_name = "Harmonic_Spectrum_" + sensitivity_name[var_idx]  + '_Sensitivity'
        if save_figures:
            fig_1.savefig(fig_1_name  + '.pdf')     
     
    return    



# ------------------------------------------------------------------ 
# Harmonic Directivty Validation
# ------------------------------------------------------------------     
def Harmonic_Directivty_Validation(PP,save_figures):

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

    # Prepare Inputs for Noise Model
    conditions.noise.total_microphone_locations            = np.array([positions_h1])
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees  
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])   
    
    # Run Noise Model   
    propeller_noise  = propeller_mid_fidelity(net.propellers,noise_data,segment,settings )   
    SPL_harmonic_1   = propeller_noise.SPL_harmonic_bpf_spectrum[0,:,0]   
    

    # Harmonic 2
    theta_2h                                               = np.array([1    ,15.45,27.19,38.04,43.94,53.88,65.40,73.48,90.61,113.3,123.7,133.4 ,141.8,149.7,158.2,169.9 ])
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

    # Prepare Inputs for Noise Model
    conditions.noise.total_microphone_locations            = np.array([positions_h2])
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic  

    # Run Noise Model   
    propeller_noise  = propeller_mid_fidelity(net.propellers,noise_data,segment,settings)    
    SPL_harmonic_2   = propeller_noise.SPL_harmonic_bpf_spectrum[0,:,1] 
 
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------      

    exp_theta_1 = np.array([0.0 ,10.178, 19.821, 27.857, 38.571,45.000, 55.714, 64.285, 80.892, 95.357, 109.28, 119.46,130.17,148.92,165 ])
    exp_harmonic_1 = np.array([ 79.8643, 84.4661,91.3567,94.4272, 97.1217,97.5170, 97.1580, 97.1762, 101.028,102.203,101.851,99.2016, 96.1709, 94.3022,93.5730,])
    exp_theta_2 = np.array([1.0650,15.443,27.692,38.875,44.201,55.384,65.502,74.023,91.597, 113.96, 124.08,134.20,142.72, 150.17, 158.16, 169.88  ])
    exp_harmonic_2 = np.array([ 85.9541,85.5725,83.6641, 85.9541, 93.2061,95.1145, 96.2595,96.6412,97.0229, 97.4045,96.2595, 92.0610, 88.6259, 87.8625, 88.6259, 91.2977,])    

    figure_width  = 5
    figure_height = 7
    
    # Polar plot of noise    
    fig1 = plt.figure('Propeller_Noise_Directivity_Validation_Harm_1')
    fig1.set_size_inches(figure_width,figure_height)    
    axis1 = fig1.add_subplot(111, projection='polar')    
    #axis1.linewidth = 1 
    l3, = axis1.plot(-exp_theta_1, exp_harmonic_1   , color = PP.Elc[0] , linestyle = '', marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw )    
    axis1.plot(exp_theta_1, exp_harmonic_1     , color = PP.Elc[0] , linestyle = '', marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw )   
    l1, = axis1.plot(-theta_1h*Units.degrees, SPL_harmonic_1, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)     
    axis1.plot(theta_1h*Units.degrees, SPL_harmonic_1, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)        
    axis1.set_ylim([0,120])
    axis1.set_yticks(np.arange(50,120,25) )     
    axis1.grid(True)     
    realign_polar_xticks(axis1)  
    plt.figlegend((l1,l3),
                  ('SUAVE','Exp.'),
                  loc=(0.25, 0.9), 
                  prop={'size': PP.legend_font} , 
                  ncol= 2, frameon=False ) 

    fig2 = plt.figure('Propeller_Noise_Directivity_Validation_Harm_2')
    fig2.set_size_inches(figure_width,figure_height)   
    axis2 = fig2.add_subplot(111, projection='polar')     
    #axis2.linewidth = 1 
    axis2.plot(-exp_theta_2*Units.degrees, exp_harmonic_2    , color = PP.Elc[0] , linestyle = '', marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw)     
    axis2.plot(exp_theta_2*Units.degrees, exp_harmonic_2   , color = PP.Elc[0] , linestyle = '', marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw )   
    axis2.plot(-theta_2h*Units.degrees, SPL_harmonic_2, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)   
    axis2.plot(theta_2h*Units.degrees, SPL_harmonic_2, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw)   
    axis2.set_ylim([0,120])   
    axis2.set_yticks(np.arange(50,120,25) )     
    axis2.grid(True)      
    realign_polar_xticks(axis2)   
    plt.figlegend((l1,l3),
                  ('SUAVE','Exp.'),
                  loc=(0.25, 0.9), 
                  prop={'size': PP.legend_font} , 
                  ncol= 2, frameon=False
                  )   

    fig1.tight_layout()
    fig2.tight_layout()    
    
    return     


# ------------------------------------------------------------------ 
# Harmonic Noise Validation 1 
# ------------------------------------------------------------------     
def Harmonic_Stength_Validation(PP,save_figures):
    '''This regression script is for validation and verification of the mid-fidelity acoustics
    analysis routine. "Experimental Data is obtained from Comparisons of predicted propeller
    noise with windtunnel ..." by Weir, D and Powers, J.
    '''   

    # Define Network 
    net                                = Battery_Propeller()     
    net.number_of_propeller_engines    = 1
    net.identical_propellers           = True
    prop                               = design_F8745D4_prop()
    net.propellers.append(prop)


    # Set-up Validation Conditions 
    a                       = 343.376
    T                       = 288.16889478  
    density                 = 1.2250	
    dynamic_viscosity       = 1.81E-5  
    theta                   = np.linspace(-np.pi/2,np.pi/2,19) + 1E-8
    S                       = np.array([4]) # np.linspace(2,10,5) 
    omega                   = np.array([2390,2710,2630]) * Units.rpm     

    # Set twist target
    three_quarter_twist     = 21 * Units.degrees 
    n                       = len(prop.twist_distribution)
    beta                    = prop.twist_distribution
    beta_75                 = beta[round(n*0.75)] 
    delta_beta              = three_quarter_twist-beta_75
    prop.twist_distribution = beta + delta_beta 

    # microphone locations
    ctrl_pts                = len(omega) 
    dim_theta               = len(theta)
    dim_S                   = len(S) 
    num_mic                 = dim_S*dim_theta

    theta                   = np.repeat(np.repeat(np.atleast_2d(theta).T ,dim_S , axis = 1)[np.newaxis,:,:],ctrl_pts, axis = 0)  
    S                       = np.repeat(np.repeat(np.atleast_2d(S)       ,dim_theta, axis = 0)[np.newaxis,:,:],ctrl_pts, axis = 0) 
    x_vals                  = S*np.sin(theta)
    y_vals                  = -S*np.cos(theta)
    z_vals                  = np.zeros_like(x_vals) 

    mic_locations           = np.zeros((ctrl_pts,num_mic,3))   
    mic_locations[:,:,0]    = x_vals.reshape(ctrl_pts,num_mic) 
    mic_locations[:,:,1]    = y_vals.reshape(ctrl_pts,num_mic) 
    mic_locations[:,:,2]    = z_vals.reshape(ctrl_pts,num_mic)     

    # Set up for Propeller Model
    prop.inputs.omega                            = np.atleast_2d(omega).T
    prop.inputs.pitch_command                    = 0.
    conditions                                   = Aerodynamics()
    conditions._size                             = 3
    conditions.freestream.density                = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity      = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound         = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature            = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector   = np.array([[77.2, 0. ,0.],[ 77.0,0.,0.], [ 77.2, 0. ,0.]])
    conditions.propulsion.throttle               = np.ones((ctrl_pts,1))*1.0
    conditions.aerodynamics.angle_of_attack      = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    conditions.frames.body.transform_to_inertial = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
    conditions.noise.total_microphone_locations  = mic_locations

    # Run Propeller model 
    F, Q, P, Cp , noise_data , etap                        = prop.spin(conditions)   
    
    # Prepare Inputs for Noise Model    
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts)
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment)

    num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
    conditions.noise.number_of_microphones                 = num_mic   

    # Run Noise Model  
    propeller_noise  = propeller_mid_fidelity(net.propellers,noise_data,segment,settings)    
    SPL_Spectrum     = propeller_noise.SPL_harmonic_bpf_spectrum             

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

    fig_size_width  = 9
    fig_size_height = 5.5
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots
    # ----------------------------------------------------------------------------------------------------------------------------------------      
    SUAVE_SPL_Case_1_60deg = SPL_Spectrum[0,6,:][:len(harmonics)]
    SUAVE_SPL_Case_1_90deg = SPL_Spectrum[0,9,:][:len(harmonics)] 
    SUAVE_SPL_Case_2_60deg = SPL_Spectrum[1,6,:][:len(harmonics)]  
    SUAVE_SPL_Case_2_90deg = SPL_Spectrum[1,9,:][:len(harmonics)]
    SUAVE_SPL_Case_3_60deg = SPL_Spectrum[2,6,:][:len(harmonics)] 
    SUAVE_SPL_Case_3_90deg = SPL_Spectrum[2,9,:][:len(harmonics)]   

    # plot results
    fig31 = plt.figure('Harmonic_Validation_Case_1_60')
    print('Case 1, $C_P$ = ' +  str(round(Cp[0,0],3)))  
    fig31.set_size_inches(fig_size_width, fig_size_height)   
    axes = fig31.add_subplot(1,1,1) 
    axes.plot(harmonics, ANOPP_PAS_Case_1_60deg, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm,  markersize = PP.m , linewidth = PP.lw,  label = r'ANOPP PAS')       
    axes.plot(harmonics, Exp_Test_Case_1_60deg , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw,  label = r'Exp.')   
    axes.plot(harmonics, SUAVE_SPL_Case_1_60deg, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw,  label = r'SUAVE')      
    axes.set_ylabel('SPL (dB)') 
    axes.set_xlabel('Harmonic no.')
    axes.legend(loc='lower left', prop={'size':  PP.legend_font})  

    fig34 = plt.figure('Harmonic_Validation_Case_1_90')
    fig34.set_size_inches(fig_size_width, fig_size_height)   
    axes = fig34.add_subplot(1,1,1)       
    axes.plot(harmonics, ANOPP_PAS_Case_1_90deg, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm,  markersize = PP.m , linewidth = PP.lw,  label = r'ANOPP PAS')    
    axes.plot(harmonics, Exp_Test_Case_1_90deg , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw,  label = r'Exp.')   
    axes.plot(harmonics, SUAVE_SPL_Case_1_90deg, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw,  label = r'SUAVE')     
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('Harmonic no.')  
    axes.legend(loc='lower left', prop={'size': PP.legend_font}) 

    # Test Case 2
    print('Case 2, $C_P$ = ' +  str(round(Cp[1,0],3)))  
    fig32 = plt.figure('Harmonic_Validation_Case_2_60')
    fig32.set_size_inches(fig_size_width, fig_size_height)   
    axes = fig32.add_subplot(1,1,1)  
    axes.plot(harmonics, ANOPP_PAS_Case_2_60deg, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm,  markersize = PP.m , linewidth = PP.lw,  label = r'ANOPP PAS')      
    axes.plot(harmonics, Exp_Test_Case_2_60deg , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw,  label = r'Exp.')   
    axes.plot(harmonics, SUAVE_SPL_Case_2_60deg, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw,  label = r'SUAVE')      
    axes.set_ylabel('SPL (dB)') 
    axes.set_xlabel('Harmonic no.')      
    axes.legend(loc='lower left', prop={'size': PP.legend_font}) 

    fig35 = plt.figure('Harmonic_Validation_Case_2_90')
    fig35.set_size_inches(fig_size_width, fig_size_height)   
    axes = fig35.add_subplot(1,1,1)                
    axes.plot(harmonics, ANOPP_PAS_Case_2_90deg, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm,  markersize = PP.m , linewidth = PP.lw,  label = r'ANOPP PAS') 
    axes.plot(harmonics, Exp_Test_Case_2_90deg , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw,  label = r'Exp.')   
    axes.plot(harmonics, SUAVE_SPL_Case_2_90deg, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw,  label = r'SUAVE')     
    axes.set_ylabel('SPL (dB)')                 
    axes.set_xlabel('Harmonic no.')  
    axes.legend(loc='lower left', prop={'size': PP.legend_font}) 

    # Test Case 3
    print('Case 3, $C_P$ = ' +  str(round(Cp[2,0],3)))  
    fig33 = plt.figure('Harmonic_Validation_Case_3_60')
    fig33.set_size_inches(fig_size_width, fig_size_height)   
    axes = fig33.add_subplot(1,1,1)  
    axes.plot(harmonics, ANOPP_PAS_Case_3_60deg, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm,  markersize = PP.m , linewidth = PP.lw,  label = r'ANOPP PAS')  
    axes.plot(harmonics, Exp_Test_Case_3_60deg , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw,  label = r'Exp.')   
    axes.plot(harmonics, SUAVE_SPL_Case_3_60deg, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw,  label = r'SUAVE')       
    axes.set_ylabel('SPL (dB)') 
    axes.legend(loc='lower left', prop={'size': PP.legend_font}) 

    fig36 = plt.figure('Harmonic_Validation_Case_3_90')
    fig36.set_size_inches(fig_size_width, fig_size_height)   
    axes = fig36.add_subplot(1,1,1)       
    axes.plot(harmonics, ANOPP_PAS_Case_3_90deg, color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm,  markersize = PP.m , linewidth = PP.lw,  label = r'ANOPP PAS') 
    axes.plot(harmonics, Exp_Test_Case_3_90deg , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm,  markersize = PP.m , linewidth = PP.lw,  label = r'Exp.')   
    axes.plot(harmonics, SUAVE_SPL_Case_3_90deg, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm,  markersize = PP.m , linewidth = PP.lw,  label = r'SUAVE')     
    axes.set_ylabel('SPL (dB)')
    axes.set_xlabel('Harmonic no.') 
    axes.legend(loc='lower left', prop={'size': PP.legend_font})    

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Regression 
    # ----------------------------------------------------------------------------------------------------------------------------------------     
    SPL_Case_1_60deg_truth = Exp_Test_Case_1_60deg
    SPL_Case_1_90deg_truth = Exp_Test_Case_1_90deg

    # Store errors 
    error = Data()
    error.SPL_Case_1_60deg  = np.max(np.abs(SUAVE_SPL_Case_1_60deg -SPL_Case_1_60deg_truth))  
    error.SPL_Case_1_90deg  = np.max(np.abs(SUAVE_SPL_Case_1_90deg -SPL_Case_1_90deg_truth)) 

    fig31.tight_layout()
    fig32.tight_layout()
    fig33.tight_layout()
    fig34.tight_layout() 
    fig35.tight_layout()
    fig36.tight_layout()    
    fig31_name = 'Harmonic_Noise_Validation_Case_1_60' 
    fig32_name = 'Harmonic_Noise_Validation_Case_1_90' 
    fig33_name = 'Harmonic_Noise_Validation_Case_2_60' 
    fig34_name = 'Harmonic_Noise_Validation_Case_2_90'  
    fig35_name = 'Harmonic_Noise_Validation_Case_3_60' 
    fig36_name = 'Harmonic_Noise_Validation_Case_3_90'    
    if save_figures:
        fig31.savefig(fig31_name  + '.pdf')               
        fig32.savefig(fig32_name  + '.pdf')               
        fig33.savefig(fig33_name  + '.pdf')               
        fig34.savefig(fig34_name  + '.pdf')     
        fig35.savefig(fig35_name  + '.pdf')  
        fig36.savefig(fig36_name  + '.pdf')  
    return    



# ------------------------------------------------------------------ 
# Examine Broadband Spectrum Shape Sensitivities 
# ------------------------------------------------------------------ 
def Broadband_Spectrum_Shape_Sensitivity(PP,save_figures):

    '''Empirical wall-pressure spectral modeling for zero and adverse pressure gradient flows''' 

    M                     = [0.5,0.75,1,1.25,1.5]   
    sensitivity_variables = [r'Re',r'$\alpha$',r'$\delta$',r'$\delta*$',r'$dp/dx$', r'$C_f$', r'$U_e$' , r'$\theta$' ]
    sensitivity_name      =  ['Re','alpha','delta','delta_star','dp_dx', 'C_f', 'Ue' , 'theta' ]
    M_name                = [r'50 % ',r'75 % ',r'100 % ',r'125 % ',r'150 % ']   

    

    for var_idx in range(len(sensitivity_variables)):
        multipliers = np.ones(8) 
        fig_1  = plt.figure() 
        fig_1.set_size_inches(PP.figure_width,PP.figure_height) 
        axis_1 = fig_1.add_subplot(1,1,1) 
        axis_1.set_ylim(50,120)
        axis_1.set_ylabel(r'$10log_{10}$ ($\Phi$)') 
        axis_1.set_xlabel('Frequency (Hz)')                
        for idx in range(len(M)):
            multipliers[var_idx] = M[idx] 

            pi           = np.pi 
            rho          = 1.2                                               
            kine_visc    = np.array([[0.0000150]])                        
            num_sec      = 1 # number of sections  
            N_r          = 1
            ctrl_pts     = 1
            BSR          = 100 # broadband spectrum resolution
            frequency    = np.linspace(1E2,1E4,BSR) 
            w            = 2*pi*frequency
            Re           = np.array([[1.5E6]]) * multipliers[0] 
            alpha        = np.array([[6.]]) *Units.degrees * multipliers[1]                                        
            V_inf        = 69.5
            chord        = Re*kine_visc/V_inf
            kine_visc    = vectorize(kine_visc,ctrl_pts,num_sec,N_r,BSR,method = 1)
            delta        = np.zeros((ctrl_pts,N_r,num_sec,BSR,2))  
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
            delta[:,:,:,:,0]        = vectorize(AP.delta[:,TE_idx]                         ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[2]             
            delta[:,:,:,:,1]        = vectorize(AP.delta[:,-TE_idx]                        ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[2]  
            delta_star[:,:,:,:,0]   = vectorize(AP.delta_star[:,TE_idx]                    ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[3]                    
            delta_star[:,:,:,:,1]   = vectorize(AP.delta_star[:,-TE_idx]                   ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[3]   
            surface_dcp_dx          = (np.diff(AP.Cp*0.5*rho*(V_inf**2),axis = 1)/(np.diff(AP.x,axis = 1)*chord))  
            dp_dx[:,:,:,:,0]        = vectorize(abs(surface_dcp_dx[:,TE_idx]),ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[4]         
            dp_dx[:,:,:,:,1]        = vectorize(abs(surface_dcp_dx[:,-TE_idx]),ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[4]               
            C_f[:,:,:,:,0]          = vectorize(AP.Cf[:,TE_idx]                            ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[5]                        
            C_f[:,:,:,:,1]          = vectorize(AP.Cf[:,-TE_idx]                           ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[5]          
            Ue[:,:,:,:,0]           = vectorize(AP.Ue_Vinf[:,TE_idx]*V_inf                 ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[6]                     
            Ue[:,:,:,:,1]           = vectorize(AP.Ue_Vinf[:,-TE_idx]*V_inf                ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[6]                                    
            Theta[:,:,:,:,0]        = vectorize(AP.theta[:,TE_idx]                         ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[7]                     
            Theta[:,:,:,:,1]        = vectorize(AP.theta[:,-TE_idx]                        ,ctrl_pts,num_sec,N_r,BSR,method = 2)* multipliers[7]    
            tau_w                   = C_f*(0.5*rho*(Ue**2))

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
            var                 = 10*np.log10((Phi_pp)/((2E-5)**2))   

            axis_1.semilogx(frequency,var[0,0,0,:,0],color = PP.colors[idx], 
                            linestyle = PP.line_style, markersize = PP.m, marker = PP.markers[idx],
                          label = M_name[idx] + sensitivity_variables[var_idx])      
            axis_1.legend(loc='upper right', ncol= 3, prop={'size': PP.legend_font_size})    


        fig_1.tight_layout()  
        fig_1_name = "Broadband_Spectrum_" + sensitivity_name[var_idx]  + '_Sensitivity'
        if save_figures:
            fig_1.savefig(fig_1_name  + '.pdf')                

    return   

# ------------------------------------------------------------------ 
# Broadband Spectrum Validation
# ------------------------------------------------------------------     
def Broadband_Spectrum_Validation(PP,save_figures):

    '''Empirical wall-pressure spectral modeling for zero and adverse pressure gradient flows''' 


    fig_1  = plt.figure('Broadband_Spectrum_Comparison')  
    fig_1.set_size_inches(PP.figure_width,PP.figure_height)  
    axis_1 = fig_1.add_subplot(1,1,1) 
    axis_1.set_ylim(50,90)
    axis_1.set_ylabel('10log10($\Phi$)') 
    axis_1.set_xlabel('Frequency (Hz)')   

    pi           = np.pi 
    rho          = 1.2                                               
    kine_visc    = np.array([[0.0000150]])                        
    num_sec      = 1 # number of sections  
    N_r          = 1
    ctrl_pts     = 1
    BSR          = 100 # broadband spectrum resolution
    frequency    = np.linspace(1E2,1E4,BSR) 
    w            = 2*pi*frequency
    Re           = np.array([[1.5E6]])  
    alpha        = np.array([[6.]]) *Units.degrees                                      
    V_inf        = 69.5
    chord        = Re*kine_visc/V_inf
    kine_visc    = vectorize(kine_visc,ctrl_pts,num_sec,N_r,BSR,method = 1)
    delta        = np.zeros((ctrl_pts,N_r,num_sec,BSR,2))  
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
    delta[:,:,:,:,0]        = vectorize(AP.delta[:,TE_idx]                         ,ctrl_pts,num_sec,N_r,BSR,method = 2)              
    delta[:,:,:,:,1]        = vectorize(AP.delta[:,-TE_idx]                        ,ctrl_pts,num_sec,N_r,BSR,method = 2)   
    delta_star[:,:,:,:,0]   = vectorize(AP.delta_star[:,TE_idx]                    ,ctrl_pts,num_sec,N_r,BSR,method = 2)                     
    delta_star[:,:,:,:,1]   = vectorize(AP.delta_star[:,-TE_idx]                   ,ctrl_pts,num_sec,N_r,BSR,method = 2)    
    surface_dcp_dx          = (np.diff(AP.Cp*0.5*rho*(V_inf**2),axis = 1)/(np.diff(AP.x,axis = 1)*chord))  
    dp_dx[:,:,:,:,0]        = vectorize(abs(surface_dcp_dx[:,TE_idx]),ctrl_pts,num_sec,N_r,BSR,method = 2)       
    dp_dx[:,:,:,:,1]        = vectorize(abs(surface_dcp_dx[:,-TE_idx]),ctrl_pts,num_sec,N_r,BSR,method = 2)           
    C_f[:,:,:,:,0]          = vectorize(AP.Cf[:,TE_idx]                            ,ctrl_pts,num_sec,N_r,BSR,method = 2)                        
    C_f[:,:,:,:,1]          = vectorize(AP.Cf[:,-TE_idx]                           ,ctrl_pts,num_sec,N_r,BSR,method = 2)          
    Ue[:,:,:,:,0]           = vectorize(AP.Ue_Vinf[:,TE_idx]*V_inf                 ,ctrl_pts,num_sec,N_r,BSR,method = 2)                     
    Ue[:,:,:,:,1]           = vectorize(AP.Ue_Vinf[:,-TE_idx]*V_inf                ,ctrl_pts,num_sec,N_r,BSR,method = 2)                                    
    Theta[:,:,:,:,0]        = vectorize(AP.theta[:,TE_idx]                         ,ctrl_pts,num_sec,N_r,BSR,method = 2)                     
    Theta[:,:,:,:,1]        = vectorize(AP.theta[:,-TE_idx]                        ,ctrl_pts,num_sec,N_r,BSR,method = 2)    
    tau_w                   = C_f*(0.5*rho*(Ue**2))

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
    var                 = 10*np.log10((Phi_pp)/((2E-5)**2))  


    reference_spectrum  = reference_wall_pressure_spectrum_model()   

    axis_1.semilogx(frequency,var[0,0,0,:,0],color = 'red', linewidth = PP.line_width,
                    linestyle = '-', marker = 's',markersize = PP.m,
                  label = 'SUAVE')      

    axis_1.semilogx(frequency,reference_spectrum,color = 'black', linewidth = PP.line_width, 
                    linestyle = '-', label = 'Ref.')      

    axis_1.legend(loc='upper right', ncol= 2, prop={'size': PP.legend_font_size})           



    fig_1.tight_layout()  
    fig_1_name = "Broadband_Spectrum_Comparison"  
    if save_figures:
        fig_1.savefig(fig_1_name  + '.pdf')

    return 

# ------------------------------------------------------------------ 
# Reference  Wall Pressure Specturm
# ------------------------------------------------------------------     
def reference_wall_pressure_spectrum_model():

    BSR          = 100 # broadband spectrum resolution
    frequency    = np.linspace(1E2,1E4,BSR) 
    w            = 2*np.pi*frequency 

    delta        = 0.0142
    delta_star   = 0.00236
    Cf           = 0.00217
    nu           = 0.0000150
    rho          = 1.2
    Ue           = 64.6
    beta_c       = 3.51
    tau_w        = Cf*(0.5*rho*(Ue**2))
    mu_tau       = (tau_w/rho)**0.5 
    R_T          = (delta/Ue)/(nu/(mu_tau**2)) 
    Delta        = delta/delta_star  
    PI           = 0.8*((beta_c + 0.5)**3/4)
    SS           = Ue/((tau_w**2)*delta_star)
    FS           = delta_star/Ue
    b            = 2
    c            = 0.75
    e            = 3.7 + 1.5*beta_c
    d            = 4.76*((1.4/Delta)**0.75)*(0.375*e - 1)
    f            = 8.8
    g            = -0.57 
    a            = (2.82*(Delta**2)*((6.13*(Delta**(-0.75)) + d)**e))*(4.2*(PI/Delta) + 1)
    criteria_b   = (19/np.sqrt(R_T))
    h            = np.minimum(3,criteria_b) + 7
    i            = 4.76

    phi_ros    = (a*(w*FS)**b)/(( i*(w*FS)**c  + d)**e + ((f*R_T**g)*(w*FS))**h  ) 
    var2       = 10*np.log10((phi_ros/SS)/((2E-5)**2))  

    return var2


# ------------------------------------------------------------------ 
# Broadband Noise Validation
# ------------------------------------------------------------------     
def Broadband_Noise_Validation(PP,save_figures):  
    APC_SF = design_APC_11x_4_7_prop()   
    APC_SF_inflow_ratio = 0.08 
    
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
    theta                                       = np.array([45., 67.5, 90.001, 112.5 , 135.])  # np.linspace(0.1,180,100)  
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
    APC_SF_thrust, APC_SF_torque, APC_SF_power, APC_SF_Cp, acoustic_outputs  , APC_SF_etap  =  APC_SF.spin(APC_SF_conditions)  
    
    # Prepare Inputs for Noise Model  
    APC_SF_conditions.noise.total_microphone_locations             = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    APC_SF_conditions.aerodynamics.angle_of_attack                 = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    APC_SF_segment                                                 = Segment() 
    APC_SF_segment.state.conditions                                = APC_SF_conditions
    APC_SF_segment.state.conditions.expand_rows(ctrl_pts)
    APC_SF_settings                                                = Data()
    APC_SF_settings                                                = setup_noise_settings(APC_SF_segment)   
    
    # Run Noise Model    
    APC_SF_propeller_noise             = propeller_mid_fidelity(net_APC_SF.propellers,acoustic_outputs,APC_SF_segment,APC_SF_settings )    
    APC_SF_1_3_Spectrum                = APC_SF_propeller_noise.SPL_1_3_spectrum 
    APC_SF_SPL_broadband_1_3_spectrum  = APC_SF_propeller_noise.SPL_broadband_1_3_spectrum  

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------      
    Exp_APC_SF_freqency_spectrum =  np.array([100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 
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

    Exp_broadband_APC = np.array([[24.8571428,27.7142857,28.8571428,26.5714285,27.1428571,27.7142857,30.2857142,32,35.4285714,
                                   39.1428571,40.5714285,39.4285714,40.5714285,40,41.1428571,40.2857142,41.7142857,44,44.5714285,
                                   44.8571428,45.1428571],
                                  [23.42857,26,27.14285,25.14285,25.71428,26.57142,28.28571,29.14285,34.28571,37.14285,
                                   37.99999,34.57142,39.14285,34,35.71428,34.85714,35.99999,43.42857,39.14285,41.14285,
                           42.57142]]) 
    
    
    fig_size_width  = 8 
    fig_size_height = 6
    
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------   

    # Figures 8 and 9 comparison 
    fig1 = plt.figure('Noise_Validation_Total_1_3_Spectrum_3600')    
    fig1.set_size_inches(fig_size_width,fig_size_height)  
    axes1 = fig1.add_subplot(1,1,1)      
    axes1.plot(Exp_APC_SF_freqency_spectrum , Exp_APC_SF_1_3_Spectrum[0,:-5]  , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw, label = 'Exp. 3600 RPM')            
    axes1.plot(Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[0,0,8:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label =' SUAVE 3600 RPM')    
    axes1.set_xscale('log') 
    axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes1.set_xlabel('Frequency (Hz)') 
    axes1.legend(loc='lower right', prop={'size': PP.legend_font}) 
    axes1.set_ylim([15,60])   
    

    fig2 = plt.figure('Noise_Validation_1_3_Spectrum_4800')    
    fig2.set_size_inches(fig_size_width,fig_size_height)  
    axes2 = fig2.add_subplot(1,1,1)           
    axes2.plot(Exp_APC_SF_freqency_spectrum , Exp_APC_SF_1_3_Spectrum[2,:-5]  , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,  label = 'Exp. 4800 RPM')       
    axes2.plot(Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[2,0,8:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,label = ' SUAVE 4800 RPM')   
    axes2.set_xscale('log') 
    axes2.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes2.set_xlabel('Frequency (Hz)') 
    axes2.legend(loc='lower right', prop={'size': PP.legend_font})  
    axes2.set_ylim([15,60])   
     

    fig3 = plt.figure('Noise_Validation_Broadband_1_3_Spectrum_45_deg')    
    fig3.set_size_inches(fig_size_width,fig_size_height)  
    axes3 = fig3.add_subplot(1,1,1)           
    axes3.plot(Exp_APC_SF_freqency_spectrum , Exp_broadband_APC[0,:], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,    label = 'Exp. 45 $\degree$ mic.')    
    axes3.plot(Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[1,4,8:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE 45 $\degree$ mic')     
    axes3.set_xscale('log') 
    axes3.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes3.set_xlabel('Frequency (Hz)') 
    axes3.legend(loc='lower right', prop={'size': PP.legend_font})  
    axes3.set_ylim([15,50])   
    

    fig4 = plt.figure('Noise_Validation_Broadband_1_3_Spectrum_22_deg')    
    fig4.set_size_inches(fig_size_width,fig_size_height)  
    axes4 = fig4.add_subplot(1,1,1)            
    axes4.plot(Exp_APC_SF_freqency_spectrum , Exp_broadband_APC[1,:], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m , linewidth = PP.lw,   label = 'Exp. 22.5 $\degree$ mic.')     
    axes4.plot(Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[1,3,8:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw,  label = 'SUAVE 22.5 $\degree$ mic.')   
    axes4.set_xscale('log') 
    axes4.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes4.set_xlabel('Frequency (Hz)') 
    axes4.legend(loc='lower right', prop={'size': PP.legend_font}) 
    axes4.set_ylim([15,50])   
    
    

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()    
    fig1_name = 'Noise_Validation_Total_1_3_Spectrum_3600' 
    fig2_name = 'Noise_Validation_1_3_Spectrum_4800' 
    fig3_name = 'Noise_Validation_Broadband_1_3_Spectrum_45_deg' 
    fig4_name = 'Noise_Validation_Broadband_1_3_Spectrum_22_deg' 
    if save_figures:
        fig1.savefig(fig1_name  + '.pdf')               
        fig2.savefig(fig2_name  + '.pdf')               
        fig3.savefig(fig3_name  + '.pdf')               
        fig4.savefig(fig4_name  + '.pdf')        
    return 



# ------------------------------------------------------------------ 
# High Fidelity Comparison
# ------------------------------------------------------------------     
def High_Fidelity_Validation_1(PP,save_figures): 

    DJI_CF = design_DJI_9_4x5_prop()
    DJI_CF_inflow_ratio = 0.05 
    
    # Atmosheric conditions 
    a                     = 343   
    density               = 1.225
    dynamic_viscosity     = 1.78899787e-05   
    T                     = 286.16889478 

    # ---------------------------------------------------------------------------------------------------------------------------
    # APC SF Rotor
    # ---------------------------------------------------------------------------------------------------------------------------
    # Define Network
    net_DJI_CF                                  = Battery_Propeller()
    net_DJI_CF.number_of_propeller_engines      = 1        
    net_DJI_CF.identical_propellers             = True  
    net_DJI_CF.propellers.append(DJI_CF)    

    # Run conditions                            
    DJI_CF_RPM                                  = np.array([6000])
    DJI_CF_omega_vector                         = DJI_CF_RPM * Units.rpm 
    ctrl_pts                                    = len(DJI_CF_omega_vector)   
    velocity                                    = DJI_CF_inflow_ratio*DJI_CF_omega_vector*DJI_CF.tip_radius 
    theta                                       = np.array([120.5])  
    S                                           = 1.51

    # Microphone Locations 
    positions = np.zeros((len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees),-S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2),-S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Define conditions 
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
    DJI_CF_thrust, DJI_CF_torque, DJI_CF_power, DJI_CF_Cp, acoustic_outputs  , DJI_CF_etap  =  DJI_CF.spin(DJI_CF_conditions)  
    
    # Prepare Inputs for Noise Model  
    DJI_CF_conditions.noise.total_microphone_locations             = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    DJI_CF_conditions.aerodynamics.angle_of_attack                 = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    DJI_CF_segment                                                 = Segment() 
    DJI_CF_segment.state.conditions                                = DJI_CF_conditions
    DJI_CF_segment.state.conditions.expand_rows(ctrl_pts)
    DJI_CF_settings                                                = Data()
    DJI_CF_settings                                                = setup_noise_settings(DJI_CF_segment)   
    
    # Run Noise Model    
    DJI_CF_propeller_noise             = propeller_mid_fidelity(net_DJI_CF.propellers,acoustic_outputs,DJI_CF_segment,DJI_CF_settings )    
    DJI_CF_1_3_Spectrum                = DJI_CF_propeller_noise.SPL_1_3_spectrum
    SUAVE_DJI_CF_freqency_spectrum     = DJI_CF_propeller_noise.one_third_frequency_spectrum

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------   
    
    CFD_Raw_Data = np.array([[ 69.55835528038423, 23.910162002945505],
                             [ 74.95606870724923, 22.4742268041237],
                             [ 80.48569557985319, 21.5169366715758],
                             [ 84.5974032599167, 23.04860088365242],
                             [ 90.1940198579585, 25.15463917525772],
                             [ 92.79883067514318, 27.4521354933726],
                             [ 95.1396762529096, 29.07952871870397],
                             [ 98.23630633704119, 29.4624447717231],
                             [ 101.07337905860273, 28.696612665684],
                             [ 104.73521672187914, 27.356406480117],
                             [ 106.99569547895555, 25.729013254786],
                             [ 109.69465621779004, 23.240058910162],
                             [ 112.06217354296606, 19.985272459499],
                             [ 115.2985384627429, 17.6877761413843],
                             [ 119.05130425848742, 15.677466863033],
                             [ 127.83390588901582, 14.050073637702],
                             [ 136.776776663866, 12.99705449189984],
                             [ 146.34526344520137, 12.135493372606],
                             [ 161.6796395554053, 12.1354933726067],
                             [ 169.93923662890387, 12.805596465390],
                             [ 176.09604965872626, 15.677466863033],
                             [ 179.89669467496418, 18.453608247422],
                             [ 182.47592092651718, 22.569955817378],
                             [ 185.09212622356455, 26.399116347569],
                             [ 184.43457924873488, 29.845360824742],
                             [ 185.752017487767, 33.100147275405,],
                             [ 186.41426139921305, 37.025036818851],
                             [ 188.41519304332115, 41.811487481590],
                             [ 190.43760226867423, 46.119293078055],
                             [ 192.48171961113133, 48.512518409425],
                             [ 194.54777808107562, 50.714285714285],
                             [ 198.74666297722766, 52.150220913107],
                             [ 203.76003700081526, 49.661266568483],
                             [ 205.21551929391302, 46.597938144329],
                             [ 207.41825970301693, 43.247422680412],
                             [ 208.15774819670213, 39.992636229749],
                             [ 208.89987311802452, 36.642120765832],
                             [ 211.14216060952106, 33.961708394698],
                             [ 214.16936132845393, 30.324005891016],
                             [ 214.93291887940973, 26.973490427098],
                             [ 218.79173484189377, 24.005891016200],
                             [ 220.3545903489194, 19.9852724594992],
                             [ 220.3545903489194, 18.0706921944035],
                             [ 221.9286095195892, 16.3475699558173],
                             [ 229.1519976562166, 12.9970544918998],
                             [ 243.44382526143528, 10.891016200294],
                             [ 265.1508629053888, 10.4123711340206],
                             [ 287.7674915310098, 10.8910162002945],
                             [ 288.7934414601462, 12.7098674521354],
                             [ 291.8932897833975, 15.0073637702503],
                             [ 295.02641122939934, 16.921944035346],
                             [ 297.13382013970914, 19.123711340206],
                             [ 299.25628252369495, 20.176730486008],
                             [ 302.4684367918573, 21.8041237113401],
                             [ 303.5467985958318, 19.9852724594992],
                             [ 306.8050063650187, 18.1664212076583],
                             [ 310.0981870540856, 15.4860088365242],
                             [ 316.7909727892321, 12.7098674521354],
                             [ 317.9203973616707, 10.3166421207658],
                             [ 324.7820082617151, 9.55081001472753],
                             [ 346.26825143395786, 9.5508100147275],
                             [ 365.2553616078901, 10.3166421207658],
                             [ 373.1385933580606, 12.4226804123711],
                             [ 377.14378487873114, 14.911634756995],
                             [ 381.1919672864412, 17.3048600883652],
                             [ 381.1919672864412, 19.1237113402061],
                             [ 382.5509945683075, 21.7083946980854],
                             [ 385.2836020363696, 24.0058910162002],
                             [ 388.035728846116, 26.97349042709866],
                             [ 390.8075144265365, 29.1752577319587],
                             [ 393.5990992025788, 31.8556701030927],
                             [ 393.5990992025788, 34.3446244477172],
                             [ 396.410624602264, 37.31222385861561],
                             [ 396.410624602264, 40.1840942562592,],
                             [ 395.00236044583676, 42.673048600883],
                             [ 399.2422330638492, 45.6406480117820],
                             [ 400.6656132328214, 46.8851251840942],
                             [ 402.09406804304507, 43.917525773195],
                             [ 403.52761558665446, 41.715758468335],
                             [ 404.9662740202847, 40.1840942562592],
                             [ 409.3130972000225, 37.1207658321060],
                             [ 410.7723820581997, 35.0147275405007],
                             [ 410.7723820581997, 32.3343151693667],
                             [ 412.23686956518543, 30.611192930780],
                             [ 415.1815267857364, 27.7393225331369],
                             [ 419.63799834944155, 24.675994108983],
                             [ 419.63799834944155, 21.612665684830],
                             [ 422.63552271731436, 18.357879234167],
                             [ 424.14230474566455, 14.624447717231],
                             [ 433.2964813432165, 12.7098674521354],
                             [ 458.68515331837864, 11.656848306332],
                             [ 485.5614548784014, 12.3269513991163],
                             [ 489.02987837394375, 14.720176730486],
                             [ 494.2790226787205, 17.6877761413843],
                             [ 497.809716802788, 20.36818851251840],
                             [ 501.36563109689257, 22.187039764359],
                             [ 508.55384208480365, 19.123711340206],
                             [ 515.8451123452334, 15.7731958762886],
                             [ 532.6349686549621, 13.2842415316642],
                             [ 546.0706574655725, 13.3799705449189],
                             [ 569.8964890764943, 15.1030927835051],
                             [ 580.1281728989516, 17.8792341678939],
                             [ 580.1281728989516, 21.2297496318114],
                             [ 586.3551472199041, 23.8144329896907],
                             [ 590.543552104429, 27.16494845360824],
                             [ 590.543552104429, 29.94108983799705],
                             [ 590.543552104429, 32.81296023564064],
                             [ 592.6489605792017, 35.9720176730486],
                             [ 592.6489605792017, 39.0353460972017],
                             [ 594.7618752655476, 42.0029455081001],
                             [ 594.7618752655476, 45.3534609720176],
                             [ 594.7618752655476, 46.9808541973490],
                             [ 596.8823229246466, 49.4698085419734],
                             [ 596.8823229246466, 51.9587628865979],
                             [ 601.1459246832014, 47.8424153166421],
                             [ 605.4399818586072, 45.1620029455081],
                             [ 609.7647119975456, 42.1944035346097],
                             [ 607.5984991503952, 38.8438880706921],
                             [ 611.9386478362862, 35.9720176730486],
                             [ 609.7647119975456, 33.3873343151693],
                             [ 611.9386478362862, 29.9410898379970],
                             [ 620.712173262591, 26.87776141384388],
                             [ 622.9251390384844, 24.2930780559646],
                             [ 622.9251390384844, 20.9425625920471],
                             [ 625.1459944897158, 17.9749631811487],
                             [ 634.1088771205186, 15.0073637702503],
                             [ 673.6571893649573, 14.1458026509572],
                             [ 695.5835528038417, 15.8689248895434],
                             [ 700.5521892419671, 18.0706921944035],
                             [ 705.5563172439086, 20.0810014727540],
                             [ 718.2235810283825, 17.7835051546391],
                             [ 728.5209415108502, 16.7304860088365],
                             [ 746.8978435885908, 14.5287187039764],
                             [ 757.6063145190465, 13.8586156111929],
                             [ 768.4683156151777, 14.1458026509572],
                             [ 776.7168935869888, 17.2091310751104],
                             [ 779.4860480793701, 20.0810014727540],
                             [ 779.4860480793701, 22.9528718703976],
                             [ 782.2650751735533, 26.0162002945508],
                             [ 801.9976701700509, 28.5051546391752],
                             [ 801.9976701700509, 31.4727540500736],
                             [ 801.9976701700509, 34.3446244477172],
                             [ 799.1485422649353, 37.6951399116347],
                             [ 799.1485422649353, 40.8541973490427],
                             [ 801.9976701700509, 44.4918998527246],
                             [ 796.3095359974772, 48.6082474226804],
                             [ 807.7264353647367, 43.3431516936671],
                             [ 807.7264353647367, 40.4712812960235],
                             [ 807.7264353647367, 37.5994108983799],
                             [ 810.6061452121388, 34.1531664212076],
                             [ 813.4961218137803, 31.2812960235640],
                             [ 816.3964017727401, 28.2179675994108],
                             [ 822.2280188278746, 25.0589101620029],
                             [ 822.2280188278746, 22.3784977908689],
                             [ 825.1594297845548, 19.6023564064801],
                             [ 825.1594297845548, 16.8262150220913],
                             [ 845.9740325991662, 13.6671575846833],
                             [ 879.7485871677239, 15.3902798232695],
                             [ 901.940198579584, 18.64506627393224],
                             [ 901.940198579584, 17.20913107511045],
                             [ 914.8715525530995, 16.2518409425625],
                             [ 924.691592211318, 13.95434462444771],
                             [ 965.0371882744997, 15.0073637702503],
                             [ 978.8731803933073, 16.9219440353460],
                             [ 985.865388493687, 19.88954344624447],
                             [ 985.865388493687, 23.14432989690721],
                             [ 985.865388493687, 26.11192930780559],
                             [ 992.9075427720785, 29.9410898379970],
                             [ 1003.5652044074799, 32.717231222385],
                             [ 1003.5652044074799, 36.546391752577],
                             [ 1007.1431194974289, 31.281296023564],
                             [ 1014.3372631510144, 29.462444771723],
                             [ 1017.9535828322735, 26.973490427098],
                             [ 1025.2249469172825, 24.197349042709],
                             [ 1021.5827954323992, 21.134020618556],
                             [ 1039.9238667702202, 18.549337260677],
                             [ 1085.2972092106315, 19.793814432989],
                             [ 1085.2972092106315, 23.335787923416],
                             [ 1096.9465621778993, 26.877761413843],
                             [ 1100.8574008961482, 28.600883652430],
                             [ 1104.7821825538322, 26.590574374079],
                             [ 1104.7821825538322, 24.005891016200],
                             [ 1108.7209568603805, 20.846833578792],
                             [ 1124.6169809799335, 19.219440353460],
                             [ 1132.650254463912, 16.8262150220913],
                             [ 1186.283698713695, 18.6450662739322],
                             [ 1207.5817413611296, 21.325478645066],
                             [ 1194.7574649314613, 26.016200294550],
                             [ 1199.017019511307, 28.8880706921944],
                             [ 1207.5817413611296, 31.759941089837],
                             [ 1216.2076420425867, 34.536082474226],
                             [ 1216.2076420425867, 30.898379970544],
                             [ 1211.887017107825, 27.4521354933726],
                             [ 1224.8951585633856, 24.771723122238],
                             [ 1255.7930739173844, 21.899852724594],
                             [ 1260.2702329193999, 19.027982326951],
                             [ 1273.7977096813606, 16.730486008836],
                             [ 1287.4703875460414, 19.123711340206],
                             [ 1301.2898250716732, 21.229749631811],
                             [ 1301.2898250716732, 24.580265095729],
                             [ 1319.9467597294647, 20.081001472754],
                             [ 1319.9467597294647, 18.262150220913],
                             [ 1324.6526397348935, 16.921944035346],
                             [ 1334.114791833106, 15.2945508100147],
                             [ 1382.4490994160763, 16.826215022091],
                             [ 1387.3778130384337, 18.262150220913],
                             [ 1397.2880185450517, 20.751104565537],
                             [ 1397.2880185450517, 23.335787923416],
                             [ 1407.2690138338476, 26.111929307805],
                             [ 1412.2862155244807, 28.888070692194],
                             [ 1402.2696359472902, 32.334315169366],
                             [ 1412.2862155244807, 35.493372606774],
                             [ 1412.2862155244807, 31.759941089837],
                             [ 1417.3213045646944, 28.026509572901],
                             [ 1432.5345346410054, 24.484536082474],
                             [ 1437.6418130777774, 21.229749631811],
                             [ 1453.0731592370723, 18.357879234167],
                             [ 1500.3681494720213, 16.921944035346]])
    
    
    
    Exp_Raw_Data = np.array([[ 69.80634503520706, 23.52724594992636],
                             [71.82235810283828, 22.0913107511045],
                             [78.22650751735542, 21.8998527245949],
                             [81.06061452121388, 21.9955817378497],
                             [87.97485871677239, 22.4742268041237],
                             [89.55432262990162, 23.8144329896907],
                             [91.48715525530994, 25.9204712812960],
                             [92.4691592211319, 27.4521354933726],
                             [95.1396762529096, 29.74963181148747],
                             [96.1608856142308, 30.80265095729013],
                             [99.64474611197916, 29.5581737849779],
                             [104.36314079234772, 28.409425625920],
                             [105.10861917815456, 26.399116347569],
                             [106.99569547895555, 24.293078055964],
                             [108.91665156043393, 21.899852724594],
                             [110.47821825538333, 19.410898379970],
                             [112.06217354296606, 16.730486008836],
                             [117.787003232039, 15.29455081001472],
                             [122.05436708884113, 14.145802650957],
                             [128.28965989371684, 12.805596465390],
                             [135.80669322560456, 13.188512518409],
                             [143.76418130777805, 10.986745213549],
                             [152.1879322454407, 15.1030927835051],
                             [159.96263413896816, 15.486008836524],
                             [168.13451594447287, 13.475699558173],
                             [176.72386807110973, 15.868924889543],
                             [179.89669467496418, 20.272459499263],
                             [182.47592092651718, 22.761413843888],
                             [184.43457924873488, 24.197349042709],
                             [185.752017487767, 28.12223858615610],
                             [187.07886634557082, 31.664212076583],
                             [187.07886634557082, 33.770250368188],
                             [188.41519304332115, 38.843888070692],
                             [190.43760226867423, 42.002945508100],
                             [190.43760226867423, 44.587628865979],
                             [191.7979208184834, 47.5552282768777],
                             [191.7979208184834, 49.6612665684830],
                             [192.48171961113133, 51.001472754050],
                             [200.1663341405894, 54.2562592047128],
                             [205.21551929391302, 50.905743740795],
                             [209.64464386638733, 48.225331369661],
                             [210.39206987470473, 43.438880706921],
                             [211.14216060952106, 40.758468335787],
                             [212.65037429370062, 37.886597938144],
                             [211.89492557113138, 35.493372606774],
                             [214.16936132845393, 32.908689248895],
                             [214.16936132845393, 30.132547864506],
                             [214.93291887940973, 26.399116347569],
                             [216.46821040289677, 23.814432989690],
                             [221.14019950564025, 21.804123711340],
                             [226.7184492994475, 19.1237113402061],
                             [232.43741014819489, 15.677466863033],
                             [242.57898160704735, 14.241531664212],
                             [246.93414344669827, 10.986745213549],
                             [254.971417983597, 10.69955817378497],
                             [265.1508629053888, 9.74226804123711],
                             [276.7197683660758, 10.0294550810014],
                             [289.82304911049175, 10.986745213549],
                             [290.85632752257004, 14.624447717231],
                             [293.9783184328186, 19.4108983799705],
                             [297.13382013970914, 23.431516936671],
                             [303.5467985958318, 21.4212076583210],
                             [305.7150696512644, 20.5596465390279],
                             [307.8988289259488, 16.2518409425625],
                             [310.0981870540856, 12.7098674521354],
                             [312.3132554601496, 9.16789396170839],
                             [320.1913407506959, 12.1354933726067],
                             [328.26815032450565, 12.326951399116], 
                             [337.748561888921, 7.731958762886592],
                             [343.8123586712754, 11.5611192930780],
                             [352.48500716826436, 11.656848306332], 
                             [361.3764227632221, 8.97643593519880],
                             [370.4921238445818, 11.2739322533136],
                             [378.48837956283484, 9.8379970544918],
                             [383.91486706022897, 15.964653902798], 
                             [386.6572168324801, 21.0382916053019],
                             [389.4191555368586, 25.4418262150220],
                             [388.035728846116, 29.65390279823269],
                             [389.4191555368586, 33.3873343151693],
                             [390.8075144265365, 37.1207658321060],
                             [392.200823099447, 42.00294550810015],
                             [403.52761558665446, 44.779086892488], 
                             [406.4100615653033, 40.8541973490427],
                             [407.85899650804095, 37.599410898379],
                             [412.23686956518543, 34.153166421207],
                             [412.23686956518543, 30.228276877761],
                             [416.66173379493756, 23.910162002945], 
                             [419.63799834944155, 18.549337260677],
                             [418.1472180446923, 17.4963181148748],
                             [425.6544587599433, 14.7201767304860],
                             [430.2233445822813, 11.8483063328424],
                             [434.84127186824776, 8.9764359351988],
                             [442.64823066162637, 8.8807069219440],
                             [450.5953523363692, 12.0397643593519],
                             [465.2614430649681, 14.3372606774668],
                             [475.3030856132226, 15.7731958762886],
                             [480.404889399088, 10.5081001472754],
                             [490.77336985171286, 15.581737849779], 
                             [494.2790226787205, 19.6980854197349],
                             [492.5230772329829, 21.9955817378497],
                             [496.0412284289005, 26.0162002945508],
                             [503.15310205463913, 22.378497790868],
                             [508.55384208480365, 16.730486008836],
                             [512.1865029496925, 13.8586156111929],
                             [514.0125525274615, 10.8910162002945],
                             [523.2409194660927, 13.7628865979381],
                             [530.7427622198568, 12.4226804123711],
                             [540.2714963640017, 15.1030927835051],
                             [546.0706574655725, 12.6141384388807],
                             [553.8998161807044, 13.2842415316642],
                             [569.8964890764943, 12.3269513991163],
                             [580.1281728989516, 11.4653902798232],
                             [580.1281728989516, 15.9646539027982],
                             [584.2720977617951, 20.2724594992636],
                             [588.4456231751216, 23.6229749631811],
                             [590.543552104429, 27.26067746686303],
                             [590.543552104429, 29.84536082474226],
                             [592.6489605792017, 34.1531664212076],
                             [596.8823229246466, 38.0780559646539],
                             [601.1459246832014, 45.1620029455081],
                             [605.4399818586072, 39.2268041237113],
                             [605.4399818586072, 34.6318114874815],
                             [609.7647119975456, 31.3770250368188],
                             [609.7647119975456, 28.2179675994108],
                             [614.1203342006601, 24.1973490427098],
                             [616.3097987228765, 21.3254786450662],
                             [618.5070691336576, 17.8792341678939],
                             [616.3097987228765, 16.9219440353460],
                             [645.4934033705825, 16.7304860088365],
                             [652.4219943870244, 13.9543446244477],
                             [661.7759402060622, 14.9116347569955],
                             [666.503084827478, 16.73048600883652],
                             [676.0589149456132, 18.2621502209131],
                             [683.3155897624569, 19.8895434462444],
                             [690.6501562071355, 18.8365243004418],
                             [695.5835528038417, 20.3681885125184],
                             [708.0717697358732, 20.8468335787923],
                             [715.6720638320966, 19.1237113402061],
                             [728.5209415108502, 16.3475699558173],
                             [733.7248536524431, 14.2415316642120],
                             [746.8978435885908, 12.3269513991163],
                             [754.914888630826, 9.837997054491893],
                             [760.3073358907059, 13.0927835051546],
                             [771.2080622410192, 17.3048600883652],
                             [779.4860480793701, 18.8365243004418],
                             [782.2650751735533, 21.5169366715758],
                             [785.0540100673812, 25.1546391752577],
                             [787.852888084184, 29.84536082474226],
                             [793.4806154101655, 35.4933726067746],
                             [796.3095359974772, 39.5139911634756],
                             [799.1485422649353, 44.2047128129602],
                             [807.7264353647367, 41.3328424153166],
                             [807.7264353647367, 37.6951399116347],
                             [810.6061452121388, 34.4403534609720],
                             [816.3964017727401, 29.7496318114874],
                             [819.3070218225927, 26.3991163475699],
                             [819.3070218225927, 22.0913107511045],
                             [816.3964017727401, 18.4536082474226],
                             [828.1012918204981, 11.5611192930780],
                             [836.9899576709188, 16.2518409425625],
                             [845.9740325991662, 17.4963181148748],
                             [852.0169262057456, 15.6774668630338],
                             [858.1029849234661, 20.0810014727540],
                             [870.4058332289866, 22.9528718703976],
                             [876.6232461070022, 20.4639175257731],
                             [882.8850707081695, 23.4315169366715],
                             [892.3617741686808, 21.3254786450662],
                             [905.1557997508451, 20.0810014727540],
                             [908.3828651975746, 17.4005891016200],
                             [921.4065893777855, 18.4536082474226],
                             [927.9883067514304, 18.9322533136966],
                             [937.9491389023832, 21.0382916053019],
                             [937.9491389023832, 14.7201767304860],
                             [948.0168885397069, 17.4963181148748],
                             [944.6490216840754, 13.4756995581737],
                             [954.7886864601287, 15.1030927835051],
                             [961.6088561423076, 19.2194403534609],
                             [978.8731803933073, 18.5493372606774],
                             [982.3630633704113, 21.9955817378497],
                             [992.9075427720785, 24.3888070692194],
                             [996.4474611197905, 27.7393225331369],
                             [996.4474611197905, 32.9086892488954],
                             [1003.5652044074799, 29.558173784977],
                             [1010.7337905860262, 26.303387334315],
                             [1017.9535828322735, 22.665684830633],
                             [1025.2249469172825, 20.368188512518],
                             [1043.631407923475, 19.1237113402061],
                             [1066.1558911075163, 20.463917525773],
                             [1085.2972092106315, 21.134020618556],
                             [1081.44164867833, 13.09278350515463],
                             [1093.0496168662514, 27.739322533136],
                             [1104.7821825538322, 29.749631811487],
                             [1112.6737737024466, 21.612665684830],
                             [1116.6406831445402, 18.740795287187],
                             [1120.6217354296596, 16.730486008836],
                             [1136.688384143262, 16.2518409425625],
                             [1144.807885102539, 19.8895434462444],
                             [1165.3612882947687, 13.188512518409],
                             [1177.8700323203875, 19.027982326951],
                             [1186.283698713695, 26.2076583210603],
                             [1190.513042584873, 31.6642120765832],
                             [1203.2917602739144, 33.674521354933],
                             [1211.887017107825, 28.5051546391752],
                             [1220.5436708884101, 23.910162002945],
                             [1220.5436708884101, 19.506627393225],
                             [1220.5436708884101, 15.486008836524],
                             [1220.5436708884101, 12.997054491899],
                             [1220.5436708884101, 5.6259204712812],
                             [1242.4568027841135, 14.050073637702],
                             [1246.8864152535054, 9.6465390279823],
                             [1255.7930739173844, 13.284241531664],
                             [1264.7633539084225, 10.220913107511],
                             [1287.4703875460414, 13.667157584683],
                             [1287.4703875460414, 8.8807069219440],
                             [1287.4703875460414, 16.826215022091],
                             [1305.9291892914302, 20.463917525773],
                             [1319.9467597294647, 15.007363770250],
                             [1315.2575975457219, 11.082474226804],
                             [1319.9467597294647, 7.9234167893961],
                             [1338.8711837690362, 12.039764359351],
                             [1367.76776663866, 0.839469808541963],
                             [1377.5378952404928, 12.135493372606],
                             [1412.2862155244807, 27.260677466863],
                             [1427.4454000094534, 10.508100147275],
                             [1437.6418130777774, 5.0515463917525],
                             [1484.4345399811975, 17.879234167893],
                             [1500.3681494720213, 8.689248895434]])

    
      
     
    
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------   
 
    fig1 = plt.figure('Noise_Validation_High_Fi_DJI')    
    fig1.set_size_inches(PP.figure_width,PP.figure_height)  
    axes1 = fig1.add_subplot(1,1,1)      
    axes1.plot(Exp_Raw_Data[:,0], Exp_Raw_Data[:,1] , color = PP.Elc[0] , linestyle = PP.Els, linewidth = PP.lw, label = 'Exp.')   
    axes1.plot(CFD_Raw_Data[:,0], CFD_Raw_Data[:,1] , color = PP.Rlc[0] , linestyle = PP.Rls, linewidth = PP.lw, label = 'CFD')          
    axes1.plot(SUAVE_DJI_CF_freqency_spectrum ,  DJI_CF_1_3_Spectrum[0,0,:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label =' SUAVE')    
    axes1.set_xscale('log') 
    axes1.set_ylabel(r'SPL (dB)')
    axes1.set_xlabel('Frequency (Hz)') 
    axes1.legend(framealpha  = 1, loc='upper right', prop={'size': PP.legend_font})  
    axes1.set_ylim([0,90])   
    axes1.set_xlim([70,1700])    
    axes1.minorticks_on()
    axes1.grid(which='major', linestyle='--', linewidth='1.', color='grey')
    axes1.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
    fig1.tight_layout() 
    fig1_name = 'Noise_Validation_High_Fidelity_DJI'
    if save_figures:
        fig1.savefig(fig1_name  + '.pdf')   
    
    
    
    return 


# ------------------------------------------------------------------ 
# High Fidelity Validation 2 
# ------------------------------------------------------------------     
def High_Fidelity_Validation_2(PP,save_figures): 
    # Define Network
    net                              = Battery_Propeller()
    net.number_of_propeller_engines  = 1                                      
    prop                             = design_SR2_8_blade_prop()     
    net.identical_propellers         = True  
    net.propellers.append(prop) 

    # Atmosheric & Run Conditions         

    a                                = 343   
    density                          = 1.225
    dynamic_viscosity                = 1.78899787e-05   
    T                                = 286.16889478  
    theta                            = np.array([46.8,50,58.5,72.2,80,90.9,100,104,110,116.8,120,130.4])  
    Mach                             = 0.6
    velocity                         = Mach * a  
    J                                = 3.06 
    n                                = velocity /(2*prop.tip_radius  *J)
    X                                = np.array([-46.7,-41.7, -30.5, - 16., -8.9 , 0.8, 8.9, 12.4, 18.0, 25.0, 28.7, 42.4]) /100
    test_omega                       = n*2*np.pi
    ctrl_pts                         = 1

    # Set twist target
    three_quarter_twist              = 59 * Units.degrees 
    n                                = len(prop.twist_distribution)
    beta                             = prop.twist_distribution
    beta_75                          = beta[round(n*0.75)] 
    delta_beta                       = three_quarter_twist-beta_75
    prop.twist_distribution          = beta +  delta_beta  

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
    F, Q, P, Cp_UI , noise_data , etap                        = prop.spin(conditions) 
     
    print(Cp_UI)

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees  
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts)
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment) 
    num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
    conditions.noise.number_of_microphones                 = num_mic 

    # Run Noise Model  
    propeller_noise  = propeller_mid_fidelity(net.propellers,noise_data,segment,settings )   
    SUAVE_SPL        = propeller_noise.SPL 


    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ------------------------------------------------------------------------------------------------------
    Exp_Raw_Data = np.array([[49.880239520958085, 131.690140845070],
                             [58.41317365269461, 136.6197183098591],
                             [72.18562874251496, 140.7042253521127],
                             [79.97005988023952, 144.0845070422535],
                             [90.74850299401197, 144.9295774647887],
                             [99.73053892215569, 143.5211267605634],
                             [104.07185628742516, 142.253521126760],
                             [110.05988023952096, 139.859154929577],
                             [116.79640718562874, 136.056338028169],
                             [119.94011976047904, 133.661971830985],
                             [130.7185628742515, 134.0845070422535]])
    CFD_Raw_Data = np.array([[46.886227544910184, 102.67605633802819],
                             [49.880239520958085, 107.605633802816],
                             [58.26347305389221, 118.8732394366197],
                             [72.18562874251496, 132.8169014084507],
                             [79.97005988023952, 137.7464788732394],
                             [90.89820359281435, 140.2816901408451],
                             [99.8802395209581, 138.16901408450707],
                             [103.92215568862275, 135.915492957746],
                             [109.91017964071855, 131.830985915492],
                             [116.79640718562874, 126.056338028169],
                             [120.08982035928142, 122.816901408450],
                             [130.5688622754491, 110.98591549295776]])
    
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots
    # ----------------------------------------------------------------------------------------------------------------------------------------      
    yerr = 0.05*CFD_Raw_Data[:,1]
    fig = plt.figure('Noise_Validation_High_Fi_SR2')    
    fig.set_size_inches(PP.figure_width,PP.figure_height)  
    axes = fig.add_subplot(1,1,1)       
    axes.plot(Exp_Raw_Data[:,0], Exp_Raw_Data[:,1] , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm , markersize = PP.m, linewidth = PP.lw, label = 'Exp.') 
    axes.errorbar(CFD_Raw_Data[:,0], CFD_Raw_Data[:,1] , linestyle = PP.Rls , color = PP.Rlc[0] , yerr=yerr , marker = PP.Rlm , markersize = PP.m,linewidth = PP.lw, label = 'CFD')
    axes.plot(theta            , SUAVE_SPL[0,:]    , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw, label ='SUAVE')             
    axes.set_ylabel(r'SPL (dB)')
    axes.set_xlabel(r'Microphone Angle ($\degree$)') 
    axes.legend(loc='upper right', prop={'size': PP.legend_font}) 
    axes.set_ylim([90,160])   
    axes.set_xlim([40,140])    
    fig.tight_layout() 
    fig_name = 'Noise_Validation_High_Fidelity_SR2'  
    if save_figures:
        fig.savefig(fig_name  + '.pdf')   
    
    return  



# ------------------------------------------------------------------ 
# Load sensitivity
# ------------------------------------------------------------------     
def Skew_function(PP,save_figures):
 
    Radius = 2    
    x = np.linspace(0,np.pi,20)
    x_vals1 = x + 0.5*np.sin(x)
    x_vals2 = x - 0.5*np.sin(x)
    x_vals3 = x  - np.sin(x)
    y_vals1 = np.sin(x) 
    y_vals2 = np.sin(x) 
    y_vals3 = np.sin(x) 

    fig = plt.figure("Test")    
    fig.set_size_inches(8,8)  
    axes = fig.add_subplot(1,1,1)              
    axes.plot(x_vals1,y_vals1 , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw ) 
    axes.plot(x_vals2,y_vals2 , color = PP.Rlc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw )
    axes.plot(x_vals3,y_vals3 , color = PP.Elc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw ) 
    axes.set_ylabel(r'Load')
    axes.set_xlabel('Radius')
    
    Area1 = np.trapz(y_vals1,x = x) 
    Area2 = np.trapz(y_vals2,x = x)
    Area3 = np.trapz(y_vals3,x = x)     
    
    Thrust = 1000 
    y_vals1_T = Thrust*y_vals1/Area1
    y_vals2_T = Thrust*y_vals2/Area2
    y_vals3_T = Thrust*y_vals3/Area3
     
    f1 = sp.interpolate.interp1d(x_vals1,y_vals1_T)
    f2 = sp.interpolate.interp1d(x_vals2,y_vals2_T)
    f3 = sp.interpolate.interp1d(x_vals3,y_vals3_T) 
    
    
    R =  x* (1.2/np.pi)
    y_vals1_T_R = f1(x)
    y_vals2_T_R = f2(x)
    y_vals3_T_R = f3(x)
    
     
    fig = plt.figure("True")    
    fig.set_size_inches(8,8)  
    axes = fig.add_subplot(1,1,1)              
    axes.plot(x,y_vals1_T_R, color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw ) 
    axes.plot(x,y_vals2_T_R, color = PP.Rlc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw )
    axes.plot(x,y_vals3_T_R, color = PP.Elc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw ) 
    axes.set_ylabel(r'Load')
    axes.set_xlabel('Radius')
    
    Area1_final = np.trapz(y_vals1_T_R,x = x)  
    Area2_final = np.trapz(y_vals2_T_R,x = x) 
    Area3_final = np.trapz(y_vals3_T_R,x = x)   
    
    print('Area under load distribution 1: ' + str(Area1_final))
    print('Area under load distribution 2: ' + str(Area2_final))
    print('Area under load distribution 3: ' + str(Area3_final))
    
    return 
    

def Total_Rotor_Spenctrum_Sensitivity_Validation(PP,save_figures):    

    # Define Network 
    net                                = Battery_Propeller()     
    net.number_of_propeller_engines    = 1
    net.identical_propellers           = True
    prop                               = design_F8745D4_prop()
    net.propellers.append(prop)


    # Set-up Validation Conditions 
    a                       = 343.376
    T                       = 288.16889478  
    density                 = 1.2250	
    dynamic_viscosity       = 1.81E-5  
    theta                   =  np.array([45., 67.5, 90.001, 112.5 , 135.])*Units.degree
    S                       = np.array([4]) # np.linspace(2,10,5) 
    omega                   = np.array([2390,2390,2390,2390]) * Units.rpm     

    # Set twist target
    three_quarter_twist     = 21 * Units.degrees 
    n                       = len(prop.twist_distribution)
    beta                    = prop.twist_distribution
    beta_75                 = beta[round(n*0.75)] 
    delta_beta              = three_quarter_twist-beta_75
    prop.twist_distribution = beta + delta_beta 

    # microphone locations
    ctrl_pts                = len(omega) 
    dim_theta               = len(theta)
    dim_S                   = len(S) 
    num_mic                 = dim_S*dim_theta

    theta                   = np.repeat(np.repeat(np.atleast_2d(theta).T ,dim_S , axis = 1)[np.newaxis,:,:],ctrl_pts, axis = 0)  
    S                       = np.repeat(np.repeat(np.atleast_2d(S)       ,dim_theta, axis = 0)[np.newaxis,:,:],ctrl_pts, axis = 0) 
    x_vals                  = S*np.sin(theta)
    y_vals                  = -S*np.cos(theta)
    z_vals                  = np.zeros_like(x_vals) 

    mic_locations           = np.zeros((ctrl_pts,num_mic,3))   
    mic_locations[:,:,0]    = x_vals.reshape(ctrl_pts,num_mic) 
    mic_locations[:,:,1]    = y_vals.reshape(ctrl_pts,num_mic) 
    mic_locations[:,:,2]    = z_vals.reshape(ctrl_pts,num_mic)     

    # Set up for Propeller Model
    prop.inputs.omega                            = np.atleast_2d(omega).T
    prop.inputs.pitch_command                    = 0.
    conditions                                   = Aerodynamics()
    conditions._size                             = 4
    conditions.freestream.density                = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity      = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound         = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature            = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector   = np.array([[77.2, 0. ,0.],[ 77.0,0.,0.], [ 77.2, 0. ,0.], [ 77.2, 0. ,0.]])
    conditions.propulsion.throttle               = np.ones((ctrl_pts,1))*1.0
    conditions.aerodynamics.angle_of_attack      = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    conditions.frames.body.transform_to_inertial = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
    conditions.noise.total_microphone_locations  = mic_locations

    # Run Propeller model 
    F, Q, P, Cp , acoustic_outputs , etap                        = prop.spin(conditions)   
    
    # Modify Thrust and Torque Distribution  
    R             = prop.tip_radius 
    r_1d          = prop.radius_distribution
    chi           = r_1d/R
    diff_r        = np.diff(r_1d)
    deltar        = np.zeros(len(r_1d))
    deltar[1:-1]  = diff_r[0:-1]/2 + diff_r[1:]/2
    deltar[0]     = diff_r[0]/2
    deltar[-1]    = diff_r[-1]/2 

    old_dT_dr   = acoustic_outputs.blade_dT_dr[0,:]
    old_dQ_dr   = acoustic_outputs.blade_dQ_dr[0,:]
    total_dT_dr = np.sum(old_dT_dr ) # np.trapz(old_dT_dr)
    total_dQ_dr = np.sum(old_dQ_dr ) #np.trapz(old_dQ_dr)  

    x       = np.linspace(0,np.pi,30) 
    x_vals3 = x - 0.5*np.sin(x)
    x_vals2 = x  
    x_vals1 = x + 0.5*np.sin(x)
    y_vals1 = np.sin(x) 
    y_vals2 = np.sin(x) 
    y_vals3 = np.sin(x)  

    Area1 = np.trapz(y_vals1,x = x) 
    Area2 = np.trapz(y_vals2,x = x)
    Area3 = np.trapz(y_vals3,x = x)    

    Skewked_Thurst_1 = y_vals1 * total_dT_dr/(Area1)
    Skewked_Thurst_2 = y_vals2 * total_dT_dr/(Area2)
    Skewked_Thurst_3 = y_vals3 * total_dT_dr/(Area3)
    Skewked_Torque_1  = y_vals1 *total_dQ_dr/(Area1)
    Skewked_Torque_2  = y_vals2 *total_dQ_dr/(Area2)
    Skewked_Torque_3  = y_vals3 *total_dQ_dr/(Area3)

    f1_T = sp.interpolate.interp1d(x_vals1,Skewked_Thurst_1)
    f2_T = sp.interpolate.interp1d(x_vals2,Skewked_Thurst_2)
    f3_T = sp.interpolate.interp1d(x_vals3,Skewked_Thurst_3) 
    f1_Q = sp.interpolate.interp1d(x_vals1,Skewked_Torque_1 )
    f2_Q = sp.interpolate.interp1d(x_vals2,Skewked_Torque_2 )
    f3_Q = sp.interpolate.interp1d(x_vals3,Skewked_Torque_3 )     

    chi_pi   = np.linspace(0,1,30)*np.pi 
    new_dT_dr1= f1_T(x)* 0.104890 
    new_dT_dr2= f2_T(x)* 0.104890 
    new_dT_dr3= f3_T(x)* 0.104890 
    new_dQ_dr1= f1_Q(x)* 0.104890
    new_dQ_dr2= f2_Q(x)* 0.104890
    new_dQ_dr3= f3_Q(x)* 0.104890

    T_Area_final  = np.trapz(old_dT_dr,x  =x)  
    T_Area1_final = np.trapz(new_dT_dr1,x =x)  
    T_Area2_final = np.trapz(new_dT_dr2,x =x) 
    T_Area3_final = np.trapz(new_dT_dr3,x =x)   
    Q_Area_final  = np.trapz(old_dQ_dr,x =x)  
    Q_Area1_final = np.trapz(new_dQ_dr1,x =x)  
    Q_Area2_final = np.trapz(new_dQ_dr2,x =x) 
    Q_Area3_final = np.trapz(new_dQ_dr3,x =x)   


    print('Area under load distribution 1: ' + str(T_Area_final))    
    print('Area under load distribution 1: ' + str(T_Area1_final))
    print('Area under load distribution 2: ' + str(T_Area2_final))
    print('Area under load distribution 3: ' + str(T_Area3_final))
    print('Area under load distribution 1: ' + str(Q_Area_final)) 
    print('Area under load distribution 1: ' + str(Q_Area1_final))
    print('Area under load distribution 2: ' + str(Q_Area2_final))
    print('Area under load distribution 3: ' + str(Q_Area3_final)) 



    acoustic_outputs.blade_dT_dr[0,:] = new_dT_dr1
    acoustic_outputs.blade_dT_dr[1,:] = new_dT_dr2
    acoustic_outputs.blade_dT_dr[2,:] = new_dT_dr3
    acoustic_outputs.blade_dQ_dr[0,:] = new_dQ_dr1
    acoustic_outputs.blade_dQ_dr[1,:] = new_dQ_dr2
    acoustic_outputs.blade_dQ_dr[2,:] = new_dQ_dr3 


    fig = plt.figure()    
    fig.set_size_inches(8,8)  
    axes = fig.add_subplot(1,1,1)              
    axes.plot(r_1d,acoustic_outputs.blade_dT_dr[0,:], color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw ) 
    axes.plot(r_1d,acoustic_outputs.blade_dT_dr[1,:], color = PP.Rlc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw )
    axes.plot(r_1d,acoustic_outputs.blade_dT_dr[2,:], color = PP.Elc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw ) 
    axes.plot(r_1d,acoustic_outputs.blade_dT_dr[3,:], color = PP.Slc[1] , linestyle = PP.Sls, marker = 's' , markersize = PP.m , linewidth = PP.lw ) 
    axes.set_ylabel(r'dT/Dr')
    axes.set_xlabel('Radius')  

    
    # Prepare Inputs for Noise Model    
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts)
    settings                                               = Data()
    settings                                               = setup_noise_settings(segment)

    num_mic                                                = len(conditions.noise.total_microphone_locations[0] )  
    conditions.noise.number_of_microphones                 = num_mic   

    # Run Noise Model  
    propeller_noise  = propeller_mid_fidelity(net.propellers,acoustic_outputs,segment,settings)     
    One_Third_Spectrum       = propeller_noise.SPL_1_3_spectrum  
    freqency_spectrum       =  propeller_noise.one_third_frequency_spectrum 
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------   

    print('Total SPL (dBA) Case 1 : ' + str(propeller_noise.SPL[0])) 
    print('Total SPL (dBA) Case 2 : ' + str(propeller_noise.SPL[1]))
    print('Total SPL (dBA) Case 3 : ' + str(propeller_noise.SPL[2]))
    print('Total SPL (dBA) Case 4 : ' + str(propeller_noise.SPL[3]))     
    
    fig_size_width  = 8 
    fig_size_height = 6
    
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------   

    # Figures 8 and 9 comparison 
    fig1 = plt.figure('Noise_Validation_Total_1_3_Spectrum_3600')    
    fig1.set_size_inches(fig_size_width,fig_size_height)  
    axes1 = fig1.add_subplot(1,1,1)                 
    axes1.plot(freqency_spectrum ,     One_Third_Spectrum[0,0,:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw) 
    axes1.plot(freqency_spectrum ,     One_Third_Spectrum[1,0,:]  , color = PP.Rlc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw)
    axes1.plot(freqency_spectrum ,     One_Third_Spectrum[2,0,:]  , color = PP.Elc[0] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw)  
    axes1.plot(freqency_spectrum ,     One_Third_Spectrum[3,0,:]  , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm , markersize = PP.m , linewidth = PP.lw)    
    axes1.set_xscale('log') 
    axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes1.set_xlabel('Frequency (Hz)')  
    axes1.set_ylim([15,120])  
    
  
    
    return 



def ANOPP2_Validation(PP,save_figures):  
    APC_SF = design_APC_11x_4_7_prop()   
    APC_SF_inflow_ratio  = 0.08
    
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
    APC_SF_RPM                                  = np.array([3600])
    APC_SF_omega_vector                         = APC_SF_RPM * Units.rpm 
    ctrl_pts                                    = len(APC_SF_omega_vector)   
    velocity                                    = APC_SF_inflow_ratio*APC_SF_omega_vector*APC_SF.tip_radius 
    theta                                       = np.linspace(0.01,179.99,35)  
    S                                           = 8*APC_SF.tip_radius 

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
    APC_SF_thrust, APC_SF_torque, APC_SF_power, APC_SF_Cp, acoustic_outputs  , APC_SF_etap  =  APC_SF.spin(APC_SF_conditions)   

    # Prepare Inputs for Noise Model  
    APC_SF_conditions.noise.total_microphone_locations             = np.repeat(positions[ np.newaxis,:,: ],ctrl_pts,axis=0)
    APC_SF_conditions.aerodynamics.angle_of_attack                 = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    APC_SF_segment                                                 = Segment() 
    APC_SF_segment.state.conditions                                = APC_SF_conditions
    APC_SF_segment.state.conditions.expand_rows(ctrl_pts)
    APC_SF_settings                                                = Data()
    APC_SF_settings                                                = setup_noise_settings(APC_SF_segment)   
    
    # Run Noise Model    
    APC_SF_propeller_noise                        = propeller_mid_fidelity(net_APC_SF.propellers,acoustic_outputs,APC_SF_segment,APC_SF_settings )     
    azimuthal_time                                = APC_SF_propeller_noise.azimuthal_time 
    SPL_1_3                                       = APC_SF_propeller_noise.SPL_harmonic_1_3_spectrum
    one_third_frequency_spectrum                  = APC_SF_propeller_noise.one_third_frequency_spectrum 
    azimuthal_harmonic_total_SPL                  = APC_SF_propeller_noise.azimuthal_harmonic_total_SPL 
    azimuthal_harmonic_total_pressure             = APC_SF_propeller_noise.azimuthal_harmonic_total_pressure  
    azimuthal_harmonic_loading_pressure           = APC_SF_propeller_noise.azimuthal_harmonic_loading_pressure    
    azimuthal_harmonic_loading_SPL                = APC_SF_propeller_noise.azimuthal_harmonic_loading_SPL         
    azimuthal_harmonic_thickness_pressure         = APC_SF_propeller_noise.azimuthal_harmonic_thickness_pressure  
    azimuthal_harmonic_thickness_SPL              = APC_SF_propeller_noise.azimuthal_harmonic_thickness_SPL          
    

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Experimental Data
    # ----------------------------------------------------------------------------------------------------------------------------------------       
    fig_size_width  = 8 
    fig_size_height = 6
    
    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------   
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 12,
                  'xtick.labelsize': 12,
                  'ytick.labelsize': 12,
                  'axes.titlesize': 12}
    plt.rcParams.update(parameters)
    
    idx = 17
    # Figures 8 and 9 comparison 
    fig1 = plt.figure('Plot_1')    
    fig1.set_size_inches(fig_size_width,fig_size_height)  
    axes1 = fig1.add_subplot(2,1,1)    
    axes1.plot(azimuthal_time[0,17,0,:,0]  ,azimuthal_harmonic_total_pressure[0,idx,0,:] ,  '-bo')   
    axes1.set_ylabel(r'Pa')
    axes1.set_xlabel('Time')  
    axes2 = fig1.add_subplot(2,1,2)    
    axes2.plot(one_third_frequency_spectrum,SPL_1_3[0,idx]/one_third_frequency_spectrum, '-bo')   
    axes2.set_xscale('log') 
    axes2.set_ylabel(r'PSD (dB/Hz)')
    axes2.set_xlabel('Frequency (Hz)') 
    axes2.set_xlim([900,10E4])  
    axes2.set_ylim([0,60])  
 

    fig2 = plt.figure('Plot_2')    
    fig2.set_size_inches(fig_size_width,fig_size_height)  
    axes3 = fig2.add_subplot(3,1,1)                
    axes4 = fig2.add_subplot(3,1,2)   
    axes5 = fig2.add_subplot(3,1,3)    
    axes3.plot(azimuthal_time[0,idx,0,:,0]  , azimuthal_harmonic_thickness_SPL[0,idx,0,:],  '-r')  
    axes4.plot(azimuthal_time[0,idx,0,:,0]  , azimuthal_harmonic_loading_SPL[0,idx,0,:],  '-b')  
    axes5.plot(azimuthal_time[0,idx,0,:,0]  , azimuthal_harmonic_total_SPL[0,idx,0,:],  '-k')  
    axes3.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes3.set_title(r'Monopole  (Thickness) Source Noise')
    axes4.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes4.set_title(r'Dipole (Loading) Source Noise')
    axes5.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes5.set_title(r'Total Harmonic Noise')
    axes5.set_xlabel('Time')  

    #fig1 = plt.figure('Plot_3')    
    #fig1.set_size_inches(fig_size_width,fig_size_height)  
    #axes1 = fig1.add_subplot(3,1,1)                
    #axes1 = fig1.add_subplot(3,1,2)   
    #axes1 = fig1.add_subplot(3,1,3)    
    #axes1.plot(one_third_frequency_spectrum , azimuthal_harmonic_thickness_SPL[0,idx,0,:] ,  '')  
    #axes1.plot(one_third_frequency_spectrum,  azimuthal_harmonic_loading_SPL[0,idx,0,:] ,  '')  
    #axes1.plot(one_third_frequency_spectrum,  azimuthal_harmonic_total_pressure[0,idx,0,:],  '')  
    #axes1.set_xscale('log') 
    #axes1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    #axes1.set_xlabel('Frequency (Hz)') 
    #axes1.legend(loc='lower right', prop={'size': PP.legend_font}) 
    #axes1.set_ylim([15,60])  

    fig1.tight_layout()    
    fig2.tight_layout()
    
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


    sts.broadband_spectrum_resolution        = 301
    sts.floating_point_precision             = np.float32
    sts.urban_canyon_microphone_z_resolution = 16 
    sts.mic_x_position                       = 0     
    sts.number_of_multiprocessing_workers    = 8
    sts.parallel_computing                   = True # TO BE REMOVED
    sts.lateral_ground_distance              = 1000 * Units.feet  
    sts.level_ground_microphone_min_x        = -50
    sts.level_ground_microphone_max_x        = 1000
    sts.level_ground_microphone_min_y        = -1000 * Units.feet 
    sts.level_ground_microphone_max_y        = 1000 * Units.feet 
    sts.level_ground_microphone_x_resolution = 16 
    sts.level_ground_microphone_y_resolution = 4      
    return sts 

# ------------------------------------------------------------------ 
# Vectorize Function
# ------------------------------------------------------------------ 
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
if __name__ == '__main__': 
    main()    
    plt.show()   
 
