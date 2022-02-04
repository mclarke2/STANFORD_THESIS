import SUAVE 
from SUAVE.Core import Units , Data 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt
from SUAVE.Methods.Propulsion import  rotor_design , propeller_design
import pickle
from SUAVE.Plots.Geometry import plot_propeller
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics                              import Aerodynamics 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars   
from SUAVE.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis                     import airfoil_analysis 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import  compute_naca_4series
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
    plot_parameters.line_style       = '-'
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 7
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.plot_grid        = True  
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
        
    #test_planform()
    single_design_point()
    #plot_results_and_pareto_fronteir(plot_parameters)
    #examine_wall_pressure_spectrum_sensitivity(plot_parameters)
    #examine_blade_geometry_sensitivity(plot_parameters)
    #examine_broadband_sensitivity(plot_parameters)
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
    
    objective_weights = np.linspace(0.0,1.0,11)
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
        #rotor.freestream_velocity             = 0.1 # 500* Units['ft/min'] #130 * Units.mph  
        rotor.freestream_velocity             = inflow_ratio*rotor.angular_velocity*rotor.tip_radius   
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
        rotor                                  = rotor_design(rotor,number_of_airfoil_section_points=100)  
      
        # save rotor geomtry
        opt_weight = str(rotor.optimization_parameters.aeroacoustic_weight)
        opt_weight = opt_weight.replace('.','_')    
        name       = 'Single_Point_Rotor_Design_Thrust_' + str(int(rotor.design_thrust)) + '_Opt_Weight_' + opt_weight
        save_blade_geometry(rotor,name)
        
        #plot_propeller(rotor)  
    
    return  


def plot_results_and_pareto_fronteir(plot_parameters):   
    
     
    objective_weights          = [0.0,0.25,0.5,0.75,1.0] 
    plot_parameters.colors     = cm.viridis(np.linspace(0,1,len(objective_weights)))    
    design_thrust              = (2300*9.81/(8))   
      
    
    axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,fig_1,fig_2,fig_3,fig_4,fig_5,fig_6  = set_up_axes(plot_parameters,design_thrust)
    
    for i in range(len(objective_weights)):   
        # save rotor geomtry
        opt_weight      = str(objective_weights[i])
        opt_weight      = opt_weight.replace('.','_')    
        rotor_file_name = 'Single_Point_Rotor_Design_Thrust_' + str(int(design_thrust)) + '_Opt_Weight_' + opt_weight
        rotor           = load_blade_geometry(rotor_file_name)  
        
        rotor_name  = r'$\alpha$ = ' + str(objective_weights[i]) 
        propeller_geoemtry_comparison_plots(rotor,axis_1,axis_2,axis_3,axis_4,plot_parameters,i, rotor_name)   
    
    
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
  


def examine_blade_geometry_sensitivity(PP):

    objective_weights = [1]     
    PP.colors         = cm.viridis(np.linspace(0,1,len(objective_weights) + 1))   
    design_thrust     = (2300*9.81/(8-1))      
    
    # Set up axes 
    fig_11 = plt.figure('Rotor_Total_SPL_Comparison')    
    fig_11.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_11 = fig_11.add_subplot(1,1,1)    
    axis_11.set_xscale('log') 
    axis_11.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_11.set_xlabel('Frequency (Hz)') 
    axis_11.legend(loc='lower right')  
    axis_11.set_ylim([0,100])


    fig_12 = plt.figure('Rotor_Harmonic_Noise_Comparison') 
    fig_12.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_12 = fig_12.add_subplot(1,1,1)      
    axis_12.set_xscale('log')
    axis_12.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_12.set_xlabel('Frequency (Hz)') 
    axis_12.legend(loc='lower right') 
    axis_12.set_ylim([0,100])


    # Figures 16 a Comparison     
    fig_13 = plt.figure('Rotor_Broadband_Noise_Comparison')    
    fig_13.set_size_inches(PP.figure_width, PP.figure_height) 
    axis_13 = fig_13.add_subplot(1,1,1)    
    axis_13.set_xscale('log')
    axis_13.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axis_13.set_xlabel('Frequency (Hz)') 
    axis_13.legend(loc='lower right')  
    axis_13.set_ylim([0,100])
    
    
    axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,fig_1,fig_2,fig_3,fig_4,fig_5,fig_6  = set_up_axes(PP,design_thrust)
    for idx in range(len(objective_weights) + 1):  
        if idx == len(objective_weights):
            rotor                                 = SUAVE.Components.Energy.Converters.Rotor() 
            rotor.tag                             = 'rotor'     
            rotor.tip_radius                      = 1.25
            rotor.hub_radius                      = 0.15 * rotor.tip_radius
            rotor.design_tip_mach                 = 0.65 # gives better noise results and more realistic blade 
            rotor.number_of_blades                = 3  
            inflow_ratio                          = 0.1
            rotor.angular_velocity                = rotor.design_tip_mach*343 /rotor.tip_radius
            #rotor.freestream_velocity             = 0.1 # 500* Units['ft/min'] #130 * Units.mph  
            rotor.freestream_velocity             = inflow_ratio*rotor.angular_velocity*rotor.tip_radius   
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
            rotor_tag                              = 'T:' + str(int(design_thrust)) + 'Adkins & Leibeck Method'
            rotor_name                             = 'Adkins & Leibeck Method'
        else:
            # save rotor geomtry
            opt_weight      = str(objective_weights[idx])
            opt_weight      = opt_weight.replace('.','_')    
            rotor_file_name = 'Single_Point_Rotor_Design_Thrust_' + str(int(design_thrust)) + '_Opt_Weight_' + opt_weight
            rotor           = load_blade_geometry(rotor_file_name)
            rotor_tag       = 'T:' + str(int(design_thrust)) + r', $\alpha$' + str(objective_weights[idx])
            rotor_name      = r'$\alpha$ = ' + str(objective_weights[idx]) 
    
    
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
        
        if  idx == len(objective_weights):
            rotor.design_SPL_dBA = np.mean(propeller_noise.SPL_dBA) 
     
        propeller_geoemtry_comparison_plots(rotor,axis_1,axis_2,axis_3,axis_4,PP,idx, rotor_name) 
        propeller_performance_comparison_plots(rotor,noise_data,axis_5,axis_6,PP,idx, rotor_name) 
        
        # ----------------------------------------------------------------------------------------------------------------------------------------
        #  Plots  
        # ----------------------------------------------------------------------------------------------------------------------------------------   
    
        axis_11.plot(One_Third_Spectrum , Total_SPL_1_3[0,0] , color = PP.colors[idx] , linestyle = PP.line_style, marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
        axis_12.plot(One_Third_Spectrum , Harmonic_1_3[0,0]  , color = PP.colors[idx] , linestyle = PP.line_style, marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)      
        axis_13.plot(One_Third_Spectrum , Broadband_1_3[0,0] , color = PP.colors[idx] , linestyle = PP.line_style, marker = PP.markers[idx] , markersize = PP.marker_size , linewidth = PP.line_width,  label = rotor_tag)  
  
  
    
    fig_11.tight_layout()
    fig_12.tight_layout()
    fig_13.tight_layout()  
    axis_11.legend(loc='upper left', ncol= 2, prop={'size': PP.legend_font_size})  
    axis_12.legend(loc='upper left', ncol= 2, prop={'size': PP.legend_font_size}) 
    axis_13.legend(loc='upper left', ncol= 2, prop={'size': PP.legend_font_size})    
    fig_11_name = 'Rotor_Total_SPL_Comparison'
    fig_12_name = 'Rotor_Harmonic_Noise_Comparison'
    fig_13_name = 'Rotor_Broadband_Noise_Comparison'
    fig_11.savefig(fig_11_name  + '.pdf')               
    fig_12.savefig(fig_12_name  + '.pdf')               
    fig_13.savefig(fig_13_name  + '.pdf')               
          
    return  



def examine_broadband_sensitivity(PP):
   
    '''Empirical wall-pressure spectral modeling for zero and adverse pressure gradient flows''' 

    # ****** DEFINE INPUTS ****** 

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
   

    reference_spectrum = reference_wall_pressure_spectrum_model()   

    fig  = plt.figure() 
    axis = fig.add_subplot(1,1,1)      
    axis.set_ylim(50,90)
    axis.set_ylabel('10log10($\Phi$)') 
    axis.set_xlabel('Frequency (Hz)')  
    axis.semilogx(frequency,var[0,0,0,:,0],'ks-')  
    axis.semilogx(frequency,reference_spectrum,'r-')    
    
    return 
  
    
def reference_wall_pressure_spectrum_model():

    BSR          = 100 # broadband spectrum resolution
    frequency    = np.linspace(1E2,1E4,BSR) 
    w            = 2*np.pi*frequency 

    delta        = 0.0142
    delta_star   = 0.00236
    Theta        = 0.00157
    Cf           = 0.00217
    dp_dx        = 12140
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



def propeller_geoemtry_comparison_plots(rotor,axis_1,axis_2,axis_3,axis_4,PP,idx,label_name = 'prop'):  
         
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

def propeller_performance_comparison_plots(rotor,outputs,axis_5,axis_6,PP,idx,label_name = 'prop'): 
    axis_5.plot(rotor.radius_distribution, outputs.blade_reynolds_number_distribution[0],
                color      = PP.colors[idx],
                marker     = PP.markers[idx],
                linestyle  = PP.line_style,
                linewidth  = PP.line_width,
                markersize = PP.marker_size,
                label      = label_name) 
    axis_6.plot(rotor.radius_distribution, outputs.blade_effective_angle_of_attack[0],
                color      = PP.colors[idx],
                marker     = PP.markers[idx],
                linestyle  = PP.line_style,
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
    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_5_name = "Rotor_Spanwise_Re_" + str(int(design_thrust))  + '_N'
    fig_5 = plt.figure(fig_5_name)     
    fig_5.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_5 = fig_5.add_subplot(1,1,1)  
    axis_5.set_ylabel(r'Re') 
    axis_5.set_xlabel('r')    
    axis_5.minorticks_on()    
    fig_5.tight_layout()   
    
    

    # ------------------------------------------------------------------
    #  Thickness Distribution
    # ------------------------------------------------------------------ 
    fig_6_name = "Rotor_Spanwise_AoA_" + str(int(design_thrust))  + '_N'
    fig_6 = plt.figure(fig_6_name)     
    fig_6.set_size_inches(PP.figure_width,PP.figure_height) 
    axis_6 = fig_6.add_subplot(1,1,1)  
    axis_6.set_ylabel(r'$\alpha$ ($\degree$)') 
    axis_6.set_xlabel('r')      
    axis_6.minorticks_on()    
    fig_6.tight_layout()       
     
    return axis_1,axis_2,axis_3,axis_4,axis_5,axis_6,fig_1,fig_2,fig_3,fig_4,fig_5,fig_6  


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