# Vehicle Comparisons

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------    
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Plots.Geometry import *  
from SUAVE.Core import Data , Units
import matplotlib.gridspec as gridspec
import scipy.ndimage as ndimage
import numpy as np    
import matplotlib.pyplot as plt 
import pandas as pd
from math import pi   
import numpy as np   
import pickle  
 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 2 
    plot_parameters.line_style       = ['-','--']
    plot_parameters.figure_width     = 10 
    plot_parameters.figure_height    = 7 
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 20 
    plot_parameters.plot_grid        = True   
    plot_parameters.markers          = ['s','^','X','o','P','D','X','*']
    plot_parameters.colors           = ['black','mediumblue','darkgreen','firebrick']   
    plot_parameters.colors2          = ['grey','darkcyan','green','red']   
    plot_parameters.lw               = 2                              # line_width                
    plot_parameters.legend_font      = 20                             # legend_font_size       
    plot_parameters.marker_size      = 14   
     

    N_gm_x = 10 
    N_gm_y = 5  
    header =  '../../../XX_Supplementary/Aircraft_Models_and_Simulations/' 
    alpha_weights = np.array([1.0,0.5,0.0])
    vehicle_name  = 'SR'
    alpha_opt_weight = 'None'
    spider_plot_max_SPL             = []
    spider_plot_maxiumum_power      = []
    spider_plot_energy_consumption  = []
    spider_plot_tip_mach            = []
    spider_plot_bat_temperature     = []
       
    
    # ------------------------------------------------------------------------------------------------------
    # STORE RESULTS FOR ADKINS AND LEIBECK   
    # ------------------------------------------------------------------------------------------------------
    sr_noise_filename_Q1       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)   
    sr_noise_filename_Q2       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)   
    sr_noise_filename_Q3       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)   
    sr_noise_filename_Q4       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)    
    sr_noise_results_raw_Q1    = load_results(sr_noise_filename_Q1)
    sr_noise_results_raw_Q2    = load_results(sr_noise_filename_Q2)
    sr_noise_results_raw_Q3    = load_results(sr_noise_filename_Q3)
    sr_noise_results_raw_Q4    = load_results(sr_noise_filename_Q4) 
    sr_noise_res_Q1            = process_results(sr_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name)
    sr_noise_res_Q2            = process_results(sr_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name)
    sr_noise_res_Q3            = process_results(sr_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name)
    sr_noise_res_Q4            = process_results(sr_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name)
    plot_flight_profile_noise_contours(1,sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4,plot_parameters,vehicle_name,alpha_opt_weight) 
  
    # append spider plot data 
    segment_no          = 4
    cpts                = sr_noise_res_Q4.num_ctrl_pts 
    max_SPL,gm_mic_loc  = compute_max_SPL_and_michrophone_locations(sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4)        
                
    start = (segment_no)*cpts
    end   = (segment_no+1)*cpts 
    spider_plot_max_SPL.append(np.max(max_SPL))
    spider_plot_maxiumum_power.append(np.max(-sr_noise_res_Q1.power[start:end]))
    spider_plot_energy_consumption.append(sr_noise_res_Q1.energy[start]-sr_noise_res_Q1.energy[end])
    spider_plot_tip_mach.append(np.max(sr_noise_res_Q1.rtm[start:end]))
    spider_plot_bat_temperature.append(np.max(sr_noise_res_Q1.pack_temp[start:end]))          
       

    # ------------------------------------------------------------------------------------------------------    
    # STORE RESULTS FOR CLARKE 
    # ------------------------------------------------------------------------------------------------------ 
    for a_i in range(len(alpha_weights)): 

        alpha             = alpha_weights[a_i]  
        alpha_opt_weight  = str(alpha)
        alpha_opt_weight  = alpha_opt_weight.replace('.','_')       
        sr_noise_filename_Q1       = 'Stopped_Rotor_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight   
        sr_noise_filename_Q2       = 'Stopped_Rotor_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight   
        sr_noise_filename_Q3       = 'Stopped_Rotor_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight   
        sr_noise_filename_Q4       = 'Stopped_Rotor_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) + '_alpha' + alpha_opt_weight    
        sr_noise_results_raw_Q1    = load_results(sr_noise_filename_Q1)
        sr_noise_results_raw_Q2    = load_results(sr_noise_filename_Q2)
        sr_noise_results_raw_Q3    = load_results(sr_noise_filename_Q3)
        sr_noise_results_raw_Q4    = load_results(sr_noise_filename_Q4) 
        sr_noise_res_Q1            = process_results(sr_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name)
        sr_noise_res_Q2            = process_results(sr_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name)
        sr_noise_res_Q3            = process_results(sr_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name)
        sr_noise_res_Q4            = process_results(sr_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name)
        plot_flight_profile_noise_contours(1,sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4,plot_parameters,vehicle_name,alpha_opt_weight) 
  
  
        # append spider plot data 
        segment_no          = 4
        cpts                = sr_noise_res_Q4.num_ctrl_pts 
        max_SPL,gm_mic_loc  = compute_max_SPL_and_michrophone_locations(sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4)        
                    
        start   = (segment_no)*cpts
        end     = (segment_no+1)*cpts
        max_SPL = np.nan_to_num(max_SPL)
        spider_plot_max_SPL.append(np.max(max_SPL))
        spider_plot_maxiumum_power.append(np.max(-sr_noise_res_Q1.power[start:end]))
        spider_plot_energy_consumption.append(sr_noise_res_Q1.energy[start]-sr_noise_res_Q1.energy[end])
        spider_plot_tip_mach.append(np.max(sr_noise_res_Q1.rtm[start:end]))
        spider_plot_bat_temperature.append(np.max(sr_noise_res_Q1.pack_temp[start:end]))          
    
    
    
    # ------------------------------------------------------------------------------------------------------    
    # PLOT RESULTS  
    # ------------------------------------------------------------------------------------------------------     
    spider_res = Data(
        maximum_SPL        = 100*(spider_plot_max_SPL/spider_plot_max_SPL[0]),
        maxiumum_power     = 100*(spider_plot_maxiumum_power/spider_plot_maxiumum_power[0]),
        energy_consumption = 100*(spider_plot_energy_consumption/spider_plot_energy_consumption[0]),
        maximum_tip_mach   = 100*(spider_plot_tip_mach/spider_plot_tip_mach[0]),
        bat_temperature    = 100*(spider_plot_bat_temperature/spider_plot_bat_temperature[0]))
    
    plot_spider_diagram(spider_res,plot_parameters)   
       
       
    return
 
# ------------------------------------------------------------------
# Process Results 
# ------------------------------------------------------------------
def process_results(res,N_gm_x ,N_gm_y,vehicle_name ):
    '''This function cleans up the data and connects segments for plots ''' 

    '''This function cleans up the data and connects segments for plots ''' 


    num_ctrl_pts   = len(res.segments[0].conditions.frames.inertial.time[:,0] )
    num_segments   = len(res.segments.keys()) 
    data_dimension = num_ctrl_pts*num_segments
    
    
    PD                 = Data()
    PD.time            = np.zeros(data_dimension)
    PD.range_nm        = np.zeros(data_dimension)  
    PD.num_gm          = res.segments[0].conditions.noise.number_ground_microphones 
    PD.gm_mic_loc      = res.segments[0].analyses.noise.settings.ground_microphone_locations 
    PD.SPL_contour     = np.zeros((data_dimension,N_gm_x*N_gm_y)) 
    PD.N_gm_x          = N_gm_x  
    PD.N_gm_y          = N_gm_y    
    PD.altitude        = np.zeros(data_dimension) 
    PD.cl              = np.zeros(data_dimension) 
    PD.cd              = np.zeros(data_dimension) 
    PD.aoa             = np.zeros(data_dimension) 
    PD.l_d             = np.zeros(data_dimension) 
    PD.eta             = np.zeros(data_dimension) 
    PD.energy          = np.zeros(data_dimension) 
    PD.volts           = np.zeros(data_dimension) 
    PD.volts_oc        = np.zeros(data_dimension) 
    PD.current         = np.zeros(data_dimension) 
    PD.battery_amp_hr  = np.zeros(data_dimension) 
    PD.C_rating        = np.zeros(data_dimension) 
    PD.eta_l           = np.zeros(data_dimension) 
    PD.prop_rpm        = np.zeros(data_dimension) 
    PD.prop_thrust     = np.zeros(data_dimension) 
    PD.prop_torque     = np.zeros(data_dimension) 
    PD.prop_effp       = np.zeros(data_dimension) 
    PD.prop_effm       = np.zeros(data_dimension) 
    PD.ptm             = np.zeros(data_dimension)   
    PD.rotor_rpm       = np.zeros(data_dimension) 
    PD.rotor_thrust    = np.zeros(data_dimension) 
    PD.rotor_torque    = np.zeros(data_dimension) 
    PD.rotor_effp      = np.zeros(data_dimension) 
    PD.rotor_effm      = np.zeros(data_dimension) 
    PD.rtm             = np.zeros(data_dimension) 
    PD.rpm             = np.zeros(data_dimension)
    PD.pack_temp       = np.zeros(data_dimension)
    PD.thrust          = np.zeros(data_dimension)
    PD.torque          = np.zeros(data_dimension)
    PD.effp            = np.zeros(data_dimension)
    PD.effm            = np.zeros(data_dimension)
    PD.tm              = np.zeros(data_dimension)  
    PD.time_normalized = np.zeros(data_dimension)    
    PD.power           = np.zeros(data_dimension) 
    PD.DL_prop         = np.zeros(data_dimension)
    PD.DL_rot          = np.zeros(data_dimension) 
    PD.PL_prop         = np.zeros(data_dimension) 
    PD.PL_rot          = np.zeros(data_dimension)  
    
    PD.aircraft_pos =  np.zeros((data_dimension,3)) 
    dim_segs        = len(res.segments)    
    PD.num_segments = dim_segs
    PD.num_ctrl_pts = num_ctrl_pts
    
    Flight_Time        = res.segments[-1].conditions.frames.inertial.time[-1,0]
    for i in range(num_segments):  
        time            = res.segments[i].conditions.frames.inertial.time[:,0] / Units.min  
        time_normalized = res.segments[i].conditions.frames.inertial.time[:,0]/Flight_Time
        range_nm        = res.segments[i].conditions.frames.inertial.position_vector[:,0] / Units.nmi #  *0.000539957 
        altitude        = res.segments[i].conditions.freestream.altitude[:,0]/Units.feet   
        cl              = res.segments[i].conditions.aerodynamics.lift_coefficient[:,0] 
        cd              = res.segments[i].conditions.aerodynamics.drag_coefficient[:,0] 
        aoa             = res.segments[i].conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d             = cl/cd         
        eta             = res.segments[i].conditions.propulsion.throttle[:,0]  
        energy          = res.segments[i].conditions.propulsion.battery_energy[:,0]*0.000277778 
        volts           = res.segments[i].conditions.propulsion.battery_voltage_under_load[:,0] 
        volts_oc        = res.segments[i].conditions.propulsion.battery_voltage_open_circuit[:,0]
        power           = res.segments[i].conditions.propulsion.battery_power_draw [:,0]
        pack_temp       = res.segments[i].conditions.propulsion.battery_pack_temperature[:,0] 
        current         = res.segments[i].conditions.propulsion.battery_current[:,0]    
        SPL_contour     = res.segments[i].conditions.noise.total_SPL_dBA           
        battery_amp_hr  = (energy)/volts 
        C_rating        = current /battery_amp_hr    
        pos             = res.segments[i].conditions.frames.inertial.position_vector   
        
        
        PD.time_normalized[i*num_ctrl_pts:(i+1)*num_ctrl_pts] = time_normalized
        PD.time[i*num_ctrl_pts:(i+1)*num_ctrl_pts]            = time          
        PD.range_nm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = range_nm      
        PD.altitude[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = altitude       
        PD.cl[i*num_ctrl_pts:(i+1)*num_ctrl_pts]              = cl             
        PD.cd[i*num_ctrl_pts:(i+1)*num_ctrl_pts]              = cd             
        PD.aoa[i*num_ctrl_pts:(i+1)*num_ctrl_pts]             = aoa            
        PD.l_d[i*num_ctrl_pts:(i+1)*num_ctrl_pts]             = l_d            
        PD.eta[i*num_ctrl_pts:(i+1)*num_ctrl_pts]             = eta            
        PD.energy[i*num_ctrl_pts:(i+1)*num_ctrl_pts]          = energy   
        PD.pack_temp[i*num_ctrl_pts:(i+1)*num_ctrl_pts]       = pack_temp
        PD.power[i*num_ctrl_pts:(i+1)*num_ctrl_pts]           = power
        PD.volts[i*num_ctrl_pts:(i+1)*num_ctrl_pts]           = volts          
        PD.volts_oc[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = volts_oc       
        PD.current[i*num_ctrl_pts:(i+1)*num_ctrl_pts]         = current        
        PD.battery_amp_hr[i*num_ctrl_pts:(i+1)*num_ctrl_pts]  = battery_amp_hr 
        PD.C_rating[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = C_rating        
        PD.aircraft_pos[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]  = pos 
        PD.SPL_contour[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]   = SPL_contour          
        
        if vehicle_name == 'SR':
            eta_l        = res.segments[i].conditions.propulsion.throttle_lift[:,0] 
            prop_rpm     = res.segments[i].conditions.propulsion.propeller_rpm[:,0] 
            prop_thrust  = np.linalg.norm(res.segments[i].conditions.propulsion.propeller_thrust,axis=1)
            prop_torque  = res.segments[i].conditions.propulsion.propeller_motor_torque[:,0]
            prop_effp    = res.segments[i].conditions.propulsion.propeller_efficiency[:,0]
            prop_effm    = res.segments[i].conditions.propulsion.propeller_motor_efficiency[:,0]   
            ptm          = res.segments[i].conditions.propulsion.propeller_tip_mach[:,0]                  
            rotor_rpm    = res.segments[i].conditions.propulsion.lift_rotor_rpm[:,0] 
            rotor_thrust = np.linalg.norm(res.segments[i].conditions.propulsion.lift_rotor_thrust,axis=1)
            rotor_torque = res.segments[i].conditions.propulsion.lift_rotor_motor_torque[:,0]
            rotor_effp   = res.segments[i].conditions.propulsion.lift_rotor_efficiency[:,0]
            rotor_effm   = res.segments[i].conditions.propulsion.lift_rotor_motor_efficiency[:,0]   
            rtm          = res.segments[i].conditions.propulsion.lift_rotor_tip_mach[:,0]    
            DL_prop      = res.segments[i].conditions.propulsion.propeller_disc_loading[:,0]   
            DL_rot       = res.segments[i].conditions.propulsion.lift_rotor_disc_loading[:,0] 
            PL_prop      = res.segments[i].conditions.propulsion.propeller_power_loading[:,0] 
            PL_rot       = res.segments[i].conditions.propulsion.lift_rotor_power_loading[:,0]
            
            PD.eta_l[i*num_ctrl_pts:(i+1)*num_ctrl_pts]           =  eta_l       
            PD.prop_rpm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        =  prop_rpm    
            PD.prop_thrust[i*num_ctrl_pts:(i+1)*num_ctrl_pts]     =  prop_thrust 
            PD.prop_torque[i*num_ctrl_pts:(i+1)*num_ctrl_pts]     =  prop_torque 
            PD.prop_effp[i*num_ctrl_pts:(i+1)*num_ctrl_pts]       =  prop_effp   
            PD.prop_effm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]       =  prop_effm   
            PD.ptm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]             =  ptm          
            PD.rotor_rpm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]       =  rotor_rpm   
            PD.rotor_thrust[i*num_ctrl_pts:(i+1)*num_ctrl_pts]    =  rotor_thrust
            PD.rotor_torque[i*num_ctrl_pts:(i+1)*num_ctrl_pts]    =  rotor_torque
            PD.rotor_effp[i*num_ctrl_pts:(i+1)*num_ctrl_pts]      =  rotor_effp  
            PD.rotor_effm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]      =  rotor_effm  
            PD.rtm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]             =  rtm      
            PD.DL_prop[i*num_ctrl_pts:(i+1)*num_ctrl_pts]         =  DL_prop    
            PD.DL_rot[i*num_ctrl_pts:(i+1)*num_ctrl_pts]          =  DL_rot
            PD.PL_prop[i*num_ctrl_pts:(i+1)*num_ctrl_pts]         =  PL_prop 
            PD.PL_rot[i*num_ctrl_pts:(i+1)*num_ctrl_pts]          =  PL_rot
            
        else:
            rpm    = res.segments[i].conditions.propulsion.propeller_rpm[:,0] 
            thrust = np.linalg.norm(res.segments[i].conditions.propulsion.propeller_thrust,axis=1)
            torque = res.segments[i].conditions.propulsion.propeller_motor_torque[:,0]
            effp   = res.segments[i].conditions.propulsion.propeller_efficiency[:,0]
            effm   = res.segments[i].conditions.propulsion.propeller_motor_efficiency[:,0]
            tm     = res.segments[i].conditions.propulsion.propeller_tip_mach[:,0]   
            DL_prop= res.segments[i].conditions.propulsion.disc_loading[:,0]
            PL_prop= res.segments[i].conditions.propulsion.power_loading[:,0]  
            
            PD.rpm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]             =  rpm    
            PD.thrust[i*num_ctrl_pts:(i+1)*num_ctrl_pts]          =  thrust 
            PD.torque[i*num_ctrl_pts:(i+1)*num_ctrl_pts]          =  torque 
            PD.effp[i*num_ctrl_pts:(i+1)*num_ctrl_pts]            =  effp   
            PD.effm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]            =  effm   
            PD.tm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]              =  tm       
            PD.DL_prop[i*num_ctrl_pts:(i+1)*num_ctrl_pts]         =  DL_prop
            PD.PL_prop[i*num_ctrl_pts:(i+1)*num_ctrl_pts]         =  PL_prop  

    return PD
 
# ------------------------------------------------------------------
# Plot Flight Profile Noise Contours 
# ------------------------------------------------------------------
def plot_flight_profile_noise_contours(idx,res_Q1,res_Q2,res_Q3,res_Q4,PP,vehicle_name,alpha_weight):     
          
    # figure parameters
    filename      = 'Noise_Contour' + vehicle_name + '_Alpha_' + alpha_weight
    fig           = plt.figure(filename) 
    fig.set_size_inches(PP.figure_width ,PP.figure_height)   

    gs            = gridspec.GridSpec(8, 8)
    axes_21       = fig.add_subplot(gs[2:,:]) # contour 
    axes_22       = fig.add_subplot(gs[:2,:]) # altitude 
      
    #   Altitude  
    axes_22.set_ylabel('Alt (ft)')  
    axes_22.axes.xaxis.set_visible(False)
    axes_22.plot(res_Q1.aircraft_pos[:,0]/Units.nmi,  -res_Q1.aircraft_pos[:,2]/Units.feet , color = PP.colors[idx], linestyle = PP.line_style[0], marker = PP.markers[idx] , markersize = PP.marker_size , linewidth= PP.line_width) 
    
    max_mi  = np.max(res_Q1.aircraft_pos[:,0]/Units.nmi) 
    axes_22.set_xlim(0, max_mi)   
    axes_22.set_ylim(0, 500)     
    axes_22.minorticks_on()   

    # set size of matrices  
    max_SPL,gm_mic_loc  = compute_max_SPL_and_michrophone_locations(res_Q1,res_Q2,res_Q3,res_Q4) 
    
    Range_x    = gm_mic_loc[:,0,0]/Units.nmi
    Range_y    = gm_mic_loc[0,:,1]/Units.nmi 
    Y, X       = np.meshgrid(Range_y, Range_x)
    levs       = np.linspace(40,80,17)    
    
    # post processing  
    #max_SPL    = ndimage.gaussian_filter(max_SPL, sigma=1.5, order=0) # SMOOTHING 
    
    CS         = axes_21.contourf(X , Y,max_SPL, levels = levs, cmap=plt.cm.jet, extend='both') 
    CS         = axes_21.contourf(X ,-Y,max_SPL, levels = levs, cmap=plt.cm.jet, extend='both')
        
    xi, yi = np.meshgrid(np.linspace(np.min(Range_x),np.max(Range_x), 10),np.linspace(-np.max(Range_y ),np.max(Range_y), 5) )
    axes_21.plot(xi, yi, 'k--', lw=1, alpha=0.5)
    axes_21.plot(xi.T, yi.T, 'k--', lw=1, alpha=0.5)
         
    fig.subplots_adjust(right=0.8)
    axes_23 = fig.add_axes([0.72, 0.0, 0.14, 1.0]) # left , heigh from base, width , height
    cbar    = fig.colorbar(CS, ax=axes_23)
    cbar.ax.set_ylabel('SPL$_{Amax}$ (dBA)', rotation =  90)     
    axes_21.set_ylabel('Spanwise $x_{fp}$ (nmi)',labelpad = 15)
    axes_21.set_xlabel('Streamwise $x_{fp}$ (nmi)')  
    plt.axis('off')	
    plt.grid(None)      
    
    return 
  
# ------------------------------------------------------------------
# Plot Flight Profile Noise Contours 
# ------------------------------------------------------------------
def compute_max_SPL_and_michrophone_locations(res_Q1,res_Q2,res_Q3,res_Q4):      

    N_gm_x       = res_Q1.N_gm_x
    N_gm_y       = res_Q1.N_gm_y  
    gm_mic_loc   = np.zeros((2*N_gm_x, 2*N_gm_y,3))   
    gm_mic_loc[0:N_gm_x,0:N_gm_y,:]   = res_Q1.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[0:N_gm_x,N_gm_y:,:]    = res_Q2.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,0:N_gm_y,:]    = res_Q3.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,N_gm_y:,:]     = res_Q4.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)   
    
    # set size of matrices  
    num_cpts     = res_Q1.num_ctrl_pts*res_Q1.num_segments
    aircraft_SPL = np.zeros((num_cpts,2*N_gm_x, 2*N_gm_y)) 
    
    aircraft_SPL[:,0:N_gm_x,0:N_gm_y] = res_Q1.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)
    aircraft_SPL[:,0:N_gm_x,N_gm_y:]  = res_Q2.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,0:N_gm_y]  = res_Q3.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,N_gm_y:]   = res_Q4.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)    
    

    aircraft_SPL = np.nan_to_num(aircraft_SPL)    
    max_SPL      = np.max(aircraft_SPL,axis=0)    
    
    return max_SPL,gm_mic_loc
  

# ------------------------------------------------------------------ 
# Plot Figures
# ------------------------------------------------------------------ 
def plot_spider_diagram(spider_res,PP):   
     
    # Set data
    df = pd.DataFrame({
    #'group': ['A. & L.','Alpha_1_0','Alpha_0_5','Alpha_0_0'],
    'group': ['A. & L.','Alpha_0_0','Alpha_0_5','Alpha_1_0'],
    'SPL$_{Max.}$': spider_res.maximum_SPL,
    'Power$_{Max.}$': spider_res.maxiumum_power,
    'Total Energy \n Consump.': spider_res.energy_consumption,
    'Max Tip Mach':  spider_res.maximum_tip_mach,
    'Battery \n Temp.$_{Max.}$': spider_res.bat_temperature, 
    })
    
    # ------- PART 1: Create background
    
    # number of variable
    categories=list(df)[1:]
    N = len(categories)
    
    # What will be the angle of each axis in the plot? (we divide the plot / number of variable)
    angles = [n / float(N) * 2 * pi for n in range(N)]
    angles += angles[:1]
    
    fig = plt.figure()
    fig.set_size_inches(8, 10)
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams['axes.linewidth'] = 2. 
    
    # Initialise the spider plot
    ax = plt.subplot(111, polar=True)
    
    # If you want the first axis to be on top:
    ax.set_theta_offset(pi / 2)
    ax.set_theta_direction(-1)
    
    # Draw one axe per variable + add labels labels yet
    plt.xticks(angles[:-1], categories, size= 20)
    ax.tick_params(axis="x" , pad= 40)
    plt.box(on=None)
    
    # Draw ylabels
    ax.set_rlabel_position(0)
    plt.yticks([90,95,100,105,110], ["90","95","100","105","110"], color="black", size=16)
    plt.ylim(90,105) 
    
    # ------- PART 2: Add plots 
    # Ind1
    lw = 4 
    
    values=df.loc[0].drop('group').values.flatten().tolist()
    values += values[:1]
    ax.plot(angles, values, color='blue', linewidth=lw, linestyle='solid', label='A. & L.')  
    ax.fill(angles, values, color='blue', alpha=0.2)
    
    # Ind2
    values=df.loc[1].drop('group').values.flatten().tolist()
    values += values[:1]
    ax.plot(angles, values, color='red', linewidth=lw, linestyle='solid', label=r'$\alpha$ = 0.0')  
    ax.fill(angles, values, color='red', alpha=0.2)
    
    # Ind2
    values=df.loc[2].drop('group').values.flatten().tolist()
    values += values[:1]
    ax.plot(angles, values, color='green', linewidth=lw, linestyle='solid', label=r'$\alpha$ = 0.5')  
    ax.fill(angles, values, color='green', alpha=0.2) 
    
    ## Ind2
    #values=df.loc[3].drop('group').values.flatten().tolist()
    #values += values[:1]
    #ax.plot(angles, values, color='orange', linewidth=lw, linestyle='solid', label=r'$\alpha$ = 0.0')  
    #ax.fill(angles, values, color='orange', alpha=0.2)

    
    # Add legend
    plt.tight_layout()
    plt.legend(loc='upper right', prop={'size': PP.legend_font}, bbox_to_anchor=(1.2, 1.3)) 
    #plt.legend(loc='center left', prop={'size': PP.legend_font}) 
   
       
    return  
  
    
# ----------------------------------------------------------------------
#  Load results
# ----------------------------------------------------------------------     
def load_results(filename):
    load_file =   filename + '.pkl'
    with open(load_file, 'rb') as file:
        results = pickle.load(file)
        
    return  results 

if __name__ == '__main__': 
    main()    
    plt.show()