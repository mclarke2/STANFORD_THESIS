# Vehicle Comparisons

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------    
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Plots.Geometry import *  
from SUAVE.Core import Data , Units
import matplotlib.gridspec as gridspec
import numpy as np    
import matplotlib.pyplot as plt    
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
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
    plot_parameters.m                = 14                             # markersize               
    plot_parameters.legend_font      = 20                             # legend_font_size       
    plot_parameters.marker_size      = 14   
     
     
     
    N_gm_x              = 8 # 12
    N_gm_y              = 3 # 4
    ## ../Supplementary/ 
    
    # ECTOL
    ectol_full_mission_filename   = 'ECTOL_Full_Mission'
    ectol_full_mission_results_raw= load_results(ectol_full_mission_filename)
    ectol_full_res                = process_results(ectol_full_mission_results_raw,N_gm_x ,N_gm_y,vehicle_name = 'ECTOL', noise_analysis = False)
 
    # STOPPED ROTOR
    sr_full_mission_filename   = 'Stopped_Rotor_Full_Mission'
    sr_full_mission_results_raw= load_results(sr_full_mission_filename)
    sr_full_res                = process_results(sr_full_mission_results_raw,N_gm_x ,N_gm_y,vehicle_name = 'SR', noise_analysis = False)
     
    # TILTWING
    tw_full_mission_filename   = 'Tiltwing_Full_Mission'
    tw_full_mission_results_raw= load_results(tw_full_mission_filename)
    tw_full_res                = process_results(tw_full_mission_results_raw,N_gm_x ,N_gm_y,vehicle_name = 'TW', noise_analysis = False)
     
    # MULTIROTOR
    mr_full_mission_filename   = 'Multirotor_Full_Mission'
    mr_full_mission_results_raw= load_results(mr_full_mission_filename)
    mr_full_res                = process_results(mr_full_mission_results_raw,N_gm_x ,N_gm_y,vehicle_name = 'MR', noise_analysis = False)
     
     
     
    axes_1,axins_1,axes_2,axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,\
               axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,\
               fig_1,fig_2,fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,fig_21,fig_10,fig_11,fig_12,fig_13,\
               fig_14,fig_15,fig_16,fig_17,fig_18,fig_19,fig_20     =  set_up_axes(plot_parameters.figure_width,plot_parameters.figure_height)
    
    
    
    plot_results(ectol_full_res ,axes_1,axins_1,axes_2,axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,
                 axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,plot_parameters,0,vehicle_name = 'ECTOL')    
    
    plot_results(sr_full_res ,axes_1,axins_1,axes_2,axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,
                 axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,plot_parameters,1,vehicle_name = 'SR')   
    
    plot_results(tw_full_res,axes_1,axins_1,axes_2,axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,
                 axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,plot_parameters,2,vehicle_name = 'TW')    
    
    plot_results(mr_full_res,axes_1,axins_1,axes_2,axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,
                 axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,plot_parameters,3,vehicle_name = 'MR')          
     

    mark_inset(axes_1, axins_1, loc1=2, loc2=4, fc="none", ec="0.5")  
    mark_inset(axes_12, axins_12, loc1=2, loc2=4, fc="none", ec="0.5")  
    axes_1.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font})  
    axes_2.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font}) 
    axes_4.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font})  
    axes_5.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font})
    axes_6.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font}) 
    axes_7.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font}) 
    axes_8.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font}) 
    axes_9.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font}) 
    axes_10.legend(loc='upper center', ncol= 5, prop={'size': plot_parameters.legend_font})  
    axes_11.legend(loc='upper center', ncol= 5, prop={'size': plot_parameters.legend_font})  
    axes_12.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font})    
    axes_13.legend(loc='upper center', ncol= 5, prop={'size': plot_parameters.legend_font})
    axes_21.legend(loc='upper center', ncol= 5, prop={'size': plot_parameters.legend_font})
    axes_14.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font})
    axes_15.legend(loc='upper center', ncol=  5 , prop={'size': plot_parameters.legend_font})    
    axes_16.legend(loc='upper center', ncol=  5 , prop={'size': plot_parameters.legend_font})  
    axes_17.legend(loc='upper center', ncol=  5 , prop={'size': plot_parameters.legend_font}) 
    axes_18.legend(loc='upper center', ncol=  5, prop={'size':  plot_parameters.legend_font})     
    axes_19.legend(loc='upper center', ncol=  5, prop={'size':  plot_parameters.legend_font})   
    axes_20.legend(loc='upper center', ncol=  5 , prop={'size': plot_parameters.legend_font}) 

    save_figures(fig_1,fig_2,fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,\
                     fig_21,fig_10,fig_11,fig_12,fig_13,fig_14,fig_15,fig_16,\
                     fig_17,fig_18,fig_19,fig_20)    
    return
 
# ------------------------------------------------------------------
# Process Results 
# ------------------------------------------------------------------
def process_results(res,N_gm_x ,N_gm_y,vehicle_name,noise_analysis):
    '''This function cleans up the data and connects segments for plots ''' 

    num_ctrl_pts   = len(res.segments[0].conditions.frames.inertial.time[:,0] )
    num_segments   = len(res.segments.keys()) 
    data_dimension = num_ctrl_pts*num_segments
    
    PD                 = Data()
    PD.time            = np.zeros(data_dimension)
    PD.range_nm        = np.zeros(data_dimension)
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
    PD.DL_prop         = np.zeros(data_dimension)
    PD.DL_rot          = np.zeros(data_dimension) 
    PD.PL_prop         = np.zeros(data_dimension) 
    PD.PL_rot          = np.zeros(data_dimension) 
    
    if noise_analysis:
        PD.num_gm       = res.segments[0].conditions.noise.number_ground_microphones 
        PD.gm_mic_loc   = res.segments[0].analyses.noise.settings.ground_microphone_locations 
        PD.SPL_contour  = np.zeros((data_dimension,N_gm_x*N_gm_y))
        PD.N_gm_x       = N_gm_x  
        PD.N_gm_y       = N_gm_y  
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
        pack_temp       = res.segments[i].conditions.propulsion.battery_pack_temperature[:,0] 
        current         = res.segments[i].conditions.propulsion.battery_current[:,0]      
        battery_amp_hr  = (energy)/volts 
        C_rating        = current /battery_amp_hr   
        if noise_analysis:
            SPL_contour     = res.segments[i].conditions.noise.total_SPL_dBA   
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
        PD.volts[i*num_ctrl_pts:(i+1)*num_ctrl_pts]           = volts          
        PD.volts_oc[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = volts_oc       
        PD.current[i*num_ctrl_pts:(i+1)*num_ctrl_pts]         = current        
        PD.battery_amp_hr[i*num_ctrl_pts:(i+1)*num_ctrl_pts]  = battery_amp_hr 
        PD.C_rating[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = C_rating     
        if noise_analysis:
            PD.SPL_contour[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]   = SPL_contour   
        PD.aircraft_pos[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]  = pos        
        
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
# Plot Aircraft Hover Noise Contours at 500 ft
# ------------------------------------------------------------------
def plot_aircraft_hover_noise_contours(res,vehicle_name): 
     
    # unpack microphone array 

    gm_N_x             = res.N_gm_x
    gm_N_y             = res.N_gm_y   
    gm_mic_loc         = res.gm_mic_loc 
    dim_gm             = res.num_gm
    Range              = np.zeros(dim_gm) 
    Span               = np.zeros(dim_gm)     
    Range              = gm_mic_loc[:,0].reshape(gm_N_x,gm_N_y)
    Span               = gm_mic_loc[:,1].reshape(gm_N_x,gm_N_y) 
    levs               = np.linspace(40,80,25)
   
    # ---------------------------------------------------------------------------
    # Stopped-Rotor 
    # --------------------------------------------------------------------------- 
    SPL_contour_gm      = res.SPL_contour 
    max_SPL_contour_gm  = np.max(SPL_contour_gm,axis=0)
    SPL_gm              = max_SPL_contour_gm.reshape(gm_N_x,gm_N_y) 
    fig_name               = 'Noise_Contour_500ft_' + vehicle_name
    fig                 = plt.figure(fig_name )  
    fig.set_size_inches(6, 6) 
    axes                = fig.add_subplot(1,1,1)   
    Range                  = Range/Units.nmi
    Span                   = Span/Units.nmi
    CS                     = axes.contourf(Range , Span,SPL_gm, levels  = levs, cmap=plt.cm.jet, extend='both')     
    cbar                   = fig.colorbar(CS)
    cbar.ax.set_ylabel('SPL (dBA)', rotation =  90)     
    axes.set_ylabel('Spanwise $x_{fp}$ (nmi)',labelpad = 15)
    axes.set_xlabel('Streamwise $x_{fp}$ (nmi)')  
    axes.grid(False)  
    axes.minorticks_on()   
    plt.savefig(fig_name + 'png')
    return  

# ------------------------------------------------------------------
# Plot Flight Profile Noise Contours 
# ------------------------------------------------------------------
def plot_flight_profile_noise_contours(res_Q1,res_Q2,res_Q3,res_Q4,line_width,ms,ls, col ,col2,vehicle_name,m,ls1, ls2):    
    
    line_width                     = 4
    plt.rcParams['axes.linewidth'] = 4.
    plt.rcParams["font.family"]    = "Times New Roman"
    plt.rcParams.update({'font.size': 36}) 
    ls            = 24 
    ms            = 14   
    figure_width  = 15
    figure_height = 10     
    
    # figure parameters
    filename = 'Noise_Contour' + vehicle_name
    fig          = plt.figure(filename) 
    fig.set_size_inches(figure_width ,figure_height)   
     
    gs = gridspec.GridSpec(8, 8)
    axes_21 = fig.add_subplot(gs[2:,:]) # contour 
    axes_22 = fig.add_subplot(gs[:2,:]) # altitude 
      
    #   Altitude  
    axes_22.set_ylabel('Alt (ft)')  
    axes_22.axes.xaxis.set_visible(False)
    axes_22.plot(res_Q1.aircraft_pos[:,0]/Units.nmi,  -res_Q1.aircraft_pos[:,2]/Units.feet , color = col , linestyle = ls1, marker = m, markersize = ms , linewidth= line_width) 
    
    max_mi = np.max(res_Q1.aircraft_pos[:,0]/Units.nmi)
    end_distance = max_mi/((res_Q1.N_gm_x-2)*2)
    axes_22.set_xlim(-end_distance, max_mi+end_distance)   
    axes_22.set_ylim(0, 3000)     
    axes_22.minorticks_on()  
 
 
    res = [res_Q1,res_Q2,res_Q3,res_Q4]
    
    for Q_idx in range(len(res)): 
        #   Noise     
        N_gm_x              = res[Q_idx].N_gm_x
        N_gm_y              = res[Q_idx].N_gm_y   
        gm_mic_loc          = res[Q_idx].gm_mic_loc
        Range               = gm_mic_loc[:,0].reshape(N_gm_x ,N_gm_y )
        Span                = gm_mic_loc[:,1].reshape(N_gm_x ,N_gm_y ) 
        SPL_contour         = res[Q_idx].SPL_contour
        SPL_contour[SPL_contour>120] = 0
        max_SPL_contour_gm  = np.max(SPL_contour,axis=0)
        SPL_gm              = max_SPL_contour_gm.reshape(N_gm_x ,N_gm_y ) 
        levs                = np.linspace(30,120,25)   # 9 is every 10 dB
        Range               = Range/Units.nmi
        Span                = Span/Units.nmi
        CS                  = axes_21.contourf(Range , Span,SPL_gm, levels  = levs, cmap=plt.cm.jet, extend='both') 
        CS                  = axes_21.contourf(Range ,-Span, SPL_gm, levels = levs, cmap=plt.cm.jet, extend='both')
        
    #xi, yi = np.meshgrid(np.linspace(np.min(Range),np.max(Range), 10),np.linspace(-np.max(Span),np.max(Span), 5) )
    #axes_21.plot(xi, yi, 'k--', lw=1, alpha=0.5)
    #axes_21.plot(xi.T, yi.T, 'k--', lw=1, alpha=0.5)
        
    fig.subplots_adjust(right=0.8)
    axes_23 = fig.add_axes([0.72, 0.0, 0.14, 1.0]) # left , heigh from base, width , height
    cbar    = fig.colorbar(CS, ax=axes_23)
    cbar.ax.set_ylabel('SPL$_{Amax}$ (dBA)', rotation =  90)     
    axes_21.set_ylabel('Spanwise $x_{fp}$ (nmi)',labelpad = 15)
    axes_21.set_xlabel('Streamwise $x_{fp}$ (nmi)')  
    plt.axis('off')	
    plt.grid(None)      
    
    plt.savefig(filename+'.png')
    
    return 
  

# ------------------------------------------------------------------ 
# Plot Figures
# ------------------------------------------------------------------ 
def plot_results(res,axes_1,axins_1,axes_2,axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,
                 axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,PP,idx,vehicle_name):
   
    line_width = PP.linewidth
    ms         = PP.marker_size 
    col        = PP.colors[idx]
    col2       = PP.colors2[idx]
    m          = PP.markers[idx]
    ls1        = PP.line_style[0]
    ls2        = PP.line_style[1] 
                     
    # ------------------------------------------------------------------
    #   Flight Conditions - Altitude 
    # ------------------------------------------------------------------  
    axes_1.plot(res.time_normalized, res.altitude, color = col , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)    
    axins_1.plot(res.time_normalized, res.altitude, color = col , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width )    
    # ------------------------------------------------------------------
    #   Flight Conditions - Altitude 
    # ------------------------------------------------------------------ 
    axes_2.plot(res.time_normalized, res.range_nm, color = col , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
   
    
    
    # ------------------------------------------------------------------
    #   Flight Conditions - AoA 
    # ------------------------------------------------------------------  
    res.aoa[abs(res.aoa)> 20] = 0 
    axes_4.plot( res.time_normalized , res.aoa , color = col , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
    
    
    # ------------------------------------------------------------------
    #   Flight Conditions - CL
    # ------------------------------------------------------------------  
    res.cl[res.cl>1.5] =1.5
    cl = res.cl 
    time = res.time_normalized    
    axes_5.plot( time , cl, color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
    
    
    # ------------------------------------------------------------------
    #   Flight Conditions - CD 
    # ------------------------------------------------------------------  
    if vehicle_name == 'TW': 
        res.cd[res.cd>0.175] = 0.175
        cd = res.cd  
    else:
        res.cd[res.cd>1.0] = 1
        cd = res.cd       
    time = res.time_normalized     
    axes_6.plot( time , cd, color = col , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
    
    
    # ------------------------------------------------------------------
    #   Flight Conditions - L/D
    # ------------------------------------------------------------------  
    axes_7.plot( res.time_normalized , cl/cd, color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
    
 
    # ------------------------------------------------------------------
    #   Electronic Conditions - Energy 
    # ------------------------------------------------------------------ 
    axes_8.plot(res.time_normalized, res.energy/1000 ,  color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)    
    
           
    # ------------------------------------------------------------------
    #   Electronic Conditions - Voltage 
    # ------------------------------------------------------------------ 
    axes_9.plot(res.time_normalized, res.volts_oc     , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width  ,label= vehicle_name) 
    axes_21.plot(res.time_normalized, res.volts  , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
    
    
    # ------------------------------------------------------------------
    #   Electronic Conditions - Power Loading
    # ------------------------------------------------------------------  
    if vehicle_name == 'SR':
        axes_10.plot(res.time_normalized, res.DL_prop   , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label= vehicle_name + r'$_{prop}$')
        axes_10.plot(res.time_normalized,  res.DL_rot , color = col2  , linestyle =ls2, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')     
    else:
        axes_10.plot(res.time_normalized,  res.DL_prop , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width,label=vehicle_name )   
    
    
    # ------------------------------------------------------------------
    #   Electronic Conditions - Power Loading
    # ------------------------------------------------------------------  
    if vehicle_name == 'SR':
        axes_11.plot(res.time_normalized, res.PL_prop , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label= vehicle_name + r'$_{prop}$')
        axes_11.plot(res.time_normalized, res.PL_rot, color = col2  , linestyle = ls2, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')     
    else:
        axes_11.plot(res.time_normalized, res.PL_prop, color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width,label=vehicle_name )  
    #axes_11.plot(res.time_normalized, res.volts_ul  , color = col2  , linestyle = '-', marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name +  ' $V_{UL}$') 
       
    
    
    # ------------------------------------------------------------------
    #   Electronic Conditions - C-Rate 
    # ------------------------------------------------------------------  
    res.C_rating[res.C_rating>30] = np.nan
    C_rat = res.C_rating
    res.time_normalized[res.C_rating>30] = np.nan
    time = res.time_normalized
    axes_12.plot(time, C_rat,color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name) 
    axins_12.plot(res.time_normalized, res.C_rating,color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width,label=vehicle_name)  
    
    # ------------------------------------------------------------------
    #   Electronic Conditions
    # ------------------------------------------------------------------ 

    res.eta[res.eta>1] = np.nan
    eta  = res.eta
    eta_l= res.eta_l
    res.time_normalized[res.eta>1] = np.nan 
    time = res.time_normalized
    
    if vehicle_name == 'SR':  
        axes_13.plot(time,  eta , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name   + r'$_{prop}$')
        axes_13.plot(time,  eta_l , color = col2  , linestyle = ls2, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')       
    else: 
        axes_13.plot(time, eta , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)  
    
 
    # ------------------------------------------------------------------
    #   Electronic Conditions - SOC
    # ------------------------------------------------------------------ 
    axes_14.plot(res.time_normalized, res.energy/res.energy[0] ,  color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)   
    
    
    # ------------------------------------------------------------------
    #   Propeller Performance _ RPM 
    # ------------------------------------------------------------------ 
    if vehicle_name == 'SR':
        axes_15.plot(res.time_normalized, res.prop_rpm , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name  + r'$_{prop}$')
        axes_15.plot(res.time_normalized, res.rotor_rpm, color = col2  , linestyle = ls2, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')  
    else: 
        axes_15.plot(res.time_normalized, res.rpm, color = col , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)  
    
    
    # ------------------------------------------------------------------
    #   Propeller Performance Thrust 
    # ------------------------------------------------------------------     
    if vehicle_name == 'SR':
        axes_16.plot(res.time_normalized, res.prop_thrust/1000 , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label= vehicle_name + r'$_{prop}$')
        axes_16.plot(res.time_normalized, res.rotor_thrust/1000, color = col2  , linestyle = ls2, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')     
    else:
        axes_16.plot(res.time_normalized, res.thrust/1000, color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width,label=vehicle_name )  
    
    
    # ------------------------------------------------------------------
    #   Propeller Performance - Torque
    # ------------------------------------------------------------------            
    if vehicle_name == 'SR': 
        axes_17.plot(res.time_normalized, res.prop_torque , color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{prop}$')
        axes_17.plot(res.time_normalized, res.rotor_torque, color = col2  , linestyle = ls2, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')       
    else:
        axes_17.plot(res.time_normalized, res.torque, color = col, linestyle = ls1, marker = m , markersize = ms , linewidth= line_width,label=vehicle_name )
    

    # ------------------------------------------------------------------
    #   Propeller Performance - Propeller Efficiency 
    # ------------------------------------------------------------------ 
    if vehicle_name == 'SR': 
        axes_18.plot(res.time_normalized, res.prop_effp , color = col  , linestyle = '-', marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{prop}$')
        axes_18.plot(res.time_normalized, res.rotor_effp, color = col2  , linestyle = '-', marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')                
 
    else:
        axes_18.plot(res.time_normalized, res.effp, color = col  , linestyle = '-', marker = m , markersize = ms , linewidth= line_width,label=vehicle_name ) 
    
    
    # ------------------------------------------------------------------
    #   Propeller Performance - Motor Efficiency
    # ------------------------------------------------------------------   
    if vehicle_name == 'SR': 
        axes_19.plot(res.time_normalized, res.prop_effm , color = col  , linestyle = '-', marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{prop}$')
        axes_19.plot(res.time_normalized, res.rotor_effm, color = col2  , linestyle = '-', marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{rot}$')
    else:
        axes_19.plot(res.time_normalized, res.effm, color = col , linestyle = '-', marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)         
    
        
    # ------------------------------------------------------------------
    #   Propeller Performance - Tip Mach 
    # ------------------------------------------------------------------    
    if vehicle_name == 'SR': 
        axes_20.plot(res.time_normalized, res.ptm, color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name + r'$_{prop}$' )
        axes_20.plot(res.time_normalized, res.rtm, color = col2  , linestyle = ls2, marker = m , markersize = ms , linewidth= line_width,label=vehicle_name + r'$_{rot}$' )        
    else:
        axes_20.plot(res.time_normalized, res.tm, color = col  , linestyle = ls1, marker = m , markersize = ms , linewidth= line_width ,label=vehicle_name)    
    
    
    return 



# ------------------------------------------------------------------ 
# Setup Axes 
# ------------------------------------------------------------------ 
def set_up_axes(figure_width,figure_height):
    
    # ------------------------------------------------------------------
    #   Flight Conditions - Altitude 
    # ------------------------------------------------------------------
    fig_1 = plt.figure("Flight_Conditions_Altitude")
    fig_1.set_size_inches(figure_width,figure_height)
    axes_1 = fig_1.add_subplot(1,1,1)
    axes_1.set_ylabel('Altitude (ft)')  
    axes_1.set_xlabel('$\hat{t}$')         
    axes_1.set_ylim(0,3000)   
    axes_1.minorticks_on()   
    axins_1 = axes_1.inset_axes([0.2, 0.05, 0.5, 0.5])
    axins_1.set_xlim(-0.002, 0.024) # apply the x-limits
    axins_1.set_ylim(0, 500) # apply the y-limits    
    axins_1.xaxis.set_visible(False)
    axins_1.yaxis.set_visible(False) 
    axins_1.indicate_inset([0.2, 0.05, 0.5, 0.5],edgecolor = 'black', alpha=1.0,linewidth =2)    
    axins_1.set_aspect('auto')   
    
    # ------------------------------------------------------------------
    #   Flight Conditions - Altitude 
    # ------------------------------------------------------------------
    fig_2 = plt.figure("Flight_Conditions_Range")
    fig_2.set_size_inches(figure_width,figure_height) 
    axes_2 = fig_2.add_subplot(1,1,1)
    axes_2.set_ylabel('Range (nmi)')  
    axes_2.set_xlabel(r'$\hat{t}$')            
    axes_2.set_ylim(0,80)   
    axes_2.minorticks_on()     
     
    
    # ------------------------------------------------------------------
    #   Flight Conditions - AoA 
    # ------------------------------------------------------------------ 
    fig_4 = plt.figure("Aero_Conditions_AoA")
    fig_4.set_size_inches(figure_width,figure_height) 
    axes_4 = fig_4.add_subplot(1,1,1)
    axes_4.set_ylabel('AoA (deg.)' )     
    axes_4.set_ylim(-15,20)   
    axes_4.minorticks_on()  
    axes_4.set_xlabel(r'$\hat{t}$') 
    
    # ------------------------------------------------------------------
    #   Flight Conditions - CL
    # ------------------------------------------------------------------ 
    fig_5 = plt.figure("Aero_Conditions_CL")
    fig_5.set_size_inches(figure_width,figure_height)   
    axes_5 = fig_5.add_subplot(1,1,1)    
    axes_5.set_ylabel(r'$C_L$')  
    axes_5.minorticks_on()  
    axes_5.set_ylim(-0.2, 1.5)    
    axes_5.set_xlabel(r'$\hat{t}$') 
     
    # ------------------------------------------------------------------
    #   Flight Conditions - CD 
    # ------------------------------------------------------------------ 
    fig_6 = plt.figure("Aero_Conditions_CD")
    fig_6.set_size_inches(figure_width,figure_height)   
    axes_6 = fig_6.add_subplot(1,1,1) 
    axes_6.set_xlabel(r'$\hat{t}$')
    axes_6.set_ylabel(r'$C_D$') 
    axes_6.minorticks_on()   
    axes_6.set_ylim(0, 0.18) 
    axes_6.set_xlabel(r'$\hat{t}$') 
    
    # ------------------------------------------------------------------
    #   Flight Conditions - L/D
    # ------------------------------------------------------------------ 
    fig_7 = plt.figure("Aero_Conditions_L_D")
    fig_7.set_size_inches(figure_width,figure_height) 
    axes_7 = fig_7.add_subplot(1,1,1)
    axes_7.set_xlabel(r'$\hat{t}$')
    axes_7.set_ylabel('L/D')  
    axes_7.minorticks_on()  
    axes_7.set_ylim(-6, 20)     
    axes_7.set_xlabel(r'$\hat{t}$') 
 
    # ------------------------------------------------------------------
    #   Electronic Conditions - Energy 
    # ------------------------------------------------------------------
    fig_8 = plt.figure("Battery_Pack_Performance_E")
    fig_8.set_size_inches(figure_width,figure_height) 
    axes_8 = fig_8.add_subplot(1,1,1)   
    axes_8.set_ylabel('$E_{bat}$ (kW-hr)')
    axes_8.minorticks_on()  
    axes_8.set_xlabel(r'$\hat{t}$') 
           
    # ------------------------------------------------------------------
    #   Electronic Conditions - Voltage 
    # ------------------------------------------------------------------
    fig_9 = plt.figure("Battery_Pack_Performance_V")
    fig_9.set_size_inches(figure_width,figure_height)  
    axes_9 = fig_9.add_subplot(1,1,1)
    axes_9.set_ylabel('$V_{OC}$ (V)')   
    axes_9.minorticks_on()  
    axes_9.set_xlabel(r'$\hat{t}$')     
    axes_9.set_ylim(400, 850)         

    # ------------------------------------------------------------------
    #   Electronic Conditions - Voltage 
    # ------------------------------------------------------------------
    fig_21 = plt.figure("Battery_Pack_Performance_Vul")
    fig_21.set_size_inches(figure_width,figure_height)  
    axes_21 = fig_21.add_subplot(1,1,1)
    axes_21.set_ylabel('$V_{UL}$ (V)')   
    axes_21.minorticks_on()  
    axes_21.set_xlabel(r'$\hat{t}$')     
    axes_21.set_ylim(400, 800)         
    
    # ------------------------------------------------------------------
    #  Performance - Disc Loading
    # ------------------------------------------------------------------
    fig_10 = plt.figure("Battery_Pack_Performance_DL")
    fig_10.set_size_inches(figure_width,figure_height)  
    axes_10 = fig_10.add_subplot(1,1,1)
    axes_10.set_ylabel('$DL$ $(N/m^2)$') 
    axes_10.minorticks_on()  
    axes_10.set_xlabel(r'$\hat{t}$')     
    axes_10.set_ylim(0, 800)        
        
    # ------------------------------------------------------------------
    #   Electronic Conditions - Power Loading
    # ------------------------------------------------------------------
    fig_11 = plt.figure("Battery_Pack_Performance_PL")
    fig_11.set_size_inches(figure_width,figure_height)  
    axes_11 = fig_11.add_subplot(1,1,1) 
    axes_11.set_ylabel('$PL$ (N/W)')   
    axes_11.minorticks_on()  
    axes_11.set_xlabel(r'$\hat{t}$')     
    axes_11.set_ylim(0, 0.11)         
    
    # ------------------------------------------------------------------
    #   Electronic Conditions - C-Rate 
    # ------------------------------------------------------------------ 
    fig_12 = plt.figure("Battery_Pack_Performance_C_rate")
    fig_12.set_size_inches(figure_width,figure_height) 
    axes_12 = fig_12.add_subplot(1,1,1) 
    axes_12.set_ylabel('C ($hr^{-1}$)')  
    axes_12.set_xlabel(r'$\hat{t}$')     
    axes_12.set_ylim(0, 6)      
    axes_12.minorticks_on()      
    fig_12.tight_layout()        
    axins_12 = axes_12.inset_axes([0.1, 0.37, 0.6, 0.5])
    axins_12.set_xlim(-0.002, 0.03) # apply the x-limits
    axins_12.set_ylim(0, 4) # apply the y-limits    
    axins_12.xaxis.set_visible(False)
    axins_12.yaxis.set_visible(False) 
    axins_12.indicate_inset( [0.1, 0.37, 0.6, 0.5],edgecolor = 'black', alpha= 1.0,linewidth =2)
    axins_12.set_aspect('auto')   
    
    # ------------------------------------------------------------------
    #   Electronic Conditions
    # ------------------------------------------------------------------
    fig_13 = plt.figure("Battery_Pack_Performance_T")
    fig_13.set_size_inches(figure_width,figure_height) 
    axes_13 = fig_13.add_subplot(1,1,1)      
    axes_13.set_ylim(0, 1.2)       
    axes_13.set_ylabel('$\zeta$') 
    axes_13.minorticks_on()  
    axes_13.set_xlabel(r'$\hat{t}$')      
 
    # ------------------------------------------------------------------
    #   Electronic Conditions - SOC
    # ------------------------------------------------------------------
    fig_14 = plt.figure("Battery_Pack_Performance_SOC")
    fig_14.set_size_inches(figure_width,figure_height) 
    axes_14 = fig_14.add_subplot(1,1,1)   
    axes_14.set_ylabel('SOC')
    axes_14.minorticks_on()     
    axes_14.set_ylim(0.5,1.02)   
    fig_14.tight_layout()          
    axes_14.text(0.0, 0.96, "1", size=20, ha="center", va="center", color= "black", bbox=dict(facecolor = 'white', boxstyle="circle") )  
    axes_14.text(0.99, 0.63, "2", size=20, ha="center", va="center",color= "black", bbox=dict(facecolor = 'white',boxstyle="circle") )  
    axes_14.set_xlabel(r'$\hat{t}$')
     
    # ------------------------------------------------------------------
    #   Propeller Performance _ RPM 
    # ------------------------------------------------------------------
 
    fig_15 = plt.figure("Propeller_Performance_RPM")
    fig_15.set_size_inches(figure_width,figure_height)   
    axes_15 = fig_15.add_subplot(1,1,1)  
    axes_15.set_ylabel('RPM')      
    axes_15.minorticks_on()  
    axes_15.set_ylim(0, 3000)
    axes_15.set_xlabel(r'$\hat{t}$') 
    
    # ------------------------------------------------------------------
    #   Propeller Performance Thrust 
    # ------------------------------------------------------------------    
    fig_16 = plt.figure("Propeller_Performance_T")
    fig_16.set_size_inches(figure_width,figure_height)   
    axes_16 = fig_16.add_subplot(1,1,1) 
    axes_16.set_ylabel('T (kN)') 
    axes_16.minorticks_on()  
    axes_16.set_ylim(0,45) 
    axes_16.set_xlabel(r'$\hat{t}$') 
    
    # ------------------------------------------------------------------
    #   Propeller Performance - Torque
    # ------------------------------------------------------------------       
    fig_17 = plt.figure("Propeller_Performance_Q")
    fig_17.set_size_inches(figure_width,figure_height) 
    axes_17 = fig_17.add_subplot(1,1,1)
    axes_17.set_xlabel(r'$\hat{t}$')    
    axes_17.set_ylabel('Q (Nm)')      
    axes_17.minorticks_on()  
    axes_17.set_ylim(0, 1500)          

    # ------------------------------------------------------------------
    #   Propeller Performance - Propeller Efficiency 
    # ------------------------------------------------------------------     
    fig_18 = plt.figure("Propeller_Performance_Prop_eff")
    fig_18.set_size_inches(figure_width,figure_height)  
    axes_18 = fig_18.add_subplot(1,1,1) 
    axes_18.set_ylabel(r'$\eta_{prop/rot}$')    
    axes_18.set_xlabel(r'$\hat{t}$')                
    axes_18.set_ylim(0, 1.1)  
    axes_18.minorticks_on()     
    
    # ------------------------------------------------------------------
    #   Propeller Performance - Motor Efficiency
    # ------------------------------------------------------------------      
 
    fig_19 = plt.figure("Propeller_Performance_Mot_eff")
    fig_19.set_size_inches(figure_width,figure_height) 
    axes_19 = fig_19.add_subplot(1,1,1)        
    axes_19.set_xlabel(r'$\hat{t}$')
    axes_19.set_ylabel(r'$\eta_{motor}$') 
    axes_19.set_ylim(0.87, 1.02)             
    axes_19.minorticks_on()      
    
    # ------------------------------------------------------------------
    #   Propeller Performance - Tip Mach 
    # ------------------------------------------------------------------   
    fig_20 = plt.figure("Propeller_Performance_Mach_Tip")
    fig_20.set_size_inches(figure_width,figure_height) 
    axes_20 = fig_20.add_subplot(1,1,1)        
    axes_20.set_xlabel(r'$\hat{t}$')
    axes_20.set_ylabel('$M_{tip}$')  
    axes_20.set_ylim(0, 0.85)  
    axes_20.minorticks_on()      
    
    return axes_1,axins_1,axes_2, axes_4,axes_5,axes_6,axes_7,axes_8,axes_9,axes_11,axes_10,\
           axes_12,axins_12,axes_13,axes_14,axes_15,axes_16,axes_17,axes_18,axes_19,axes_20,axes_21,\
           fig_1,fig_2, fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,fig_21,fig_10,fig_11,fig_12,fig_13,\
           fig_14,fig_15,fig_16,fig_17,fig_18,fig_19,fig_20 


# ------------------------------------------------------------------ 
# Save Figures
# ------------------------------------------------------------------ 
def save_figures(fig_1,fig_2, fig_4,fig_5,fig_6,fig_7,fig_8,fig_9,\
                 fig_21,fig_10,fig_11,fig_12,fig_13,fig_14,fig_15,fig_16,\
                 fig_17,fig_18,fig_19,fig_20):
    
    fig_1.savefig("Flight_Conditions_Altitude.png") 
    fig_2.savefig("Flight_Conditions_Range.png")  
    fig_4.savefig("Aero_Conditions_AoA.png") 
    fig_5.savefig("Aero_Conditions_CL.png") 
    fig_6.savefig("Aero_Conditions_CD.png") 
    fig_7.savefig("Aero_Conditions_L_D.png") 
    fig_8.savefig("Battery_Pack_Performance_E.png") 
    fig_9.savefig("Battery_Pack_Performance_V.png") 
    fig_10.savefig("Battery_Pack_Performance_DL.png") 
    fig_11.savefig("Battery_Pack_Performance_PL.png") 
    fig_12.savefig("Battery_Pack_Performance_C_rate.png") 
    fig_13.savefig("Battery_Pack_Performance_T.png") 
    fig_14.savefig("Battery_Pack_Performance_SOC.png")
    fig_15.savefig("Propeller_Performance_RPM.png") 
    fig_16.savefig("Propeller_Performance_T.png") 
    fig_17.savefig("Propeller_Performance_Q.png") 
    fig_18.savefig("Propeller_Performance_Prop_eff.png") 
    fig_19.savefig("Propeller_Performance_Mot_eff.png") 
    fig_20.savefig("Propeller_Performance_Mach_Tip.png")  
    fig_21.savefig("Battery_Pack_Performance_Vul.png") 
    
    return  


# ----------------------------------------------------------------------
#  Add Grid Lines 
# ----------------------------------------------------------------------
def add_grid_lines(axes): 

    axes.minorticks_on() 
        
    return 

 
# ----------------------------------------------------------------------
#  Load results
# ----------------------------------------------------------------------     
def load_results(filename):
    load_file =   filename + '.pkl'
    # Store data (serialize)
    with open(load_file, 'rb') as file:
        results = pickle.load(file)
        
    return  results 

if __name__ == '__main__': 
    main()    
    plt.show()