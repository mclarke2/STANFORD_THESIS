# Vehicle Comparisons

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------    
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Plots.Geometry import *  
from SUAVE.Core import Data , Units 
import scipy.ndimage as ndimage
import plotly.graph_objects as go  
import numpy as np    
import matplotlib.pyplot as plt    
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
    plot_parameters.line_width       = 3
    plot_parameters.line_style       = ['-','--']
    plot_parameters.figure_width     = 12 
    plot_parameters.figure_height    = 10 
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 20 
    plot_parameters.plot_grid        = True   
    plot_parameters.markers          = ['s','^','X','o','P','D','X','*']
    plot_parameters.colors           = ['black','mediumblue','darkgreen','firebrick']   
    plot_parameters.colors2          = ['grey','darkcyan','green','red']                  
    plot_parameters.legend_font      = 20                             # legend_font_size       
    plot_parameters.marker_size      = 14    
    N_gm_x = 20 # 12
    N_gm_y = 5 # 4
    header =  '../Aircraft_Models_and_Simulations/' 
    
    plot_ECTOL_results = False
    plot_SR_results    = True
    plot_TW_results    = False
    plot_MR_results    = False
    
    
    if plot_ECTOL_results:  
        ectol_noise_filename_Q1       = header + 'ECTOL_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        ectol_noise_filename_Q2       = header + 'ECTOL_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        ectol_noise_filename_Q3       = header + 'ECTOL_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        ectol_noise_filename_Q4       = header + 'ECTOL_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)  
        ectol_noise_results_raw_Q1    = load_results(ectol_noise_filename_Q1)
        ectol_noise_results_raw_Q2    = load_results(ectol_noise_filename_Q2)
        ectol_noise_results_raw_Q3    = load_results(ectol_noise_filename_Q3)
        ectol_noise_results_raw_Q4    = load_results(ectol_noise_filename_Q4)  
        ectol_noise_res_Q1            = process_results(ectol_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name  = 'ECTOL')
        ectol_noise_res_Q2            = process_results(ectol_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name  = 'ECTOL')
        ectol_noise_res_Q3            = process_results(ectol_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name  = 'ECTOL')
        ectol_noise_res_Q4            = process_results(ectol_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name  = 'ECTOL')    
        plot_flight_profile_max_noise_contour(0,ectol_noise_res_Q1,ectol_noise_res_Q2,ectol_noise_res_Q3,ectol_noise_res_Q4,plot_parameters,vehicle_name = 'ECTOL')
   
    
    N_gm_x = 10 # 12
    N_gm_y = 5 # 4   
    if plot_SR_results:
        sr_noise_filename_Q1       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        sr_noise_filename_Q2       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        sr_noise_filename_Q3       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        sr_noise_filename_Q4       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) 
        sr_noise_results_raw_Q1    = load_results(sr_noise_filename_Q1)
        sr_noise_results_raw_Q2    = load_results(sr_noise_filename_Q2)
        sr_noise_results_raw_Q3    = load_results(sr_noise_filename_Q3)
        sr_noise_results_raw_Q4    = load_results(sr_noise_filename_Q4) 
        sr_noise_res_Q1            = process_results(sr_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name = 'SR')
        sr_noise_res_Q2            = process_results(sr_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name = 'SR')
        sr_noise_res_Q3            = process_results(sr_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name = 'SR')
        sr_noise_res_Q4            = process_results(sr_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name = 'SR')  
        plot_flight_profile_max_noise_contour(1,sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4,plot_parameters,vehicle_name = 'SR') 
     
    if plot_TW_results:
        tw_noise_filename_Q1       = header + 'Tiltwing_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        tw_noise_filename_Q2       = header + 'Tiltwing_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        tw_noise_filename_Q3       = header + 'Tiltwing_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        tw_noise_filename_Q4       = header + 'Tiltwing_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) 
        tw_noise_results_raw_Q1    = load_results(tw_noise_filename_Q1)
        tw_noise_results_raw_Q2    = load_results(tw_noise_filename_Q2)
        tw_noise_results_raw_Q3    = load_results(tw_noise_filename_Q3)
        tw_noise_results_raw_Q4    = load_results(tw_noise_filename_Q4) 
        tw_noise_res_Q1            = process_results(tw_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name  = 'TW')
        tw_noise_res_Q2            = process_results(tw_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name  = 'TW')
        tw_noise_res_Q3            = process_results(tw_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name  = 'TW')
        tw_noise_res_Q4            = process_results(tw_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name  = 'TW')      
        plot_flight_profile_max_noise_contour(2,tw_noise_res_Q1,tw_noise_res_Q2, tw_noise_res_Q3,tw_noise_res_Q4,plot_parameters,vehicle_name = 'TW') 
    
     
    if plot_MR_results:
        mr_noise_filename_Q1       = header + 'Multirotor_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        mr_noise_filename_Q2       = header + 'Multirotor_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        mr_noise_filename_Q3       = header + 'Multirotor_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
        mr_noise_filename_Q4       = header + 'Multirotor_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y) 
        mr_noise_results_raw_Q1    = load_results(mr_noise_filename_Q1)
        mr_noise_results_raw_Q2    = load_results(mr_noise_filename_Q2)
        mr_noise_results_raw_Q3    = load_results(mr_noise_filename_Q3)
        mr_noise_results_raw_Q4    = load_results(mr_noise_filename_Q4) 
        mr_noise_res_Q1            = process_results(mr_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name  = 'MR')
        mr_noise_res_Q2            = process_results(mr_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name  = 'MR')
        mr_noise_res_Q3            = process_results(mr_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name  = 'MR')
        mr_noise_res_Q4            = process_results(mr_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name  = 'MR')   
        plot_flight_profile_max_noise_contour(3,mr_noise_res_Q1,mr_noise_res_Q2, mr_noise_res_Q3,mr_noise_res_Q4,plot_parameters,vehicle_name = 'MR') 

    return
 
# ------------------------------------------------------------------
# Process Results 
# ------------------------------------------------------------------
def process_results(res,N_gm_x ,N_gm_y,vehicle_name ):
    '''This function cleans up the data and connects segments for plots ''' 

    num_ctrl_pts   = len(res.segments[0].conditions.frames.inertial.time[:,0] )
    num_segments   = len(res.segments.keys()) 
    data_dimension = num_ctrl_pts*num_segments
    
    PD              = Data()
    PD.time         = np.zeros(data_dimension)
    PD.range_nm     = np.zeros(data_dimension)
    PD.altitude     = np.zeros(data_dimension) 
    PD.time_normalized = np.zeros(data_dimension)   
    PD.num_gm       = res.segments[0].conditions.noise.number_ground_microphones 
    PD.gm_mic_loc   = res.segments[0].analyses.noise.settings.ground_microphone_locations 
    PD.SPL_contour  = np.zeros((data_dimension,N_gm_x*N_gm_y)) 
    PD.N_gm_x       = N_gm_x  
    PD.N_gm_y       = N_gm_y  
    PD.aircraft_pos = np.zeros((data_dimension,3)) 
    dim_segs        = len(res.segments)    
    PD.num_segments = dim_segs
    PD.num_ctrl_pts = num_ctrl_pts
    
    Flight_Time        = res.segments[-1].conditions.frames.inertial.time[-1,0]
    for i in range(num_segments):  
        time            = res.segments[i].conditions.frames.inertial.time[:,0] / Units.min  
        time_normalized = res.segments[i].conditions.frames.inertial.time[:,0]/Flight_Time
        range_nm        = res.segments[i].conditions.frames.inertial.position_vector[:,0] / Units.nmi #  *0.000539957 
        altitude        = res.segments[i].conditions.freestream.altitude[:,0]/Units.feet    
        SPL_contour = res.segments[i].conditions.noise.total_SPL_dBA   
        pos             = res.segments[i].conditions.frames.inertial.position_vector   
        
        
        PD.time_normalized[i*num_ctrl_pts:(i+1)*num_ctrl_pts] = time_normalized
        PD.time[i*num_ctrl_pts:(i+1)*num_ctrl_pts]            = time          
        PD.range_nm[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = range_nm      
        PD.altitude[i*num_ctrl_pts:(i+1)*num_ctrl_pts]        = altitude       
        PD.SPL_contour[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]   = SPL_contour   
        PD.aircraft_pos[i*num_ctrl_pts:(i+1)*num_ctrl_pts,:]  = pos       
        
            
    return PD 


# ------------------------------------------------------------------
# Plot Flight Profile Max Noise Contours 
# ------------------------------------------------------------------
def plot_flight_profile_max_noise_contour(idx,res_Q1,res_Q2,res_Q3,res_Q4,PP,vehicle_name):     
          
    # figure parameters
    filename      = 'Noise_Contour_' + vehicle_name  

    # set size of matrices 
    N_gm_x       = res_Q1.N_gm_x
    N_gm_y       = res_Q1.N_gm_y 
    num_cpts     = res_Q1.num_ctrl_pts*res_Q1.num_segments
    Aircraft_pos = res_Q1.aircraft_pos
    aircraft_SPL = np.zeros((num_cpts,2*N_gm_x, 2*N_gm_y))
    gm_mic_loc   = np.zeros((2*N_gm_x, 2*N_gm_y,3)) 
    
    aircraft_SPL[:,0:N_gm_x,0:N_gm_y] = res_Q1.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)
    aircraft_SPL[:,0:N_gm_x,N_gm_y:]  = res_Q2.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,0:N_gm_y]  = res_Q3.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,N_gm_y:]   = res_Q4.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)  
    max_SPL                           = np.max(aircraft_SPL,axis=0)    
    
    gm_mic_loc[0:N_gm_x,0:N_gm_y,:]   = res_Q1.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[0:N_gm_x,N_gm_y:,:]    = res_Q2.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,0:N_gm_y,:]    = res_Q3.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,N_gm_y:,:]     = res_Q4.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)    
    Range_x                           = gm_mic_loc[:,0,0] # /Units.nmi
    Range_y                           = gm_mic_loc[0,:,1] # /Units.nmi 
    Y, X                              = np.meshgrid(Range_y, Range_x)  
    
    Range  = X
    Span   = Y
    SPL_gm = max_SPL 
                            
    # Level Ground Noise Contour 
    ground_surface      = np.zeros(Range.shape)   
    
    # ---------------------------------------------------------------------------
    # Comprehensive contour including buildings 
    # ---------------------------------------------------------------------------
    plot_data           = []
    ground_contour_l     = contour_surface_slice(Range, Span,ground_surface , SPL_gm)
    ground_contour_r     = contour_surface_slice(Range, -Span,ground_surface , SPL_gm)
    plot_data.append(ground_contour_l)
    plot_data.append(ground_contour_r)
    
    # Aircraft Trajectory
    aircraft_trajectory = go.Scatter3d(x=Aircraft_pos[:,0], y=Aircraft_pos[:,1], z=-Aircraft_pos[:,2],
                                mode='markers', 
                                marker=dict(size=6,color='black',opacity=0.8),
                                line=dict(color='black',width=2))    
    plot_data.append(aircraft_trajectory)
     
    # Define Colorbar Bounds  
    min_SPL     = 30 
    max_SPL     = 80
    min_alt     = 0
    max_alt     = np.max(-Aircraft_pos[:,2]) 
    
    # Adjust Plot Camera 
    #camera        = dict(up=dict(x=1, y=1, z=1), center=dict(x=0, y=0, z=0), eye=dict(x=-1., y=-1., z=.25))  
    
    camera = dict( up=dict(x=0, y=0, z=1), center=dict(x=0, y=0, z=-0.1), eye=dict(x=-1.5, y=-1.5, z=0.5) )    
    fig = go.Figure(data=plot_data)
    fig.update_layout(
             title_text= filename, 
             title_x = 0.5,
             width   = 1000,
             height  = 1000,
             font_family = "Times New Roman",
             font_size=14,          
             scene_aspectmode= "manual",
            # margin=dict(t=100, r=100, l=100, b=100),   
             scene = dict(xaxis = dict( backgroundcolor="white", gridcolor="white", showbackground=False, zerolinecolor="white",),
                          yaxis = dict( backgroundcolor="white", gridcolor="white", showbackground=False, zerolinecolor="white",),
                          zaxis = dict( backgroundcolor="white", gridcolor="white", showbackground=False, zerolinecolor="white",),
                          xaxis_title='Range (m)',
                          yaxis_title='Sideline Distance (m)',
                          zaxis_title='Altitude (m)'),             
             coloraxis=dict(colorscale='Jet',
                            colorbar_thickness=30,
                            colorbar_nticks=20,
                            colorbar_title_text = 'SPL max (dBA)',
                            colorbar_tickfont_size=18,
                            colorbar_title_side="right",
                            colorbar_ypad = 20,
                            colorbar_len= 0.5,
                            **colorax(min_SPL, max_SPL)),
             scene_camera=camera)  
    
    fig.update_scenes(aspectratio =  dict(x=1, y=1, z=0.5)) 
    fig.show()
    
    image_name = filename + '.png'
    #fig.write_image(image_name)
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