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
     

    fig = plt.figure("Flight_Conditions_Noise")
    fig.set_size_inches(plot_parameters.figure_width,plot_parameters.figure_height)
    axes = fig.add_subplot(1,1,1)   
    axes.set_ylim(40,80)   
    axes.set_ylabel('SPL$_{Amax}$ (dBA)')  
    #axes.set_xlabel(r'$\hat{t}$') 
    axes.minorticks_on()   
     
    N_gm_x = 3# 12
    N_gm_y = 3 # 4
    header =  '../../XX_Supplementary/Aircraft_Models_and_Simulations/' 
     
     
    sr_noise_filename_Q1       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q1'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    sr_noise_filename_Q2       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q2'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    sr_noise_filename_Q3       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q3'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    sr_noise_filename_Q4       = header + 'Stopped_Rotor_Approach_Departure_Noise_Q4'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    sr_hover_filename          = header + 'Stopped_Rotor_Hover_Mission' 
    sr_noise_results_raw_Q1    = load_results(sr_noise_filename_Q1)
    sr_noise_results_raw_Q2    = load_results(sr_noise_filename_Q2)
    sr_noise_results_raw_Q3    = load_results(sr_noise_filename_Q3)
    sr_noise_results_raw_Q4    = load_results(sr_noise_filename_Q4)
    sr_hover_results_raw       = load_results(sr_hover_filename) 
    sr_noise_res_Q1            = process_results(sr_noise_results_raw_Q1,N_gm_x ,N_gm_y,vehicle_name = 'SR')
    sr_noise_res_Q2            = process_results(sr_noise_results_raw_Q2,N_gm_x ,N_gm_y,vehicle_name = 'SR')
    sr_noise_res_Q3            = process_results(sr_noise_results_raw_Q3,N_gm_x ,N_gm_y,vehicle_name = 'SR')
    sr_noise_res_Q4            = process_results(sr_noise_results_raw_Q4,N_gm_x ,N_gm_y,vehicle_name = 'SR')
    sr_hover_results           = process_results(sr_hover_results_raw,N_gm_x ,N_gm_y,vehicle_name    = 'SR')
    plot_results(1,sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4,axes,plot_parameters,vehicle_name = 'SR')    
    plot_aircraft_hover_noise_contours(1,sr_hover_results,plot_parameters,vehicle_name = 'SR') 
    plot_flight_profile_noise_contours(1,sr_noise_res_Q1,sr_noise_res_Q2, sr_noise_res_Q3,sr_noise_res_Q4,plot_parameters,vehicle_name = 'SR') 
 
    fig.tight_layout()
    axes.legend(loc='upper center', ncol= 4, prop={'size': plot_parameters.legend_font})    
    fig.savefig("Flight_Conditions_Noise.pdf")  
          
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
# Plot Aircraft Hover Noise Contours at 500 ft
# ------------------------------------------------------------------
def plot_aircraft_hover_noise_contours(res_Q1,res_Q2,res_Q3,res_Q4,vehicle_name): 
     
     
    # set size of matrices 
    N_gm_x       = res_Q1.N_gm_x
    N_gm_y       = res_Q1.N_gm_y 
    num_cpts     = res_Q1.num_ctrl_pts*res_Q1.num_segments
    aircraft_SPL = np.zeros((num_cpts,2*N_gm_x, 2*N_gm_y))
    gm_mic_loc   = np.zeros((2*N_gm_x, 2*N_gm_y,3))
    
    
    aircraft_SPL[:,0:N_gm_x,0:N_gm_y] = res_Q1.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)
    aircraft_SPL[:,0:N_gm_x,N_gm_y:]  = res_Q2.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,0:N_gm_y]  = res_Q3.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,N_gm_y:]   = res_Q4.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 

    gm_mic_loc[0:N_gm_x,0:N_gm_y,:]   = res_Q1.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[0:N_gm_x,N_gm_y:,:]    = res_Q2.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,0:N_gm_y,:]    = res_Q3.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,N_gm_y:,:]     = res_Q4.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)    
    
    # ---------------------------------------------------------------------------
    # Stopped-Rotor 
    # ---------------------------------------------------------------------------    
    levs                = np.linspace(40,80,25)    
    SPL_gm              = np.max(aircraft_SPL,axis=0)
    fig_name            = 'Noise_Contour_500ft_' + vehicle_name
    fig                 = plt.figure(fig_name )  
    fig.set_size_inches(6, 6) 
    axes                = fig.add_subplot(1,1,1)   
    Range               = Range/Units.nmi
    Span                = Span/Units.nmi
    CS                  = axes.contourf(Range , Span,SPL_gm, levels  = levs, cmap=plt.cm.jet, extend='both')     
    cbar                = fig.colorbar(CS)
    cbar.ax.set_ylabel('SPL (dBA)', rotation =  90)     
    axes.set_ylabel('Spanwise $x_{fp}$ (nmi)',labelpad = 15)
    axes.set_xlabel('Streamwise $x_{fp}$ (nmi)')  
    axes.grid(False)  
    axes.minorticks_on()   
    plt.savefig(fig_name + 'pdf')
    return  

# ------------------------------------------------------------------
# Plot Flight Profile Noise Contours 
# ------------------------------------------------------------------
def plot_flight_profile_noise_contours(idx,res_Q1,res_Q2,res_Q3,res_Q4,PP,vehicle_name):     
          
    # figure parameters
    filename      = 'Noise_Contour' + vehicle_name
    fig           = plt.figure(filename) 
    fig.set_size_inches(PP.figure_width ,PP.figure_height)   

    fig.tight_layout(rect= (0.05,0.05,1,1))
          
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
    N_gm_x       = res_Q1.N_gm_x
    N_gm_y       = res_Q1.N_gm_y 
    num_cpts     = res_Q1.num_ctrl_pts*res_Q1.num_segments
    aircraft_SPL = np.zeros((num_cpts,2*N_gm_x, 2*N_gm_y))
    gm_mic_loc   = np.zeros((2*N_gm_x, 2*N_gm_y,3)) 
    
    aircraft_SPL[:,0:N_gm_x,0:N_gm_y] = res_Q1.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)
    aircraft_SPL[:,0:N_gm_x,N_gm_y:]  = res_Q2.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,0:N_gm_y]  = res_Q3.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,N_gm_y:]   = res_Q4.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)   
    
    gm_mic_loc[0:N_gm_x,0:N_gm_y,:]   = res_Q1.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[0:N_gm_x,N_gm_y:,:]    = res_Q2.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,0:N_gm_y,:]    = res_Q3.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,N_gm_y:,:]     = res_Q4.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)   
  
    
    Range_x    = gm_mic_loc[:,0,0]/Units.nmi
    Range_y    = gm_mic_loc[0,:,1]/Units.nmi 
    Y, X       = np.meshgrid(Range_y, Range_x)
    levs       = np.linspace(40,80,17)    
    
    # post processing 
    #aircraft_SPL = np.nan_to_num(aircraft_SPL) #  CONVERT NANs TO NUMS
    max_SPL    = np.max(aircraft_SPL,axis=0)   
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
# Plot Figures
# ------------------------------------------------------------------ 
def plot_results(idx,res_Q1,res_Q2,res_Q3,res_Q4,axes,PP,vehicle_name):   
    
    # set size of matrices 
    N_gm_x       = res_Q1.N_gm_x
    N_gm_y       = res_Q1.N_gm_y 
    num_cpts     = res_Q1.num_ctrl_pts*res_Q1.num_segments
    aircraft_SPL = np.zeros((num_cpts,2*N_gm_x, 2*N_gm_y))
    gm_mic_loc   = np.zeros((2*N_gm_x, 2*N_gm_y,3))
    
    
    aircraft_SPL[:,0:N_gm_x,0:N_gm_y] = res_Q1.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y)
    aircraft_SPL[:,0:N_gm_x,N_gm_y:]  = res_Q2.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,0:N_gm_y]  = res_Q3.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 
    aircraft_SPL[:,N_gm_x:,N_gm_y:]   = res_Q4.SPL_contour.reshape(num_cpts,N_gm_x ,N_gm_y) 

    gm_mic_loc[0:N_gm_x,0:N_gm_y,:]   = res_Q1.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[0:N_gm_x,N_gm_y:,:]    = res_Q2.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,0:N_gm_y,:]    = res_Q3.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)
    gm_mic_loc[N_gm_x:,N_gm_y:,:]     = res_Q4.gm_mic_loc.reshape(N_gm_x ,N_gm_y,3)    
  
    Range      = gm_mic_loc[:,0,0]/Units.nmi
    max_SPL    = np.max(aircraft_SPL,axis=0)      
    axes.plot(Range,max_SPL[:,1], color = PP.colors[idx] , linestyle = PP.line_style[0],
              marker = PP.markers[idx] , markersize = PP.marker_size , linewidth= PP.line_width ,label=vehicle_name)   
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