# ----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------- 
from SUAVE.Core import Units
import numpy as np
import matplotlib.pyplot as plt  
import matplotlib.ticker as ticker   
import matplotlib.cm as cm 
import matplotlib.patches as patches
import itertools
from labellines import labelLines

import pickle 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    vehicle_name = 'ECTOL'# GOOD ! 
    ectol_filenames = ['../ECTOL_EROL/ECTOL_365_Days_60.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_65.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_70.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_75.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_80.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_85.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_90.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_95.0_Nmi.pkl',
                       '../ECTOL_EROL/ECTOL_365_Days_100.0_Nmi.pkl']
    ectol_ranges       = np.array([60,65,70,75,80,85,90,95,100])
    E    = 331257600*0.8 # Energy 
    W    = 1156.66054*9.81 # Weight
    L_D  = 9.289 # L/D ratio
    V    = 175.* Units['mph'] # Cruise Velocity
    plot_results(vehicle_name,ectol_filenames,ectol_ranges,E,W,L_D,V)
    
    
    vehicle_name = 'SR' # 90 mi needs correction
    sr_filenames = ['../SR_EROL/SR_365_Days_60.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_65.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_70.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_75.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_80.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_85.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_90.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_95.0_Nmi.pkl',
                       '../SR_EROL/SR_365_Days_100.0_Nmi.pkl']
    sr_ranges       = np.array([60,65,70,75,80,85,90,95,100])
    E    = 644112000.0*0.8 # Energy
    W    = 2800*9.81# Weight
    L_D  = 16.65 # L/D ratio
    V    = 175.* Units['mph']# Cruise Velocity
    plot_results(vehicle_name,sr_filenames,sr_ranges,E,W,L_D,V)
    
    
    vehicle_name = 'TW' # 90 mi needs correction
    tw_filenames =   ['../TW_EROL/TW_365_Days_60.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_65.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_70.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_75.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_80.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_85.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_90.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_95.0_Nmi.pkl',
                       '../TW_EROL/TW_365_Days_100.0_Nmi.pkl']  
    tw_ranges       =np.array([60,65,70,75,80,85,90,95,100])
    E    = 897156000.0*0.8 # Energy
    W    = 2700*9.81 # Weight
    L_D  = 16.538 # L/D ratio
    V    = 175.* Units['mph'] # Cruise Velocity
    plot_results(vehicle_name,tw_filenames,tw_ranges,E,W,L_D,V)
    
    vehicle_name = 'MR' # GOOD ! 
    mr_filenames =    ['../MR_EROL/MR_365_Days_20.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_22.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_25.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_28.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_30.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_32.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_35.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_38.0_Nmi.pkl',
                       '../MR_EROL/MR_365_Days_40.0_Nmi.pkl']    
    mr_ranges       = np.array([20,22.5,25,27.5,30,32.5,35,37.5,40])    
    E    = 2269804680.0*0.8 # Energy
    W    = 3800*9.81 # Weight
    L_D  = -0.1025 # L/D ratio
    V    = 75. * Units['mph']     # Cruise Velocity
    plot_results(vehicle_name,mr_filenames,mr_ranges,E,W,L_D,V) 
    
    return 

def plot_results(vehicle_name,filenames,ranges,E,W,L_D,V):
    # Plot settings  
    line_width                     = 4
    plt.rcParams['axes.linewidth'] = 4.
    plt.rcParams["font.family"]    = "Times New Roman"
    plt.rcParams.update({'font.size': 36})
    figure_width  = 11
    figure_height = 8
    ls            = 24 # legend font size 
    
    cases = 9    
    col1  =  cm.viridis(np.linspace(0,1,cases)) # ['black','darkblue','darkgreen','firebrick','goldenrod']  
    col2  =  cm.viridis(np.linspace(0,1,cases)) # ['grey','blue','green','red','orange']
    ls1   = ['-']*cases
    ls2   = ['--']*cases  
    m1    = ['o']*cases 
    m2    = ['^']*cases 
    ms1   = 14  
    ms2   = 14    
    
    day_step  = 10 
    iter_rate = int(np.ceil(365/day_step))
    days      = np.linspace(0,365,iter_rate) 
    
    # Initialize 2d contour matrices as nans 
    Capacity_Fade_contour          = np.empty((cases,iter_rate)) 
    Capacity_Fade_contour[:]       = np.nan    
    Resistance_Growth_contour      = np.empty((cases,iter_rate)) 
    Resistance_Growth_contour[:]   = np.nan    
    Max_Temp_contour               = np.empty((cases,iter_rate)) 
    Max_Temp_contour[:]            = np.nan    
    Max_C_rate_contour             = np.empty((cases,iter_rate)) 
    Max_C_rate_contour[:]          = np.nan    
    Charge_TP_contour              = np.empty((cases,iter_rate)) 
    Charge_TP_contour[:]           = np.nan   
    Bat_Energy_contour             = np.empty((cases,iter_rate)) 
    Bat_Energy_contour[:]          = np.nan   
    Last_day                       = np.zeros(cases)
    
    # setup axes  
    axis1,fig1, axis2, fig2, axis3,fig3, axis4,fig4,axis5,fig5,fig12, axis12  = setup_line_plots(vehicle_name,figure_width,figure_height) 
    
    ''' NOTE: The dimension of most results is number of days x number of segments of the LAST flight of each day ;
    that ism 365 x 9 for the ECTOL for example'''
    for i in range(len(filenames)): 

        with open(filenames[i], 'rb') as file:  
            res = pickle.load(file)   
        
        range_label = str(ranges[i]) + ' nmi'
        
        bat_SOC_end_of_flight  = res.Bat_Final_Energy[:,-2]/res.Bat_Start_Energy[:,0]  # CHANGE!!! 
        try: 
            cut_off_day = np.where(bat_SOC_end_of_flight < 0.2)[0][0]  
        except:
            cut_off_day = 364
        time                   = res.Mission_time[:cut_off_day,0][::day_step]
        num_vals               = len(time)
        Capacity_Fade          = res.E_Fade[:cut_off_day,0][::day_step]
        Resistance_Growth      = res.R_Growth[:cut_off_day,0][::day_step]
        Max_Daily_Temp         = np.max(res.Max_Seg_Temp,axis = 1)[:cut_off_day][::day_step]
        Charge_Throughput      = res.Charge[:cut_off_day,0][::day_step]
        Max_Daily_C_rate       = np.max(res.Max_Seg_C_rate,axis = 1)[:cut_off_day][::day_step]
        Bat_Energy             = res.Bat_Start_Energy[:cut_off_day,-2][::day_step]*(0.000277778/1000)  # CHANGE!!! 
        
        Capacity_Fade_contour[i,:num_vals]     = Capacity_Fade     
        Resistance_Growth_contour[i,:num_vals] = Resistance_Growth   
        Max_Temp_contour[i,:num_vals]          = Max_Daily_Temp   
        Max_C_rate_contour[i,:num_vals]        = Charge_Throughput
        Charge_TP_contour[i,:num_vals]         = Max_Daily_C_rate 
        Bat_Energy_contour[i,:num_vals]        = Bat_Energy
        
        Last_day[i] = cut_off_day
        
        # ---------------------------------------
        # Plot 1 : Capacity and Resistance 
        # --------------------------------------- 
        axis1.plot(time, Capacity_Fade , color = col1[i] ,\
                   linestyle = ls1[i], marker = m1[i] , markersize = ms1 , linewidth= line_width ,label = ',' )  
        axis1.plot(time, Resistance_Growth, color = col2[i] ,\
                   linestyle = ls2[i], marker = m2[i] , markersize = ms2 , linewidth= line_width,label=range_label )     
        axis1.set_ylim([0,5]) 
        axis1.set_xlim(0,370)
        axis1.set_ylabel(r'C/C$_{0}$ and R/R$_{0}$ ')    
        axis1.set_xlabel('Time (day)')  
        #axis1.legend(loc='upper left', prop={'size': ls},ncol = 2)
        format_axis(axis1) 
        fig1.tight_layout()    
        
    
        # ---------------------------------------    
        # Plot 2: Battery Temperature 
        # ---------------------------------------
        axis2.plot(time, Max_Daily_Temp  , color = col1[i] , \
                  linestyle = ls1[i], marker = m1[i] , markersize = ms1 , linewidth= line_width ,label= range_label)     
        axis2.set_ylabel('Cell Temperature (K)')    
        axis2.set_xlabel('Time (day)')  
        axis2.set_ylim(300,340) 
        axis2.set_xlim(0,370)
        axis2.legend(loc='upper center', prop={'size': ls}, ncol = 3)
        format_axis(axis2) 
        fig2.tight_layout()   
        
        
        # ---------------------------------------    
        # Plot 3 : Charge Throughput 
        # --------------------------------------- 
        axis3.plot(time, Charge_Throughput, color = col1[i] , linestyle = ls1[i],\
                marker = m1[i] , markersize = ms1 , linewidth= line_width ,label= range_label)     
        axis3.set_ylabel('Charge (Ah)')    
        axis3.set_xlabel('Time (day)')  
        axis3.set_ylim(0,15000)
        axis3.set_xlim(0,370)
        axis3.legend(loc='upper center', prop={'size': ls}, ncol = 3)
        format_axis(axis3) 
        fig3.tight_layout()  
        
        
        # ---------------------------------------    
        # Plot 4 : C rate 
        # ---------------------------------------
        axis4.plot(time,Max_Daily_C_rate, color = col1[i] , \
                 linestyle = ls1[i], marker = m1[i] , markersize = ms1 , linewidth= line_width ,label= range_label)    
        axis4.set_ylabel('Max C-Rate (1/hr)')    
        axis4.set_xlabel('Time (day)')  
        axis4.set_ylim(2,30)
        axis4.set_xlim(0,370)
        axis4.legend(loc='upper center', prop={'size': ls}, ncol = 3)
        format_axis(axis4)   
        fig4.tight_layout() 
        

        # ---------------------------------------    
        # Plot 5 : Energy
        # ---------------------------------------
        axis5.plot(time, Bat_Energy, color = col1[i] , \
                 linestyle = ls1[i], marker = m1[i] , markersize = ms1 , linewidth= line_width ,label= range_label)    
        axis5.set_ylabel('Battery Energy (kWh)')    
        axis5.set_xlabel('Time (day)')  
        axis5.set_ylim(0,500)
        axis5.set_xlim(0,370)
        axis5.legend(loc='upper center', prop={'size': ls}, ncol = 3)
        format_axis(axis5)   
        fig5.tight_layout() 
    
    handles, labels = axis1.get_legend_handles_labels()
    axis1.legend(flip(handles, 4), flip(labels, 4), loc='upper center', prop={'size': ls}, ncol=4)
        
    save_filename_1 =  vehicle_name + "_Capacity_IRG_1Yr" + '.png'
    fig1.savefig(save_filename_1)            
    
    save_filename_2 =  vehicle_name + "_Battery_Temperature_1Yr" + '.png'
    fig2.savefig(save_filename_2) 

    save_filename_3 =  vehicle_name + "_Charge_1Yr" + '.png'
    fig3.savefig(save_filename_3) 

    save_filename_4 =  vehicle_name + "_C_Rate_1Yr" + '.png'
    fig5.savefig(save_filename_4) 

    save_filename_5 =  vehicle_name + "_Energy_1Yr" + '.png'
    fig5.savefig(save_filename_5)        
    
    fig6, axis6,fig7, axis7,fig8, axis8, fig9,axis9, fig10,axis10, fig11,axis11 = setup_contour_plots(vehicle_name,figure_width,figure_height) 
    Days_2D,  Range_2D = np.meshgrid(days,ranges)
    
    c_bar_discretization = 20 
    levs6  = np.linspace(0.5,1,c_bar_discretization)
    levs7  = np.linspace(1,3,c_bar_discretization)
    levs8  = np.linspace(300,340,c_bar_discretization)
    levs9  = np.linspace(0,15,c_bar_discretization)
    levs10 = np.linspace(0,15000,c_bar_discretization)
    levs11 = np.linspace(20,250,c_bar_discretization)   
    
    # ---------------------------------------
    # Plot 6 : Capacity Contour
    # ---------------------------------------         
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.2f')    
    sfmt2 = ticker.FormatStrFormatter('%.0f')    
    CS6    = axis6.contourf(Days_2D,Range_2D,Capacity_Fade_contour ,lw=3, levels = levs6,extend='both',corner_mask=True )   
    cbar6  =  fig6.colorbar(CS6   , ax = axis6 , format= sfmt ) 
    cbar6.ax.set_ylabel('E/E$_{0}$' , labelpad=20) 
    add_hash_background(axis6)
    fig6.tight_layout()
    save_filename_6 =  vehicle_name + "_Capacity_Contour_1Yr" + '.png'
    fig6.savefig(save_filename_6)
    
    # ---------------------------------------
    # Plot 7 : Interal Resitance
    # ---------------------------------------    
    CS7           = axis7.contourf(Days_2D,Range_2D,Resistance_Growth_contour ,lw=3, levels = levs7,extend='both',corner_mask=True)
    cbar7  =  fig7.colorbar(CS7   , ax = axis7 , format= sfmt)
    cbar7.ax.set_ylabel('R/R$_{0}$' , labelpad=20) 
    add_hash_background(axis7)
    fig7.tight_layout()
    save_filename_7 =   vehicle_name + "_IRG_Contour_1Yr" + '.png'
    fig7.savefig(save_filename_7)
    
    # ---------------------------------------
    # Plot 8 : Cell Temperature
    # ---------------------------------------  
    CS8           = axis8.contourf(Days_2D,Range_2D,Max_Temp_contour ,lw=3, levels = levs8,extend='both',corner_mask=True)
    cbar8  =  fig8.colorbar(CS8   , ax = axis8 , format= sfmt2)
    cbar8.ax.set_ylabel('Cell Temperature (K)' , labelpad=20)
    add_hash_background(axis8)  
    fig8.tight_layout()
    save_filename_8 =   vehicle_name + "_Battery_Temperature_Contour_1Yr" + '.png'
    fig8.savefig(save_filename_8)
    
    # ---------------------------------------
    # Plot 9 : Daily Max Instantaneous C-Rate
    # ---------------------------------------  
    CS9    = axis9.contourf(Days_2D,Range_2D,Charge_TP_contour,lw=3, levels = levs9,extend='both',corner_mask=True) 
    cbar9  = fig9.colorbar(CS9 , ax = axis9 , format= sfmt)
    cbar9.ax.set_ylabel('Max C-Rate (1/hr)' , labelpad=20)  
    add_hash_background(axis9)
    fig9.tight_layout()
    save_filename_9 =   vehicle_name + "_C_Rate_Contour_1Yr" + '.png'
    fig9.savefig(save_filename_9)
    
    # ---------------------------------------
    # Plot 10 : Charge Throughput
    # ---------------------------------------  
    CS10    = axis10.contourf(Days_2D,Range_2D,Max_C_rate_contour ,lw=3, levels = levs10,extend='both',corner_mask=True)
    cbar10  =  fig10.colorbar(CS10   , ax = axis10 , format= sfmt)
    cbar10.ax.set_ylabel('Charge Throughput(Ah)' , labelpad=20)
    add_hash_background(axis10)  
    fig10.tight_layout()
    save_filename_10 =   vehicle_name + "_Charge_Contour_1Yr" + '.png'
    fig10.savefig(save_filename_10)
    
    
    # ---------------------------------------
    # Plot 11 : Battery Energy (kWh)
    # ---------------------------------------  
    CS11    = axis11.contourf(Days_2D,Range_2D,Bat_Energy_contour,lw=3, levels = levs11,extend='both',corner_mask=True) 
    cbar11  = fig11.colorbar(CS11 , ax = axis11 , format= sfmt)
    cbar11.ax.set_ylabel('Battery Energy (kWh)' , labelpad=20)
    add_hash_background(axis11)  
    fig11.tight_layout()
    save_filename_11 =  vehicle_name + "_Energy_Contour_1Yr" + '.png'
    fig11.savefig(save_filename_11)
    
    
    
    # Comparison with traditional method  
    eta  = np.array([0.2,0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6,0.65,0.7,0.75,0.8,0.85])
    time = np.linspace(1,360,10)
    for i in range(len(eta)):
        R     = E*(1/W)*eta[i]*L_D
        Range = np.ones_like(time)*R/Units.nmi
        axis12.plot( time , Range , linewidth = 1.5 , color = 'black' , linestyle = '--', marker = None ,label= '$\eta$ = ' + str(eta[i]))  
    axis12.set_ylabel('Range (nmi)')    
    axis12.set_xlabel('Time (day)')  
    axis12.set_xlim(0,360)
    axis12.set_ylim(ranges[0],ranges[-1])
    axis12.plot(Last_day,ranges, color = 'black' , linestyle = '-', marker = None, linewidth= line_width ,label= vehicle_name)   
    labelLines(axis12.get_lines()) 
    format_axis(axis12)   
    fig12.tight_layout()  

    save_filename_12 =  vehicle_name + "_Comparison" + '.png'    
    fig12.savefig(save_filename_12)
    
    return 

def flip(items, ncol):
    return itertools.chain(*[items[i::ncol] for i in range(ncol)])


def setup_contour_plots(vehicle_name,figure_width,figure_height):
    
    save_filename_6 =  vehicle_name + "_Capacity_Contour_1Yr"
    fig6  = plt.figure(save_filename_6)        
    fig6.set_size_inches(figure_width,figure_height)      
    axis6 = fig6.add_subplot(1,1,1)  
    axis6.set_xlabel('Time (day)')
    axis6.set_ylabel('Range (nmi)')         
    axis6.grid(False)  
    axis6.minorticks_on()     
    
    save_filename_7 =  vehicle_name + "_IRG_Contour_1Yr"
    fig7  = plt.figure(save_filename_7)        
    fig7.set_size_inches(figure_width,figure_height)      
    axis7 = fig7.add_subplot(1,1,1)  
    axis7.set_xlabel('Time (day)')
    axis7.set_ylabel('Range (nmi)')         
    axis7.grid(False)  
    axis7.minorticks_on() 
    
    save_filename_8 =  vehicle_name + "_Battery_Temperature_Contour_1Yr"
    fig8  = plt.figure(save_filename_8)        
    fig8.set_size_inches(figure_width,figure_height)      
    axis8 = fig8.add_subplot(1,1,1)  
    axis8.set_xlabel('Time (day)')
    axis8.set_ylabel('Range (nmi)')         
    axis8.grid(False)   
    axis8.minorticks_on() 
    
    
    save_filename_9 =  vehicle_name + "_C_Rate_Contour_1Yr"
    fig9  = plt.figure(save_filename_9)        
    fig9.set_size_inches(figure_width,figure_height)      
    axis9 = fig9.add_subplot(1,1,1)  
    axis9.set_xlabel('Time (day)')
    axis9.set_ylabel('Range (nmi)')         
    axis9.grid(False)   
    axis9.minorticks_on() 
    
    save_filename_10 =  vehicle_name + "_Charge_Contour_1Yr"
    fig10  = plt.figure(save_filename_10)        
    fig10.set_size_inches(figure_width,figure_height)      
    axis10 = fig10.add_subplot(1,1,1)  
    axis10.set_xlabel('Time (day)')
    axis10.set_ylabel('Range (nmi)')         
    axis10.grid(False)   
    axis10.minorticks_on() 
    
    save_filename_11 =  vehicle_name + "_Energy_Contour_1Yr"
    fig11  = plt.figure(save_filename_11)        
    fig11.set_size_inches(figure_width,figure_height)      
    axis11 = fig11.add_subplot(1,1,1)  
    axis11.set_xlabel('Time (day)')
    axis11.set_ylabel('Range (nmi)')         
    axis11.grid(False)   
    axis11.minorticks_on()     
    
    return fig6, axis6,fig7, axis7,fig8, axis8, fig9,axis9, fig10,axis10, fig11,axis11

def add_hash_background(axis):
    xmin, xmax = axis.get_xlim()
    ymin, ymax = axis.get_ylim()
    xy = (xmin,ymin)
    width = xmax - xmin
    height = ymax - ymin
    
    # create the patch and place it in the back of countourf (zorder!)
    p = patches.Rectangle(xy, width, height, hatch='/', fill=None, zorder=-10)
    axis.add_patch(p)
    return 

def setup_line_plots(vehicle_name,figure_width,figure_height):
    
    save_filename_1 =  vehicle_name + "_Capacity_IRG_1Yr"
    fig1 = plt.figure(save_filename_1)        
    fig1.set_size_inches(figure_width,figure_height)   
    axis1 = fig1.add_subplot(1,1,1)  

    save_filename_2 = vehicle_name + "_Battery_Temperature_1Yr"
    fig2 = plt.figure(save_filename_2)  
    fig2.set_size_inches(figure_width,figure_height)   
    axis2 = fig2.add_subplot(1,1,1)  

    save_filename_3 = vehicle_name +"_Charge_1Yr"
    fig3 = plt.figure(save_filename_3) 
    fig3.set_size_inches(figure_width,figure_height)   
    axis3 = fig3.add_subplot(1,1,1)    
    
    save_filename_4 = vehicle_name + "_C_Rate_1Yr"
    fig4 = plt.figure(save_filename_4)
    fig4.set_size_inches(figure_width,figure_height)   
    axis4 = fig4.add_subplot(1,1,1)      

    save_filename_5 = vehicle_name + "_Energy_1Yr"
    fig5 = plt.figure(save_filename_5)
    fig5.set_size_inches(figure_width,figure_height)   
    axis5 = fig5.add_subplot(1,1,1)     
    

    save_filename_12 = vehicle_name + "_Performance_Comparison"
    fig12 = plt.figure(save_filename_12)
    fig12.set_size_inches(figure_width,figure_height)   
    axis12 = fig12.add_subplot(1,1,1)      
    
    return axis1,fig1, axis2, fig2, axis3,fig3, axis4,fig4,axis5,fig5,fig12, axis12 


def plot_contour_plots(vehicle_name,filenames,ranges):
    
 
    num_simulated_days  = np.array([50,100,150,200,250,300,365])
    aircraft_ranges     = np.array([40,60,80,100,120,140,160]) 

    ECTOL_Capacity_Fade      = np.zeros((len(num_simulated_days),len(aircraft_ranges)))
    ECTOL_Resistance_Growth  = np.zeros_like(ECTOL_Capacity_Fade)
    ECTOL_Max_Battery_Temp   = np.zeros_like(ECTOL_Capacity_Fade)
    ECTOL_Charge_Throughout  = np.zeros_like(ECTOL_Capacity_Fade)
    ECTOL_Max_C_Rate         = np.zeros_like(ECTOL_Capacity_Fade)
    SR_Capacity_Fade       = np.zeros_like(ECTOL_Capacity_Fade)
    SR_Resistance_Growth   = np.zeros_like(ECTOL_Capacity_Fade)
    SR_Max_Battery_Temp    = np.zeros_like(ECTOL_Capacity_Fade)
    SR_Charge_Throughout   = np.zeros_like(ECTOL_Capacity_Fade)
    SR_Max_C_Rate          = np.zeros_like(ECTOL_Capacity_Fade)
    TW_Capacity_Fade       = np.zeros_like(ECTOL_Capacity_Fade)
    TW_Resistance_Growth   = np.zeros_like(ECTOL_Capacity_Fade)
    TW_Max_Battery_Temp    = np.zeros_like(ECTOL_Capacity_Fade)
    TW_Charge_Throughout   = np.zeros_like(ECTOL_Capacity_Fade) 
    TW_Max_C_Rate          = np.zeros_like(ECTOL_Capacity_Fade) 
    MR_Capacity_Fade       = np.zeros_like(ECTOL_Capacity_Fade)
    MR_Resistance_Growth   = np.zeros_like(ECTOL_Capacity_Fade)
    MR_Max_Battery_Temp    = np.zeros_like(ECTOL_Capacity_Fade)
    MR_Charge_Throughout   = np.zeros_like(ECTOL_Capacity_Fade) 
    MR_Max_C_Rate          = np.zeros_like(ECTOL_Capacity_Fade) 


    for nsd in range(num_simulated_days):
        for ar in range(aircraft_ranges): 

            # load results 
            ECTOL_results_path    = 'ECTOL_EROL/ECTOL' + '_' + str(num_simulated_days[nsd]) + '_Days_' + str(aircraft_ranges[ar]) + '_Miles'  
            with open(ECTOL_results_path, 'rb') as file_1:
                ECTOL_results = pickle.load(file_1)  
            # store results  
            ECTOL_Capacity_Fade[nsd,ar]     = ECTOL_results.C_Fade[-1]
            ECTOL_Resistance_Growth[nsd,ar] = ECTOL_results.R_Growth[-1]
            ECTOL_Max_Battery_Temp[nsd,ar]  = max(ECTOL_results.Max_Seg_Temp[-1,:])
            ECTOL_Charge_Throughout[nsd,ar] = ECTOL_results.Charge[-1]
            ECTOL_Max_C_Rate[nsd,ar]        = max(ECTOL_results.Max_Seg_C_rate[-1,:])

                ## load results 
            #SR_results_path    = 'SR_EROL/SR' + '_' + str(num_simulated_days[nsd]) + '_Days_' + str(aircraft_ranges[ar]) + '_Miles'  
            #with open(SR_results_path, 'rb') as file_2:
                #SR_results = pickle.load(file_2)   
            ## store results  
            #SR_Capacity_Fade[nsd,ar]      = SR_results.C_Fade[-1]
            #SR_Resistance_Growth[nsd,ar]  = SR_results.R_Growth[-1]
            #SR_Max_Battery_Temp[nsd,ar]   = max(SR_results.Max_Seg_Temp[-1,:])
            #SR_Charge_Throughout[nsd,ar]  = SR_results.Charge[-1]
            #SR_Max_C_Rate[nsd,ar]         = max(SR_results.Max_Seg_C_rate[-1,:])

                # load results 
            TW_results_path    = 'TW_EROL/TW' + '_' + str(num_simulated_days[nsd]) + '_Days_' + str(aircraft_ranges[ar]) + '_Miles'  
            with open(TW_results_path, 'rb') as file_3:
                TW_results = pickle.load(file_3) 
            # store results  
            TW_Capacity_Fade[nsd,ar]      = TW_results.C_Fade[-1]
            TW_Resistance_Growth[nsd,ar]  = TW_results.R_Growth[-1]
            TW_Max_Battery_Temp[nsd,ar]   = max(TW_results.Max_Seg_Temp[-1,:])
            TW_Charge_Throughout[nsd,ar]  = TW_results.Charge[-1]
            TW_Max_C_Rate[nsd,ar]         = max(TW_results.Max_Seg_C_rate[-1,:])

                # load results 
            MR_results_path    = 'MR_EROL/MR' + '_' + str(num_simulated_days[nsd]) + '_Days_' + str(aircraft_ranges[ar]) + '_Miles' 
            with open(MR_results_path, 'rb') as file_4:
                MR_results = pickle.load(file_4)     
            # store results  
            MR_Capacity_Fade[nsd,ar]      = MR_results.C_Fade[-1]
            MR_Resistance_Growth[nsd,ar]  = MR_results.R_Growth[-1]
            MR_Max_Battery_Temp[nsd,ar]   = max(MR_results.Max_Seg_Temp[-1,:])
            MR_Charge_Throughout[nsd,ar]  = MR_results.Charge[-1]
            MR_Max_C_Rate[nsd,ar]         = max(MR_results.Max_Seg_C_rate[-1,:])



    # Plot settings   
    plt.rcParams['axes.linewidth'] = 4.
    plt.rcParams["font.family"]    = "Times New Roman"
    plt.rcParams.update({'font.size': 36})
    figure_width  = 14
    figure_height = 8 

    levs                 = np.linspace(65,120,35)
    DAYS, RANGE  = np.meshgrid(num_simulated_days, aircraft_ranges)  

    # ---------------------------------------
    # Plot 1 : Capacity and Resistance 
    # ---------------------------------------

    save_filename_1 = "Capacity_EROL"
    fig = plt.figure(save_filename_1)        
    fig.set_size_inches(figure_width,figure_height) 


    ECTOL_axis        = fig.add_subplot(1,4,1)  
    ECTOL_CS          = ECTOL_axis.contourf(DAYS, RANGE ,ECTOL_Capacity_Fade,lw=3, levels = levs)   
    ECTOL_axis.set_title('General Aviation')  
    ECTOL_axis.set_xlabel('Range (mi)')
    ECTOL_axis.set_ylabel('Time (days)')

    SR_axis        = fig.add_subplot(1,4,2)  
    SR_CS          = SR_axis.contourf(DAYS, RANGE ,SR_Capacity_Fade, lw=3,levels = levs)       
    SR_axis.set_title('Stopped-rotor')  
    SR_axis.set_xlabel('Range (mi)')   


    TW_axis        = fig.add_subplot(1,4,3)  
    TW_CS          = TW_axis.contourf(DAYS, RANGE ,TW_Capacity_Fade, lw=3,levels = levs)       
    TW_axis.set_title('Tilt-wing')  
    TW_axis.set_xlabel('Range (mi)')    


    MR_axis        = fig.add_subplot(1,4,4)  
    MR_CS          = MR_axis.contourf(DAYS, RANGE ,MR_Capacity_Fade,lw=3, levels = levs)       
    MR_axis.set_title('Multi-rotor')  
    MR_axis.set_xlabel('Range (mi)')        

    format_axis(axis) 
    plt.tight_layout()     

    # ---------------------------------------
    # Plot 3 : Capacity and Resistance 
    # ---------------------------------------

    save_filename_2 = "Resistance_EROL"
    fig = plt.figure(save_filename_2)        
    fig.set_size_inches(figure_width,figure_height) 


    ECTOL_axis        = fig.add_subplot(1,4,1)  
    ECTOL_CS          = ECTOL_axis.contourf(DAYS, RANGE ,ECTOL_Resistance_Growth ,lw=3, levels = levs)   
    ECTOL_axis.set_title('General Aviation')  
    ECTOL_axis.set_xlabel('Range (mi)')
    ECTOL_axis.set_ylabel('Time (days)')

    SR_axis        = fig.add_subplot(1,4,2)  
    SR_CS          = SR_axis.contourf(DAYS, RANGE ,SR_Resistance_Growth, lw=3,levels = levs)       
    SR_axis.set_title('Stopped-rotor')  
    SR_axis.set_xlabel('Range (mi)')   


    TW_axis        = fig.add_subplot(1,4,3)  
    TW_CS          = TW_axis.contourf(DAYS, RANGE ,TW_Resistance_Growth, lw=3,levels = levs)       
    TW_axis.set_title('Tilt-wing')  
    TW_axis.set_xlabel('Range (mi)')    


    MR_axis        = fig.add_subplot(1,4,4)  
    MR_CS          = MR_axis.contourf(DAYS, RANGE ,MR_Resistance_Growth,lw=3, levels = levs)       
    MR_axis.set_title('Multi-rotor')  
    MR_axis.set_xlabel('Range (mi)')        

    format_axis(axis) 
    plt.tight_layout()     



    # ---------------------------------------    
    # Plot 3: Battery Temperature 
    # ---------------------------------------

    save_filename_3 = "Battery_Temperature_1Yr"
    fig = plt.figure(save_filename_3)  
    fig.set_size_inches(figure_width,figure_height)   
    axis = fig.add_subplot(1,1,1)     

    ECTOL_axis        = fig.add_subplot(1,4,1)  
    ECTOL_CS          = ECTOL_axis.contourf(DAYS, RANGE ,ECTOL_Max_Battery_Temp,lw=3, levels = levs)   
    ECTOL_axis.set_title('General Aviation')  
    ECTOL_axis.set_xlabel('Range (mi)')
    ECTOL_axis.set_ylabel('Time (days)')

    SR_axis        = fig.add_subplot(1,4,2)  
    SR_CS          = SR_axis.contourf(DAYS, RANGE ,SR_Max_Battery_Temp, lw=3,levels = levs)       
    SR_axis.set_title('Stopped-rotor')  
    SR_axis.set_xlabel('Range (mi)')   


    TW_axis        = fig.add_subplot(1,4,3)  
    TW_CS          = TW_axis.contourf(DAYS, RANGE ,TW_Max_Battery_Temp, lw=3,levels = levs)       
    TW_axis.set_title('Tilt-wing')  
    TW_axis.set_xlabel('Range (mi)')    


    MR_axis        = fig.add_subplot(1,4,4)  
    MR_CS          = MR_axis.contourf(DAYS, RANGE ,MR_Max_Battery_Temp,lw=3, levels = levs)       
    MR_axis.set_title('Multi-rotor')  
    MR_axis.set_xlabel('Range (mi)')    

    format_axis(axis) 
    plt.tight_layout()   
    fig = plt.figure(save_filename_2)  


    # ---------------------------------------    
    # Plot 4 : Charge Throughput 
    # --------------------------------------- 

    save_filename_4 = "Charge_1Yr"
    fig = plt.figure(save_filename_4) 
    fig.set_size_inches(figure_width,figure_height)   
    axis = fig.add_subplot(1,1,1)         

    ECTOL_axis        = fig.add_subplot(1,4,1)  
    ECTOL_CS          = ECTOL_axis.contourf(DAYS, RANGE ,ECTOL_Charge_Throughout,lw=3, levels = levs)   
    ECTOL_axis.set_title('General Aviation')  
    ECTOL_axis.set_xlabel('Range (mi)')
    ECTOL_axis.set_ylabel('Time (days)')

    SR_axis        = fig.add_subplot(1,4,2)  
    SR_CS          = SR_axis.contourf(DAYS, RANGE ,SR_Charge_Throughout, lw=3,levels = levs)       
    SR_axis.set_title('Stopped-rotor')  
    SR_axis.set_xlabel('Range (mi)')   


    TW_axis        = fig.add_subplot(1,4,3)  
    TW_CS          = TW_axis.contourf(DAYS, RANGE ,TW_Charge_Throughout, lw=3,levels = levs)       
    TW_axis.set_title('Tilt-wing')  
    TW_axis.set_xlabel('Range (mi)')    


    MR_axis        = fig.add_subplot(1,4,4)  
    MR_CS          = MR_axis.contourf(DAYS, RANGE ,MR_Charge_Throughout,lw=3, levels = levs)       
    MR_axis.set_title('Multi-rotor')  
    MR_axis.set_xlabel('Range (mi)')        

    format_axis(axis) 
    plt.tight_layout()  
    fig = plt.figure(save_filename_3)  


    # ---------------------------------------    
    # Plot 5 : C rate 
    # ---------------------------------------

    save_filename_5 = "C_Rate_1Yr"
    fig = plt.figure(save_filename_5)
    fig.set_size_inches(figure_width,figure_height)   
    axis = fig.add_subplot(1,1,1)       


    ECTOL_axis        = fig.add_subplot(1,4,1)  
    ECTOL_CS          = ECTOL_axis.contourf(DAYS, RANGE ,ECTOL_Max_C_Rate,lw=3, levels = levs)   
    ECTOL_axis.set_title('General Aviation')  
    ECTOL_axis.set_xlabel('Range (mi)')
    ECTOL_axis.set_ylabel('Time (days)')

    SR_axis        = fig.add_subplot(1,4,2)  
    SR_CS          = SR_axis.contourf(DAYS, RANGE ,SR_Max_C_Rate, lw=3,levels = levs)       
    SR_axis.set_title('Stopped-rotor')  
    SR_axis.set_xlabel('Range (mi)')   


    TW_axis        = fig.add_subplot(1,4,3)  
    TW_CS          = TW_axis.contourf(DAYS, RANGE ,TW_Max_C_Rate, lw=3,levels = levs)       
    TW_axis.set_title('Tilt-wing')  
    TW_axis.set_xlabel('Range (mi)')    


    MR_axis        = fig.add_subplot(1,4,4)  
    MR_CS          = MR_axis.contourf(DAYS, RANGE ,MR_Max_C_Rate,lw=3, levels = levs)       
    MR_axis.set_title('Multi-rotor')  
    MR_axis.set_xlabel('Range (mi)')    

    format_axis(axis)   
    plt.tight_layout() 
    fig = plt.figure(save_filename_4)   

    return

def format_axis(axis):  
    axis.minorticks_on()
    #axis.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
    #axis.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
    return axis 



 
if __name__ == '__main__': 
    main()    
    plt.show()