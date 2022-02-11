# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units
import matplotlib.pyplot as plt
import matplotlib.cm as cm 
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D  
import numpy as np 
import matplotlib 
import matplotlib.ticker as ticker 
import copy, time
import pickle
from SUAVE.Core import Data  
from scipy.interpolate import interp1d, interp2d, RectBivariateSpline 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 
    days = 365
    multiplier = 1
    #plot_battery_age(days,multiplier)
    
    plot_battery_age_vs_climb_rate(days,multiplier) 
    
    #plot_battery_age_vs_climate(days,multiplier)
    
    #plot_battery_age_vs_range(days,multiplier)

def plot_battery_age(days,multiplier):
   
    # Define rates of flight profile   
    Climb_Speeds            = np.array([700])    # Units['ft/min']
    Climb_Rates             = np.array([120])    # Units['mph']   
    Descent_Speeds          = np.array([300])    # Units['ft/min']
    Descent_Rates           = np.array([110])    # Units['mph'] 
    
    # ------------------------------------------------------------
    # LOAD DATA
    # ------------------------------------------------------------ 
    filename = 'full_mission_' + str(days) + '_days.pkl'
    #filename = 'full_mission_' + str(days) + '_days.pkl'
    # open with pickle 
    with open(filename, 'rb') as file:
        mission_res = pickle.load(file)  
    
    res = process_data(mission_res,multiplier,mission_res.num_cl_rates,mission_res.num_des_rates)     
    
    # ------------------------------------------------------------
    # 2D PLOT RESULTS
    # ------------------------------------------------------------   
    line_width       = 3  
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['axes.linewidth'] = 2.
    
    fig1 = plt.figure("Nominal_Mission_Ener_Res")
    fig1.set_size_inches(8,7) 
    axes = fig1.add_subplot(2,1,1)  
    axes.set_ylim([0.5*100,2.1*100]) 
    axes.set_xlim([0,days+5])  
    axes.set_ylabel('% $E_{Fade}$  and  $R_{Growth}$')  
    axes.plot(res.Mission_time[0,0,:]+1, res.E_Fade[0,0,:]*100   , color = 'black' , linestyle = '-', linewidth= line_width ,label = '$E_{Fade}$'  ) 
    axes.plot(res.Mission_time[0,0,:]+1, res.R_Growth[0,0,:]*100 , color = 'grey' , linestyle = '-', linewidth= line_width,label = '$R_{Growth}$' )  
    axes.legend(loc='upper center', prop={'size': 16}, ncol = 2)    
    
 
    axes2 = fig1.add_subplot(2,1,2)  
    axes2.set_ylim([10.,50])
    axes2.set_xlim([0,days+5]) 
    axes2.set_xlabel('Day of Year')  
    axes2.set_ylabel('Temperature ($\degree$C)')  
    axes2.plot(res.Mission_time[0,0,:]+1, res.Max_Temp[0,0,:] , color = 'grey' , linestyle = '-', linewidth= line_width ,label = 'Max. Battery Temp.'  ) 
    axes2.plot(res.Mission_time[0,0,:]+1, res.Amb_Temp[0,0,:] , color = 'black' , linestyle = '-', linewidth= line_width,label = 'Ambient Temp.' )  
    axes2.legend(loc='upper center', prop={'size': 16})         
      
    
    return     
  
    
def plot_battery_age_vs_climate(days,multiplier):
   
    # Plot of Daily Temperatures  
    # NY
    day_range = np.linspace(1,days,days)
    NY_daily_temp = 6 -0.191*day_range+ 4.59E-3*(day_range**2) - 2.02E-5*(day_range**3) + 2.48E-8*(day_range**4)
    
    # LA 
    LA_daily_temp = 18.2 - 9.7E-3*(day_range) + 2.41E-4*(day_range**2) -7.74E-6*(day_range**3) \
                + 1.38E-7*(day_range**4) - 1.01E-9*(day_range**5) + 3.67E-12*(day_range**6) \
                - 6.78E-15*(day_range**7) + 5.1E-18*(day_range**8)
    
    # HOU 
    HOU_daily_temp = 17.1 - 0.0435*(day_range) + 2.77E-3*(day_range**2) -2.2E-5*(day_range**3) \
                + 9.72E-8*(day_range**4) - 2.53E-10*(day_range**5) + 2.67E-13*(day_range**6)  
    
    # SF 
    SF_daily_temp = 13.5 + (day_range)*(-0.00882) + (day_range**2)*(0.00221) + \
                (day_range**3)*(-0.0000314) + (day_range**4)*(0.000000185)  +\
                (day_range**5)*(-0.000000000483)  + (day_range**6)*(4.57E-13)
    
    # CHI
    CHI_daily_temp  = -0.145 + (day_range)*(-0.11) + (day_range**2)*(4.57E-3) + \
                        (day_range**3)*(-2.71E-5) + (day_range**4)*(7.92E-8)  +\
                        (day_range**5)*(-1.66E-10)  + (day_range**6)*(1.76E-13)     
     
    
    # ------------------------------------------------------------
    # LOAD DATA
    # ------------------------------------------------------------  
    filename = 'climates_mission_' + str(days) + '_days.pkl'
    # open with pickle 
    with open(filename, 'rb') as file:
        mission_res = pickle.load(file)  
    res = process_data(mission_res,multiplier,4,1)       
                                                 
    #filename2 = 'chicago_mission_' + str(days) + '_days.pkl'
    ## open with pickle 
    #with open(filename2, 'rb') as file2:
        #mission_res2 = pickle.load(file2)  
    #res2 = process_data(mission_res2,multiplier,1,1)    
    
    # ------------------------------------------------------------
    # 2D PLOT RESULTS
    # ------------------------------------------------------------  
    ls  = ['darkorange','red','blue','darkgreen','darkorange']
    ls2 = ['darkorange','red','blue','darkgreen','darkorange']
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['axes.linewidth'] = 2.
    line_width       = 3  
    axis_tick_size   = 14      
       
    
    fig1 = plt.figure("Climate_Comparison_Cap_Res",constrained_layout=True)    
    gs = fig1.add_gridspec(10, 4)
    fig1.set_size_inches(10,8) 
    axes = fig1.add_subplot(gs[1:, :])
    
    axes.set_ylim([0.65*100,2*100]) 
    axes.set_xlim([0,days+5]) 
    axes.set_xlabel('Day of Year')  
    axes.set_ylabel('% $E_{Fade}$ and $R_{Growth}$')  
    axes.plot( res.Mission_time[0,0,:]+1, res.E_Fade[0,0,:]*100 , color = ls[0] , linestyle = '-', linewidth= line_width, label = 'NY $E_{Fade}$'  )
    axes.plot(res.Mission_time[0,0,:] +1, res.R_Growth[0,0,:]*100 , color = ls2[0] , linestyle = '--', linewidth= line_width, label = 'NY $R_{Rrowth}$' )
    axes.plot( res.Mission_time[1,0,:]+1, res.E_Fade[1,0,:]*100 , color = ls[1] , linestyle = '-', linewidth= line_width, label = 'LA $E_{Fade}$'  )
    axes.plot(res.Mission_time[1,0,:] +1, res.R_Growth[1,0,:]*100 , color = ls2[1] , linestyle = '--', linewidth= line_width, label = 'LA $R_{Rrowth}$' )
    axes.plot( res.Mission_time[2,0,:]+1, res.E_Fade[2,0,:]*100 , color = ls[2] , linestyle = '-', linewidth= line_width, label = 'HOU $E_{Fade}$'  )
    axes.plot(res.Mission_time[2,0,:] +1, res.R_Growth[2,0,:]*100 , color = ls2[2] , linestyle = '--', linewidth= line_width, label = 'HOU $R_{Rrowth}$' )
    axes.plot( res.Mission_time[3,0,:]+1, res.E_Fade[3,0,:]*100 , color = ls[3] , linestyle = '-', linewidth= line_width, label = 'SF $E_{Fade}$'  )
    axes.plot(res.Mission_time[3,0,:] +1, res.R_Growth[3,0,:]*100 , color = ls2[3] , linestyle = '--', linewidth= line_width, label = 'SF $R_{Rrowth}$' )
    #axes.plot( res.Mission_time[0,0,:]+1, res.E_Fade[0,0,:]*100 , color = ls[4] , linestyle = '-', linewidth= line_width, label = 'CHI $E_{Fade}$'  )  
    #axes.plot(res.Mission_time[0,0,:] +1, res.R_Growth[0,0,:]*100 , color = ls2[4] , linestyle = '--', linewidth= line_width, label = 'CHI $R_{Rrowth}$' )
    axes.legend( prop={'size': 18}, ncol = 3 , bbox_to_anchor=(0.5,1.04), loc="lower center", borderaxespad=0)    
    
    
    from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
    axins = zoomed_inset_axes(axes, 10, loc=7)
    axins.plot(res.Mission_time[0,0,:] +1, res.E_Fade[0,0,:]*100, color = ls[0] , linestyle = '-', linewidth= line_width)
    axins.plot(res.Mission_time[0,0,:] +1, res.E_Fade[1,0,:]*100, color = ls[1] , linestyle = '-', linewidth= line_width)
    axins.plot(res.Mission_time[0,0,:] +1, res.E_Fade[2,0,:]*100, color = ls[2] , linestyle = '-', linewidth= line_width)
    axins.plot(res.Mission_time[0,0,:] +1, res.E_Fade[3,0,:]*100, color = ls[3] , linestyle = '-', linewidth= line_width)
    axins.plot(res.Mission_time[0,0,:] +1, res.E_Fade[0,0,:]*100, color = ls[4] , linestyle = '-', linewidth= line_width)
    x1, x2, y1, y2 = 358, 366, 72, 78 # specify the limits
    axins.set_xlim(x1, x2) # apply the x-limits
    axins.set_ylim(y1, y2) # apply the y-limits    
    plt.yticks(visible=False)
    plt.xticks(visible=False)    
    from mpl_toolkits.axes_grid1.inset_locator import mark_inset
    mark_inset(axes, axins, loc1=3, loc2=4, fc="none", ec="0.5")    
    

    fig2 = plt.figure("Climate_Comparison_Temperature" ,constrained_layout=True)    
    gs = fig2.add_gridspec(10, 4)
    fig2.set_size_inches(10,8) 
    axes2 = fig2.add_subplot(gs[1:, :]) 
    axes2.set_ylim([-5.,50])  
    axes2.set_xlim([0,days+5]) 
    axes2.set_xlabel('Day of Year')  
    axes2.set_ylabel('Temperature ($\degree$C)')  
    axes2.plot( res.Mission_time[0,0,:]+1, res.Max_Temp[0,0,:] ,  color = ls[0] , linestyle = '--', linewidth= line_width, label = 'NY Max. Temp.'  )
    axes2.plot(day_range+1 ,  NY_daily_temp ,  color = ls2[0] , linestyle = '-', linewidth= line_width, label = 'NY Amb. Temp.' )
    axes2.plot( res.Mission_time[1,0,:]+1, res.Max_Temp[1,0,:] ,  color = ls[1] , linestyle = '--', linewidth= line_width, label = 'LA Max. Temp.'  )
    axes2.plot(day_range +1,  LA_daily_temp ,  color = ls2[1] , linestyle = '-', linewidth= line_width, label = 'LA Amb. Temp.' )
    axes2.plot( res.Mission_time[2,0,:]+1, res.Max_Temp[2,0,:] ,  color = ls[2] , linestyle = '--', linewidth= line_width, label = 'HOU Max. Temp.'  )
    axes2.plot(day_range+1 , HOU_daily_temp ,  color = ls2[2] , linestyle = '-', linewidth= line_width, label = 'HOU Amb. Temp.' )
    axes2.plot( res.Mission_time[3,0,:]+1, res.Max_Temp[3,0,:] ,  color = ls[3] , linestyle = '--', linewidth= line_width, label = 'SF Max. Temp.'  )
    axes2.plot(day_range+1,  SF_daily_temp ,  color = ls2[3] , linestyle = '-', linewidth= line_width, label = 'SF Amb. Temp.' )
    #axes2.plot(day_range+1, CHI_daily_temp ,  color = ls[4] , linestyle = '--', linewidth= line_width, label = 'CHI Max. Temp.'  )
    #axes2.plot(day_range+1, CHI_daily_temp ,  color = ls2[4] , linestyle = '-', linewidth= line_width, label = 'CHI Amb. Temp.' )
    axes2.legend( prop={'size': 18}, ncol = 3 , bbox_to_anchor=(0.5,1.04), loc="lower center", borderaxespad=0)   
    return  

def plot_battery_age_vs_range(days,multiplier): 
    
    # ------------------------------------------------------------
    # LOAD DATA
    # ------------------------------------------------------------  
    filename = 'range_comparison_mission_' + str(days) + '_days.pkl'
    # open with pickle 
    with open(filename, 'rb') as file:
        mission_res = pickle.load(file)  
    res = process_data(mission_res,multiplier,4,1)     
                                                 
    # ------------------------------------------------------------
    # 2D PLOT RESULTS
    # ------------------------------------------------------------  
    ls  = ['black','red','blue','darkgreen','darkorange']
    ls2 = ['black','red','blue','darkgreen','darkorange'] 
    line_width       = 3   
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['axes.linewidth'] = 2.
    
    fig1 = plt.figure("Range_Comparison",constrained_layout=True)
    gs = fig1.add_gridspec(10, 4)
    fig1.set_size_inches(9,8) 
    axes = fig1.add_subplot(gs[1:, :])
    axes.set_ylim([60,250]) 
    axes.set_xlim([0,days+5]) 
    axes.set_xlabel('Day of Year')  
    axes.set_ylabel('% $E_{Fade}$  and  $R_{Growth}$') 
    axes.plot(res.Mission_time[0,0,:] + 1,  res.E_Fade[0,0,:]*100  ,  color = ls[0] , linestyle = '-', linewidth= line_width,label = '40-mi $E_{Fade}$'  ) 
    axes.plot(res.Mission_time[0,0,:] + 1,  res.R_Growth[0,0,:]*100  ,  color = ls2[0] , linestyle = '--', linewidth= line_width,label = '40-mi $R_{Growth}$' )    
    axes.plot(res.Mission_time[1,0,:] + 1,  res.E_Fade[1,0,:]*100  ,  color = ls[1] , linestyle = '-', linewidth= line_width,label = '50-mi $E_{Fade}$'  )
    axes.plot(res.Mission_time[1,0,:] + 1,  res.R_Growth[1,0,:]*100  ,  color = ls2[1] , linestyle = '--', linewidth= line_width,label = '50-mi $R_{Growth}$' )
    axes.plot(res.Mission_time[2,0,:] + 1,  res.E_Fade[2,0,:]*100  ,  color = ls[2] , linestyle = '-', linewidth= line_width,label = '60-mi $E_{Fade}$'  )
    axes.plot(res.Mission_time[2,0,:] + 1,  res.R_Growth[2,0,:]*100  ,  color = ls2[2] , linestyle = '--', linewidth= line_width,label = '60-mi $R_{Growth}$' )
    axes.plot(res.Mission_time[3,0,:] + 1,  res.E_Fade[3,0,:]*100  ,  color = ls[3] , linestyle = '-', linewidth= line_width,label = '70-mi $E_{Fade}$'  ) 
    axes.plot(res.Mission_time[3,0,:] + 1,  res.R_Growth[3,0,:]*100  ,  color = ls2[3] , linestyle = '--', linewidth= line_width,label = '70-mi $R_{Growth}$' ) 
    axes.legend(prop={'size': 18},ncol=  3, bbox_to_anchor=(0.5,1.02), loc="lower center", borderaxespad=0)  
    
    
    
    E = np.array([ res.E_Fade[0,0,-1] , res.E_Fade[1,0,-1] ,  res.E_Fade[2,0,-1] ,  res.E_Fade[3,0,-1] ])
    R = np.array([ res.R_Growth[0,0,-1] , res.R_Growth[1,0,-1] ,  res.R_Growth[2,0,-1] ,  res.R_Growth[3,0,-1] ]) 
    
    E_vals = abs(1- E)*100
    R_vals = abs(1 - R)*100
    # plot results
    fig8 = plt.figure('Range_Barchart') 
    fig8.set_size_inches(10,8) 
    ax8 = fig8.add_subplot(1,1,1) 
    ax9 = ax8.twinx()
    bar_width = 0.35  
    r1 = np.arange(len(E))
    r2 = [x + bar_width for x in r1]      
    pop = ax8.bar(r1, E_vals , width=bar_width, color='k', align='center')  
    gdp = ax9.bar(r2, R_vals, width=bar_width,color='grey',align='center')   
    plt.xticks([r + bar_width*0.5 for r in range(len(E))], ['40','50','60', '70'])
    ax8.set_xlabel('Range (mi)')
    ax8.set_ylabel('% $E_{Fade}$ Reduction') 
    ax9.set_ylabel('% $R_{Growth}$ Increase')     
    plt.legend([pop, gdp],['$E_{Fade}$', '$R_{Growth}$'],loc='upper left')   
    
    return     

   
def plot_battery_age_vs_climb_rate(days,multiplier):
    
    # Define rates of flight profile  
    Climb_Rates_init        = np.array([450,550,650,750])  # Units['ft/min'] 
    Descent_Rates_init      = np.array([200,250,320,400])  # Units['ft/min'] 
    
    Climb_Rates             = np.linspace(450,750,25)               #  Units['ft/min'] 
    Descent_Rates           = np.linspace(200,400,25)              # Units['ft/min'] 
    # ------------------------------------------------------------
    # LOAD DATA
    # ------------------------------------------------------------   
    filename = 'const_rate_mission_' + str(days) + '_days.pkl'
    # open with pickle 
    with open(filename, 'rb') as file_1:
        mission_res = pickle.load(file_1) 
    
    res = process_data(mission_res,multiplier,mission_res.num_cl_rates,mission_res.num_des_rates)   
    
    
    Aging_time       = res.Aging_time
    Mission_time     = res.Mission_time 
    CF               = res.E_Fade 
    RG               = res.R_Growth 
    Charge           = res.Charge 
    ADT              = res.Avg_Temp  
    MDT              = res.Max_Temp 
    Max_Daily_C_rate = res.Max_C_rate  
    Avg_Daily_C_Rate = res.Avg_C_rate  
    FFS              = res.SOC  
      
    # ------------------------------------------------------------
    # INTERPOLATE DATA
    # ------------------------------------------------------------  
    MDT_surrogate  = interp2d(Climb_Rates_init, Descent_Rates_init, MDT[:,:,-1]) #,kind = 'linear')  
    Max_Daily_Temp = MDT_surrogate(Climb_Rates,Descent_Rates)    
    
    CF_surrogate  = interp2d(Climb_Rates_init, Descent_Rates_init, CF[:,:,-1]) #,kind = 'linear')  
    C_Fade        = CF_surrogate(Climb_Rates,Descent_Rates)    
    C_Fade        = (1 - C_Fade)*100/1
    
    RG_surrogate  = interp2d(Climb_Rates_init, Descent_Rates_init, RG[:,:,-1] ) #,kind = 'linear')  
    R_Growth      = RG_surrogate(Climb_Rates,Descent_Rates)     
    R_Growth      = (R_Growth-1)*100/1
    
    FFS_surrogate    = interp2d(Climb_Rates_init, Descent_Rates_init, FFS[:,:,0]) #,kind = 'linear')  
    Final_Flight_SOC = FFS_surrogate(Climb_Rates,Descent_Rates)  
    Final_Flight_DOD = (1-Final_Flight_SOC)*100 
    
    # ------------------------------------------------------------
    # CONTOUR PLOT RESULTS
    # ------------------------------------------------------------ 
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['axes.linewidth'] = 2. 
    axis_tick_size   = 14   
    

    
    fig3 = plt.figure("Battery_Capacity_Contour")
    fig4 = plt.figure("Battery_Resistance_Contour")
    fig5 = plt.figure("End_of_Mission_SOC")
    fig6 = plt.figure("Max_Daily_Temp")
    
    fig3.set_size_inches(8,6)   
    fig4.set_size_inches(8,6)
    fig5.set_size_inches(8,6)
    fig6.set_size_inches(8,6)
                           
    axes31 = fig3.add_subplot(1,1,1) 
    axes32 = fig4.add_subplot(1,1,1) 
    axes33 = fig5.add_subplot(1,1,1) 
    axes34 = fig6.add_subplot(1,1,1)        
    
    Climb_Rates_mesh , Descent_Rates_mesh = np.meshgrid(Climb_Rates , Descent_Rates) 
    CS_31 = axes31.contourf( Descent_Rates_mesh, Climb_Rates_mesh , C_Fade[:,:]   , 100 , cmap=cm.jet  )        
    CS_32 = axes32.contourf( Descent_Rates_mesh, Climb_Rates_mesh , R_Growth[:,:]      , 100 , cmap=cm.jet  ) 
    CS_33 = axes33.contourf( Descent_Rates_mesh, Climb_Rates_mesh , Final_Flight_DOD[:,:]   , 100 , cmap=cm.jet  )  
    CS_34 = axes34.contourf( Descent_Rates_mesh, Climb_Rates_mesh , Max_Daily_Temp[:,:]      , 100 , cmap=cm.jet  )  
    
    axes31.set_ylabel("Climb Rate (ft/min)"  )
    axes32.set_ylabel("Climb Rate (ft/min)"  )
    axes33.set_ylabel("Climb Rate (ft/min)"  )
    axes34.set_ylabel("Climb Rate (ft/min)"  )      
    axes31.set_xlabel("Descent Rate (ft/min)")
    axes32.set_xlabel("Descent Rate (ft/min)")
    axes33.set_xlabel("Descent Rate (ft/min)")
    axes34.set_xlabel("Descent Rate (ft/min)") 
    
    axes31.tick_params(axis = 'x', labelsize = axis_tick_size ) 
    axes32.tick_params(axis = 'x', labelsize = axis_tick_size ) 
    axes33.tick_params(axis = 'x', labelsize = axis_tick_size ) 
    axes34.tick_params(axis = 'x', labelsize = axis_tick_size ) 
    axes31.tick_params(axis = 'y', labelsize = axis_tick_size ) 
    axes32.tick_params(axis = 'y', labelsize = axis_tick_size ) 
    axes33.tick_params(axis = 'y', labelsize = axis_tick_size ) 
    axes34.tick_params(axis = 'y', labelsize = axis_tick_size )    
    
    #cbar_num_format = "%.3f"
    sfmt = ticker.ScalarFormatter(useMathText=True) 
    sfmt = ticker.FormatStrFormatter('%.2f') 
    cbar31 = fig3.colorbar(CS_31, ax = axes31, format= sfmt )    
    cbar32 = fig3.colorbar(CS_32, ax = axes32, format= sfmt)
    cbar33 = fig3.colorbar(CS_33, ax = axes33, format= sfmt)
    cbar34 = fig3.colorbar(CS_34, ax = axes34, format= sfmt)
    
    ticklabs31 = cbar31.ax.get_yticklabels()
    ticklabs32 = cbar32.ax.get_yticklabels()
    ticklabs33 = cbar33.ax.get_yticklabels()
    ticklabs34 = cbar34.ax.get_yticklabels()
    
    cbar31.ax.set_yticklabels(ticklabs31, fontsize=axis_tick_size)
    cbar32.ax.set_yticklabels(ticklabs32, fontsize=axis_tick_size)
    cbar33.ax.set_yticklabels(ticklabs33, fontsize=axis_tick_size)
    cbar34.ax.set_yticklabels(ticklabs34, fontsize=axis_tick_size)
    
    cbar31.set_label('% $E_{Fade}$ Reduction')
    cbar32.set_label('% $R_{Growth}$ Increase')
    cbar33.set_label('DOD')
    cbar34.set_label('Max Battery Temp. $\degree$C')   
     
    return

def process_data(mis_res,multiplier, cl, des):
    
    res = Data 
    
    count = int(mis_res.num_days/multiplier)
    res_Aging_time   = np.zeros((cl,des,count ))
    res_Mission_time = np.zeros_like(res_Aging_time)
    res_E_Fade       = np.zeros_like(res_Aging_time)
    res_Charge       = np.zeros_like(res_Aging_time)
    res_R_Growth     = np.zeros_like(res_Aging_time)
    res_Max_Temp     = np.zeros_like(res_Aging_time)
    res_Avg_Temp     = np.zeros_like(res_Aging_time)
    res_Amb_Temp     = np.zeros_like(res_Aging_time)
    res_Max_C_rate   = np.zeros_like(res_Aging_time)
    res_Avg_C_rate   = np.zeros_like(res_Aging_time)
    res_SOC          = np.zeros_like(res_Aging_time) 
     
    
    
    for i in range( cl):
        for j in range(des):
            for k in range(count): 
                idx = k*multiplier
                res_Aging_time[i,j,k]   = idx + 1 
                res_Mission_time[i,j,k] = mis_res.Mission_time[i,j,idx,-1]      
                res_E_Fade[i,j,k]       = mis_res.E_Fade[i,j,idx,-1]          
                res_Charge[i,j,k]       = mis_res.Charge[i,j,idx,-1]         
                res_R_Growth[i,j,k]     = mis_res.R_Growth[i,j,idx,-1]       
                res_Max_Temp[i,j,k]     = max(mis_res.Max_Temp[i,j,idx,:-1])   
                res_Avg_Temp[i,j,k]     = np.mean(mis_res.Avg_Temp[i,j,idx,:-1]) 
                res_Amb_Temp[i,j,k]     = np.mean(mis_res.Amb_Temp[i,j,idx,:-1])       
                res_Max_C_rate[i,j,k]   = max(mis_res.Max_C_rate[i,j,idx,:-1]) 
                res_Avg_C_rate[i,j,k]   = np.mean(mis_res.Avg_C_rate[i,j,idx,:])     
                res_SOC[i,j,k]          = mis_res.SOC[i,j,idx,-2] # SOC of last segment 
    
    res.Aging_time   = res_Aging_time
    res.Mission_time = res_Mission_time
    res.E_Fade       = res_E_Fade
    res.Charge       = res_Charge
    res.R_Growth     = res_R_Growth
    res.Max_Temp     = res_Max_Temp
    res.Avg_Temp     = res_Avg_Temp
    res.Amb_Temp     = res_Amb_Temp
    res.Max_C_rate   = res_Max_C_rate
    res.Avg_C_rate   = res_Avg_C_rate
    res.SOC          = res_SOC    
    
    
    
    return res 
if __name__ == '__main__': 
    main()    
    plt.show()