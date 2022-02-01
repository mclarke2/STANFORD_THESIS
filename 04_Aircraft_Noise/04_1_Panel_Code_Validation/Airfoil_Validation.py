# airfoil_panel_method_test.py
# 
# Created:  
# Modified: Mar 2021, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis      import airfoil_analysis  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import  compute_naca_4series
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry\
     import import_airfoil_geometry   
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_polars \
     import import_airfoil_polars  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars \
     import compute_airfoil_polars   
from SUAVE.Plots.Geometry import plot_propeller 
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
import matplotlib.pyplot as plt  
import matplotlib.cm as cm
import os 
import numpy as np
import time 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():    
    #airfoil_test() 
    propeller_bl_test()
    
    return 

def airfoil_test():
    
    # Define Panelization 

    npanel   = 400  
    # Batch analysis of single airfoil - NACA 4412 
    Re_batch             = np.atleast_2d(np.array([1E4, 1E5, 1E6])).T
    AoA_batch            = np.atleast_2d(np.array([0.,4.,8.])*Units.degrees).T       
    airfoil_geometry     = compute_naca_4series(0.04,0.4,0.12,npoints=npanel)
    SUAVE_data = airfoil_analysis(airfoil_geometry,AoA_batch,Re_batch, npanel)   
    
    # X foil
    Re_tags  = ['1E4','1E5','1E6']
    AoA_tags = ['0','4','8']
    xfoil_npanel   = 300 
    Xfoil_data = import_NACA_4412_xfoil_results(Re_tags,AoA_tags,xfoil_npanel)  
     
    # Plot trailing edge properties  
    plot_trailing_edge_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags)   
    
    plot_full_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags) 
     
    return 
 
def plot_trailing_edge_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags):   
    
    # determine dimension of angle of attack and reynolds number 
    AoA_range = np.array([0,4,8]) 
    Te_idx = 4 
    
    # create array of colors for difference reynolds numbers 
    colors  =['blue','darkblue']
    colors2 =['red','darkred']
    markers = ['o','v','s','P','p','^','D','X','*']
    
    fig1  = plt.figure('Airfoil_Polars') 
    fig1.set_size_inches(10, 8) 
    axis11 = fig1.add_subplot(3,3,1)     
    axis12 = fig1.add_subplot(3,3,2)     
    axis13 = fig1.add_subplot(3,3,3)     
    axis14 = fig1.add_subplot(3,3,4)     
    axis15 = fig1.add_subplot(3,3,5)     
    axis16 = fig1.add_subplot(3,3,6)     
    axis17 = fig1.add_subplot(3,3,7)     
    axis18 = fig1.add_subplot(3,3,8)   
    axis19 = fig1.add_subplot(3,3,9)  
    axis11.set_title('Re = 1E4')
    axis12.set_title('Re = 1E5')
    axis13.set_title('Re = 1E6')    
    axis17.set_xlabel('AoA')
    axis18.set_xlabel('AoA')
    axis19.set_xlabel('AoA')
    axis11.set_ylabel('Cl')
    axis14.set_ylabel('Cd')
    axis17.set_ylabel('Cm')  
    

    fig2  = plt.figure('Airfoil_BL_TE_1') 
    fig2.set_size_inches(10, 8) 
    axis21 = fig2.add_subplot(3,3,1)     
    axis22 = fig2.add_subplot(3,3,2)     
    axis23 = fig2.add_subplot(3,3,3)     
    axis24 = fig2.add_subplot(3,3,4)     
    axis25 = fig2.add_subplot(3,3,5)     
    axis26 = fig2.add_subplot(3,3,6)     
    axis27 = fig2.add_subplot(3,3,7)     
    axis28 = fig2.add_subplot(3,3,8)   
    axis29 = fig2.add_subplot(3,3,9) 
    axis21.set_title('Re = 1E4')
    axis22.set_title('Re = 1E5')
    axis23.set_title('Re = 1E6') 
    axis27.set_xlabel('AoA')
    axis28.set_xlabel('AoA')
    axis29.set_xlabel('AoA')
    axis21.set_ylabel(r'$Ue/V_{inf}$')
    axis24.set_ylabel(r'$\theta$')
    axis27.set_ylabel(r'$\delta$ *')   
    

    fig3  = plt.figure('Airfoil_BL_TE_2') 
    fig3.set_size_inches(10, 8) 
    axis31 = fig3.add_subplot(3,3,1)     
    axis32 = fig3.add_subplot(3,3,2)     
    axis33 = fig3.add_subplot(3,3,3)     
    axis34 = fig3.add_subplot(3,3,4)     
    axis35 = fig3.add_subplot(3,3,5)     
    axis36 = fig3.add_subplot(3,3,6)     
    axis37 = fig3.add_subplot(3,3,7)     
    axis38 = fig3.add_subplot(3,3,8)   
    axis39 = fig3.add_subplot(3,3,9) 
    axis31.set_title('Re = 1E4')
    axis32.set_title('Re = 1E5')
    axis33.set_title('Re = 1E6') 
    axis37.set_xlabel('AoA')
    axis38.set_xlabel('AoA')
    axis39.set_xlabel('AoA')
    axis31.set_ylabel(r'Cf')
    axis34.set_ylabel(r'H')
    axis37.set_ylabel(r'Cp')   
     
    
    # Cl
    axis11.plot(AoA_range ,  SUAVE_data.Cl[:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0] , label = 'SUAVE'  )  
    axis11.plot(AoA_range ,  Xfoil_data.Cl[:,0] ,color = colors2[0], linestyle = '--' ,marker =   markers[1] , label = 'Xfoil')   
    axis12.plot(AoA_range,   SUAVE_data.Cl[:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis12.plot(AoA_range,   Xfoil_data.Cl[:,1]  ,color = colors2[0], linestyle = '--' ,marker =   markers[1])    
    axis13.plot(AoA_range,   SUAVE_data.Cl[:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis13.plot(AoA_range,   Xfoil_data.Cl[:,2] ,color = colors2[0], linestyle = '--' ,marker =   markers[1])              
     
    # CdAoA_range
    axis14.plot(AoA_range ,  SUAVE_data.Cd[:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis14.plot(AoA_range ,  Xfoil_data.Cd[:,0] ,color = colors2[0], linestyle = '--' ,marker =   markers[1]  )  
    axis15.plot(AoA_range,   SUAVE_data.Cd[:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis15.plot(AoA_range,   Xfoil_data.Cd[:,1]  ,color = colors2[0], linestyle = '--' ,marker =   markers[1])     
    axis16.plot(AoA_range,   SUAVE_data.Cd[:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis16.plot(AoA_range,   Xfoil_data.Cd[:,2] ,color = colors2[0], linestyle = '--' ,marker =   markers[1])             
  
    # CmAoA_range
    axis17.plot(AoA_range , SUAVE_data.Cm[:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0]  )  
    axis17.plot(AoA_range , Xfoil_data.Cm[:,0] ,color = colors2[0], linestyle = '--' ,marker =   markers[1]  )   
    axis18.plot(AoA_range,  SUAVE_data.Cm[:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis18.plot(AoA_range,  Xfoil_data.Cm[:,1]  ,color = colors2[0], linestyle = '--' ,marker =   markers[1])    
    axis19.plot(AoA_range,  SUAVE_data.Cm[:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis19.plot(AoA_range,  Xfoil_data.Cm[:,2] ,color = colors2[0], linestyle = '--' ,marker =   markers[1])   
     
    # Ue/V_inf
    axis21.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.Ue_Vinf[Te_idx ,:,0]  ,color = colors[0] , linestyle = '-' ,marker =    markers[0]  , label = 'SUAVE Lower Surf'  )  
    axis21.plot(Xfoil_data.x[Te_idx ,:,0] ,  -Xfoil_data.Ue_Vinf[Te_idx ,:,0] ,color = colors2[0], linestyle = '--' ,marker =   markers[1] , label = 'Xfoil Lower Surf')  
    axis21.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.Ue_Vinf[-Te_idx,:,0]  ,color = colors[1] , linestyle = '-' ,marker =  markers[0]  , label = 'SUAVE Upper Surf'  )     
    axis21.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.Ue_Vinf[-Te_idx,:,0]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1] , label = 'Xfoil Upper Surf' )        
    axis22.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.Ue_Vinf[Te_idx ,:,1]  ,color = colors[0]  , linestyle = '-' ,marker =    markers[0]   )  
    axis22.plot(Xfoil_data.x[Te_idx ,:,1],   -Xfoil_data.Ue_Vinf[Te_idx ,:,1] ,color = colors2[0] , linestyle = '--' ,marker =   markers[1]) 
    axis22.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.Ue_Vinf[-Te_idx,:,1]  ,color = colors[1]  , linestyle = '-' ,marker =  markers[0]  )  
    axis22.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.Ue_Vinf[-Te_idx,:,1]  ,color = colors2[1] , linestyle = '--' ,marker =  markers[1])   
    axis23.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.Ue_Vinf[Te_idx ,:,2]  ,color = colors[0]  , linestyle = '-' ,marker =    markers[0]   )  
    axis23.plot(Xfoil_data.x[Te_idx ,:,2],   -Xfoil_data.Ue_Vinf[Te_idx ,:,2] ,color = colors2[0] , linestyle = '--' ,marker =   markers[1])         
    axis23.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.Ue_Vinf[-Te_idx,:,2]  ,color = colors[1]  , linestyle = '-' ,marker =  markers[0]  )          
    axis23.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.Ue_Vinf[-Te_idx,:,2]  ,color = colors2[1] , linestyle = '--' ,marker =  markers[1])   
    
    # theta
    axis24.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.theta[Te_idx ,:,0],color = colors[0] , linestyle = '-' ,marker =    markers[0]    )  
    axis24.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.theta[Te_idx ,:,0],color = colors2[0], linestyle = '--' ,marker =   markers[1] ) 
    axis24.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.theta[-Te_idx,:,0],color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  )  
    axis24.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.theta[-Te_idx,:,0],color = colors2[1],  linestyle = '--' ,marker =  markers[1])  
    axis25.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.theta[Te_idx ,:,1],color = colors[0] ,   linestyle = '-' ,marker =    markers[0]   )  
    axis25.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.theta[Te_idx ,:,1],color = colors2[0],  linestyle = '--' ,marker =   markers[1]) 
    axis25.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.theta[-Te_idx,:,1],color = colors[1] ,   linestyle = '-' ,marker =  markers[0]  )  
    axis25.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.theta[-Te_idx,:,1],color = colors2[1],   linestyle = '--' ,marker =  markers[1])     
    axis26.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.theta[Te_idx ,:,2],color = colors[0] , linestyle = '-' ,marker =    markers[0]   )  
    axis26.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.theta[Te_idx ,:,2],color = colors2[0], linestyle = '--' ,marker =   markers[1])         
    axis26.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.theta[-Te_idx,:,2],color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  )          
    axis26.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.theta[-Te_idx,:,2],color = colors2[1],  linestyle = '--' ,marker =  markers[1])            
    
    # Delta Star 
    axis27.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.delta_star[Te_idx ,:,0],color = colors[0] , linestyle = '-' ,marker =    markers[0]   )  
    axis27.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.delta_star[Te_idx ,:,0],color = colors2[0], linestyle = '--' ,marker =   markers[1]  ) 
    axis27.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.delta_star[-Te_idx,:,0],color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  )  
    axis27.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.delta_star[-Te_idx,:,0],color = colors2[1],  linestyle = '--' ,marker =  markers[1])    
    axis28.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.delta_star[Te_idx ,:,1],color = colors[0] ,   linestyle = '-' ,marker =    markers[0]   )  
    axis28.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.delta_star[Te_idx ,:,1],color = colors2[0],  linestyle = '--' ,marker =   markers[1]) 
    axis28.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.delta_star[-Te_idx,:,1],color = colors[1] ,   linestyle = '-' ,marker =  markers[0]  )  
    axis28.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.delta_star[-Te_idx,:,1],color = colors2[1],  linestyle = '--' ,marker =  markers[1])   
    axis29.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.delta_star[Te_idx ,:,2],color = colors[0] , linestyle = '-' ,marker =    markers[0]   )  
    axis29.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.delta_star[Te_idx ,:,2],color = colors2[0], linestyle = '--' ,marker =   markers[1])         
    axis29.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.delta_star[-Te_idx,:,2],color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  )          
    axis29.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.delta_star[-Te_idx,:,2],color = colors2[1],  linestyle = '--' ,marker =  markers[1])   
    

    # Cf
    axis31.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.Cf[Te_idx ,:,2],color = colors[0] , linestyle = '-' ,marker =    markers[0]  , label = 'SUAVE Lower Surf'  )  
    axis31.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.Cf[Te_idx ,:,2],color = colors2[0], linestyle = '--' ,marker =   markers[1] , label = 'Xfoil Lower Surf') 
    axis31.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.Cf[-Te_idx,:,2],color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  , label = 'SUAVE Upper Surf'  )  
    axis31.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.Cf[-Te_idx,:,2],color = colors2[1],  linestyle = '--' ,marker =  markers[1] , label = 'Xfoil Upper Surf' )   
    axis32.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.Cf[Te_idx ,:,2],color = colors[0] ,   linestyle = '-' ,marker =    markers[0]   )  
    axis32.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.Cf[Te_idx ,:,2],color = colors2[0],  linestyle = '--' ,marker =   markers[1]) 
    axis32.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.Cf[-Te_idx,:,2],color = colors[1] ,   linestyle = '-' ,marker =  markers[0]  )  
    axis32.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.Cf[-Te_idx,:,2],color = colors2[1],   linestyle = '--' ,marker =  markers[1])  
    axis33.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.Cf[Te_idx ,:,2],color = colors[0] , linestyle = '-' ,marker =    markers[0]   )  
    axis33.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.Cf[Te_idx ,:,2],color = colors2[0], linestyle = '--' ,marker =   markers[1])         
    axis33.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.Cf[-Te_idx,:,2],color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  )          
    axis33.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.Cf[-Te_idx,:,2],color = colors2[1],  linestyle = '--' ,marker =  markers[1])            
     
    # H
    axis34.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.H[Te_idx ,:,2],color = colors[0] ,linestyle = '-' ,marker =    markers[0])  
    axis34.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.H[Te_idx ,:,2],color = colors2[0],linestyle = '--' ,marker =   markers[1] ) 
    axis34.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.H[-Te_idx,:,2],color = colors[1] , linestyle = '-' ,marker =  markers[0]  )  
    axis34.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.H[-Te_idx,:,2],color = colors2[1], linestyle = '--' ,marker =  markers[1])    
    axis35.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.H[Te_idx ,:,2],color = colors[0] , linestyle = '-' ,marker =    markers[0]   )  
    axis35.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.H[Te_idx ,:,2],color = colors2[0], linestyle = '--' ,marker =   markers[1]) 
    axis35.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.H[-Te_idx,:,2],color = colors[1] , linestyle = '-' ,marker =  markers[0]  )  
    axis35.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.H[-Te_idx,:,2],color = colors2[1], linestyle = '--' ,marker =  markers[1])    
    axis36.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.H[Te_idx ,:,2],color = colors[0] ,linestyle = '-' ,marker =    markers[0]   )  
    axis36.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.H[Te_idx ,:,2],color = colors2[0],linestyle = '--' ,marker =   markers[1])         
    axis36.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.H[-Te_idx,:,2],color = colors[1] , linestyle = '-' ,marker =  markers[0]  )          
    axis36.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.H[-Te_idx,:,2],color = colors2[1], linestyle = '--' ,marker =  markers[1])            
    
    # Cp
    axis37.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.Cp[Te_idx ,:,2] ,color = colors[0] ,linestyle = '-' ,marker =    markers[0] )  
    axis37.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.Cp[Te_idx ,:,2] ,color = colors2[0],linestyle = '--' ,marker =   markers[1]) 
    axis37.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.Cp[-Te_idx,:,2] ,color = colors[1] , linestyle = '-' ,marker =  markers[0]  )  
    axis37.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.Cp[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
    axis38.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.Cp[Te_idx ,:,2] ,color = colors[0] ,  linestyle = '-' ,marker =    markers[0]   )  
    axis38.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.Cp[Te_idx ,:,2] ,color = colors2[0], linestyle = '--' ,marker =   markers[1]) 
    axis38.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.Cp[-Te_idx,:,2] ,color = colors[1] ,  linestyle = '-' ,marker =  markers[0]  )  
    axis38.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.Cp[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
    axis39.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.Cp[Te_idx ,:,2] ,color = colors[0] ,linestyle = '-' ,marker =    markers[0]   )  
    axis39.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.Cp[Te_idx ,:,2] ,color = colors2[0],linestyle = '--' ,marker =   markers[1])         
    axis39.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.Cp[-Te_idx,:,2] ,color = colors[1] , linestyle = '-' ,marker =  markers[0]  )          
    axis39.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.Cp[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
        
    # add legends for plotting
    plt.tight_layout()
    
    axis11.legend(loc='upper right')     
    axis21.legend(loc='upper right') 
    axis31.legend(loc='upper right') 
     
     
     
     
     
     
     
    return  


 
def plot_full_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags):   
    
    # determine dimension of angle of attack and reynolds number 
    nAoA = len(SUAVE_data.AoA)
    nRe  = len(SUAVE_data.Re)
    
    # create array of colors for difference reynolds numbers 
    colors  =['blue','darkblue']
    colors2 =['red','darkred']
    markers = ['o','v','s','P','p','^','D','X','*']
    
    fig1  = plt.figure('Ue_Vinf') 
    fig1.set_size_inches(10, 8)   
    fig2  = plt.figure('theta') 
    fig2.set_size_inches(10, 8) 
    fig3  = plt.figure('Delta Star') 
    fig3.set_size_inches(10, 8) 
    fig4  = plt.figure('Cf') 
    fig4.set_size_inches(10, 8) 
    fig5  = plt.figure('H') 
    fig5.set_size_inches(10, 8) 
    fig6  = plt.figure('Cp') 
    fig6.set_size_inches(10, 8) 
    
     
    mid = int(len(SUAVE_data.x[:,0,0])/2)
    mid2 = int(len(Xfoil_data.x[:,0,0])/2)
      
    idx  = 0
    for i in range(nAoA):
        for j in range(nRe):
            idx += 1
            axis1 = fig1.add_subplot(3,3,idx)  
            axis2 = fig2.add_subplot(3,3,idx)  
            axis3 = fig3.add_subplot(3,3,idx)  
            axis4 = fig4.add_subplot(3,3,idx)  
            axis5 = fig5.add_subplot(3,3,idx)  
            axis6 = fig6.add_subplot(3,3,idx)  
            
            Common_Title = ' AoA: ' +  AoA_tags[i] + '$/degree$, Re: ' + Re_tags[j] 
            axis1.set_title('$Ue/V_{inf}$' +  Common_Title )
            axis2.set_title('\theta' +  Common_Title  )
            axis3.set_title('$\delta$*' +  Common_Title  )
            axis4.set_title('$Cf$' +  Common_Title  )
            axis5.set_title('H' +  Common_Title  )
            axis6.set_title( '$C_p$' +  Common_Title ) 
             
            axis3.set_ylim(-0.005, 0.025) 
            axis4.set_ylim(0, 0.2)     
            axis6.set_ylim(1.2,-7)  
            axis1.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.Ue_Vinf[:mid ,i,j],color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'SUAVE Lower Surf')   
            axis1.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.Ue_Vinf[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]    , label = 'SUAVE Upper Surf') 
            axis1.plot(Xfoil_data.x[:mid2,i,j],   -Xfoil_data.Ue_Vinf[:mid2,i,j],color = colors2[0], linestyle = '-' ,marker =  markers[0]     , label = 'Xfoil Lower Surf')  
            axis1.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.Ue_Vinf[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1]    , label = 'Xfoil Upper Surf')               
           
            axis2.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.theta[:mid ,i,j],color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'SUAVE Lower Surf')   
            axis2.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.theta[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]    , label = 'SUAVE Upper Surf') 
            axis2.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.theta[:mid2,i,j],color = colors2[0], linestyle = '-' ,marker =  markers[0]     , label = 'Xfoil Lower Surf')  
            axis2.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.theta[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1]     , label = 'Xfoil Upper Surf')        
            
            axis3.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.delta_star[:mid ,i,j],color = colors[0], linestyle = '-' ,marker =    markers[0]  , label = 'SUAVE Lower Surf')    
            axis3.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.delta_star[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]  , label = 'SUAVE Upper Surf') 
            axis3.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.delta_star[:mid2,i,j],color = colors2[0], linestyle = '-' ,marker =  markers[0]   , label = 'Xfoil Lower Surf')   
            axis3.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.delta_star[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1]   , label = 'Xfoil Upper Surf')      
            

            axis4.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.Cf[:mid ,i,j],color = colors[0], linestyle = '-' ,marker =    markers[0]   , label = 'SUAVE Lower Surf')   
            axis4.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.Cf[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]   , label = 'SUAVE Upper Surf') 
            axis4.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.Cf[:mid2,i,j],color = colors2[0], linestyle = '-' ,marker =  markers[0]    , label = 'Xfoil Lower Surf')  
            axis4.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.Cf[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1]    , label = 'Xfoil Upper Surf')                
           
            axis5.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.H[:mid ,i,j],color = colors[0], linestyle = '-' ,marker =    markers[0]  , label = 'SUAVE Lower Surf')       
            axis5.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.H[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]  , label = 'SUAVE Upper Surf')  
            axis5.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.H[:mid2,i,j],color = colors2[0], linestyle = '-' ,marker =  markers[0]   , label = 'Xfoil Lower Surf')   
            axis5.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.H[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1]   , label = 'Xfoil Upper Surf')       
            
            axis6.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.Cp[:mid ,i,j],color = colors[0], linestyle = '-' ,marker =    markers[0]   , label = 'SUAVE Lower Surf')      
            axis6.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.Cp[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]   , label = 'SUAVE Upper Surf') 
            axis6.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.Cp[:mid2,i,j],color = colors2[0], linestyle = '-' ,marker =  markers[0]    , label = 'Xfoil Lower Surf')   
            axis6.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.Cp[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1]    , label = 'Xfoil Upper Surf')         
                
            if (i == 0) and (j == 0):  
                axis1.legend(loc='upper right') 
                axis2.legend(loc='upper right')
                axis3.legend(loc='upper right')
                axis4.legend(loc='upper right')
                axis5.legend(loc='upper right')
                axis6.legend(loc='upper right')
               
    return    


def propeller_bl_test():

    ti = time.time()     
    net                       = Battery_Propeller()   
    net.number_of_propeller_engines     = 2    
    
    # Design Gearbox 
    gearbox                   = SUAVE.Components.Energy.Converters.Gearbox()
    gearbox.gearwheel_radius1 = 1
    gearbox.gearwheel_radius2 = 1
    gearbox.efficiency        = 0.95
    gearbox.inputs.torque     = 885.550158704757
    gearbox.inputs.speed      = 207.16160479940007
    gearbox.inputs.power      = 183451.9920076409
    gearbox.compute() 
    
    propeller                          = SUAVE.Components.Energy.Converters.Propeller()  
    propeller.number_of_blades         = 3
    propeller.number_of_engines        = 1
    propeller.freestream_velocity      = 49.1744 
    propeller.tip_radius               = 1.0668
    propeller.hub_radius               = 0.21336 
    propeller.design_tip_mach          = 0.65
    propeller.angular_velocity         = gearbox.inputs.speed # 207.16160479940007 
    propeller.design_Cl                = 0.7
    propeller.design_altitude          = 1. * Units.km      
    propeller.airfoil_geometry         = ['../../XX_Suplementary/Aircraft_Models/Airfoils/NACA_4412.txt',
                                          '../../XX_Suplementary/Aircraft_Models/Airfoils/Clark_y.txt']

    propeller.airfoil_polars           = [['../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt'],
                                       ['../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/Clark_y_polar_Re_50000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/Clark_y_polar_Re_100000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/Clark_y_polar_Re_200000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/Clark_y_polar_Re_500000.txt',
                                        '../../XX_Suplementary/Aircraft_Models/Airfoils/Polars/Clark_y_polar_Re_1000000.txt']] 

    #propeller.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1]  
    propeller.airfoil_polar_stations  = [0,0,0,0,0,0,1,1,1,1]  
    propeller.design_thrust           = 3054.4809132125697
    propeller                         = propeller_design(propeller,number_of_stations=10,surrogate_type= 'svr')   
    
    # Find the operating conditions
    atmosphere            = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions =  atmosphere.compute_values(propeller.design_altitude)
    
    V  = propeller.freestream_velocity 
    
    conditions                                          = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
    conditions._size                                    = 1
    conditions.freestream                               = Data()
    conditions.propulsion                               = Data()
    conditions.frames                                   = Data()
    conditions.frames.body                              = Data()
    conditions.frames.inertial                          = Data()
    conditions.freestream.update(atmosphere_conditions)
    conditions.freestream.dynamic_viscosity             = atmosphere_conditions.dynamic_viscosity
    conditions.frames.inertial.velocity_vector          = np.array([[V,0,0]])
    conditions.propulsion.throttle                      = np.array([[1.0]])
    conditions.frames.body.transform_to_inertial        = np.array([np.eye(3)]) 
    conditions.frames.inertial.velocity_vector   = np.array([[V,0,0]]) 
    
    # Create and attach this propeller 
    propeller.inputs.omega  = np.array(propeller.angular_velocity,ndmin=2) 
    
    # propeller with airfoil results 
    propeller.inputs.pitch_command                = 0.0*Units.degree
    F, Q, P, Cplast,outputs , eta = propeller.spin(conditions)  
     
    rho                = conditions.freestream.density                 
    dyna_visc          = conditions.freestream.dynamic_viscosity            
    alpha_blade        = outputs.disc_effective_angle_of_attack 
    Vt_2d              = outputs.disc_tangential_velocity  
    Va_2d              = outputs.disc_axial_velocity                
    blade_chords       = propeller.chord_distribution         
    r                  = propeller.radius_distribution      
    num_sec            = len(r) 
    num_azi            = len(outputs.disc_effective_angle_of_attack[0,0,:])   
    U_blade            = np.sqrt(Vt_2d**2 + Va_2d **2)
    Re_blade           = U_blade*np.repeat(np.repeat(blade_chords[np.newaxis,:],1,axis=0)[:,:,np.newaxis],num_azi,axis=2)*\
                          np.repeat(np.repeat((rho/dyna_visc),num_sec,axis=1)[:,:,np.newaxis],num_azi,axis=2)  
    

    # ------------------------------------------------------------
    # ****** TRAILING EDGE BOUNDARY LAYER PROPERTY CALCULATIONS  ******    
    bl_results = evaluate_boundary_layer_surrogates( propeller,alpha_blade,Re_blade)
    

    tf = time.time()
    print ('Time taken: ' + str(round((tf-ti)/60,3))  + ' min')
    
    plot_surrogate_validation(propeller,bl_results) 

    return 
  
def plot_surrogate_validation(propeller,bl_results):
    
    af_sur     = propeller.airfoil_bl_surrogates
    # determine dimension of angle of attack and reynolds number  
    Re         = np.array([1E2,1E3,1E4,1E5,1E6,1E7])
    AoA        = np.linspace(-4,14,10)   
    a_loc      = propeller.airfoil_polar_stations 
    
    # create array of colors for difference reynolds numbers 
    colors  =['blue','darkblue']
    colors2 =['red','darkred']
    markers = ['o','v','s','P','p','^','D','X','*']
    
    fig1  = plt.figure('Prop_Ue_Vinf') 
    fig1.set_size_inches(10, 8)   
    fig2  = plt.figure('Prop_theta') 
    fig2.set_size_inches(10, 8) 
    fig3  = plt.figure('Prop_Delta Star') 
    fig3.set_size_inches(10, 8) 
    fig4  = plt.figure('Prop_Cf') 
    fig4.set_size_inches(10, 8) 
    fig5  = plt.figure('Prop_H') 
    fig5.set_size_inches(10, 8) 
    fig6  = plt.figure('Prop_Cp') 
    fig6.set_size_inches(10, 8) 
    
    idx = 0 
    blade_section_idx = 3 
    for Re_i in range(len(Re)): 
        idx += 1
        axis_1 = fig1.add_subplot(2,3,idx)  
        axis_2 = fig2.add_subplot(2,3,idx)  
        axis_3 = fig3.add_subplot(2,3,idx)  
        axis_4 = fig4.add_subplot(2,3,idx)  
        axis_5 = fig5.add_subplot(2,3,idx)  
        axis_6 = fig6.add_subplot(2,3,idx)  
        
        Common_Title = 'Re: ' + str(Re[Re_i]) 
        axis_5.set_title('$Ue/V_{inf}$ ' +  Common_Title )
        axis_1.set_title('Theta ' +  Common_Title  ) 
        axis_3.set_title('$\delta$* ' +  Common_Title  )
        axis_4.set_title('$Cf$ ' +  Common_Title  )
        axis_5.set_title('H ' +  Common_Title  )
        axis_2.set_title('$C_p$ ' +  Common_Title ) 
          
        axis_1.plot(AoA,af_sur.lower_surface_theta_vals[a_loc[blade_section_idx],:,Re_i]       ,color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Lower Surf')       
        axis_1.plot(AoA,af_sur.upper_surface_theta_vals[a_loc[blade_section_idx],:,Re_i]      ,color = colors2[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Upper Surf')  
        axis_2.plot(AoA,af_sur.lower_surface_dcp_dx_vals[a_loc[blade_section_idx],:,Re_i]      ,color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Lower Surf')       
        axis_2.plot(AoA,af_sur.upper_surface_dcp_dx_vals[a_loc[blade_section_idx],:,Re_i]     ,color = colors2[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Upper Surf')   
        axis_3.plot(AoA,af_sur.lower_surface_delta_star_vals[a_loc[blade_section_idx],:,Re_i]  ,color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Lower Surf')      
        axis_3.plot(AoA,af_sur.upper_surface_delta_star_vals[a_loc[blade_section_idx],:,Re_i] ,color = colors2[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Upper Surf')   
        axis_4.plot(AoA,af_sur.lower_surface_cf_vals[a_loc[blade_section_idx],:,Re_i]          ,color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Lower Surf')       
        axis_4.plot(AoA,af_sur.upper_surface_cf_vals[a_loc[blade_section_idx],:,Re_i]         ,color = colors2[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Upper Surf')   
        axis_5.plot(AoA,af_sur.lower_surface_Ue_vals[a_loc[blade_section_idx],:,Re_i]          ,color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Lower Surf')        
        axis_5.plot(AoA,af_sur.upper_surface_Ue_vals[a_loc[blade_section_idx],:,Re_i]         ,color = colors2[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Upper Surf')     
        axis_6.plot(AoA,af_sur.lower_surface_H_vals[a_loc[blade_section_idx],:,Re_i]           ,color = colors[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Lower Surf')       
        axis_6.plot(AoA,af_sur.upper_surface_H_vals[a_loc[blade_section_idx],:,Re_i]          ,color = colors2[0], linestyle = '-' ,marker =    markers[0]    , label = 'True Upper Surf')   
        

        Re_sec  = np.ones_like(AoA)*Re[Re_i]/1E6
        cond    = np.vstack([Re_sec,AoA]).T    

        ls_theta_sur      = af_sur.lower_surface_theta_surrogates[a_loc[blade_section_idx]].predict(cond) 
        ls_delta_star_sur = af_sur.lower_surface_delta_star_surrogates[a_loc[blade_section_idx]].predict(cond) 
        ls_cf_sur         = af_sur.lower_surface_cf_surrogates[a_loc[blade_section_idx]].predict(cond) 
        ls_dcp_dx_sur     = af_sur.lower_surface_dcp_dx_surrogates[a_loc[blade_section_idx]].predict(cond) 
        ls_Ue_sur         = af_sur.lower_surface_Ue_surrogates[a_loc[blade_section_idx]].predict(cond) 
        ls_H_sur          = af_sur.lower_surface_H_surrogates[a_loc[blade_section_idx]].predict(cond)  
        us_theta_sur      = af_sur.upper_surface_theta_surrogates[a_loc[blade_section_idx]].predict(cond)  
        us_delta_star_sur = af_sur.upper_surface_delta_star_surrogates[a_loc[blade_section_idx]].predict(cond) 
        us_cf_sur         = af_sur.upper_surface_cf_surrogates[a_loc[blade_section_idx]].predict(cond) 
        us_dcp_dx_sur     = af_sur.upper_surface_dcp_dx_surrogates[a_loc[blade_section_idx]].predict(cond) 
        us_Ue_sur         = af_sur.upper_surface_Ue_surrogates[a_loc[blade_section_idx]].predict(cond) 
        us_H_sur          = af_sur.upper_surface_H_surrogates[a_loc[blade_section_idx]].predict(cond)
        


        #ls_theta_sur      = af_sur.lower_surface_theta_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #ls_delta_star_sur = af_sur.lower_surface_delta_star_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #ls_cf_sur         = af_sur.lower_surface_cf_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #ls_dcp_dx_sur     = af_sur.lower_surface_dcp_dx_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #ls_Ue_sur         = af_sur.lower_surface_Ue_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #ls_H_sur          = af_sur.lower_surface_H_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #us_theta_sur      = af_sur.upper_surface_theta_surrogates[blade_section_idx](AoA ,Re_sec,grid=False)  
        #us_delta_star_sur = af_sur.upper_surface_delta_star_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #us_cf_sur         = af_sur.upper_surface_cf_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #us_dcp_dx_sur     = af_sur.upper_surface_dcp_dx_surrogates[blade_section_idx](AoA ,Re_sec,grid=False)  
        #us_Ue_sur         = af_sur.upper_surface_Ue_surrogates[blade_section_idx](AoA ,Re_sec,grid=False) 
        #us_H_sur          = af_sur.upper_surface_H_surrogates[blade_section_idx](AoA ,Re_sec,grid=False)         

          
        axis_1.plot(AoA,ls_theta_sur       ,color = colors[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Lower Surf')       
        axis_1.plot(AoA,us_theta_sur       ,color = colors2[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Upper Surf')  
        axis_2.plot(AoA,ls_dcp_dx_sur      ,color = colors[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Lower Surf')       
        axis_2.plot(AoA,us_dcp_dx_sur      ,color = colors2[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Upper Surf')   
        axis_3.plot(AoA,ls_delta_star_sur  ,color = colors[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Lower Surf')      
        axis_3.plot(AoA,us_delta_star_sur  ,color = colors2[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Upper Surf')   
        axis_4.plot(AoA,ls_cf_sur          ,color = colors[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Lower Surf')       
        axis_4.plot(AoA,us_cf_sur          ,color = colors2[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Upper Surf')   
        axis_5.plot(AoA,ls_Ue_sur          ,color = colors[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Lower Surf')        
        axis_5.plot(AoA,us_Ue_sur          ,color = colors2[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Upper Surf')     
        axis_6.plot(AoA,ls_H_sur           ,color = colors[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Lower Surf')       
        axis_6.plot(AoA,us_H_sur           ,color = colors2[1], linestyle = '-' ,marker =    markers[1]    , label = 'Sur. Upper Surf')    
                        
    
        if (Re_i == 0):  
            axis_1.legend(loc='upper left') 
            axis_2.legend(loc='upper left')
            axis_3.legend(loc='upper left')
            axis_4.legend(loc='upper left')
            axis_5.legend(loc='upper left')
            axis_6.legend(loc='upper left')    
    return 


def import_NACA_4412_xfoil_results(Re_tags,AoA_tags,Npanels): 
    
    Xfoil_data = Data()  
    NRe = len(Re_tags)
    NAoA = len(AoA_tags)
    x_pts        = np.zeros((Npanels,NAoA,NRe))
    y_pts        = np.zeros_like(x_pts)
    Cp           = np.zeros_like(x_pts)
    Ue_Vinf      = np.zeros_like(x_pts)
    delta_star   = np.zeros_like(x_pts)
    theta        = np.zeros_like(x_pts)
    Cf           = np.zeros_like(x_pts)
    H            = np.zeros_like(x_pts)
    
    for i in range(NAoA):
        for j in range(NRe): 
            # read files  
            bl_file_name = 'Airfoil_Data/NACA_4412_Re' + Re_tags[j] + '_A' + AoA_tags[i] + '_1'
            f_1 = open(bl_file_name) 
            data_block_1 = f_1.readlines()
            f_1.close()  

            cp_file_name = 'Airfoil_Data/NACA_4412_Re' + Re_tags[j] + '_A' + AoA_tags[i] + '_2'
            f_2 = open(cp_file_name) 
            data_block_2 = f_2.readlines()
            f_2.close()          
            
            header_1 = 1
            header_2 = 3
            # Loop through each value: append to each column
            for lc in range(Npanels): 
                x_pts[lc,i,j]      = float(data_block_1[lc + header_1][10:20].strip())
                y_pts[lc,i,j]      = float(data_block_1[lc + header_1][20:28].strip())         
                Ue_Vinf[lc,i,j]    = float(data_block_1[lc + header_1][28:38].strip())   
                delta_star[lc,i,j] = float(data_block_1[lc + header_1][38:48].strip())   
                theta[lc,i,j]      = float(data_block_1[lc + header_1][48:58].strip()) 
                Cf[lc,i,j]         = float(data_block_1[lc + header_1][58:68].strip())          
                H [lc,i,j]         = float(data_block_1[lc + header_1][68:78].strip())  
                Cp[lc,i,j]         = float(data_block_2[lc + header_2][20:28].strip()) 
            
    
    Xfoil_data.x          = np.flip(x_pts,axis = 0)    
    Xfoil_data.y          = np.flip(y_pts   ,axis = 0)   
    Xfoil_data.Cp         = np.flip(Cp,axis = 0)         
    Xfoil_data.Ue_Vinf    = np.flip(Ue_Vinf ,axis = 0)   
    Xfoil_data.delta_star = np.flip(delta_star ,axis = 0)
    Xfoil_data.theta      = np.flip(theta,axis = 0)      
    Xfoil_data.Cf         = np.flip(Cf   ,axis = 0)      
    Xfoil_data.H          = np.flip( H ,axis = 0)  
    
    
    Xfoil_data.Cl      = np.array([[0.0003,0.4466,0.4737],
                                   [0.2672,0.8935,0.913],
                                   [0.4767,1.2859,1.3048]])   
    Xfoil_data.Cm      = np.array([[-0.0428,-0.1077,-0.1034],
                                  [-0.0458,-0.1032,-0.1017],
                                  [-0.0542,-0.0902,-0.0931]])        
    Xfoil_data.Cd      = np.array([[0.04733,0.01762,0.00691],
                                   [0.0659,0.01937,0.00723],
                                   [0.09828,0.02355,0.01178]])             
    
    
    
    return Xfoil_data 


if __name__ == '__main__': 
    main() 
    plt.show()