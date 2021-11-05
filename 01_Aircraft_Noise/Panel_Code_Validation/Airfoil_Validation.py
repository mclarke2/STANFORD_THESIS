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
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_boundary_layer_properties\
     import evaluate_boundary_layer_surrogates
import matplotlib.pyplot as plt  
import matplotlib.cm as cm
import os 
import numpy as np

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():    
    # Define Panelization 
    npanel   = 300 
    
    # Batch analysis of single airfoil - NACA 4412 
    Re_batch             = np.atleast_2d(np.array([1E4, 1E5, 1E6])).T
    AoA_batch            = np.atleast_2d(np.array([0.,4.,8.])*Units.degrees).T       
    airfoil_geometry     = compute_naca_4series(0.4,0.4,0.12,npoints=npanel)
    SUAVE_data = airfoil_analysis(airfoil_geometry,AoA_batch,Re_batch, npanel)   
    
    # X foil
    Re_tags  = ['1E4','1E5','1E6']
    AoA_tags = ['0','4','8']
    Xfoil_data = import_NACA_4412_xfoil_results(Re_tags,AoA_tags,npanel)  
     
    # Plot trailing edge properties  
    plot_trailing_edge_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags)   
    
    plot_full_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags) 
    
    #airfoil_bl_surs = compute_airfoil_boundary_layer_properties(airfoil_geometry,npanel=100)
    
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
    axis11.set_xlabel('Cl')
    axis14.set_xlabel('Cd')
    axis17.set_xlabel('Cm')  
    

    fig2  = plt.figure('Airfoil_BL_TE_1') 
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
    axis21.set_xlabel(r'$Ue/V_{inf}$')
    axis24.set_xlabel(r'$\theta$')
    axis27.set_xlabel(r'$\delta$ *')   
    

    fig3  = plt.figure('Airfoil_BL_TE_2') 
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
    axis31.set_xlabel(r'Cf')
    axis34.set_xlabel(r'H')
    axis37.set_xlabel(r'Cp')   
     
    
    # Cl
    axis11.plot(AoA_range ,  SUAVE_data.Cl[:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0] , label = 'SUAVE'  )  
    axis11.plot(AoA_range ,  Xfoil_data.Cl[:,0] ,color = colors[1], linestyle = '--' ,marker =   markers[1] , label = 'Xfoil')   
    axis12.plot(AoA_range,   SUAVE_data.Cl[:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis12.plot(AoA_range,   Xfoil_data.Cl[:,1]  ,color = colors[1], linestyle = '--' ,marker =   markers[1])    
    axis13.plot(AoA_range,   SUAVE_data.Cl[:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis13.plot(AoA_range,   Xfoil_data.Cl[:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])              
     
    # CdAoA_range
    axis14.plot(AoA_range ,  SUAVE_data.Cd[:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis14.plot(AoA_range ,  Xfoil_data.Cd[:,0] ,color = colors[1], linestyle = '--' ,marker =   markers[1]  )  
    axis15.plot(AoA_range,   SUAVE_data.Cd[:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis15.plot(AoA_range,   Xfoil_data.Cd[:,1]  ,color = colors[1], linestyle = '--' ,marker =   markers[1])     
    axis16.plot(AoA_range,   SUAVE_data.Cd[:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis16.plot(AoA_range,   Xfoil_data.Cd[:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])             
  
    # CmAoA_range
    axis17.plot(AoA_range ,  SUAVE_data.Cm[:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0]  )  
    axis17.plot(AoA_range ,  SUAVE_data.Cm[:,0] ,color = colors[1], linestyle = '--' ,marker =   markers[1]  )   
    axis18.plot(AoA_range,   SUAVE_data.Cm[:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis18.plot(AoA_range,   SUAVE_data.Cm[:,1]  ,color = colors[1], linestyle = '--' ,marker =   markers[1])    
    axis19.plot(AoA_range,   SUAVE_data.Cm[:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis19.plot(AoA_range,   SUAVE_data.Cm[:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])   
     
    # Ue/V_inf
    axis21.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.Ue_Vinf[Te_idx ,:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0] , label = 'SUAVE'   )  
    axis21.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.Ue_Vinf[Te_idx ,:,0] ,color = colors[1], linestyle = '--' ,marker =   markers[1] , label = 'Xfoil') 
    axis21.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.Ue_Vinf[-Te_idx,:,0] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis21.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.Ue_Vinf[-Te_idx,:,0] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
    axis22.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.Ue_Vinf[Te_idx ,:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis22.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.Ue_Vinf[Te_idx ,:,1]  ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis22.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.Ue_Vinf[-Te_idx,:,1]  ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis22.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.Ue_Vinf[-Te_idx,:,1]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1])   
    axis23.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.Ue_Vinf[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis23.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.Ue_Vinf[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])         
    axis23.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.Ue_Vinf[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )          
    axis23.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.Ue_Vinf[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])   
    
    # theta
    axis24.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.theta[Te_idx ,:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0]    )  
    axis24.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.theta[Te_idx ,:,0] ,color = colors[1], linestyle = '--' ,marker =   markers[1] ) 
    axis24.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.theta[-Te_idx,:,0] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis24.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.theta[-Te_idx,:,0] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])  
    axis25.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.theta[Te_idx ,:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis25.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.theta[Te_idx ,:,1]  ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis25.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.theta[-Te_idx,:,1]  ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis25.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.theta[-Te_idx,:,1]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
    axis26.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.theta[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis26.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.theta[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])         
    axis26.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.theta[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )          
    axis26.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.theta[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])            
    
    # Delta Star 
    axis27.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.delta_star[Te_idx ,:,0] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis27.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.delta_star[Te_idx ,:,0] ,color = colors[1], linestyle = '--' ,marker =   markers[1]  ) 
    axis27.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.delta_star[-Te_idx,:,0] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis27.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.delta_star[-Te_idx,:,0] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])    
    axis28.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.delta_star[Te_idx ,:,1]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis28.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.delta_star[Te_idx ,:,1]  ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis28.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.delta_star[-Te_idx,:,1]  ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis28.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.delta_star[-Te_idx,:,1]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1])   
    axis29.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.delta_star[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis29.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.delta_star[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])         
    axis29.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.delta_star[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )          
    axis29.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.delta_star[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])   
    

    # Cf
    axis31.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.Cf[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]  , label = 'SUAVE'  )  
    axis31.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.Cf[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1] , label = 'Xfoil') 
    axis31.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.Cf[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis31.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.Cf[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])   
    axis32.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.Cf[Te_idx ,:,2]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis32.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.Cf[Te_idx ,:,2]  ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis32.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.Cf[-Te_idx,:,2]  ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis32.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.Cf[-Te_idx,:,2]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1])  
    axis33.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.Cf[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis33.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.Cf[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])         
    axis33.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.Cf[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )          
    axis33.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.Cf[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])            
     
    # H
    axis34.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.H[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0])  
    axis34.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.H[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1] ) 
    axis34.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.H[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis34.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.H[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])    
    axis35.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.H[Te_idx ,:,2]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis35.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.H[Te_idx ,:,2]  ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis35.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.H[-Te_idx,:,2]  ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis35.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.H[-Te_idx,:,2]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1])    
    axis36.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.H[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis36.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.H[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])         
    axis36.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.H[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )          
    axis36.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.H[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])            
    
    # Cp
    axis37.plot(SUAVE_data.x[Te_idx ,:,0] ,  SUAVE_data.Cp[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0] )  
    axis37.plot(Xfoil_data.x[Te_idx ,:,0] ,  Xfoil_data.Cp[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis37.plot(SUAVE_data.x[-Te_idx,:,0] ,  SUAVE_data.Cp[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis37.plot(Xfoil_data.x[-Te_idx,:,0] ,  Xfoil_data.Cp[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
    axis38.plot(SUAVE_data.x[Te_idx ,:,1],   SUAVE_data.Cp[Te_idx ,:,2]   ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis38.plot(Xfoil_data.x[Te_idx ,:,1],   Xfoil_data.Cp[Te_idx ,:,2]  ,color = colors[1], linestyle = '--' ,marker =   markers[1]) 
    axis38.plot(SUAVE_data.x[-Te_idx,:,1],   SUAVE_data.Cp[-Te_idx,:,2]  ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )  
    axis38.plot(Xfoil_data.x[-Te_idx,:,1],   Xfoil_data.Cp[-Te_idx,:,2]  ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
    axis39.plot(SUAVE_data.x[Te_idx ,:,2],   SUAVE_data.Cp[Te_idx ,:,2] ,color = colors[0], linestyle = '-' ,marker =    markers[0]   )  
    axis39.plot(Xfoil_data.x[Te_idx ,:,2],   Xfoil_data.Cp[Te_idx ,:,2] ,color = colors[1], linestyle = '--' ,marker =   markers[1])         
    axis39.plot(SUAVE_data.x[-Te_idx,:,2],   SUAVE_data.Cp[-Te_idx,:,2] ,color = colors2[0], linestyle = '-' ,marker =  markers[0]  )          
    axis39.plot(Xfoil_data.x[-Te_idx,:,2],   Xfoil_data.Cp[-Te_idx,:,2] ,color = colors2[1], linestyle = '--' ,marker =  markers[1])     
        
    # add legends for plotting
    plt.tight_layout()
    
    axis11.legend(loc='upper right')     
    axis21.legend(loc='upper right') 
    axis31.legend(loc='upper right') 
     
     
     
     
     
     
     
    return  


## @ingroup Plots
def plot_full_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags):   
    
    # determine dimension of angle of attack and reynolds number 
    nAoA = len(SUAVE_data.AoA)
    nRe  = len(SUAVE_data.Re)
    
    # create array of colors for difference reynolds numbers 
    colors  =['blue','darkblue']
    colors2 =['red','darkred']
    markers = ['o','v','s','P','p','^','D','X','*']
    
    fig1  = plt.figure('Ue_Vinf')   
    fig2  = plt.figure('theta') 
    fig3  = plt.figure('Delta Star') 
    fig4  = plt.figure('Cf') 
    fig5  = plt.figure('H') 
    fig6  = plt.figure('Cp') 
    
     
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
            axis6.set_ylim(1.2,-7)  
            axis1.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.Ue_Vinf[:mid ,i,j],color = colors[j], linestyle = '-' ,marker =    markers[0]   )  
            axis1.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.Ue_Vinf[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]) 
            axis1.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.Ue_Vinf[:mid2,i,j],color = colors2[j], linestyle = '-' ,marker =  markers[0]  )  
            axis1.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.Ue_Vinf[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1])              
           
            axis2.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.theta[:mid ,i,j],color = colors[j], linestyle = '-' ,marker =    markers[0]   )  
            axis2.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.theta[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]) 
            axis2.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.theta[:mid2,i,j],color = colors2[j], linestyle = '-' ,marker =  markers[0]  )  
            axis2.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.theta[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1])     
            
            axis3.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.delta_star[:mid ,i,j],color = colors[j], linestyle = '-' ,marker =    markers[0]   )  
            axis3.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.delta_star[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]) 
            axis3.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.delta_star[:mid2,i,j],color = colors2[j], linestyle = '-' ,marker =  markers[0]  )  
            axis3.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.delta_star[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1])    
            

            axis4.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.Cf[:mid ,i,j],color = colors[j], linestyle = '-' ,marker =    markers[0]   )  
            axis4.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.Cf[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]) 
            axis4.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.Cf[:mid2,i,j],color = colors2[j], linestyle = '-' ,marker =  markers[0]  )  
            axis4.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.Cf[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1])              
           
            axis5.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.H[:mid ,i,j],color = colors[j], linestyle = '-' ,marker =    markers[0]   )  
            axis5.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.H[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]) 
            axis5.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.H[:mid2,i,j],color = colors2[j], linestyle = '-' ,marker =  markers[0]  )  
            axis5.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.H[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1])     
            
            axis6.plot(SUAVE_data.x[:mid ,i,j],   SUAVE_data.Cp[:mid ,i,j],color = colors[j], linestyle = '-' ,marker =    markers[0]   )  
            axis6.plot(SUAVE_data.x[mid: ,i,j],   SUAVE_data.Cp[mid: ,i,j],color = colors[1], linestyle = '--' ,marker =   markers[1]) 
            axis6.plot(Xfoil_data.x[:mid2,i,j],   Xfoil_data.Cp[:mid2,i,j],color = colors2[j], linestyle = '-' ,marker =  markers[0]  )  
            axis6.plot(Xfoil_data.x[mid2:,i,j],   Xfoil_data.Cp[mid2:,i,j],color = colors2[1], linestyle = '--' ,marker =  markers[1])        
                
                
    ## add legends for plotting
    #plt.tight_layout()
    #if show_legend:
        #lines1, labels1 = fig2.axes[0].get_legend_handles_labels()
        #fig2.legend(lines1, labels1, loc='upper center', ncol=5)
        #plt.tight_layout()
        #axis8.legend(loc='upper right')     
    return    



def surrogates():
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
    
    prop_a                          = SUAVE.Components.Energy.Converters.Propeller()  
    prop_a.number_of_blades         = 3
    prop_a.number_of_engines        = 1
    prop_a.freestream_velocity      = 49.1744 
    prop_a.tip_radius               = 1.0668
    prop_a.hub_radius               = 0.21336 
    prop_a.design_tip_mach          = 0.65
    prop_a.angular_velocity         = gearbox.inputs.speed # 207.16160479940007 
    prop_a.design_Cl                = 0.7
    prop_a.design_altitude          = 1. * Units.km      
    prop_a.airfoil_geometry         = ['../Vehicles/Airfoils/NACA_4412.txt','../Vehicles/Airfoils/Clark_y.txt']

    prop_a.airfoil_polars           = [['../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_50000.txt',
                                        '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_100000.txt',
                                        '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_200000.txt',
                                        '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_500000.txt',
                                        '../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt'],
                                       ['../Vehicles/Airfoils/Polars/Clark_y_polar_Re_50000.txt',
                                        '../Vehicles/Airfoils/Polars/Clark_y_polar_Re_100000.txt',
                                        '../Vehicles/Airfoils/Polars/Clark_y_polar_Re_200000.txt',
                                        '../Vehicles/Airfoils/Polars/Clark_y_polar_Re_500000.txt',
                                        '../Vehicles/Airfoils/Polars/Clark_y_polar_Re_1000000.txt']] 

    prop_a.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1]  
    prop_a.design_thrust           = 3054.4809132125697
    prop_a                         = propeller_design(prop_a)  
    
    # plot propeller 
    plot_propeller(prop_a)
  
    
    # Find the operating conditions
    atmosphere            = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions =  atmosphere.compute_values(prop_a.design_altitude)
    
    V  = prop_a.freestream_velocity 
    
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
    prop_a.inputs.omega  = np.array(prop_a.angular_velocity,ndmin=2) 
    
    # propeller with airfoil results 
    prop_a.inputs.pitch_command                = 0.0*Units.degree
    F_a, Q_a, P_a, Cplast_a ,output_a , etap_a = prop_a.spin(conditions)  
    plot_results(output_a, prop_a,'blue','-','s')   
     
    rho                = conditions.freestream.density                 
    dyna_visc          = conditions.freestream.dynamic_viscosity            
    alpha_blade        = output_a.disc_effective_angle_of_attack 
    Vt_2d              = output_a.disc_tangential_velocity  
    Va_2d              = output_a.disc_axial_velocity                
    blade_chords       = prop_a.chord_distribution         
    r                  = prop_a.radius_distribution      
    num_sec            = len(r) 
    num_azi            = len(output_a.disc_effective_angle_of_attack[0,0,:])   
    U_blade            = np.sqrt(Vt_2d**2 + Va_2d **2)
    Re_blade           = U_blade*np.repeat(np.repeat(blade_chords[np.newaxis,:],1,axis=0)[:,:,np.newaxis],num_azi,axis=2)*\
                          np.repeat(np.repeat((rho/dyna_visc),num_sec,axis=1)[:,:,np.newaxis],num_azi,axis=2)  

    # ------------------------------------------------------------
    # ****** TRAILING EDGE BOUNDARY LAYER PROPERTY CALCULATIONS  ****** 
    bl_results = evaluate_boundary_layer_surrogates(prop_a,alpha_blade,Re_blade)
    
    bl_results.ls_theta 
    bl_results.ls_delta   
    bl_results.ls_delta_star 
    bl_results.ls_cf      
    bl_results.ls_Ue        
    bl_results.ls_H          
    bl_results.us_theta  
    bl_results.us_delta    
    bl_results.us_delta_star
    bl_results.us_cf     
    bl_results.us_Ue
    bl_results.us_H   
    

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
            
    
    Xfoil_data.x          = x_pts      
    Xfoil_data.y          = y_pts      
    Xfoil_data.Cp         = Cp         
    Xfoil_data.Ue_Vinf    = Ue_Vinf    
    Xfoil_data.delta_star = delta_star 
    Xfoil_data.theta      = theta      
    Xfoil_data.Cf         = Cf         
    Xfoil_data.H          = H   
    
    
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