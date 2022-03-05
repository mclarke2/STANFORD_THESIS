# Airfoil Validation.py
# 
# Created:  
# Modified: Mar 2021, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
from SUAVE.Core import Units, Data 
from SUAVE.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis      import airfoil_analysis 
import matplotlib.pyplot as plt   
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series \
     import  compute_naca_4series
from SUAVE.Plots.Performance.Airfoil_Plots import * 
import matplotlib.cm as cm 
import numpy as np    


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():     

    # Define Panelization 

    npanel               = 300  
    Re_batch             = np.atleast_2d(np.array([1E4, 1E5, 1E6])).T
    AoA_batch            = np.atleast_2d(np.array([0.,4.,8.])*Units.degrees).T       
    airfoil_geometry     = compute_naca_4series(0.04,0.4,0.12,npoints=npanel)
    SUAVE_data           = airfoil_analysis(airfoil_geometry,AoA_batch,Re_batch, npanel)   

    # X foil
    Re_tags  = ['1E4','1E5','1E6']
    AoA_tags = ['0','4','8']
    xfoil_npanel   = 300 
    Xfoil_data     = import_NACA_4412_xfoil_results(Re_tags,AoA_tags,xfoil_npanel)  


    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'xtick.labelsize': 28,
                  'legend.fontsize': 20,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 2 
    plot_parameters.line_style       = '-' 
    plot_parameters.figure_width     = 8 
    plot_parameters.figure_height    = 6
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 20 
    plot_parameters.plot_grid        = True      
    plot_parameters.colors           = ['blue','darkblue']
    plot_parameters.colors2          = ['red','darkred'] 
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']

    # Plot trailing edge properties  
    plot_trailing_edge_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags,plot_parameters)   
    

    plot_pressure_distribution(SUAVE_data,Xfoil_data,Re_tags,AoA_tags,plot_parameters)      

    return 

def plot_trailing_edge_boundary_layer_properties(SUAVE_data,Xfoil_data,Re_tags,AoA_tags,PP):   

    # determine dimension of angle of attack and reynolds number  

    mid = int(len(SUAVE_data.x[0,0,:])/2) 
    mid2 = int(len(Xfoil_data.x[0,0,:])/2)   
    
    Te_idx_SUAVE = 10  
    Te_idx_SUAVE_cf = 30
    Te_idx_Xfoil = 5
    Te_idx = 1 

    fig01  = plt.figure() 
    fig02  = plt.figure() 
    fig03  = plt.figure() 
    fig04  = plt.figure() 
    fig05  = plt.figure() 
    fig06  = plt.figure() 
    fig07  = plt.figure() 
    fig08  = plt.figure() 
    fig09  = plt.figure()
    fig10  = plt.figure()
    fig11  = plt.figure() 
    fig12  = plt.figure() 
    fig13  = plt.figure() 
    fig14  = plt.figure() 
    fig15  = plt.figure() 
    fig16  = plt.figure() 
    fig17  = plt.figure() 
    fig18  = plt.figure() 
  
    fig01.set_size_inches(PP.figure_width,PP.figure_height) 
    fig02.set_size_inches(PP.figure_width,PP.figure_height) 
    fig03.set_size_inches(PP.figure_width,PP.figure_height) 
    fig04.set_size_inches(PP.figure_width,PP.figure_height) 
    fig05.set_size_inches(PP.figure_width,PP.figure_height) 
    fig06.set_size_inches(PP.figure_width,PP.figure_height) 
    fig07.set_size_inches(PP.figure_width,PP.figure_height) 
    fig08.set_size_inches(PP.figure_width,PP.figure_height) 
    fig09.set_size_inches(PP.figure_width,PP.figure_height) 
    fig10.set_size_inches(PP.figure_width,PP.figure_height)
    fig11.set_size_inches(PP.figure_width,PP.figure_height) 
    fig12.set_size_inches(PP.figure_width,PP.figure_height) 
    fig13.set_size_inches(PP.figure_width,PP.figure_height) 
    fig14.set_size_inches(PP.figure_width,PP.figure_height) 
    fig15.set_size_inches(PP.figure_width,PP.figure_height) 
    fig16.set_size_inches(PP.figure_width,PP.figure_height) 
    fig17.set_size_inches(PP.figure_width,PP.figure_height) 
    fig18.set_size_inches(PP.figure_width,PP.figure_height)  
    

    axis01 = fig01.add_subplot(1,1,1)     
    axis02 = fig02.add_subplot(1,1,1)     
    axis03 = fig03.add_subplot(1,1,1)     
    axis04 = fig04.add_subplot(1,1,1)     
    axis05 = fig05.add_subplot(1,1,1)     
    axis06 = fig06.add_subplot(1,1,1)     
    axis07 = fig07.add_subplot(1,1,1)     
    axis08 = fig08.add_subplot(1,1,1)   
    axis09 = fig09.add_subplot(1,1,1) 
    axis10 = fig10.add_subplot(1,1,1)   
    axis11 = fig11.add_subplot(1,1,1)     
    axis12 = fig12.add_subplot(1,1,1)     
    axis13 = fig13.add_subplot(1,1,1)     
    axis14 = fig14.add_subplot(1,1,1)     
    axis15 = fig15.add_subplot(1,1,1)     
    axis16 = fig16.add_subplot(1,1,1)     
    axis17 = fig17.add_subplot(1,1,1)     
    axis18 = fig18.add_subplot(1,1,1) 
 
    
    x     = np.arange(len( AoA_tags))  # the label locations
    width = 0.2  # the width of the bars 
    
    rects011 = axis01.bar(x - 1.5*width,   SUAVE_data.Ue_Vinf[:,0,Te_idx_SUAVE]    , width , color= PP.colors[0] , label='SUAVE Lower Surf.')
    rects012 = axis01.bar(x -   width/2,   -Xfoil_data.Ue_Vinf[:,0,Te_idx_Xfoil]   , width, color= PP.colors2[0] , label='XFoil Lower Surf.')
    rects013 = axis01.bar(x +   width/2,    SUAVE_data.Ue_Vinf[:,0,-Te_idx_SUAVE]  , width, color= PP.colors[1]  , label='SUAVE Upper Surf.')
    rects014 = axis01.bar(x + 1.5*width,    Xfoil_data.Ue_Vinf[:,0,-Te_idx_Xfoil]  , width, color= PP.colors2[1] , label='XFoil Upper Surf.') 
    plt.tight_layout()
    rects021 = axis02.bar(x - 1.5*width,   SUAVE_data.Ue_Vinf[:,1,Te_idx_SUAVE]    , width, color= PP.colors[0]  , label='SUAVE Lower Surf.')
    rects022 = axis02.bar(x -   width/2,   -Xfoil_data.Ue_Vinf[:,1,Te_idx_Xfoil]   , width, color= PP.colors2[0] , label='XFoil Lower Surf.')
    rects023 = axis02.bar(x +   width/2,    SUAVE_data.Ue_Vinf[:,1,-Te_idx_SUAVE]  , width, color= PP.colors[1]  , label='SUAVE Upper Surf.')
    rects024 = axis02.bar(x + 1.5*width,    Xfoil_data.Ue_Vinf[:,1,-Te_idx_Xfoil]  , width, color= PP.colors2[1] , label='XFoil Upper Surf.') 
    plt.tight_layout()
    rects031 = axis03.bar(x - 1.5*width,   SUAVE_data.Ue_Vinf[:,2,Te_idx_SUAVE]    , width, color= PP.colors[0]  , label='SUAVE Lower Surf.')
    rects032 = axis03.bar(x -   width/2,   -Xfoil_data.Ue_Vinf[:,2,Te_idx_Xfoil]   , width, color= PP.colors2[0] , label='XFoil Lower Surf.')
    rects033 = axis03.bar(x +   width/2,    SUAVE_data.Ue_Vinf[:,2,-Te_idx_SUAVE]  , width, color= PP.colors[1]  , label='SUAVE Upper Surf.')
    rects034 = axis03.bar(x + 1.5*width,    Xfoil_data.Ue_Vinf[:,2,-Te_idx_Xfoil]  , width, color= PP.colors2[1] , label='XFoil Upper Surf.')
    plt.tight_layout()
    
    rects041 = axis04.bar(x - 1.5*width, SUAVE_data.theta[:,0,Te_idx_SUAVE]     , width  , color= PP.colors[0] )
    rects042 = axis04.bar(x -   width/2, Xfoil_data.theta[:,0,Te_idx_Xfoil]      , width , color= PP.colors2[0])
    rects043 = axis04.bar(x +   width/2, SUAVE_data.theta[:,0,-Te_idx_SUAVE]  , width , color= PP.colors[1]   )
    rects044 = axis04.bar(x + 1.5*width, Xfoil_data.theta[:,0,-Te_idx_Xfoil]  , width , color= PP.colors2[1]  ) 
    plt.tight_layout()
    rects051 = axis05.bar(x - 1.5*width, SUAVE_data.theta[:,1,Te_idx_SUAVE]       , width , color= PP.colors[0]  )
    rects052 = axis05.bar(x -   width/2, Xfoil_data.theta[:,1,Te_idx_Xfoil]       , width , color= PP.colors2[0] )
    rects053 = axis05.bar(x +   width/2, SUAVE_data.theta[:,1,-Te_idx_SUAVE]  , width , color= PP.colors[1]     )
    rects054 = axis05.bar(x + 1.5*width, Xfoil_data.theta[:,1,-Te_idx_Xfoil]  , width , color= PP.colors2[1]    ) 
    plt.tight_layout()
    rects061 = axis06.bar(x - 1.5*width,SUAVE_data.theta[:,2,Te_idx_SUAVE]      , width , color= PP.colors[0]  )
    rects062 = axis06.bar(x -   width/2,Xfoil_data.theta[:,2,Te_idx_Xfoil]       , width, color= PP.colors2[0] )
    rects063 = axis06.bar(x +   width/2, SUAVE_data.theta[:,2,-Te_idx_SUAVE]   , width, color= PP.colors[1]  )
    rects064 = axis06.bar(x + 1.5*width, Xfoil_data.theta[:,2,-Te_idx_Xfoil]   , width, color= PP.colors2[1] )
    plt.tight_layout()
    
    rects071 = axis07.bar(x - 1.5*width, SUAVE_data.delta_star[:,0,Te_idx_SUAVE]   , width, color= PP.colors[0]  )
    rects072 = axis07.bar(x -   width/2, Xfoil_data.delta_star[:,0,Te_idx_Xfoil]   , width, color= PP.colors2[0] )
    rects073 = axis07.bar(x +   width/2, SUAVE_data.delta_star[:,0,-Te_idx_SUAVE]  , width, color= PP.colors[1]  )
    rects074 = axis07.bar(x + 1.5*width, Xfoil_data.delta_star[:,0,-Te_idx_Xfoil]  , width, color= PP.colors2[1] ) 
    plt.tight_layout()
    rects081 = axis08.bar(x - 1.5*width, SUAVE_data.delta_star[:,1,Te_idx_SUAVE]   , width, color= PP.colors[0]  )
    rects082 = axis08.bar(x -   width/2, Xfoil_data.delta_star[:,1,Te_idx_Xfoil]   , width, color= PP.colors2[0] )
    rects083 = axis08.bar(x +   width/2, SUAVE_data.delta_star[:,1,-Te_idx_SUAVE]  , width, color= PP.colors[1]  )
    rects084 = axis08.bar(x + 1.5*width, Xfoil_data.delta_star[:,1,-Te_idx_Xfoil]  , width, color= PP.colors2[1] ) 
    plt.tight_layout()
    rects091 = axis09.bar(x - 1.5*width, SUAVE_data.delta_star[:,2,Te_idx_SUAVE]   , width, color= PP.colors[0]  )
    rects092 = axis09.bar(x -   width/2, Xfoil_data.delta_star[:,2,Te_idx_Xfoil]   , width, color= PP.colors2[0] )
    rects093 = axis09.bar(x +   width/2, SUAVE_data.delta_star[:,2,-Te_idx_SUAVE]  , width, color= PP.colors[1]  )
    rects094 = axis09.bar(x + 1.5*width, Xfoil_data.delta_star[:,2,-Te_idx_Xfoil]  , width, color= PP.colors2[1] )
    plt.tight_layout()
    
    rects101 = axis10.bar(x - 1.5*width,  SUAVE_data.Cf[:,0,Te_idx_SUAVE_cf]   , width, color= PP.colors[0]  )
    rects102 = axis10.bar(x -   width/2,  Xfoil_data.Cf[:,0,Te_idx_Xfoil]   , width, color= PP.colors2[0] )
    rects103 = axis10.bar(x +   width/2,  SUAVE_data.Cf[:,0,-Te_idx_SUAVE_cf] , width, color= PP.colors[1]  )
    rects104 = axis10.bar(x + 1.5*width,  Xfoil_data.Cf[:,0,-Te_idx_Xfoil] , width, color= PP.colors2[1] ) 
    plt.tight_layout()
    rects111 = axis11.bar(x - 1.5*width,  SUAVE_data.Cf[:,1,Te_idx_SUAVE_cf]   , width, color= PP.colors[0]  ) 
    rects112 = axis11.bar(x -   width/2,  Xfoil_data.Cf[:,1,Te_idx_Xfoil]   , width, color= PP.colors2[0] ) 
    rects113 = axis11.bar(x +   width/2,  SUAVE_data.Cf[:,1,-Te_idx_SUAVE_cf] , width, color= PP.colors[1]  ) 
    rects114 = axis11.bar(x + 1.5*width,  Xfoil_data.Cf[:,1,-Te_idx_Xfoil] , width, color= PP.colors2[1] ) 
    plt.tight_layout()
    rects121 = axis12.bar(x - 1.5*width,  SUAVE_data.Cf[:,2,Te_idx_SUAVE_cf]   , width, color= PP.colors[0]  ) 
    rects122 = axis12.bar(x -   width/2,  Xfoil_data.Cf[:,2,Te_idx_Xfoil]   , width, color= PP.colors2[0] ) 
    rects123 = axis12.bar(x +   width/2,  SUAVE_data.Cf[:,2,-Te_idx_SUAVE_cf] , width, color= PP.colors[1]   ) 
    rects124 = axis12.bar(x + 1.5*width,  Xfoil_data.Cf[:,2,-Te_idx_Xfoil] , width, color= PP.colors2[1]  ) 
    plt.tight_layout()
 
    rects131 = axis13.bar(x - 1.5*width,   SUAVE_data.H[:,0,Te_idx_SUAVE]    , width,color = PP.colors[0] )
    rects132 = axis13.bar(x -   width/2,   Xfoil_data.H[:,0,Te_idx_Xfoil]   , width,color = PP.colors2[0] )
    rects133 = axis13.bar(x +   width/2,   SUAVE_data.H[:,0,-Te_idx_SUAVE] , width,color = PP.colors[1]  )
    rects134 = axis13.bar(x + 1.5*width,   Xfoil_data.H[:,0,-Te_idx_Xfoil] , width,color = PP.colors2[1] ) 
    plt.tight_layout()
    rects141 = axis14.bar(x - 1.5*width,   SUAVE_data.H[:,1,Te_idx_SUAVE]   , width,color = PP.colors[0]   )
    rects142 = axis14.bar(x -   width/2,   Xfoil_data.H[:,1,Te_idx_Xfoil]   , width,color = PP.colors2[0]  )
    rects143 = axis14.bar(x +   width/2,   SUAVE_data.H[:,1,-Te_idx_SUAVE] , width ,color = PP.colors[1]  )
    rects144 = axis14.bar(x + 1.5*width,   Xfoil_data.H[:,1,-Te_idx_Xfoil] , width ,color = PP.colors2[1] )
    plt.tight_layout()
    rects151 = axis15.bar(x - 1.5*width,  SUAVE_data.H[:,2,Te_idx_SUAVE]   , width ,color = PP.colors[0]  )
    rects152 = axis15.bar(x -   width/2,  Xfoil_data.H[:,2,Te_idx_Xfoil]   , width ,color = PP.colors2[0] )
    rects153 = axis15.bar(x +   width/2,   SUAVE_data.H[:,2,-Te_idx_SUAVE] , width ,color = PP.colors[1]  )
    rects154 = axis15.bar(x + 1.5*width,   Xfoil_data.H[:,2,-Te_idx_Xfoil] , width ,color = PP.colors2[1] )
    plt.tight_layout()
    
          
    rects161 = axis16.bar(x - 1.5*width, SUAVE_data.Cp[:,0,:mid][:,Te_idx]    , width, color= PP.colors[0] )
    rects162 = axis16.bar(x -   width/2, Xfoil_data.Cp[:,0,:mid2][:,Te_idx]    , width, color= PP.colors2[0])
    rects163 = axis16.bar(x +   width/2, SUAVE_data.Cp[:,0,mid:][:,-Te_idx ]  , width, color= PP.colors[1] )
    rects164 = axis16.bar(x + 1.5*width, Xfoil_data.Cp[:,0,mid2:][:,-Te_idx ]  , width, color= PP.colors2[1]) 
    plt.tight_layout() 
    rects171 = axis17.bar(x - 1.5*width, SUAVE_data.Cp[:,1,:mid ][:,Te_idx ]    , width, color= PP.colors[0]  )
    rects172 = axis17.bar(x -   width/2, Xfoil_data.Cp[:,1,:mid2][:,Te_idx ]    , width, color= PP.colors2[0] )
    rects173 = axis17.bar(x +   width/2, SUAVE_data.Cp[:,1,mid: ][:,-Te_idx ]  , width, color= PP.colors[1]  )
    rects174 = axis17.bar(x + 1.5*width, Xfoil_data.Cp[:,1,mid2:][:,-Te_idx ]  , width, color= PP.colors2[1] )
    plt.tight_layout()
    rects181 = axis18.bar(x - 1.5*width, SUAVE_data.Cp[:,2,:mid][:,Te_idx ]    , width, color= PP.colors[0]  )
    rects182 = axis18.bar(x -   width/2, Xfoil_data.Cp[:,2,:mid2][:,Te_idx ]    , width, color= PP.colors2[0] )
    rects183 = axis18.bar(x +   width/2, SUAVE_data.Cp[:,2,mid: ][:,-Te_idx ]  , width, color= PP.colors[1]  )
    rects184 = axis18.bar(x + 1.5*width, Xfoil_data.Cp[:,2,mid2:][:,-Te_idx ]  , width, color= PP.colors2[1] ) 
    plt.tight_layout()
 
    axis01.set_xticks(x,AoA_tags) 
    axis02.set_xticks(x,AoA_tags)
    axis03.set_xticks(x,AoA_tags)
    axis04.set_xticks(x,AoA_tags) 
    axis05.set_xticks(x,AoA_tags)
    axis06.set_xticks(x,AoA_tags)
    axis07.set_xticks(x,AoA_tags) 
    axis08.set_xticks(x,AoA_tags)
    axis09.set_xticks(x,AoA_tags)
    axis10.set_xticks(x,AoA_tags) 
    axis11.set_xticks(x,AoA_tags)
    axis12.set_xticks(x,AoA_tags)
    axis13.set_xticks(x,AoA_tags) 
    axis14.set_xticks(x,AoA_tags)
    axis15.set_xticks(x,AoA_tags)
    axis16.set_xticks(x,AoA_tags) 
    axis17.set_xticks(x,AoA_tags)
    axis18.set_xticks(x,AoA_tags) 

   
    axis01.set_xlabel('AoA') 
    axis02.set_xlabel('AoA') 
    axis03.set_xlabel('AoA') 
    axis04.set_xlabel('AoA') 
    axis05.set_xlabel('AoA') 
    axis06.set_xlabel('AoA') 
    axis07.set_xlabel('AoA')
    axis08.set_xlabel('AoA')
    axis09.set_xlabel('AoA')
    axis10.set_xlabel('AoA')
    axis11.set_xlabel('AoA') 
    axis12.set_xlabel('AoA') 
    axis13.set_xlabel('AoA') 
    axis14.set_xlabel('AoA') 
    axis15.set_xlabel('AoA') 
    axis16.set_xlabel('AoA') 
    axis17.set_xlabel('AoA')
    axis18.set_xlabel('AoA') 
    
    

    axis01.set_ylabel(r'$U_e/U_{inf}$')
    axis02.set_ylabel(r'$U_e/U_{inf}$')
    axis03.set_ylabel(r'$U_e/U_{inf}$')
    axis04.set_ylabel(r'$\theta$')
    axis05.set_ylabel(r'$\theta$')
    axis06.set_ylabel(r'$\theta$')
    axis07.set_ylabel(r'$\delta$ *')    
    axis08.set_ylabel(r'$\delta$ *')    
    axis09.set_ylabel(r'$\delta$ *')     
    axis10.set_ylabel(r'$C_f$') 
    axis11.set_ylabel(r'$C_f$')
    axis12.set_ylabel(r'$C_f$')
    axis13.set_ylabel(r'$H$')
    axis14.set_ylabel(r'$H$')
    axis15.set_ylabel(r'$H$')
    axis16.set_ylabel(r'$C_p$')
    axis17.set_ylabel(r'$C_p$') 
    axis18.set_ylabel(r'$C_p$')   
    
    

    axis01.set_ylim([0,1.8])
    axis02.set_ylim([0,1.8])
    axis03.set_ylim([0,1.8])
    axis04.set_ylim([0,0.03 ])
    axis05.set_ylim([0,0.015 ])
    axis06.set_ylim([0,0.01 ])
    axis07.set_ylim([0,0.2])  
    axis08.set_ylim([0,0.04])  
    axis09.set_ylim([0,0.02])   
    axis10.set_ylim([0,0.005])
    axis11.set_ylim([0,0.005])
    axis12.set_ylim([0,0.005])
    axis13.set_ylim([0,4.5])
    axis14.set_ylim([0,4])
    axis15.set_ylim([0,4])
    axis16.set_ylim([-1,1])
    axis17.set_ylim([-1,1])
    axis18.set_ylim([-1,1])
    
    
    # add legends for plotting 
    axis01.legend(loc='upper left')  
    axis02.legend(loc='upper left')   
    axis03.legend(loc='upper left')    
    #axis04.legend(loc='upper left')    
    #axis07.legend(loc='upper left')   
    #axis10.legend(loc='upper right')    
    #axis13.legend(loc='upper left')   
    #axis16.legend(loc='upper left')   


    fig01.tight_layout()
    fig02.tight_layout()
    fig03.tight_layout()
    fig04.tight_layout()
    fig05.tight_layout()
    fig06.tight_layout()
    fig07.tight_layout()
    fig08.tight_layout()
    fig09.tight_layout()
    fig10.tight_layout()
    fig11.tight_layout()
    fig12.tight_layout()
    fig13.tight_layout()
    fig14.tight_layout()
    fig15.tight_layout()
    fig16.tight_layout()
    fig17.tight_layout()
    fig18.tight_layout()  

    fig01_name = 'Airfoil_BL_Ue_Uinf_' + Re_tags[0] 
    fig02_name = 'Airfoil_BL_Ue_Uinf_' + Re_tags[1]
    fig03_name = 'Airfoil_BL_Ue_Uinf_' + Re_tags[2]
    fig04_name = 'Airfoil_BL_theta_' +Re_tags[0] 
    fig05_name = 'Airfoil_BL_theta_' +Re_tags[1]
    fig06_name = 'Airfoil_BL_theta_' +Re_tags[2]
    fig07_name = 'Airfoil_BL_delta_star_' + Re_tags[0] 
    fig08_name = 'Airfoil_BL_delta_star_' + Re_tags[1]      
    fig09_name = 'Airfoil_BL_delta_star_' + Re_tags[2] 
    fig10_name = 'Airfoil_BL_Cf_' + Re_tags[0] 
    fig11_name = 'Airfoil_BL_Cf_' + Re_tags[1]
    fig12_name = 'Airfoil_BL_Cf_' + Re_tags[2]
    fig13_name = 'Airfoil_BL_H_' +Re_tags[0] 
    fig14_name = 'Airfoil_BL_H_' +Re_tags[1]
    fig15_name = 'Airfoil_BL_H_' +Re_tags[2]
    fig16_name = 'Airfoil_BL_Cp_' +Re_tags[0] 
    fig17_name = 'Airfoil_BL_Cp_' +Re_tags[1] 
    fig18_name = 'Airfoil_BL_Cp_' +Re_tags[2]   

    fig01.savefig(fig01_name + '.pdf')
    fig02.savefig(fig02_name + '.pdf')
    fig03.savefig(fig03_name + '.pdf')
    fig04.savefig(fig04_name + '.pdf')
    fig05.savefig(fig05_name + '.pdf')
    fig06.savefig(fig06_name + '.pdf')
    fig07.savefig(fig07_name + '.pdf')
    fig08.savefig(fig08_name + '.pdf')
    fig09.savefig(fig09_name + '.pdf')
    fig10.savefig(fig10_name + '.pdf') 
    fig11.savefig(fig11_name + '.pdf')
    fig12.savefig(fig12_name + '.pdf')
    fig13.savefig(fig13_name + '.pdf')
    fig14.savefig(fig14_name + '.pdf')
    fig15.savefig(fig15_name + '.pdf')
    fig16.savefig(fig16_name + '.pdf')
    fig17.savefig(fig17_name + '.pdf')
    fig18.savefig(fig18_name + '.pdf') 

    return   


def plot_pressure_distribution(SUAVE_data,Xfoil_data,Re_tags,AoA_tags,PP): 

    fig01  = plt.figure() 
    fig02  = plt.figure() 
    fig03  = plt.figure()  
  
    fig01.set_size_inches(PP.figure_width,PP.figure_height) 
    fig02.set_size_inches(PP.figure_width,PP.figure_height) 
    fig03.set_size_inches(PP.figure_width,PP.figure_height)  
    

    axis01 = fig01.add_subplot(1,1,1)     
    axis02 = fig02.add_subplot(1,1,1)     
    axis03 = fig03.add_subplot(1,1,1)      
  
    
    mid = int(len(SUAVE_data.x[:,0,0])/2)
    mid2 = int(len(Xfoil_data.x[:,0,0])/2)     
     
    axis01.plot(SUAVE_data.x[0,0,:mid ],   SUAVE_data.Cp[0,0,:mid ] , linewidth = PP.line_width ,color = PP.colors[0], linestyle = '-' ,marker  =  PP.markers[0]   , label = 'SUAVE, AoA = ' + AoA_tags[0])      
    axis01.plot(SUAVE_data.x[0,0,mid: ],   SUAVE_data.Cp[0,0,mid: ] , linewidth = PP.line_width ,color = PP.colors[0], linestyle = '--' ,marker =  PP.markers[0] ) 
    axis01.plot(Xfoil_data.x[0,0,:mid2],   Xfoil_data.Cp[0,0,:mid2] , linewidth = PP.line_width ,color = PP.colors2[0], linestyle = '-' ,marker  =  PP.markers[0]    , label = 'Xfoil, AoA = ' + AoA_tags[0])   
    axis01.plot(Xfoil_data.x[0,0,mid2:],   Xfoil_data.Cp[0,0,mid2:] , linewidth = PP.line_width ,color = PP.colors2[0], linestyle = '--' ,marker =  PP.markers[0]   )    
    axis01.plot(SUAVE_data.x[1,0,:mid ],   SUAVE_data.Cp[1,0,:mid ] , linewidth = PP.line_width ,color = PP.colors[1], linestyle = '-' ,marker  =  PP.markers[1]   , label = 'SUAVE, AoA = ' + AoA_tags[1])      
    axis01.plot(SUAVE_data.x[1,0,mid: ],   SUAVE_data.Cp[1,0,mid: ] , linewidth = PP.line_width ,color = PP.colors[1], linestyle = '--' ,marker =  PP.markers[1] ) 
    axis01.plot(Xfoil_data.x[1,0,:mid2],   Xfoil_data.Cp[1,0,:mid2] , linewidth = PP.line_width ,color = PP.colors2[1], linestyle = '-' ,marker  =  PP.markers[1]    , label = 'Xfoil, AoA = ' + AoA_tags[1])   
    axis01.plot(Xfoil_data.x[1,0,mid2:],   Xfoil_data.Cp[1,0,mid2:] , linewidth = PP.line_width ,color = PP.colors2[1], linestyle = '--' ,marker =  PP.markers[1]   )     
     
    axis02.plot(SUAVE_data.x[0,1,:mid ],   SUAVE_data.Cp[0,1,:mid ], linewidth = PP.line_width ,color = PP.colors[0], linestyle = '-' ,marker  =  PP.markers[0]   , label = 'SUAVE, AoA = ' + AoA_tags[0])      
    axis02.plot(SUAVE_data.x[0,1,mid: ],   SUAVE_data.Cp[0,1,mid: ], linewidth = PP.line_width ,color = PP.colors[0], linestyle = '--' ,marker =  PP.markers[0] ) 
    axis02.plot(Xfoil_data.x[0,1,:mid2],   Xfoil_data.Cp[0,1,:mid2], linewidth = PP.line_width ,color = PP.colors2[0], linestyle = '-' ,marker  =  PP.markers[0]    , label = 'Xfoil, AoA = ' + AoA_tags[0])   
    axis02.plot(Xfoil_data.x[0,1,mid2:],   Xfoil_data.Cp[0,1,mid2:], linewidth = PP.line_width ,color = PP.colors2[0], linestyle = '--' ,marker =  PP.markers[0]   )    
    axis02.plot(SUAVE_data.x[1,1,:mid ],   SUAVE_data.Cp[1,1,:mid ], linewidth = PP.line_width ,color = PP.colors[1], linestyle = '-' ,marker  =  PP.markers[1]   , label = 'SUAVE, AoA = ' + AoA_tags[1])      
    axis02.plot(SUAVE_data.x[1,1,mid: ],   SUAVE_data.Cp[1,1,mid: ], linewidth = PP.line_width ,color = PP.colors[1], linestyle = '--' ,marker =  PP.markers[1] ) 
    axis02.plot(Xfoil_data.x[1,1,:mid2],   Xfoil_data.Cp[1,1,:mid2], linewidth = PP.line_width ,color = PP.colors2[1], linestyle = '-' ,marker  =  PP.markers[1]    , label = 'Xfoil, AoA = ' + AoA_tags[1])   
    axis02.plot(Xfoil_data.x[1,1,mid2:],   Xfoil_data.Cp[1,1,mid2:], linewidth = PP.line_width ,color = PP.colors2[1], linestyle = '--' ,marker =  PP.markers[1]   )       
    
    axis03.plot(SUAVE_data.x[0,2,:mid ],   SUAVE_data.Cp[0,2,:mid ], linewidth = PP.line_width ,color = PP.colors[0], linestyle = '-' ,marker  =  PP.markers[0]   , label = 'SUAVE, AoA = ' + AoA_tags[0])      
    axis03.plot(SUAVE_data.x[0,2,mid: ],   SUAVE_data.Cp[0,2,mid: ], linewidth = PP.line_width ,color = PP.colors[0], linestyle = '--' ,marker =  PP.markers[0] ) 
    axis03.plot(Xfoil_data.x[0,2,:mid2],   Xfoil_data.Cp[0,2,:mid2], linewidth = PP.line_width ,color = PP.colors2[0], linestyle = '-' ,marker  =  PP.markers[0]    , label = 'Xfoil, AoA = ' + AoA_tags[0])   
    axis03.plot(Xfoil_data.x[0,2,mid2:],   Xfoil_data.Cp[0,2,mid2:], linewidth = PP.line_width ,color = PP.colors2[0], linestyle = '--' ,marker =  PP.markers[0]   )    
    axis03.plot(SUAVE_data.x[1,2,:mid ],   SUAVE_data.Cp[1,2,:mid ], linewidth = PP.line_width ,color = PP.colors[1], linestyle = '-' ,marker  =  PP.markers[1]   , label = 'SUAVE, AoA = ' + AoA_tags[1])      
    axis03.plot(SUAVE_data.x[1,2,mid: ],   SUAVE_data.Cp[1,2,mid: ], linewidth = PP.line_width ,color = PP.colors[1], linestyle = '--' ,marker =  PP.markers[1] ) 
    axis03.plot(Xfoil_data.x[1,2,:mid2],   Xfoil_data.Cp[1,2,:mid2], linewidth = PP.line_width ,color = PP.colors2[1], linestyle = '-' ,marker  =  PP.markers[1]    , label = 'Xfoil, AoA = ' + AoA_tags[1])   
    axis03.plot(Xfoil_data.x[1,2,mid2:],   Xfoil_data.Cp[1,2,mid2:], linewidth = PP.line_width ,color = PP.colors2[1], linestyle = '--' ,marker =  PP.markers[1]   )              
      

     
    axis01.set_xlabel('x') 
    axis02.set_xlabel('x') 
    axis03.set_xlabel('x')   
 
    axis01.set_ylabel(r'$C_p$')
    axis02.set_ylabel(r'$C_p$') 
    axis03.set_ylabel(r'$C_p$')   
    
    

    axis01.set_ylim([1,-2])
    axis02.set_ylim([1,-2])
    axis03.set_ylim([1,-2])  
    
    # add legends for plotting 
    axis01.legend(loc='upper right') 
    axis02.legend(loc='upper right')
    axis03.legend(loc='upper right')   

    fig01.tight_layout()
    fig02.tight_layout()
    fig03.tight_layout() 
    fig01_name = 'Airfoil_Cp_Distribution_' +Re_tags[0] 
    fig02_name = 'Airfoil_Cp_Distribution_' +Re_tags[1] 
    fig03_name = 'Airfoil_Cp_Distribution_' +Re_tags[2]   

    fig01.savefig(fig01_name + '.pdf')
    fig02.savefig(fig02_name + '.pdf')
    fig03.savefig(fig03_name + '.pdf') 
    
    
    return 



    

def import_NACA_4412_xfoil_results(Re_tags,AoA_tags,Npanels): 

    Xfoil_data = Data()  
    NRe = len(Re_tags)
    NAoA = len(AoA_tags)
    x_pts        = np.zeros((NAoA,NRe,Npanels))
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
                x_pts[i,j,lc]      = float(data_block_1[lc + header_1][10:20].strip())
                y_pts[i,j,lc]      = float(data_block_1[lc + header_1][20:28].strip())         
                Ue_Vinf[i,j,lc]    = float(data_block_1[lc + header_1][28:38].strip())   
                delta_star[i,j,lc] = float(data_block_1[lc + header_1][38:48].strip())   
                theta[i,j,lc]      = float(data_block_1[lc + header_1][48:58].strip()) 
                Cf[i,j,lc]         = float(data_block_1[lc + header_1][58:68].strip())          
                H [i,j,lc]         = float(data_block_1[lc + header_1][68:78].strip())  
                Cp[i,j,lc]         = float(data_block_2[lc + header_2][20:28].strip()) 


    Xfoil_data.x          = np.flip(x_pts,axis = 2)    
    Xfoil_data.y          = np.flip(y_pts   ,axis = 2)   
    Xfoil_data.Cp         = np.flip(Cp,axis = 2)         
    Xfoil_data.Ue_Vinf    = np.flip(Ue_Vinf ,axis = 2)   
    Xfoil_data.delta_star = np.flip(delta_star ,axis = 2)
    Xfoil_data.theta      = np.flip(theta,axis = 2)      
    Xfoil_data.Cf         = np.flip(Cf   ,axis = 2)      
    Xfoil_data.H          = np.flip( H ,axis = 2)  


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

