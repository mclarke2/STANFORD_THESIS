
import SUAVE  
from SUAVE.Core import Units, Data
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.dbA_noise                    import A_weighting   

# Python Imports 
import time 
import numpy as np  
import scipy as sp
from scipy.special import jv 
from scipy.special import fresnel
import matplotlib.pyplot as plt  
import matplotlib.cm as cm 

# import propeller/rotors geometries  
import sys
sys.path.append('../../XX_Supplementary')

from Propellers_Rotors.design_SR2_4_blade_prop import design_SR2_4_blade_prop 
from Propellers_Rotors.design_SR2_8_blade_prop import design_SR2_8_blade_prop
from Propellers_Rotors.design_SR7_8_blade_prop import design_SR7_8_blade_prop 
from Propellers_Rotors.design_BO_105_prop      import design_BO_105_prop 
from Propellers_Rotors.design_Hubbard_prop     import design_Hubbard_prop     
from Propellers_Rotors.design_F8745D4_prop     import design_F8745D4_prop     
from Propellers_Rotors.design_DJI_9_4x5_prop   import design_DJI_9_4x5_prop   
from Propellers_Rotors.design_APC_11x_4_7_prop import design_APC_11x_4_7_prop   
from Propellers_Rotors.design_APC_11x45_prop   import design_APC_11x45_prop   
from Propellers_Rotors.design_APC_10x7_prop    import design_APC_10x7_prop 
from Propellers_Rotors.design_APC_11x8_prop    import design_APC_11x8_prop
from Propellers_Rotors.design_SR1_prop         import design_SR1_prop 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():  


    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
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
    plot_parameters.colors           = cm.viridis(np.linspace(0,1,5))     
    plot_parameters.lw               = 2                              # line_width               
    plot_parameters.m                = 14                             # markersize               
    plot_parameters.legend_font      = 20                             # legend_font_size         
    plot_parameters.Slc              = ['black','dimgray','silver' ]  # SUAVE_line_colors        
    plot_parameters.Slm              = '^'                            # SUAVE_line_markers       
    plot_parameters.Sls              = '-'                            # SUAVE_line_styles        
    plot_parameters.Elc              = ['firebrick','red','tomato']  # Experimental_line_colors 
    plot_parameters.Elm              = 's'                            # Experimental_line_markers
    plot_parameters.Els              = '-'                            # Experimental_line_styles 
    plot_parameters.Rlc              = ['mediumblue','blue','cyan']   # Ref_Code_line_colors     
    plot_parameters.Rlm              = 'o'                            # Ref_Code_line_markers    
    plot_parameters.Rls              = '--'                           # Ref_Code_line_styles     

    ti                = time.time()      


    f          = np.linspace(1E2,1E5,1000) 
    SPL        = np.zeros(len(f))
    A_f        = A_weighting(SPL,f)
    
    name = 'A_weighting'
    fig = plt.figure(name)    
    fig.set_size_inches(plot_parameters.figure_width,plot_parameters.figure_height)  
    axis = fig.add_subplot(1,1,1)     
    axis.semilogx(f,A_f,color = 'black', linewidth = 3 )   
    axis.set_ylabel('Attenuation (dB)') 
    axis.set_xlabel('Frequency (Hz)')  
    fig.tight_layout()
    axis.grid(True) 
    axis.grid(b=True, which='major', color='k', linestyle='-')  
    axis.grid(b=True, which='minor', color='grey', linestyle=':')    
    axis.minorticks_on()
    fig.savefig(name + 'png' )
           
    return 



if __name__ == '__main__': 
    main()    
    plt.show()   
 
