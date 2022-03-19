import SUAVE 
from SUAVE.Core import Units , Data  

# Package Imports 
import matplotlib.cm as cm 
import numpy as np 
import matplotlib.pyplot as plt 

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    

    # Add to rotor definition  

    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'legend.fontsize': 22,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 3
    plot_parameters.line_styles      = ['--',':','-',':','--']
    plot_parameters.figure_width     = 10
    plot_parameters.figure_height    = 7
    plot_parameters.marker_size      = 10
    plot_parameters.legend_font_size = 20
    plot_parameters.plot_grid        = True   

    plot_parameters.colors           = [['black','firebrick','darkblue'],
                                        ['dimgray','red','blue'], 
                                        ['darkgray','salmon','deepskyblue']]  
     
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']   
    
        
    test_rotor_planform_function(plot_parameters)   
    return 

# ------------------------------------------------------------------ 
# Test Rotor Planform Function
# ------------------------------------------------------------------ 
def test_rotor_planform_function(PP): 
    c_r     = 0.3
    c_t     = 0.1   
    b       = 1 # span 
    r       = 11
    N       = r-1                      # number of spanwise divisions
    n       = np.linspace(N,0,r)       # vectorize
    theta_n = n*(np.pi/2)/r            # angular stations
    y_n     = b*np.cos(theta_n)        # y locations based on the angular spacing
    eta_n   = np.abs(y_n/b)            # normalized coordinates 
    p       = [0.25,0.5,1,2] 
    markers = ['s','o','P','D','v'] 
    q       = [0.25,0.5,1,1.5]         # q must be positive 
    colors  = ['red','blue','black','green','orange']
    
    fig = plt.figure()
    fig.set_size_inches(PP.figure_width,PP.figure_height)    
    axis = fig.add_subplot(1,1,1) 
    axis.set_ylabel(r'$c_n$')
    axis.set_xlabel(r'$\eta_n$')
    for i in range(len(p)):
        for j in range(len(q)):
            c_n = c_r*(1 - eta_n**p[i])**q[j] + c_t*eta_n
            line_label = 'p = ' + str(p[i]) +  ', q = ' + str(q[j]) 
            axis.plot(y_n,c_n,linestyle = '-', linewidth = PP.line_width, marker = markers[i],markersize = PP.marker_size , color = colors[j], label  = line_label) 
            
    #axis.legend(loc='upper center', ncol = 4 )     
    #axis.set_ylim(0,0.5)  
    fig.tight_layout()    
    fig_name = 'Rotor_Planform_Shape_Function'          
    fig.savefig(fig_name  + '.pdf')              
    return 


if __name__ == '__main__': 
    main() 
    plt.show()