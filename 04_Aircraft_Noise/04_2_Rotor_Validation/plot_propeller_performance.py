# Imports 
import matplotlib.pyplot as plt 

   
def plot_propeller_performance(F, Q, P, Cp , output,prop, col1 = 'bo-', col2 = 'rs-'):
     
    r                 = prop.radius_distribution/prop.tip_radius
    beta              = prop.twist_distribution         
    c                 = prop.chord_distribution         
    max_t             = prop.max_thickness_distribution 
    t_c               = prop.thickness_to_chord          
    T_distribution    = output.blade_thrust_distribution[0,:] 	 
    dT_dr_distribution= output.blade_dT_dr[0,:] 	       	
    Q_distribution    = output.blade_torque_distribution[0,:]	 
    dQ_dr_distribution= output.blade_dQ_dr[0,:] 	 
    
    # ----------------------------------------------------------------------------	
    # 2D - Plots  Plots    	
    # ---------------------------------------------------------------------------- 	     	

    fig2 = plt.figure('performance')  	
    fig2.set_size_inches(12, 8)    	
    axes21 = fig2.add_subplot(2,2,1)      	
    axes21.plot(r, T_distribution,col1)      	
    axes21.set_xlabel('Radial Location')	
    axes21.set_ylabel('Trust, N')	                   
 
    axes23 = fig2.add_subplot(2,2,2)      	            
    axes23.plot(r, dT_dr_distribution,col1)      
    axes23.set_xlabel('Radial Location')	            
    axes23.set_ylabel('dT/dr') 	                            
 
    axes24 = fig2.add_subplot(2,2,3)  	                    
    axes24.plot(r, Q_distribution,col1)     	    
    axes24.set_xlabel('Radial Location')	            
    axes24.set_ylabel('Torque, N-m') 	                           
 
    axes26 = fig2.add_subplot(2,2,4)  	 
    axes26.plot(r, dQ_dr_distribution,col1)     	
    axes26.set_xlabel('Radial Location')	
    axes26.set_ylabel('dQ/dr') 
    
    
    fig3 = plt.figure('blade properties')     	
    fig3.set_size_inches(8, 8)   	
    axes31 = fig3.add_subplot(2,2,1)       	
    axes31.plot(r, beta*57.5,col1)      	
    axes31.set_xlabel('Radial Location')	
    axes31.set_ylabel('Twist')	 
        
    axes32 = fig3.add_subplot(2,2,2)      	            
    axes32.plot(r,  c,col1)     
    axes32.set_xlabel('Radial Location')	            
    axes32.set_ylabel('Chord') 	                            
         
    axes33 = fig3.add_subplot(2,2,3)      	            
    axes33.plot(r, t_c,col1)      
    axes33.set_xlabel('Radial Location')	            
    axes33.set_ylabel('thickness_to_chord') 	                            
        
    axes34 = fig3.add_subplot(2,2,4)  	                    
    axes34.plot(r, max_t,col1)     	    
    axes34.set_xlabel('Radial Location')	            
    axes34.set_ylabel('max thickness') 	                   
    
    return 