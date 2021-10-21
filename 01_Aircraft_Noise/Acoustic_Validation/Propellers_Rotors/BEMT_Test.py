
import SUAVE
from SUAVE.Core import Units 
import numpy as np 
import copy 
from SUAVE.Core import Units, Data  
from SUAVE.Methods.Propulsion import propeller_design 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_naca_4series import compute_naca_4series 
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller     
import matplotlib.pyplot as plt
import matplotlib.cm as cm
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():  
    '''
    In this test case, input data (propeller and flight conditions) is obtained from drone
    acoustic tests by Christian in "Auralization of tonal noise .."
    
    A comparison is made between Blade Element Momentum Theory (BEMT) and Blade Element Theory 
    using the BEMT branch. The following shows the breakdown on the converter and the theory
    used 
    
    In this comparison we will use the Rotor network which has both the BEMT and BET
    formulation. A flag (to be removed upon pulling in the branch) is used to force 
    switch between the prediction methods
    '''
    prop_RPM                  = 2000
    rot_RPM                   = 2500
    net                       = Battery_Propeller()   
    net.number_of_engines     = 2   
    
    #------------------------------------------------------------------
    # PROPELLER
    #------------------------------------------------------------------  
   
    prop_a                          = SUAVE.Components.Energy.Converters.Propeller() 
    prop_a.tag                      = "Prop_W_Aifoil"
    prop_a.number_of_blades         = 3
    prop_a.number_of_engines        = 1
    prop_a.freestream_velocity      = 50
    prop_a.tip_radius               = 1.0668
    prop_a.hub_radius               = 0.21336 
    prop_a.design_tip_mach          = 0.65
    prop_a.angular_velocity         = prop_RPM * Units.rpm   
    prop_a.design_Cl                = 0.7
    prop_a.design_altitude          = 1. * Units.km      
    prop_a.airfoil_geometry         = ['NACA_4412.txt','Clark_y.txt']
    prop_a.airfoil_polars           = [['NACA_4412_polar_Re_50000.txt','NACA_4412_polar_Re_100000.txt','NACA_4412_polar_Re_200000.txt',
                                        'NACA_4412_polar_Re_500000.txt','NACA_4412_polar_Re_1000000.txt'],
                                       ['Clark_y_polar_Re_50000.txt','Clark_y_polar_Re_100000.txt','Clark_y_polar_Re_200000.txt',
                                        'Clark_y_polar_Re_500000.txt','Clark_y_polar_Re_1000000.txt']] 
    prop_a.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1]  
    prop_a.design_thrust           = 3054.4809132125697
    prop_a                         = propeller_design(prop_a)  
     
    
    # Design a Rotor with airfoil  geometry defined  
    rot_a                          = SUAVE.Components.Energy.Converters.Rotor() 
    rot_a.tag                      = "Rot_W_Aifoil"
    rot_a.tip_radius               = 2.8 * Units.feet
    rot_a.hub_radius               = 0.35 * Units.feet      
    rot_a.number_of_blades         = 2   
    rot_a.design_tip_mach          = 0.65
    rot_a.number_of_engines        = 12
    rot_a.disc_area                = np.pi*(rot_a.tip_radius**2)        
    rot_a.induced_hover_velocity   = 12.756071638899549
    rot_a.freestream_velocity      = 500. * Units['ft/min']  
    rot_a.angular_velocity         = rot_RPM * Units.rpm
    rot_a.design_Cl                = 0.7
    rot_a.design_altitude          = 20 * Units.feet                            
    rot_a.design_thrust            = 2271.2220451593753 
    rot_a.airfoil_geometry         = ['NACA_4412.txt']
    rot_a.airfoil_polars           = [['NACA_4412_polar_Re_50000.txt','NACA_4412_polar_Re_100000.txt',
                                       'NACA_4412_polar_Re_200000.txt','NACA_4412_polar_Re_500000.txt',
                                       'NACA_4412_polar_Re_1000000.txt']]
    rot_a.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    rot_a                          = propeller_design(rot_a) 
     
    
    # Find the operating conditions
    atmosphere            = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions =  atmosphere.compute_values(rot_a.design_altitude)
    
    V  = 50
    Vr = 500. * Units['ft/min'] 
    
    conditions                                          = Data()
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
    
    conditions_r = copy.deepcopy(conditions)
    conditions.frames.inertial.velocity_vector   = np.array([[V,0,0]])
    conditions_r.frames.inertial.velocity_vector = np.array([[0,Vr,0]])
    
    # Create and attach this propeller 
    prop_a.inputs.omega  = np.array(prop_a.angular_velocity,ndmin=2) 
    rot_a.inputs.omega   = np.array(rot_a.angular_velocity,ndmin=2) 
    
    # propeller with airfoil results 
    F_a, Q_a, P_a, Cplast_a ,output_a , etap_a = prop_a.spin(conditions)   
    va_ind            = output_a.blade_axial_induced_velocity[0,:] 	
    vt_ind            = output_a.blade_tangential_induced_velocity[0,:] 	
    r                 = prop_a.radius_distribution	
    T_distribution    = output_a.blade_thrust_distribution[0,:] 	 	
    dT_dr_distribution= output_a.blade_dT_dr[0,:] 	
    vt                = output_a.blade_tangential_velocity[0,:]   	
    va                = output_a.blade_axial_velocity[0,:]        	
    Q_distribution    = output_a.blade_torque_distribution[0,:]	 
    dQ_dr_distribution= output_a.blade_dQ_dr[0,:] 	
    disc_circulation  = output_a.disc_circulation[0,:,:]	
    T_distribution_2d = output_a.disc_thrust_distribution[0,:,:]  	
    Q_distribution_2d = output_a.disc_torque_distribution[0,:,:]	
    va_2d             = output_a.disc_tangential_velocity[0,:,:] 	
    vt_2d             = output_a.disc_axial_velocity[0,:,:]      	
    theta_2d          = output_a.disc_azimuthal_distribution[0,:,:]	
    r_2d              = output_a.disc_radial_distribution[0,:,:]   
    
    # rotor with airfoil results 
    Fr_a, Qr_a, Pr_a, Cplastr_a ,outputr_a , etapr = rot_a.spin(conditions_r)   
     
    
    print('RPM: ' + str(round(prop_RPM, 3)))
    print('Design Thrust: ' + str(round(prop_a.design_thrust, 3)) + '   BEMT Thrust: ' + str(round(F_a[0][0], 3)))
    print('Design Torque: ' + str(round(prop_a.design_torque, 3)) + '    BEMT Torque: ' + str(round(Q_a[0][0], 3)))
    print('Design Torque: ' + str(round(prop_a.design_power, 3)) + '  BEMT Torque: ' + str(round(P_a[0][0], 3)))
    print('Design CP    : ' + str(round(prop_a.design_power_coefficient[0], 3)) + '      BEMT CP    : ' + str(round(Cplast_a[0][0], 3)))
    print('Design CT    : ' + str(round(prop_a.design_thrust_coefficient[0], 3)) + '      BEMT CT    : ' + str(round(output_a.thrust_coefficient[0][0], 3)) + '\n\n')
    
    print('RPM: ' + str(round(rot_RPM, 3)))
    print('Design Thrust: ' + str(round(rot_a.design_thrust, 3)) + '   BEMT Thrust: ' + str(round(Fr_a[0][0], 3)))
    print('Design Torque: ' + str(round(rot_a.design_torque, 3)) + '     BEMT Torque: ' + str(round(Qr_a[0][0], 3)))
    print('Design Torque: ' + str(round(rot_a.design_power, 3)) + '  BEMT Torque: ' + str(round(Pr_a[0][0], 3)))
    print('Design CP    : ' + str(round(rot_a.design_power_coefficient[0], 3)) + '       BEMT CP    : ' + str(round(Cplastr_a[0][0], 3)))
    print('Design CT    : ' + str(round(rot_a.design_thrust_coefficient[0], 3)) + '      BEMT CT    : ' + str(round(output_a.thrust_coefficient[0][0], 3)))
   
        
    # ----------------------------------------------------------------------------	
    # 2D - Plots  Plots    	
    # ---------------------------------------------------------------------------- 	
    # perpendicular velocity, up Plot 	
    fig1 = plt.figure(1)     	
    fig1.set_size_inches(8, 4)   	
    axes11 = fig1.add_subplot(1,2,1)  	
    axes11.plot(r, va,'rs-'  , label = 'axial BEMT')    	
    axes11.plot(r, vt,'bo-'  , label = 'tangential BEMT')         	
    axes11.set_xlabel('Radial Location')	
    axes11.set_ylabel('Velocity') 	
    axes11.legend(loc='lower right') 	

    axes12 = fig1.add_subplot(1,2,2)           	
    axes12.plot(r, va_ind ,'rs-' ,label = 'axial BEMT')       	
    axes12.plot(r, vt_ind ,'bo-'  ,label = 'tangential BEMT')         	
    axes12.set_xlabel('Radial Location')	
    axes12.set_ylabel('Induced Velocity') 	
    axes12.legend(loc='lower right')    	

    fig2 = plt.figure(2)  	
    fig2.set_size_inches(12, 8)    	
    axes21 = fig2.add_subplot(2,2,1)      	
    axes21.plot(r, T_distribution,'bo-')      	
    axes21.set_xlabel('Radial Location')	
    axes21.set_ylabel('Trust, N')	                           

    rot_a.hub_radius                                        
    axes23 = fig2.add_subplot(2,2,2)      	            
    axes23.plot(r, dT_dr_distribution,'bo-')      
    axes23.set_xlabel('Radial Location')	            
    axes23.set_ylabel('dT/dr') 	                            
 
    axes24 = fig2.add_subplot(2,2,3)  	                    
    axes24.plot(r, Q_distribution,'bo-')     	    
    axes24.set_xlabel('Radial Location')	            
    axes24.set_ylabel('Torque, N-m') 	                         
 
    axes26 = fig2.add_subplot(2,2,4)  	 
    axes26.plot(r, dQ_dr_distribution,'bo-')     	
    axes26.set_xlabel('Radial Location')	
    axes26.set_ylabel('dQ/dr') 
    
    return 
    

def compute_MCA(lamda,zeta,chi,V,R,Cl,B,T,x,a,nu):
    '''taken from propeller design model'''
    tanphit = lamda*(1.+zeta/2.)   # Tangent of the flow angle at the tip
    phit    = np.arctan(tanphit)   # Flow angle at the tip
    tanphi  = tanphit/chi          # Flow angle at every station
    f       = (B/2.)*(1.-chi)/np.sin(phit) 
    F       = (2./np.pi)*np.arccos(np.exp(-f)) #Prandtl momentum loss factor
    phi     = np.arctan(tanphi)  #Flow angle at every station
    
    #Step 3, determine the product Wc, and RE
    G       = F*x*np.cos(phi)*np.sin(phi) #Circulation function
    Wc      = 4.*np.pi*lamda*G*V*R*zeta/(Cl*B)
    Ma      = Wc/a
    RE      = Wc/nu

    #Step 4, determine epsilon and alpha from airfoil data
    
    #This is an atrocious fit of DAE51 data at RE=50k for Cd
    #There is also RE scaling
    Cdval   = (0.108*(Cl**4)-0.2612*(Cl**3)+0.181*(Cl**2)-0.0139*Cl+0.0278)*((50000./RE)**0.2)

    #More Cd scaling from Mach from AA241ab notes for turbulent skin friction
    Tw_Tinf = 1. + 1.78*(Ma**2)
    Tp_Tinf = 1. + 0.035*(Ma**2) + 0.45*(Tw_Tinf-1.)
    Tp      = Tp_Tinf*T
    Rp_Rinf = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
    
    Cd      = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval
    
    alpha   = Cl/(2.*np.pi)
    epsilon = Cd/Cl
    
    #Step 5, change Cl and repeat steps 3 and 4 until epsilon is minimized
    
    #Step 6, determine a and a', and W
    
    a       = (zeta/2.)*(np.cos(phi)**2.)*(1.-epsilon*np.tan(phi))
    aprime  = (zeta/(2.*x))*np.cos(phi)*np.sin(phi)*(1.+epsilon/np.tan(phi))
    W       = V*(1.+a)/np.sin(phi)
    
    #Step 7, compute the chord length and blade twist angle 
    c       = Wc/W
    MCA = c/4. - c[0]/4.   
    
    return MCA

if __name__ == '__main__': 
    main()    
    plt.show()   