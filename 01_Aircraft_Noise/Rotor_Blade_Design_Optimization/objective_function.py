

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------  
# SUAVE Imports 
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Components.Energy.Networks.Battery_Propeller                                   import Battery_Propeller 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars  import compute_airfoil_polars
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions                                           import Aerodynamics
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry\
     import import_airfoil_geometry      
 
import numpy as np   
from common_functions import evaluate_rotor_aerodynamics , linear_discretize , spline_discretize

# objective function
def aeroacoustic_objective(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                       total_pivots,pivot_points,linear_interp_flag,design_thrust,
                       design_power,design_taper,CL_max,design_FM): 

    # Discretized propeller into stations using linear interpolation
    if linear_interp_flag:
        c    = linear_discretize(x[:total_pivots],chi,pivot_points)     
        beta = linear_discretize(x[total_pivots:],chi,pivot_points)  
    else: 
        c    = spline_discretize(x[:total_pivots],chi,pivot_points)     
        beta = spline_discretize(x[total_pivots:],chi,pivot_points)  
     
    
    # Define Network
    net                                   = Battery_Propeller()
    net.number_of_propeller_engines       = 1              
    prop.twist_distribution               = beta 
    prop.chord_distribution               = c 
    prop.radius_distribution              = R*chi  
    prop.mid_chord_alignment              = c/4. - c[0]/4.
    airfoil_geometry_data                 = prop.airfoil_data          
    t_max                                 = np.take(airfoil_geometry_data.max_thickness,a_loc,axis=0)*c    
    prop.max_thickness_distribution       = t_max 
    prop.mid_chord_aligment               = np.zeros_like(prop.chord_distribution)  
    prop.number_of_airfoil_section_points = 100 
  
    
    net.identical_propellers           = True  
    net.propellers.append(prop)  

    # Atmosheric & Run Conditions          

    theta                   = np.linspace(1,179,21)
    S                       = 4. 
    ctrl_pts                = len(omega) 

    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees)  ,S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2)  ,S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Set up for Propeller Model
    prop.inputs.omega                                      = np.atleast_2d(omega).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector             = np.array([[ V, 0. ,0.]])
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    thrust , torque, power, Ct, Cp, Cl, noise_data , etap = evaluate_rotor_aerodynamics(B,c,beta,R,chi,Rh,a_geo,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V,omega)


    noise_datas = Data()
    noise_datas.propeller = noise_data     
     
    # Store Noise Data 
    noise                                                  = SUAVE.Analyses.Noise.Fidelity_One()
    segment                                                = Segment()
    settings                                               = noise.settings
    conditions.noise.sources.propeller                     = noise_datas
    conditions.noise.number_of_microphones                 = len(theta)
    segment.state.conditions                               = conditions

    # Run Fidelity One   
    propeller_noise  = propeller_mid_fidelity(net,noise_datas,segment,settings)   
    
    return 
 
 