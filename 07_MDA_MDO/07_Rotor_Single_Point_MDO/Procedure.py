# Procedure.py 
# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------     
# SUAVE imports 
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Analyses.Process                                                               import Process   
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions                                           import Aerodynamics
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry import import_airfoil_geometry 
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics                              import Aerodynamics 

# Python package imports 
import numpy as np
from numpy import linalg as LA

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------     
def set_up(): 
    procedure                   = Process()
    procedure.modify_rotor      = modify_blade_geometry
    procedure.post_process      = post_process    
    return procedure  

# ----------------------------------------------------------------------      
#   Modify Rotor Geoemtry
# ----------------------------------------------------------------------   
def modify_blade_geometry(nexus):  
    # Pull out the vehicles
    vehicle = nexus.vehicle_configurations.rotor_testbench 
    rotor   = vehicle.networks.battery_propeller.lift_rotors.rotor 
    
    # Update geometry of blade
    c       = updated_blade_geometry(rotor.radius_distribution ,rotor.chord_r,rotor.chord_p,rotor.chord_q,rotor.chord_t)     
    beta    = updated_blade_geometry(rotor.radius_distribution ,rotor.twist_r,rotor.twist_p,rotor.twist_q,rotor.twist_t)   
    
    rotor.chord_distribution               = c
    rotor.twist_distribution               = beta  
    rotor.mid_chord_alignment              = c/4. - c[0]/4.
    airfoil_geometry_data                  = import_airfoil_geometry(rotor.airfoil_geometry) 
    t_max                                  = np.take(airfoil_geometry_data.max_thickness,rotor.airfoil_polar_stations,axis=0)*c
    rotor.airfoil_data                     = airfoil_geometry_data
    rotor.max_thickness_distribution       = t_max   
     
    # diff the new data
    vehicle.store_diff() 
    
    return nexus    

def updated_blade_geometry(chi,c_r,p,q,c_t):   
    b       = chi[-1]
    r       = len(chi)                
    n       = np.linspace(r-1,0,r)          
    theta_n = n*(np.pi/2)/r              
    y_n     = b*np.cos(theta_n)          
    eta_n   = np.abs(y_n/b)            
    x_cos   = c_r*(1 - eta_n**p)**q + c_t*eta_n 
    x_lin   = np.interp(chi,eta_n, x_cos)  
    return x_lin 

def post_process(nexus):  
    summary       = nexus.summary 
    vehicle       = nexus.vehicle_configurations.rotor_testbench  
    lift_rotors   = vehicle.networks.battery_propeller.lift_rotors
    
    # -------------------------------------------------------
    # RUN AEROACOUSTICS MODELS
    # -------------------------------------------------------    
    # unpack rotor properties 
    rotor         = lift_rotors.rotor 
    c             = rotor.chord_distribution 
    omega         = rotor.angular_velocity 
    V             = rotor.freestream_velocity   
    alt           = rotor.design_altitude
    alpha         = rotor.optimization_parameters.aeroacoustic_weight
    epsilon       = rotor.optimization_parameters.slack_constaint 
    ideal_SPL     = rotor.optimization_parameters.ideal_SPL  
    
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  

    # Define microphone locations
    theta     = np.array([45,90,135])*Units.degrees + 1E-1
    S         = 10. 
    ctrl_pts  = 1 
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees)  ,S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2)  ,S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Define run conditions 
    rotor.inputs.omega                               = np.atleast_2d(omega).T
    conditions                                       = Aerodynamics()   
    conditions.freestream.density                    = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity          = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound             = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector       = np.array([[V, 0. ,0.]])
    conditions.propulsion.throttle                   = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial     = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    thrust , torque, power, Cp  , noise_data , etap  = rotor.spin(conditions) 

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations      = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack          = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                          = Segment() 
    segment.state.conditions                         = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 

    # Store Noise Data 
    noise                                            = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                         = noise.settings   
    num_mic                                          = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones           = num_mic  
    acoustic_outputs                                 = Data()
    acoustic_outputs.propeller                       = noise_data 
    
    # Run noise model    
    if alpha != 1: 
        propeller_noise  = propeller_mid_fidelity(lift_rotors,acoustic_outputs,segment,settings)   
        mean_SPL         = np.mean(propeller_noise.SPL_dBA) 
        Acoustic_Metric  = mean_SPL 
    else:
        Acoustic_Metric  = 0 
        mean_SPL         = 0
   
    # -------------------------------------------------------
    # CONTRAINTS
    # -------------------------------------------------------
    # thrust/power constraint
    if rotor.design_thrust == None:
        summary.thrust_power_residual = epsilon*rotor.design_power - abs(power[0][0] - rotor.design_power)
        ideal_aero                    = (rotor.design_power/V)
        Aerodynamic_Metric            = thrust[0][0]
    else: 
        summary.thrust_power_residual = epsilon*rotor.design_thrust - abs(thrust[0][0] - rotor.design_thrust)
        ideal_aero                    = rotor.design_thrust*V
        Aerodynamic_Metric            = power[0][0]     

    # q to p ratios 
    summary.chord_p_to_q_ratio = rotor.chord_p/rotor.chord_q
    summary.twist_p_to_q_ratio = rotor.twist_p/rotor.twist_q
    
    # Cl constraint  
    mean_CL = np.mean(noise_data.lift_coefficient[0])
    
    # blade taper consraint 
    blade_taper = c[-1]/c[0]
    summary.blade_taper_constraint_1  = blade_taper 
    summary.blade_taper_constraint_2  = blade_taper

    # -------------------------------------------------------
    # OBJECTIVE FUNCTION
    # -------------------------------------------------------     
    summary.Aero_Acoustic_Obj =  LA.norm((Aerodynamic_Metric - ideal_aero)/ideal_aero)*alpha \
                                + LA.norm((Acoustic_Metric - ideal_SPL)/ideal_SPL)*(1-alpha)
        
    # -------------------------------------------------------
    # PRINT ITERATION PERFOMRMANCE
    # -------------------------------------------------------                
    print("Aero_Acoustic_Obj       : " + str(summary.Aero_Acoustic_Obj)) 
    if rotor.design_thrust == None: 
        print("Power                   : " + str(power[0][0])) 
    if rotor.design_power == None: 
        print("Thrust                  : " + str(thrust[0][0]))   
    print("Average SPL             : " + str(mean_SPL))  
    print("Thrust/Power Residual   : " + str(summary.thrust_power_residual)) 
    print("Blade Taper             : " + str(blade_taper))
    print("Mean CL                 : " + str(mean_CL))  
    print("\n\n") 
    
    return nexus   