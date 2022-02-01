# Procedure.py 
# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------     
import SUAVE
from SUAVE.Core import Units, Data
import numpy as np
import scipy as sp
from scipy.interpolate import CubicSpline
from SUAVE.Analyses.Process import Process   
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
from SUAVE.Analyses.Mission.Segments.Conditions                                           import Aerodynamics
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry 
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics import Aerodynamics 

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------    

def set_up(): 
    procedure                   = Process()
    procedure.modify_rotor  = update_geometry
    procedure.post_process      = post_process   
        
    return procedure  
# ----------------------------------------------------------------------      
#   Modify Vehicle 
# ----------------------------------------------------------------------  

def update_geometry(nexus): 
    '''
    This function takes the updated design variables and modifies the aircraft 
    '''
    # Pull out the vehicles
    vehicle = nexus.vehicle_configurations.rotor_testbench
    
    rotor        = vehicle.networks.battery_propeller.lift_rotors.rotor
    chi          = rotor.radius_distribution
    pivot_points = rotor.radial_pivots[1:-1] 
     
    if rotor.linear_interp_flag:
        c    = linear_discretize(rotor.chord_pivots,chi,pivot_points)     
        beta = linear_discretize(rotor.twist_pivots,chi,pivot_points)  
    else: 
        c    = spline_discretize(rotor.chord_pivots,chi,pivot_points)     
        beta = spline_discretize(rotor.twist_pivots,chi,pivot_points) 
    
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
  
def post_process(nexus):  
    summary       = nexus.summary 
    vehicle       = nexus.vehicle_configurations.rotor_testbench  
    network       = vehicle.networks.battery_propeller 
    
    # unpack rotor properties 
    rotor         = network.lift_rotors.rotor  
    alpha         = rotor.noise_aero_acoustic_obj_weight  
    epsilon       = rotor.optimization_slack_constaint
    c             = rotor.chord_distribution 
    omega         = rotor.angular_velocity
    R             = rotor.tip_radius 
    V             = rotor.freestream_velocity  
    alt           = rotor.design_altitude
    
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  

    # Run Conditions     
    theta                   = np.linspace(30,150,5) + 1E-1
    S                       = 50. 
    ctrl_pts                = 1

    # microphone locations
    positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            positions[i][:] = [-S*np.cos(theta[i]*Units.degrees)  ,S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2)  ,S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Set up for Propeller Model
    rotor.inputs.omega                                     = np.atleast_2d(omega).T
    conditions                                             = Aerodynamics()   
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * rho
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * mu
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T 
    conditions.frames.inertial.velocity_vector             = np.array([[V, 0. ,0.]])
    conditions.propulsion.throttle                         = np.ones((ctrl_pts,1))*1.0
    conditions.frames.body.transform_to_inertial           = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])

    # Run Propeller model 
    thrust , torque, power, Cp  , noise_data , etap        = rotor.spin(conditions) 

    # Prepare Inputs for Noise Model  
    conditions.noise.total_microphone_locations            = np.repeat(positions[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angle_of_attack                = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                                = Segment() 
    segment.state.conditions                               = conditions
    segment.state.conditions.expand_rows(ctrl_pts) 

    # Store Noise Data 
    noise                                                  = SUAVE.Analyses.Noise.Fidelity_One() 
    settings                                               = noise.settings   
    num_mic                                                = len(conditions.noise.total_microphone_locations[0])  
    conditions.noise.number_of_microphones                 = num_mic  
    acoustic_outputs                                       = Data()
    acoustic_outputs.propeller                             = noise_data 
    
    # Run noise model    
    if alpha != 1: 
        propeller_noise   = propeller_mid_fidelity(network,acoustic_outputs,segment,settings,source = 'lift_rotors')   
        Acoustic_Metric   = np.max(np.nan_to_num(propeller_noise.SPL_dBA))/65
    else:
        Acoustic_Metric  = 0 
 
    # thrust/power constraint
    if rotor.design_thrust == None:
        summary.thrust_power_residual = epsilon*rotor.design_power - abs(power[0][0] - rotor.design_power)
        Aerodynamic_Metric = thrust[0][0]/1000  
    if rotor.design_power == None:
        summary.thrust_power_residual = epsilon*rotor.design_thrust - abs(thrust[0][0] - rotor.design_thrust)
        Aerodynamic_Metric = power[0][0]/100000  
        
    # Cl constraint  
    summary.design_cl_residual  =  epsilon*rotor.design_Cl - abs(rotor.design_Cl - np.mean(noise_data.lift_coefficient))

    # blade taper consraint 
    blade_taper = c[-1]/c[0]
    summary.blade_taper_residual  = blade_taper - rotor.minimum_taper_constraint

    # figure of merit constaint  
    Area   = np.pi*(R**2)
    FM     = (thrust[0][0]*np.sqrt(thrust[0][0]/2*rho*Area))/power
    summary.figure_of_merit_residual  = epsilon*rotor.design_figure_of_merit  - abs(FM[0][0] - rotor.design_figure_of_merit)   
    
    # monotimic increasing twist and chord constaints
    summary.c0_min_c1 = rotor.chord_pivots[0] - rotor.chord_pivots[1]
    summary.c1_min_c2 = rotor.chord_pivots[1] - rotor.chord_pivots[2]
    summary.c2_min_c3 = rotor.chord_pivots[2] - rotor.chord_pivots[3]
    summary.t0_min_t1 = rotor.twist_pivots[0] - rotor.twist_pivots[1]
    summary.t1_min_t2 = rotor.twist_pivots[1] - rotor.twist_pivots[2]
    summary.t2_min_t3 = rotor.twist_pivots[2] - rotor.twist_pivots[3] 
    if rotor.blade_optimization_pivots == 5:
        summary.c3_min_c4 = rotor.chord_pivots[3] - rotor.chord_pivots[4]
        summary.t3_min_t4 = rotor.twist_pivots[3] - rotor.twist_pivots[4]  

    # print 
    summary.Aero_Acoustic_Obj =  Aerodynamic_Metric*alpha + Acoustic_Metric *(1-alpha)  
    print("Aero_Acoustic_Obj       : " + str(summary.Aero_Acoustic_Obj)) 
    if rotor.design_thrust == None: 
        print("Power                   : " + str(power[0][0])) 
    if rotor.design_power == None: 
        print("Thrust                  : " + str(thrust[0][0]))   
    print("Average SPL             : " + str(Acoustic_Metric)) 
    print("c0 - c1                 : " + str(summary.c0_min_c1))
    print("c1 - c2                 : " + str(summary.c1_min_c2))
    print("c2 - c3                 : " + str(summary.c2_min_c3))
    if rotor.blade_optimization_pivots == 5:
        print("c3 - c4                 : " + str(summary.c3_min_c4))
    print("t0 - t1                 : " + str(summary.t0_min_t1))
    print("t1 - t2                 : " + str(summary.t1_min_t2))
    print("t2 - t3                 : " + str(summary.t2_min_t3))
    if rotor.blade_optimization_pivots == 5:
        print("t3 - t4                 : " + str(summary.t3_min_t4)) 
    print("Thrust/Power Residual   : " + str(summary.thrust_power_residual)) 
    print("Blade Taper Residual    : " + str(summary.blade_taper_residual) )
    print("Design CL Residual      : " + str(summary.design_cl_residual))  
    #print("Figure of Merit Residual: " + str(summary.figure_of_merit_residual)) 
    print("\n\n") 
    
    return nexus  


def linear_discretize(x_pivs,chi,pivot_points):
    
    chi_pivs       = np.zeros(len(x_pivs))
    chi_pivs[0]    = chi[0]
    chi_pivs[-1]   = chi[-1]
    locations      = np.array(pivot_points)*(chi[-1]-chi[0]) + chi[0]
    
    # vectorize 
    chi_2d = np.repeat(np.atleast_2d(chi).T,len(pivot_points),axis = 1)
    pp_2d  = np.repeat(np.atleast_2d(locations),len(chi),axis = 0)
    idxs   = (np.abs(chi_2d - pp_2d)).argmin(axis = 0) 
    
    chi_pivs[1:-1] = chi[idxs]
    
    x_bar  = np.interp(chi,chi_pivs, x_pivs) 
    
    return x_bar 


def spline_discretize(x_pivs,chi,pivot_points):
    chi_pivs       = np.zeros(len(x_pivs))
    chi_pivs[0]    = chi[0]
    chi_pivs[-1]   = chi[-1]
    locations      = np.array(pivot_points)*(chi[-1]-chi[0]) + chi[0]
    
    # vectorize 
    chi_2d = np.repeat(np.atleast_2d(chi).T,len(pivot_points),axis = 1)
    pp_2d  = np.repeat(np.atleast_2d(locations),len(chi),axis = 0)
    idxs   = (np.abs(chi_2d - pp_2d)).argmin(axis = 0) 
    
    chi_pivs[1:-1] = chi[idxs]
    
    fspline = CubicSpline(chi_pivs,x_pivs)
    x_bar   = fspline(chi)
    
    return x_bar

