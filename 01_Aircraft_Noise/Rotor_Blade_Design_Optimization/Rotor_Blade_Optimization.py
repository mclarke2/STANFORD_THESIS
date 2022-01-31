# Optimize.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------  
import SUAVE
from SUAVE.Core import Units, Data 
import numpy as np
import Vehicles 
import Procedure 
from SUAVE.Analyses.Mission.Segments.Segment                                              import Segment 
from SUAVE.Methods.Noise.Fidelity_One.Propeller.propeller_mid_fidelity                    import propeller_mid_fidelity
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from SUAVE.Optimization       import Nexus      
import numpy as np 
import scipy as sp 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars \
     import compute_airfoil_polars 
from SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics import Aerodynamics 
import time 

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def low_noise_rotor_design(rotor,
                           number_of_stations       = 20, 
                           number_of_airfoil_section_points = 100,
                           solver_name              = 'differential_evolution',  
                           print_iter               = True):  
    
    
    N             = number_of_stations       # this number determines the discretization of the propeller into stations 
    B             = rotor.number_of_blades
    R             = rotor.tip_radius
    Rh            = rotor.hub_radius 
    design_thrust = rotor.design_thrust
    design_power  = rotor.design_power
    a_geo         = rotor.airfoil_geometry
    a_pol         = rotor.airfoil_polars        
    a_loc         = rotor.airfoil_polar_stations 
    
    
    # Determine target values 
    if (design_thrust == None) and (design_power== None):
        raise AssertionError('Specify either design thrust or design power!')
    
    elif (design_thrust!= None) and (design_power!= None):
        raise AssertionError('Specify either design thrust or design power!')
    
    if rotor.rotation == None:
        rotor.rotation = list(np.ones(int(B))) 
        
    if  a_pol != None and a_loc != None:
        if len(a_loc) != N:
            raise AssertionError('\nDimension of airfoil sections must be equal to number of stations on rotor')
        # compute airfoil polars for airfoils 
        airfoil_polars  = compute_airfoil_polars(a_geo, a_pol)  
        cl_sur = airfoil_polars.lift_coefficient_surrogates 
        cd_sur = airfoil_polars.drag_coefficient_surrogates  
    else:
        raise AssertionError('\nDefine rotor airfoil') 
 
    chi0    = Rh/R # Where the rotor blade actually starts
    chi     = np.linspace(chi0,1,N+1) # Vector of nondimensional radii
    chi     = chi[0:N]
    
    
    airfoil_geometry_data                  = import_airfoil_geometry(rotor.airfoil_geometry)
    t_c                                    = np.take(airfoil_geometry_data.thickness_to_chord,a_loc,axis=0) 
    rotor.number_of_blades                 = int(B)  
    rotor.thickness_to_chord               = t_c  
    rotor.radius_distribution              = chi
    rotor.airfoil_cl_surrogates            = cl_sur
    rotor.airfoil_cd_surrogates            = cd_sur 
    rotor.airfoil_flag                     = True  
    rotor.minimum_taper_constraint         = 0.3     
    rotor.chord_r                          = 0.1*R     
    rotor.chord_p                          = 1.0       
    rotor.chord_q                          = 0.5       
    rotor.chord_t                          = 0.05*R    
    rotor.twist_r                          = np.pi/6   
    rotor.twist_p                          = 1.0       
    rotor.twist_q                          = 0.5       
    rotor.twist_t                          = np.pi/10  
    rotor.number_of_airfoil_section_points = number_of_airfoil_section_points
    
    
    
    ti = time.time()   
    optimization_problem = low_noise_rotor_optimization_setup(rotor)
    output = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, sense_step = 1E-4, tolerance = 1E-3)  
    print (output) 
     
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Rotor Otimization Simulation Time: ' + str(elapsed_time))   

    rotor = set_optimized_rotor_planform(rotor,optimization_problem)
    
    return rotor
  
def low_noise_rotor_optimization_setup(rotor): 
    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------  
    R = rotor.tip_radius  
    inputs = []
    inputs.append([ 'chord_r'    , 0.1*R     , 0.05*R , 0.2*R    , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_p'    , 2         , 0.25   , 2.0      , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_q'    , 1         , 0.25   , 1.5      , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_t'    , 0.05*R    , 0.05*R , 0.2*R    , 1.0     ,  1*Units.less])  
    inputs.append([ 'twist_r'    , np.pi/6   ,  0     , np.pi/4  , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_p'    , 1         , 0.25   , 2.0      , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_q'    , 0.5       , 0.25   , 1.5      , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_t'    , np.pi/10  ,  0     , np.pi/4  , 1.0     ,  1*Units.less]) 
    problem.inputs = np.array(inputs,dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------- 
    problem.objective = np.array([ 
                               # [ tag         , scaling, units ]
                                 [  'Aero_Acoustic_Obj'  ,  1.0   ,    1*Units.less] 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------  
    constraints = [] 
    constraints.append([ 'thrust_power_residual'    ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]), 
    constraints.append([ 'blade_taper_constraint_1'     ,  '>'  ,  0.3 ,   1.0   , 1*Units.less]), 
    constraints.append([ 'blade_taper_constraint_2'     ,  '<'  ,  0.8 ,   1.0   , 1*Units.less]),  
    #constraints.append([ 'design_cl_residual'       ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]), 
    constraints.append([ 'chord_p_to_q_ratio'       ,  '>'  ,  0.5,   1.0    , 1*Units.less])   
    constraints.append([ 'twist_p_to_q_ratio'       ,  '>'  ,  0.5,   1.0    , 1*Units.less])     
    #constraints.append([ 'blade_solidity_residual'  ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),  
    #constraints.append([ 'figure_of_merit_residual' ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]) 
    problem.constraints =  np.array(constraints,dtype=object)                                      
    # -------------------------------------------------------------------
    #  Aliases
    # ------------------------------------------------------------------- 
    aliases = []
    aliases.append([ 'chord_r'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_r' ])
    aliases.append([ 'chord_p'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_p' ])
    aliases.append([ 'chord_q'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_q' ])
    aliases.append([ 'chord_t'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_t' ]) 
    aliases.append([ 'twist_r'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_r' ])
    aliases.append([ 'twist_p'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_p' ])
    aliases.append([ 'twist_q'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_q' ])
    aliases.append([ 'twist_t'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_t' ]) 
    aliases.append([ 'Aero_Acoustic_Obj'         , 'summary.Aero_Acoustic_Obj'       ])  
    aliases.append([ 'thrust_power_residual'     , 'summary.thrust_power_residual'   ]) 
    aliases.append([ 'blade_taper_constraint_1'      , 'summary.blade_taper_constraint_1'    ])  
    aliases.append([ 'blade_taper_constraint_2'      , 'summary.blade_taper_constraint_2'    ])  
    #aliases.append([ 'design_cl_residual'        , 'summary.design_cl_residual'      ]) 
    aliases.append([ 'chord_p_to_q_ratio'        , 'summary.chord_p_to_q_ratio'    ])  
    aliases.append([ 'twist_p_to_q_ratio'        , 'summary.twist_p_to_q_ratio'    ])     
    #aliases.append([ 'blade_solidity_residual'   , 'summary.blade_solidity_residual' ])
    #aliases.append([ 'figure_of_merit_residual'  , 'summary.figure_of_merit_residual']) 
    
    problem.aliases = aliases
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Vehicles.rotor_blade_setup(rotor)
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = None 
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = None
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Procedure.set_up()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()     
    return nexus  
 
def set_optimized_rotor_planform(rotor,optimization_problem):
    r                        = rotor.radius_distribution
    R                        = rotor.tip_radius     
    chi                      = r/R 
    B                        = rotor.number_of_blades 
    omega                    = rotor.angular_velocity
    V                        = rotor.freestream_velocity  
    alt                      = rotor.design_altitude
    network                  = optimization_problem.vehicle_configurations.rotor_testbench.networks.battery_propeller
    rotor_opt                = network.lift_rotors.rotor 
    rotor.chord_distribution = rotor_opt.chord_distribution
    rotor.twist_distribution = rotor_opt.twist_distribution
    c                        = rotor.chord_distribution
    a_geo                    = rotor.airfoil_geometry 
    a_loc                    = rotor.airfoil_polar_stations  
  
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1 

    # Run Conditions     
    theta                   = np.array([45,90,135])*Units.degrees + 1E-1
    S                       = 10.  

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

    propeller_noise   = propeller_mid_fidelity(network.lift_rotors,acoustic_outputs,segment,settings)   
    mean_SPL          =  np.mean(propeller_noise.SPL_dBA) 
    
    if rotor.design_power == None: 
        rotor.design_power = power[0][0]
    if rotor.design_thrust == None: 
        rotor.design_thrust = thrust[0][0]
        
    design_torque = power[0][0]/omega
    
    # blade solidity
    r          = chi*R                    # Radial coordinate   
    blade_area = sp.integrate.cumtrapz(B*c, r-r[0])
    sigma      = blade_area[-1]/(np.pi*R**2)   
    
    MCA    = c/4. - c[0]/4.
    airfoil_geometry_data = import_airfoil_geometry(a_geo) 
    t_max = np.take(airfoil_geometry_data.max_thickness,a_loc,axis=0)*c 
    t_c   =  np.take(airfoil_geometry_data.thickness_to_chord,a_loc,axis=0)  
    
    rotor.design_torque              = design_torque
    rotor.max_thickness_distribution = t_max 
    rotor.radius_distribution        = r 
    rotor.number_of_blades           = int(B) 
    rotor.design_power_coefficient   = Cp[0][0] 
    rotor.design_thrust_coefficient  = noise_data.thrust_coefficient[0][0] 
    rotor.mid_chord_alignment        = MCA
    rotor.thickness_to_chord         = t_c 
    rotor.design_SPL_dBA             = mean_SPL
    rotor.blade_solidity             = sigma   
    rotor.airfoil_flag               = True    
    
    return rotor

if __name__ == '__main__':
    low_noise_rotor_design()
