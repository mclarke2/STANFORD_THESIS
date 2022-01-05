# Optimize.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------  
import SUAVE
from SUAVE.Core import Units, Data 
import numpy as np
import Vehicles 
import Procedure
from Procedure import linear_discretize, spline_discretize
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from SUAVE.Optimization       import Nexus      
import numpy as np 
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
    
    
    airfoil_geometry_data           = import_airfoil_geometry(rotor.airfoil_geometry, npoints = rotor.number_of_airfoil_section_points)
    t_c                             = np.take(airfoil_geometry_data.thickness_to_chord,a_loc,axis=0) 
    rotor.number_of_blades          = int(B)  
    rotor.thickness_to_chord        = t_c  
    rotor.radius_distribution       = chi
    rotor.airfoil_cl_surrogates     = cl_sur
    rotor.airfoil_cd_surrogates     = cd_sur 
    rotor.airfoil_flag              = True 
    
    ti = time.time()   
    optimization_problem = low_noise_rotor_optimization_setup(rotor)
    output = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, sense_step = 5E-2, tolerance = 1E-2)  
    print (output) 
    
    if solver_name == 'SLSQP':
        optimized_varables = output.x
    elif solver_name == 'differential_evolution':
        optimized_varables = output.x
         
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Rotor Otimization Simulation Time: ' + str(elapsed_time))   

    rotor = set_optimized_rotor_planform(rotor,optimized_varables)
     
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
    inputs.append([ 'chord_0'    , rotor.chord_pivots[0]  , 0.05*R , 0.3*R    , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_1'    , rotor.chord_pivots[1]  , 0.05*R , 0.3*R    , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_2'    , rotor.chord_pivots[2]  , 0.05*R , 0.3*R    , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_3'    , rotor.chord_pivots[3]  , 0.05*R , 0.3*R    , 1.0     ,  1*Units.less])
    if rotor.blade_optimization_pivots == 5:
        inputs.append([ 'chord_4', rotor.chord_pivots[4]  , 0.05*R , 0.3*R    , 1.0     ,  1*Units.less]) 
    inputs.append([ 'twist_0'    , rotor.twist_pivots[0]  ,  0     , np.pi/4  , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_1'    , rotor.twist_pivots[1]  ,  0     , np.pi/4  , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_2'    , rotor.twist_pivots[2]  ,  0     , np.pi/4  , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_3'    , rotor.twist_pivots[3]  ,  0     , np.pi/4  , 1.0     ,  1*Units.less])
    if rotor.blade_optimization_pivots == 5:
        inputs.append([ 'twist_4', rotor.twist_pivots[4]  ,  0     , np.pi/4  , 1.0     ,  1*Units.less])  
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
    constraints.append([ 'c0_min_c1'                ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),
    constraints.append([ 'c1_min_c2'                ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),
    constraints.append([ 'c2_min_c3'                ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),
    if rotor.blade_optimization_pivots == 5:
        constraints.append([ 'c3_min_c4'            ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),
    constraints.append([ 't0_min_t1'                ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]), 
    constraints.append([ 't1_min_t2'                ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]), 
    constraints.append([ 't2_min_t3'                ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),
    if rotor.blade_optimization_pivots == 5:
        constraints.append([ 't3_min_t4'            ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]), 
    constraints.append([ 'thrust_power_residual'    ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]), 
    constraints.append([ 'blade_taper_residual'     ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),   
    constraints.append([ 'design_cl_residual'       ,  '>'  ,  0.0 ,   1.0   , 1*Units.less])
    #constraints.append([ 'blade_solidity_residual'  ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]),  
    #constraints.append([ 'figure_of_merit_residual' ,  '>'  ,  0.0 ,   1.0   , 1*Units.less]) 
    problem.constraints =  np.array(constraints,dtype=object)                                      
    # -------------------------------------------------------------------
    #  Aliases
    # ------------------------------------------------------------------- 
    aliases = []
    aliases.append([ 'chord_0'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_pivots[0]' ])
    aliases.append([ 'chord_1'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_pivots[1]' ])
    aliases.append([ 'chord_2'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_pivots[2]' ])
    aliases.append([ 'chord_3'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_pivots[3]' ])
    if rotor.blade_optimization_pivots == 5:
        aliases.append([ 'chord_4'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_pivots[4]' ])
    aliases.append([ 'twist_0'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_pivots[0]' ])
    aliases.append([ 'twist_1'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_pivots[1]' ])
    aliases.append([ 'twist_2'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_pivots[2]' ])
    aliases.append([ 'twist_3'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_pivots[3]' ])
    if rotor.blade_optimization_pivots == 5:
        aliases.append([ 'twist_4'                   , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_pivots[4]' ]) 
    aliases.append([ 'Aero_Acoustic_Obj'         , 'summary.Aero_Acoustic_Obj'       ])  
    aliases.append([ 'thrust_power_residual'     , 'summary.thrust_power_residual'   ]) 
    aliases.append([ 'blade_taper_residual'      , 'summary.blade_taper_residual'    ])  
    aliases.append([ 'design_cl_residual'        , 'summary.design_cl_residual'      ]) 
    aliases.append([ 'c0_min_c1'                 , 'summary.c0_min_c1'       ]) 
    aliases.append([ 'c1_min_c2'                 , 'summary.c1_min_c2'       ])  
    aliases.append([ 'c2_min_c3'                 , 'summary.c2_min_c3'       ]) 
    if rotor.blade_optimization_pivots == 5:
        aliases.append([ 'c3_min_c4'                 , 'summary.c3_min_c4'       ])  
    aliases.append([ 't0_min_t1'                 , 'summary.t0_min_t1'       ])  
    aliases.append([ 't1_min_t2'                 , 'summary.t1_min_t2'       ])  
    aliases.append([ 't2_min_t3'                 , 'summary.t2_min_t3'       ])  
    if rotor.blade_optimization_pivots == 5:
        aliases.append([ 't3_min_t4'                 , 'summary.t3_min_t4'       ])   
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
 
def set_optimized_rotor_planform(rotor,optimized_varables):
    r             = rotor.radius_distribution
    R             = rotor.tip_radius     
    chi           = r/R
    total_pivots  = rotor.blade_optimization_pivots
    B             = rotor.number_of_blades
    pivot_points  = rotor.radial_pivots
    omega         = rotor.angular_velocity
    V             = rotor.freestream_velocity  
    alt           = rotor.design_altitude
    
    if rotor.linear_interp_flag:
        c    = linear_discretize(optimized_varables[:total_pivots],chi,pivot_points)     
        beta = linear_discretize(optimized_varables[total_pivots:],chi,pivot_points)  
    else: 
        c    = spline_discretize(optimized_varables[:total_pivots],chi,pivot_points)     
        beta = spline_discretize(optimized_varables[total_pivots:],chi,pivot_points) 
  
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt)   
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]  
    ctrl_pts       = 1
    
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
     
    if rotor.design_power == None: 
        rotor.design_power = power[0][0]
    if rotor.design_thrust == None: 
        rotor.design_thrust = thrust[0][0]
        
    design_torque = power[0][0]/omega[0][0]
    
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
    rotor.twist_distribution         = beta
    rotor.chord_distribution         = c
    rotor.radius_distribution        = r 
    rotor.number_of_blades           = int(B) 
    rotor.design_power_coefficient   = Cp[0][0] 
    rotor.design_thrust_coefficient  = noise_data.Ct[0][0] 
    rotor.mid_chord_alignment        = MCA
    rotor.thickness_to_chord         = t_c 
    rotor.blade_solidity             = sigma   
    rotor.airfoil_flag               = True    
    
    return rotor

if __name__ == '__main__':
    low_noise_rotor_design()
