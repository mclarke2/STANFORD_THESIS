# Optimize.py 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------  
import SUAVE
from SUAVE.Core import Units, Data 
import numpy as np
import Vehicles
import Missions
import Procedure
import SUAVE.Optimization.Package_Setups.scipy_setup as scipy_setup
from SUAVE.Optimization       import Nexus      
import numpy as np 
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry  
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars \
     import compute_airfoil_polars 

import time 

# ----------------------------------------------------------------------        
#   Run the whole thing
# ----------------------------------------------------------------------  
def low_noise_rotor_design(rotor,
                           number_of_stations = 20, 
                           solver                   = 'SLSQP', 
                           objective_weight         = 0.5,
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
    #rotor.airfoil_data              = airfoil_geometry_data 
    rotor.number_of_blades          = int(B)  
    rotor.thickness_to_chord        = t_c  
    rotor.radius_distribution       = chi
    rotor.airfoil_cl_surrogates     = cl_sur
    rotor.airfoil_cd_surrogates     = cd_sur 
    rotor.airfoil_flag              = True 
    
    ti = time.time()  
    solver_name       = 'SLSQP' 
    optimization_problem = low_noise_rotor_optimization_setup(rotor)
    output = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, sense_step = 1E-2, tolerance = 1E-3)  
    print (output)    
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Rotor Otimization Simulation Time: ' + str(elapsed_time))    
   
   
   

   
     
    #if prop.design_power == None: 
        #prop.design_power = power[0][0]
        
    #design_torque = power[0][0]/omega[0][0]
    
    ## blade solidity
    #r          = chi*R                    # Radial coordinate   
    #blade_area = sp.integrate.cumtrapz(B*c, r-r[0])
    #sigma      = blade_area[-1]/(np.pi*R**2)   
    
    #MCA    = c/4. - c[0]/4.
    #airfoil_geometry_data = import_airfoil_geometry(a_geo) 
    #t_max = np.take(airfoil_geometry_data.max_thickness,a_loc,axis=0)*c 
    #t_c   =  np.take(airfoil_geometry_data.thickness_to_chord,a_loc,axis=0)  
    
    #rotor.design_torque              = design_torque
    #rotor.max_thickness_distribution = t_max
    #rotor.twist_distribution         = beta
    #rotor.chord_distribution         = c
    #rotor.radius_distribution        = r 
    #rotor.number_of_blades           = int(B) 
    #rotor.design_power_coefficient   = Cp[0][0] 
    #rotor.design_thrust_coefficient  = Ct[0][0] 
    #rotor.mid_chord_alignment        = MCA
    #rotor.thickness_to_chord         = t_c 
    #rotor.blade_solidity             = sigma   
    #rotor.airfoil_flag               = True     
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
    for i in range(rotor.blade_optimization_pivots):
        #             [ tag              , initial                  ,  lb    , ub       , scaling ,     units ] 
        inputs.append([ 'chord_' + str(i+1), rotor.chord_pivots[i]  , 0.05*R , 0.4*R    , 1.0     ,  1*Units.less])
        inputs.append([ 'twist_' + str(i+1), rotor.twist_pivots[i]  ,  0     , np.pi/4  , 1.0     ,  1*Units.degrees])
    problem.inputs = np.array(inputs,dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------- 
    problem.objective = np.array([ 
                               # [ tag         , scaling, units ]
                                 [  'SPL_dBA'  ,  10.0   ,    1*Units.less] 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------
    constraints = [] 
    for i in range(rotor.blade_optimization_pivots - 1):
        #             [ tag                                     , sense, edge , scaling , units ]
        constraints.append([ 'c' + str(i+1) + 'min_c' + str(i+2),  '>' ,  0.0 ,   1.0   , 1*Units.less])
        constraints.append([ 't' + str(i+1) + 'min_t' + str(i+2),  '>' ,  0.0 ,   1.0   , 1*Units.less]) 
    constraints.append([ 'thrust_power_residual'    ,   '>' ,  0.0 ,   1.0   , 1*Units.less])  
    constraints.append([ 'blade_taper_residual'     ,   '>' ,  0.0 ,   1.0   , 1*Units.less])    
    constraints.append([ 'max_cl_residual'          ,   '>' ,  0.0 ,   1.0   , 1*Units.less])
    constraints.append([ 'blade_solidity_residual'  ,   '>' ,  0.0 ,   1.0   , 1*Units.less])  
    constraints.append([ 'figure_of_merit_residual' ,   '>' ,  0.0 ,   1.0   , 1*Units.less]) 
    problem.constraints = np.array(constraints,dtype=object) \
        
    # -------------------------------------------------------------------
    #  Aliases
    # ------------------------------------------------------------------- 
    aliases = [   
        [ 'SPL_dBA'                   , 'summary.SPL_dBA' ],  
        [ 'thrust_power_residual'     , 'summary.thrust_power_residual'  ], 
        [ 'blade_taper_residual'      , 'summary.blade_taper_residual'   ],  
        [ 'max_cl_residual'           , 'summary.max_cl_residual'        ], 
        [ 'blade_solidity_residual'   , 'summary.blade_solidity_residual'], 
        [ 'figure_of_merit_residual'  , 'summary.figure_of_merit_residual'],    ]      
    
 
    for i in range(rotor.blade_optimization_pivots):    
        aliases.append([ 'chord_' + str(i+1)  , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.chord_pivots[' + str(i) + ']' ])
        aliases.append([ 'twist_'  + str(i+1) , 'vehicle_configurations.*.networks.battery_propeller.lift_rotors.rotor.twist_pivots[' + str(i) + ']' ], )
        
        if i == rotor.blade_optimization_pivots - 1: 
            pass 
        else:
            constraints.append([ 'c' + str(i+1) + 'min_c' + str(i+2), 'summary.' + 'c' + str(i+1) + 'min_c' + str(i+2)   ])
            constraints.append([ 't' + str(i+1) + 'min_t' + str(i+2), 'summary.' + 't' + str(i+1) + 'min_t' + str(i+2)  ])    
    problem.aliases =  aliases
    
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
 
if __name__ == '__main__':
    low_noise_rotor_design()
