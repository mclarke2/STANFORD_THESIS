# Optimize.py
#  
# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------  
# SUAVE Imports 
import SUAVE 
from SUAVE.Core                                                                           import Units, Data  
import SUAVE.Optimization.Package_Setups.scipy_setup                                      as scipy_setup
import SUAVE.Optimization.Package_Setups.pyoptsparse_setup                                as pyoptsparse_setup
from SUAVE.Optimization                                                                   import Nexus      

# Python package imports  
import numpy as np 
import time 

import Battery_Opt_Vehicle
import Battery_Opt_Analyses
import Battery_Opt_Mission
import Battery_Opt_Procedure

def main():
    
    use_pyoptsparse = False
    solver_name     = 'SLSQP'
    
    # start optimization 
    ti = time.time()   
    optimization_problem = problem_setup() 
    if use_pyoptsparse:
        output = pyoptsparse_setup.Pyoptsparse_Solve(optimization_problem,solver='SNOPT',FD='parallel',
                                                      sense_step= 1E-3) 
    else: 
        output = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, sense_step = 1E-4,
                                         tolerance = 1E-3)    
    tf           = time.time()
    elapsed_time = round((tf-ti)/60,2)
    print('Rotor Optimization Simulation Time: ' + str(elapsed_time))   
    
    # print optimization results 
    print (output)  
     
    return


# ----------------------------------------------------------------------        
#   Inputs, Objective, & Constraints
# ----------------------------------------------------------------------  

def problem_setup():

    nexus = Nexus()
    problem = Data()
    nexus.optimization_problem = problem
 

    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------   
    inputs = []
    inputs.append([ 'tip_mach'   , tm_0      , tm_ll  , tm_ul    , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_r'    , 0.1*R     , 0.05*R , 0.2*R    , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_p'    , 2         , 0.25   , 2.0      , 1.0     ,  1*Units.less]) 
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
    constraints.append([ 'thrust_power_residual'    ,  '>'  ,  0.0 ,   1.0   , 1*Units.less])  
    constraints.append([ 'blade_taper_constraint_1' ,  '>'  ,  0.3 ,   1.0   , 1*Units.less])  
    constraints.append([ 'blade_taper_constraint_2' ,  '<'  ,  0.7 ,   1.0   , 1*Units.less])  
    problem.constraints =  np.array(constraints,dtype=object)                
    
    # -------------------------------------------------------------------
    #  Aliases
    # ------------------------------------------------------------------- 
    aliases = []
    aliases.append([ 'tip_mach'                  , 'vehicle_configurations.*.networks.battery_propeller.battery.rotor.design_tip_mach' ])
    aliases.append([ 'chord_r'                   , 'vehicle_configurations.*.networks.battery_propeller.battery.rotor.chord_r' ])
    aliases.append([ 'chord_p'                   , 'vehicle_configurations.*.networks.battery_propeller.battery.rotor.chord_p' ])  
    problem.aliases = aliases
     
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = Battery_Opt_Vehicle.setup()
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = Battery_Opt_Analyses.analyses_setup(nexus.vehicle_configurations)
       
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = Battery_Opt_Mission.setup(nexus.analyses,nexus.vehicle_configurations.base)
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = Battery_Opt_Procedure.setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary = Data()    
    nexus.total_number_of_iterations = 0
    return nexus 

if __name__ == '__main__':
    main() 
