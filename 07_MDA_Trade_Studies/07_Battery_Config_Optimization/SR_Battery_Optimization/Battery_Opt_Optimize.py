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
        output = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, sense_step = 1E-1,
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
    inputs.append([ 'cell_in_series'    , 140  , 120  , 160  , 10.0 ,  1*Units.less])
    inputs.append([ 'number_of_modules' , 16   , 10   , 20   , 10.0  ,  1*Units.less])
    inputs.append([ 'layout_ratio'      , 0.5  , 0.3  , 0.7  , 1.0   ,  1*Units.less]) 
    problem.inputs = np.array(inputs,dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------- 
    problem.objective = np.array([ 
                               # [ tag         , scaling, units ]
                                 [  'battery_energy'  ,  1.0   ,    1*Units.less] 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------  
    constraints = [] 
    constraints.append([ 'max_module_voltage_residual'  ,  '>'  ,  0.0 ,   1.0   , 1*Units.less])  
    constraints.append([ 'max_C_rate'                   ,  '<'  ,  6.0 ,   1.0   , 1*Units.less])    
    problem.constraints =  np.array(constraints,dtype=object)                
    
    # -------------------------------------------------------------------
    #  Aliases
    # ------------------------------------------------------------------- 
    aliases = []
    aliases.append([ 'cell_in_series'               , 'vehicle_configurations.*.networks.lift_cruise.battery.pack_config.series' ])
    aliases.append([ 'number_of_modules'            , 'vehicle_configurations.*.networks.lift_cruise.battery.module_config.number_of_modules' ])
    aliases.append([ 'layout_ratio'                 , 'vehicle_configurations.*.networks.lift_cruise.battery.module_config.layout_ratio' ])  
    aliases.append([ 'battery_energy'               , 'summary.battery_energy' ])
    aliases.append([ 'max_module_temperature'       , 'summary.max_module_temperature' ])  
    aliases.append([ 'max_module_voltage_residual'  , 'summary.max_module_voltage_residual' ])  
    aliases.append([ 'max_C_rate'                   , 'summary.max_C_rate' ])  
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
