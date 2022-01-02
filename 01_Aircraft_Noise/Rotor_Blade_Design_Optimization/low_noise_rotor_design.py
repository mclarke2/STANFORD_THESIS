## @ingroup Methods-Propulsion
# rotor_design.py
# 
# Created: May 2021, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import SUAVE
import numpy as np
import scipy as sp
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry \
     import import_airfoil_geometry
from scipy.optimize import minimize 
from scipy.optimize import NonlinearConstraint 
import scipy as sp
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars \
     import compute_airfoil_polars

from constraint_functions import constraint_thrust_power,  constraint_max_cl,  constraint_blade_taper, constraint_monotonic_chord, constraint_monotonic_twist,  constraint_blade_solidity, constraint_figure_of_merit 
from common_functions import evaluate_rotor_aerodynamics, linear_discretize, spline_discretize
from objective_function import aeroacoustic_objective


# ----------------------------------------------------------------------
#  Propeller Design
# ----------------------------------------------------------------------

def low_noise_rotor_design(prop, number_of_stations = 20,
                 design_taper             = 0.3,
                 pivot_points             = [0.25,0.5,0.75],
                 linear_interp_flag       = True,
                 solver                   = 'differential_evolution',
                 CL_max                   = 1.1,
                 design_FM                = 0.75,
                 objective_weight         = 0.5,
                 print_iter               = True):
    
    """ Optimizes propeller chord and twist given input parameters.
          
          Inputs:
          Either design power or thrust
          prop_attributes.
            hub radius                       [m]
            tip radius                       [m]
            rotation rate                    [rad/s]
            freestream velocity              [m/s]
            number of blades               
            number of stations
            design lift coefficient
            airfoil data
            
          Outputs:
          Twist distribution                 [array of radians]
          Chord distribution                 [array of meters]
              
          Assumptions/ Source:
          Based on Design of Optimum Propellers by Adkins and Liebeck
          
    """    
    # Unpack
    N             = number_of_stations       # this number determines the discretization of the propeller into stations 
    B             = prop.number_of_blades
    R             = prop.tip_radius
    Rh            = prop.hub_radius
    omega         = prop.angular_velocity    # Rotation Rate in rad/s
    Va            = prop.induced_hover_velocity
    Vinf          = prop.freestream_velocity # Freestream Velocity 
    alt           = prop.design_altitude
    design_thrust = prop.design_thrust
    design_power  = prop.design_power
    a_geo         = prop.airfoil_geometry
    a_pol         = prop.airfoil_polars        
    a_loc         = prop.airfoil_polar_stations    
    
    if (design_thrust == None) and (design_power== None):
        raise AssertionError('Specify either design thrust or design power!')
    
    elif (design_thrust!= None) and (design_power!= None):
        raise AssertionError('Specify either design thrust or design power!')
    
    if prop.rotation == None:
        prop.rotation = list(np.ones(int(B))) 
        
    if  a_pol != None and a_loc != None:
        if len(a_loc) != N:
            raise AssertionError('\nDimension of airfoil sections must be equal to number of stations on rotor')
        # compute airfoil polars for airfoils 
        airfoil_polars  = compute_airfoil_polars(a_geo, a_pol)  
        cl_sur = airfoil_polars.lift_coefficient_surrogates 
        cd_sur = airfoil_polars.drag_coefficient_surrogates    
        
    else:
        raise AssertionError('\nDefine rotor airfoil') 
 
    airfoil_geometry_data           =  import_airfoil_geometry(prop.airfoil_geometry, npoints = prop.number_of_airfoil_section_points)
    t_c                             =  np.take(airfoil_geometry_data.thickness_to_chord,a_loc,axis=0)
    prop.airfoil_data               = airfoil_geometry_data 
    prop.number_of_blades           = int(B)  
    prop.thickness_to_chord         = t_c   
    prop.airfoil_cl_surrogates      = cl_sur
    prop.airfoil_cd_surrogates      = cd_sur 
    prop.airfoil_flag               = True 
    
        
    # Calculated total velocity 
    V       = Vinf + Va
    chi0    = Rh/R # Where the rotor blade actually starts
    chi     = np.linspace(chi0,1,N+1) # Vector of nondimensional radii
    chi     = chi[0:N]
         
    # Calculate atmospheric properties
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(alt) 
    p              = atmo_data.pressure[0]
    T              = atmo_data.temperature[0]
    rho            = atmo_data.density[0]
    a              = atmo_data.speed_of_sound[0]
    mu             = atmo_data.dynamic_viscosity[0]
    nu             = mu/rho  
    V              = np.array(V,ndmin=2)
    omega          = np.array(omega,ndmin=2)
    
    # Define initial conditions 
    total_pivots = len(pivot_points) + 2
    
    # Define bounds  
    c_lb      = 0.05*R    # small bound is 40 % of radius 
    c_ub      = 0.4*R     # upper bound is 40 % of radius 
    beta_lb   = 0         #  
    beta_ub   = np.pi/4   # upper bound approx. 60 degrees  
    c_bnds    = []
    beta_bnds = []
    for i in range(total_pivots):
        c_bnds.append((c_lb,c_ub)) 
        beta_bnds.append((beta_lb,beta_ub))  
    de_bnds = c_bnds + beta_bnds
    bnds    = tuple(de_bnds)

    # Define initial conditions 
    total_pivots = len(pivot_points) + 2   
    c_0          = np.linspace(0.2,0.4,4)*R # 0.2
    beta_0       = np.linspace(0.2,np.pi/4,4)   # 0.3     
    
    # Define static arguments 
    args = (prop,B,R,chi,Rh,a_geo,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,total_pivots,pivot_points,
            linear_interp_flag,design_thrust,design_power,design_taper,CL_max,design_FM)  
    
    init_scaling = np.linspace(0.2,1,3) 
    success_flag = False 
    for i_ in range(len(init_scaling)): 
        #initials      = list(np.concatenate([np.ones(total_pivots)*c_0[i_],np.ones(total_pivots)*beta_0[i_]]))   
        initials      = np.concatenate([np.linspace(0.4,0.1,total_pivots),np.linspace(0.8,0.4,total_pivots),])*init_scaling[i_] 
        if solver == 'SLSQP': 
            # Define constaints 
            cons = [{'type':'ineq', 'fun': constraint_thrust_power   ,'args': args},
                    {'type':'ineq', 'fun': constraint_blade_taper    ,'args': args},
                    {'type':'ineq', 'fun': constraint_monotonic_chord,'args': args},
                    {'type':'ineq', 'fun': constraint_monotonic_twist,'args': args},
                    #{'type':'ineq', 'fun': constraint_blade_solidity, 'args': args},
                    {'type':'ineq', 'fun': constraint_max_cl         ,'args': args}, 
                    ] 
            
            opts= {'eps':1e-2,'maxiter': 500, 'disp': print_iter, 'ftol': 1e-4} 
            try:  
                sol = minimize(aeroacoustic_objective,initials, args=args,method=solver, bounds=bnds , constraints= cons, options = opts)
                success_flag = sol.success
            except: 
                success_flag = False   
      
        elif solver == 'differential_evolution': 
            opts  ={'eps':1e-2, 'disp':print_iter , 'ftol': 1e-4}
            diff_evo_cons = create_diff_evo_cons(prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                            total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)  
            
            sol = sp.optimize.differential_evolution(aeroacoustic_objective, bounds= de_bnds,args= args, strategy='best1bin', maxiter=2000, \
                                                         tol=0.01, mutation=(0.5, 1), recombination=0.7, callback=None,seed = 2,\
                                                         disp=print_iter, polish=True, init='latinhypercube', atol=0, updating='immediate',\
                                                         workers=1,constraints=diff_evo_cons) 
        
            if sol.success == True:
                success_flag = sol.success
            else:
                success_flag = False  
            
        if success_flag:
            break 
            
            
            
    # one last check of chord and twist
    print(sol)
    
    mcc  = constraint_monotonic_chord(sol.x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                               total_pivots,pivot_points,linear_interp_flag,design_thrust,
                               design_power,design_taper,CL_max,design_FM)
    print(mcc)
    mtc = constraint_monotonic_twist(sol.x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
               total_pivots,pivot_points,linear_interp_flag,design_thrust,
               design_power,design_taper,CL_max,design_FM)
    
    print(mtc)
    
    # run one last time to get properties to pack
    if linear_interp_flag:
        c    = linear_discretize(sol.x[:total_pivots],chi,pivot_points)     
        beta = linear_discretize(sol.x[total_pivots:],chi,pivot_points)  
    else: 
        c    = spline_discretize(sol.x[:total_pivots],chi,pivot_points)     
        beta = spline_discretize(sol.x[total_pivots:],chi,pivot_points) 
    
    
    thrust , torque, power, Ct, Cp, Cl, outputs , etap = evaluate_rotor_aerodynamics(prop,B,R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega)
     
    if prop.design_power == None: 
        prop.design_power = power[0][0]
        
    design_torque = power[0][0]/omega[0][0]
    
    # blade solidity
    r          = chi*R                    # Radial coordinate   
    blade_area = sp.integrate.cumtrapz(B*c, r-r[0])
    sigma      = blade_area[-1]/(np.pi*R**2)   
    
    MCA    = c/4. - c[0]/4.
    airfoil_geometry_data = import_airfoil_geometry(a_geo) 
    t_max = np.take(airfoil_geometry_data.max_thickness,a_loc,axis=0)*c 
    t_c   =  np.take(airfoil_geometry_data.thickness_to_chord,a_loc,axis=0)  
    
    prop.design_torque              = design_torque
    prop.max_thickness_distribution = t_max
    prop.twist_distribution         = beta
    prop.chord_distribution         = c
    prop.radius_distribution        = r 
    prop.number_of_blades           = int(B) 
    prop.design_power_coefficient   = Cp[0][0] 
    prop.design_thrust_coefficient  = Ct[0][0] 
    prop.mid_chord_alignment        = MCA
    prop.thickness_to_chord         = t_c 
    prop.blade_solidity             = sigma  
    prop.airfoil_cl_surrogates      = cl_sur
    prop.airfoil_cd_surrogates      = cd_sur 
    prop.airfoil_flag               = True 

    return prop
  

def create_diff_evo_cons(prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                       total_pivots,pivot_points,linear_interp_flag,design_thrust,
                       design_power,design_taper,CL_max,design_FM):
    constraints = []
    def fun1(x): 
        con1 = constraint_thrust_power(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                               total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)

        return np.atleast_1d(con1)

    def fun2(x): 
        con2 = constraint_blade_taper(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                              total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)

        return np.atleast_1d(con2)


    def fun3(x): 
        con3 = constraint_monotonic_chord(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                                  total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)

        return np.atleast_1d(con3)


    def fun4(x): 
        con4 = constraint_monotonic_twist(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                                  total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)

        return np.atleast_1d(con4)

    def fun5(x): 
        con5 = constraint_blade_solidity(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                                 total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)
        return np.atleast_1d(con5)

    def fun6(x): 
        con6 = constraint_figure_of_merit(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                                 total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)
        return np.atleast_1d(con6)

    def fun7(x): 
        con7 = constraint_max_cl(x,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                                                 total_pivots,pivot_points,linear_interp_flag,design_thrust,
                                            design_power,design_taper,CL_max,design_FM)


        return np.atleast_1d(con7)            

    constraints.append(NonlinearConstraint(fun1, 0 ,np.inf)) 
    constraints.append(NonlinearConstraint(fun2, 0 ,np.inf))  
    constraints.append(NonlinearConstraint(fun3, 0 ,np.inf))  
    constraints.append(NonlinearConstraint(fun4, 0 ,np.inf))
    #constraints.append(NonlinearConstraint(fun5, 0 ,np.inf)) 
    #constraints.append(NonlinearConstraint(fun6, 0 ,np.inf)) 
    constraints.append(NonlinearConstraint(fun7, 0 ,np.inf)) 
    
    return tuple(constraints)     
