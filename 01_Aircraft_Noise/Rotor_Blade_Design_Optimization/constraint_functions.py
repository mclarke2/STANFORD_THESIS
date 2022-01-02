 
# ----------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------- 
import numpy as np
import scipy as sp   

from common_functions import evaluate_rotor_aerodynamics , linear_discretize , spline_discretize
 
def constraint_thrust_power(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                            total_pivots,pivot_points,linear_interp_flag,design_thrust,
                            design_power,design_taper,CL_max,design_FM): 
    
    # Discretized propeller into stations using linear interpolation 
    if linear_interp_flag:
        c    = linear_discretize(x[:total_pivots],chi,pivot_points)     
        beta = linear_discretize(x[total_pivots:],chi,pivot_points)  
    else: 
        c    = spline_discretize(x[:total_pivots],chi,pivot_points)     
        beta = spline_discretize(x[total_pivots:],chi,pivot_points) 
    
    thrust , torque, power, Ct, Cp, Cl, outputs , etap = evaluate_rotor_aerodynamics(prop, B, c, beta , R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega)
    
    if design_thrust == None:
        constraint = 0.005*design_thrust - abs(power[0][0] - design_power)    # error bound = 0.5 % 
    
    if design_power == None:
        constraint = 0.005*design_thrust - abs(thrust[0][0] - design_thrust)   
        
    return  constraint


def constraint_max_cl(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                            total_pivots,pivot_points,linear_interp_flag,design_thrust,
                            design_power,design_taper,CL_max,design_FM): 
    
    # Discretized propeller into stations using linear interpolation 
    if linear_interp_flag:
        c    = linear_discretize(x[:total_pivots],chi,pivot_points)     
        beta = linear_discretize(x[total_pivots:],chi,pivot_points)  
    else: 
        c    = spline_discretize(x[:total_pivots],chi,pivot_points)     
        beta = spline_discretize(x[total_pivots:],chi,pivot_points) 
    
    thrust , torque, power, Ct, Cp, Cl, outputs , etap = evaluate_rotor_aerodynamics(prop,B, c, beta , R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega)

    # Cl constraint 
    cl_diff = CL_max - Cl
    CL_constraint = sum(cl_diff[cl_diff<0])*10  
    return CL_constraint


def constraint_blade_taper(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                           total_pivots,pivot_points,linear_interp_flag,design_thrust,
                           design_power,design_taper,CL_max,design_FM): 
    blade_taper = x[total_pivots-1]/x[0]
    taper_con   = blade_taper - design_taper
    
    return  taper_con


def constraint_monotonic_chord(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                           total_pivots,pivot_points,linear_interp_flag,design_thrust,
                           design_power,design_taper,CL_max,design_FM):  
  
    violation = 0
    for pi in range(total_pivots-1):
        if (x[pi] - x[pi+1]) < 0:
            violation += (x[pi] - x[pi+1])*10 
            
    return  violation 

def constraint_monotonic_twist(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                           total_pivots,pivot_points,linear_interp_flag,design_thrust,
                           design_power,design_taper,CL_max,design_FM):  
    violation = 0
    for pi in range(total_pivots-1):
        if ( x[total_pivots+ pi] - x[total_pivots + pi+1]) < 0:
            violation += ( x[total_pivots+ pi] - x[total_pivots + pi+1])*10 
            
    return  violation


def constraint_blade_solidity(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                              total_pivots,pivot_points,linear_interp_flag,design_thrust,
                              design_power,design_taper,CL_max,design_FM): 
    
    # Discretized propeller into stations using linear interpolation
    c    = linear_discretize(x[:total_pivots],chi,pivot_points)     
    
    # blade solidity
    r          = chi*R                    
    blade_area = sp.integrate.cumtrapz(B*c, r-r[0])
    sigma      = blade_area[-1]/(np.pi*R**2)   
    
    sigma_con  = 0.2 - sigma  # solidity no greater than 20% - typically 5-20% from Ananth
    return sigma_con 


def constraint_figure_of_merit(x,prop,B, R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega,
                              total_pivots,pivot_points,linear_interp_flag,design_thrust,
                              design_power,design_taper,CL_max,design_FM): 
    
    # Discretized propeller into stations using linear interpolation 
    if linear_interp_flag:
        c    = linear_discretize(x[:total_pivots],chi,pivot_points)     
        beta = linear_discretize(x[total_pivots:],chi,pivot_points)  
    else: 
        c    = spline_discretize(x[:total_pivots],chi,pivot_points)     
        beta = spline_discretize(x[total_pivots:],chi,pivot_points) 
    
    thrust , torque, power, Ct, Cp, Cl, outputs , etap = evaluate_rotor_aerodynamics(prop,B, c, beta , R,chi,Rh,a_geo ,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V, omega)
     
    Area   = np.pi*(R**2)
    FM  = (thrust*np.sqrt(thrust/2*rho*Area))/power
    FM_constraint    = (FM[0][0] - design_FM)*10
    
    return FM_constraint
 