import SUAVE
from SUAVE.Core import Units, Data
import copy
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Propulsion import rotor_design
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_mass
from SUAVE.Methods.Aerodynamics.Fidelity_Zero.Lift import compute_max_lift_coeff  
from SUAVE.Methods.Utilities.Chebyshev  import chebyshev_data

import numpy as np
import pylab as plt
from copy import deepcopy

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():  
    
    # Calculate atmospheric properties
    prop_alt       = 2500 * Units.feet   
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(prop_alt)
    prop_T         = atmo_data.temperature[0]
    prop_rho       = atmo_data.density[0]
    prop_a         = atmo_data.speed_of_sound[0]
    prop_mu        = atmo_data.dynamic_viscosity[0]
     
    rot_alt        = 500 * Units.feet   
    atmosphere     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmo_data      = atmosphere.compute_values(rot_alt) 
    rot_T          = atmo_data.temperature[0]
    rot_rho        = atmo_data.density[0]
    rot_a          = atmo_data.speed_of_sound[0]
    rot_mu         = atmo_data.dynamic_viscosity[0]
    
    ctrl_pts = 1
    
    
    #------------------------------------------------------------------
    # DEFINE PROPELLER & ROTOR 
    #----------------------------------------------------------------- 
    prop                           = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades          = 2
    prop.freestream_velocity       = 135.*Units['mph']    
    prop.angular_velocity          = 2500.  * Units.rpm  
    prop.tip_radius                = 76./2. * Units.inches  
    prop.hub_radius                = 8.     * Units.inches
    prop.design_Cl                 = 0.7
    prop.design_altitude           = 2500. * Units.feet 
    prop.design_thrust             = 400.     
    prop.origin                    = [[2.5, 1.63, 1.01],[2.5, -1.63, 1.01]]   
    prop.airfoil_geometry          = ['NACA_4412.txt']
    prop.airfoil_polars            = [['NACA_4412_polar_Re_50000.txt','NACA_4412_polar_Re_100000.txt',
                                      'NACA_4412_polar_Re_200000.txt','NACA_4412_polar_Re_500000.txt',
                                      'NACA_4412_polar_Re_1000000.txt']]   
    prop.airfoil_polar_stations    = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    
    # Design a Rotor with airfoil  geometry defined  
    rot                          = SUAVE.Components.Energy.Converters.Rotor()  
    rot.tip_radius               = 1.2 
    rot.hub_radius               = 0.2 * rot.tip_radius
    rot.number_of_blades         = 3  
    rot.freestream_velocity      = 1.24  
    rot.angular_velocity         = 141.54523785   
    rot.design_Cl                = 0.7
    rot.design_altitude          = 500 * Units.feet                  
    rot.design_thrust            = 2697.75 
    rot.induced_hover_velocity   = 15.63326
    rot.VTOL_flag                = True 
    rot.airfoil_geometry          = ['NACA_4412.txt']
    rot.airfoil_polars            = [['NACA_4412_polar_Re_50000.txt','NACA_4412_polar_Re_100000.txt',
                                      'NACA_4412_polar_Re_200000.txt','NACA_4412_polar_Re_500000.txt',
                                      'NACA_4412_polar_Re_1000000.txt']]   
    rot.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
     
    #------------------------------------------------------------------
    # PROPELLER & ROTOR DESIGN SCRIPT
    #------------------------------------------------------------------     
    prop_pd = copy.deepcopy(prop)
    rot_pd = copy.deepcopy(rot)
    prop_pd  = propeller_design(prop_pd)  
    rot_pd  = propeller_design(rot_pd)  
     
    prop_rd = copy.deepcopy(prop) 
    rot_rd = copy.deepcopy(rot) 
    prop_rd  = rotor_design(prop_rd) 
    rot_rd  = rotor_design(rot_rd) 
    
    
    #------------------------------------------------------------------
    # PROPELLER & ROTOR CONDITIONS 
    #------------------------------------------------------------------       
    prop_pd.inputs.omega                        = np.array(2500.  * Units.rpm ,ndmin=2)
    prop_rd.inputs.omega                        = np.array(2500.  * Units.rpm ,ndmin=2)
    conditions                                  = Data() 
    conditions.freestream                       = Data()
    conditions.propulsion                       = Data()
    conditions.frames                           = Data()  
    conditions.frames.inertial                  = Data()  
    conditions.frames.body                      = Data() 
    conditions.freestream.density               = np.ones((ctrl_pts,1)) * prop_rho 
    conditions.freestream.dynamic_viscosity     = np.ones((ctrl_pts,1)) * prop_mu
    conditions.freestream.speed_of_sound        = np.ones((ctrl_pts,1)) * prop_a 
    conditions.freestream.temperature           = np.ones((ctrl_pts,1)) * prop_T
    velocity_vector                             = np.array([[ 135.* Units['mph'] , 0. ,0.]])
    conditions.propulsion.throttle              = np.ones((ctrl_pts,1)) * 1.0
    conditions.frames.inertial.velocity_vector  = np.tile(velocity_vector,(ctrl_pts,1))    
    conditions.frames.body.transform_to_inertial= np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])
    
           
    rot_pd.inputs.omega    = np.array(141.54523785,ndmin=2)
    rot_rd.inputs.omega    = np.array(141.54523785,ndmin=2)
    conditions_r                                  = Data() 
    conditions_r.freestream                       = Data()
    conditions_r.propulsion                       = Data()
    conditions_r.frames                           = Data()  
    conditions_r.frames.inertial                  = Data()  
    conditions_r.frames.body                      = Data() 
    conditions_r.freestream.density               = np.ones((ctrl_pts,1)) * rot_rho 
    conditions_r.freestream.dynamic_viscosity     = np.ones((ctrl_pts,1)) * rot_mu
    conditions_r.freestream.speed_of_sound        = np.ones((ctrl_pts,1)) * rot_a 
    conditions_r.freestream.temperature           = np.ones((ctrl_pts,1)) * rot_T
    velocity_vector                               = np.array([[ 1.24 , 0. ,0.]])
    conditions_r.propulsion.throttle              = np.ones((ctrl_pts,1)) * 1.0
    conditions_r.frames.inertial.velocity_vector  = np.tile(velocity_vector,(ctrl_pts,1))    
    conditions_r.frames.body.transform_to_inertial= np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]])   
 
    #------------------------------------------------------------------
    # PROPELLER & ROTOR RUN SCRITPS
    #------------------------------------------------------------------    
    
    # Run Propeller designed with propeller design 
    F_P_PD, Q_P_PD, P_P_PD, Cp_P_PD , noise_P_PD , etap_P_PD = prop_pd.spin(conditions)      
    r_P_PD                 = prop_pd.radius_distribution
    T_distribution_P_PD    = noise_P_PD.blade_thrust_distribution[0] 
    vt_P_PD                = noise_P_PD.blade_tangential_velocity[0] 
    va_P_PD                = noise_P_PD.blade_axial_velocity[0] 
    Q_distribution_P_PD    = noise_P_PD.blade_torque_distribution[0]
    
    # Run Propeller designed with rotor design 
    F_P_RD, Q_P_RD, P_P_RD, Cp_P_RD , noise_P_RD , etap_P_RD = prop_rd.spin(conditions)      
    r_P_RD                 = prop_rd.radius_distribution
    T_distribution_P_RD    = noise_P_RD.blade_thrust_distribution[0] 
    vt_P_RD                = noise_P_RD.blade_tangential_velocity[0] 
    va_P_RD                = noise_P_RD.blade_axial_velocity[0] 
    Q_distribution_P_RD    = noise_P_RD.blade_torque_distribution[0] 
    
    print('\n\nPropeller Design') 
    print('Design Thrust   : ' + str(prop.design_thrust))
    print('PD Method Thrust: ' + str(F_P_PD[0][0]))
    print('RD Nethod Thrust: ' + str(F_P_RD[0][0]))
    print('Power')
    print('PD Method Power: ' + str(P_P_PD[0][0]))
    print('RD Nethod Power: ' + str(P_P_RD[0][0])) 
    print('Torque')
    print('PD Method Torque: ' + str(Q_P_PD[0][0]))
    print('RD Nethod Torque: ' + str(Q_P_RD[0][0]))        
    
    
    # Run Propeller designed with propeller design 
    F_R_PD, Q_R_PD, P_R_PD, Cp_R_PD , noise_R_PD , etap_R_PD = rot_pd.spin(conditions_r)      
    r_R_PD                 = rot_pd.radius_distribution
    T_distribution_R_PD    = noise_R_PD.blade_thrust_distribution[0] 
    vt_R_PD                = noise_R_PD.blade_tangential_velocity[0] 
    va_R_PD                = noise_R_PD.blade_axial_velocity[0] 
    Q_distribution_R_PD    = noise_R_PD.blade_torque_distribution[0] 
    
    
    # Run Propeller designed with propeller design 
    F_R_RD, Q_R_RD, P_R_RD, Cp_R_RD , noise_R_RD , etap_R_RD = rot_rd.spin(conditions_r)      
    r_R_RD                 = rot_rd.radius_distribution
    T_distribution_R_RD    = noise_R_RD.blade_thrust_distribution[0] 
    vt_R_RD                = noise_R_RD.blade_tangential_velocity[0] 
    va_R_RD                = noise_R_RD.blade_axial_velocity[0] 
    Q_distribution_R_RD    = noise_R_RD.blade_torque_distribution[0] 
    
    
    print('\n\nPropeller Design') 
    print('Design Thrust   : ' + str(rot.design_thrust))
    print('PD Method Thrust: ' + str(F_R_PD[0][0]))
    print('RD Nethod Thrust: ' + str(F_R_RD[0][0]))
    print('Power')
    print('PD Method Power: ' + str(P_R_PD[0][0]))
    print('RD Nethod Power: ' + str(P_R_RD[0][0])) 
    print('Torque')
    print('PD Method Torque: ' + str(Q_R_PD[0][0]))
    print('RD Nethod Torque: ' + str(Q_R_RD[0][0]))            
        
    # ----------------------------------------------------------------------------
    # 2D - Plots  Plots    
    # ---------------------------------------------------------------------------- 
    # perpendicular velocity, up Plot 
    fig1 = plt.figure(1) 
    axes1 = fig1.add_subplot(1,2,1)
    axes1.plot(r_P_PD, va_P_PD,'ro-', label = 'prop-design method')      
    axes1.plot(r_P_RD , va_P_RD,'bs-' , label = 'rotor-design method')    
    axes1.set_xlabel('Radial Location')
    axes1.set_ylabel('Axial Velocity')  
    axes2 = fig1.add_subplot(1,2,2)
    axes2.plot(r_R_PD , va_R_PD,'ro-', label = 'prop-design method')      
    axes2.plot(r_R_RD , va_R_RD,'bs-' , label = 'rotor-design method')    
    axes2.set_xlabel('Radial Location')
    axes2.set_ylabel('Axial Velocity') 
    axes2.legend(loc='lower right') 
    
     
    fig2 = plt.figure(2) 
    axes1 = fig2.add_subplot(1,2,1)
    axes1.plot(r_P_PD, vt_P_PD,'ro-', label = 'prop-design method')      
    axes1.plot(r_P_RD ,vt_P_RD,'bs-' , label = 'rotor-design method')    
    axes1.set_xlabel('Radial Location')
    axes1.set_ylabel('Tangential Velocity')  
    axes2 = fig2.add_subplot(1,2,2)
    axes2.plot(r_R_PD , vt_R_PD,'ro-', label = 'prop-design method')      
    axes2.plot(r_R_RD , vt_R_RD,'bs-' , label = 'rotor-design method')    
    axes2.set_xlabel('Radial Location')
    axes2.set_ylabel('Tangential Velocity') 
    axes2.legend(loc='lower right')
    
     
    fig3 = plt.figure(3) 
    axes1 = fig3.add_subplot(1,2,1)
    axes1.plot(r_P_PD, T_distribution_P_PD,'ro-', label = 'prop-design method')      
    axes1.plot(r_P_RD ,T_distribution_P_RD,'bs-' , label = 'rotor-design method')    
    axes1.set_xlabel('Radial Location')
    axes1.set_ylabel('Trust, N')  
    axes2 = fig3.add_subplot(1,2,2)
    axes2.plot(r_R_PD , T_distribution_R_PD,'ro-', label = 'prop-design method')      
    axes2.plot(r_R_RD , T_distribution_R_RD,'bs-' , label = 'rotor-design method')    
    axes2.set_xlabel('Radial Location')
    axes2.set_ylabel('Thrust, N') 
    axes2.legend(loc='lower right')
    
     
    fig4 = plt.figure(4) 
    axes1 = fig4.add_subplot(1,2,1)
    axes1.plot(r_P_PD, Q_distribution_P_PD,'ro-', label = 'prop-design method')      
    axes1.plot(r_P_RD ,Q_distribution_P_RD,'bs-' , label = 'rotor-design method')    
    axes1.set_xlabel('Radial Location')
    axes1.set_ylabel('Torque, Nm')  
    axes2 = fig4.add_subplot(1,2,2)
    axes2.plot(r_R_PD , Q_distribution_R_PD,'ro-', label = 'prop-design method')      
    axes2.plot(r_R_RD , Q_distribution_R_RD,'bs-' , label = 'rotor-design method')    
    axes2.set_xlabel('Radial Location')
    axes2.set_ylabel('Torque, Nm') 
    axes2.legend(loc='lower right')
    
    fig5 = plt.figure(5) 
    axes1 = fig5.add_subplot(2,2,1)
    axes1.plot(r_P_PD, prop_pd.twist_distribution/Units.degrees,'ro-', label = 'prop-design method')      
    axes1.plot(r_P_RD ,prop_rd.twist_distribution/Units.degrees,'bs-' , label = 'rotor-design method')    
    axes1.set_xlabel('Radial Location')
    axes1.set_ylabel('Twist distribution')  
    axes2 = fig5.add_subplot(2,2,2)
    axes2.plot(r_R_PD , rot_pd.twist_distribution/Units.degrees,'ro-', label = 'prop-design method')      
    axes2.plot(r_R_RD , rot_rd.twist_distribution/Units.degrees,'bs-' , label = 'rotor-design method')    
    axes2.set_xlabel('Radial Location')
    axes2.set_ylabel('Twist distribution')  
    axes3 = fig5.add_subplot(2,2,3) 
    axes3.plot(r_P_PD, prop_pd.chord_distribution,'ro-', label = 'prop-design method')      
    axes3.plot(r_P_RD ,prop_rd.chord_distribution,'bs-' , label = 'rotor-design method')    
    axes3.set_xlabel('Radial Location')
    axes3.set_ylabel('Chord distribution')  
    axes4 = fig5.add_subplot(2,2,4)
    axes4.plot(r_R_PD , rot_pd.chord_distribution,'ro-', label = 'prop-design method')      
    axes4.plot(r_R_RD , rot_rd.chord_distribution,'bs-' , label = 'rotor-design method')    
    axes4.set_xlabel('Radial Location')
    axes4.set_ylabel('Chord distribution') 
    axes4.legend(loc='lower right')        
      
 
    return  
 

if __name__ == '__main__': 
    main()    
    plt.show()   