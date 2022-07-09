# Stopped_Rotor_CRM.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data   
import pickle
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry   import * 
from SUAVE.Components.Energy.Networks.Lift_Cruise                         import Lift_Cruise
from SUAVE.Methods.Power.Battery.Sizing                                   import initialize_from_mass 
from SUAVE.Methods.Geometry.Two_Dimensional.Planform                      import segment_properties
from SUAVE.Methods.Power.Battery.Sizing                                   import initialize_from_circuit_configuration 
from SUAVE.Methods.Weights.Correlations.Propulsion                        import nasa_motor
from SUAVE.Methods.Propulsion.electric_motor_sizing                       import size_optimal_motor
from SUAVE.Methods.Propulsion                                             import propeller_design   
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                           import empty
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform


from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight    import converge_evtol_weight  

import time 
import os
import numpy as np
import pylab as plt
from copy import deepcopy 

try:
    import vsp 
    from SUAVE.Input_Output.OpenVSP.vsp_write import write 
except ImportError:
    # This allows SUAVE to build without OpenVSP
    pass 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    # 466.577 mins  
    simulated_days   = 1
    flights_per_day  = 1 
    aircraft_range   = 55 *Units.nmi
    reserve_segment  = False
    plot_geometry    = False
    recharge_battery = False
    run_analysis     = True
    plot_mission     = True
    control_points   = 10
    N_gm_x           = 10
    N_gm_y           = 5
    
    run_noise_model   = False
    hover_noise_test  = False
    run_full_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                     hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                     control_points,N_gm_x,N_gm_y) 
    

    #run_noise_model   = True 
    #hover_noise_test  = False   
    #run_approach_departure_noise_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      #hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      #control_points,N_gm_x,N_gm_y)
    

    #run_noise_model   = True 
    #hover_noise_test  = False   
    #run_full_noise_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      #hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      #control_points,N_gm_x,N_gm_y)    


    #hover_noise_test  = True 
    #run_noise_model   = True    
    #run_hover_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      #hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      #control_points,N_gm_x,N_gm_y) 
    
    return 


def run_full_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                     hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                     control_points,N_gm_x,N_gm_y):  
 
    min_y             = 1E-1
    max_y             = 0.25*Units.nmi
    min_x             = 1E-1
    max_x             = aircraft_range 
    
    ti                = time.time() 
    vehicle           = vehicle_setup()
    #write(vehicle,"Stopped_Rotor") 
    configs           = configs_setup(vehicle) 
    configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
    base_mission      = full_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
    missions_analyses = missions_setup(base_mission) 
    analyses          = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses 
    configs.finalize()
    analyses.finalize()      
    mission           = analyses.missions.base
    mission_results   = mission.evaluate()   
    filename          = 'Stopped_Rotor_Full_Mission' 
    save_results(mission_results,filename)   
            
    # weight plot breakdown 
    print('WEIGHT BREAKDOWN')
    breakdown = empty(configs.base,contingency_factor=1.0)
    print(breakdown)
    
    # plot geoemtry 
    if plot_geometry: 
        plot_vehicle(configs.base, elevation_angle = 90,azimuthal_angle =  180,axis_limits =8,plot_control_points = False)       
        
    if plot_mission: 
        plot_results(mission_results,run_noise_model)   
        
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = mission_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')
  
    return 


def run_full_noise_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      control_points,N_gm_x,N_gm_y): 

    Y_LIM = np.linspace(1E-3,5*Units.nmi,3)    
    end_distance = aircraft_range/((N_gm_x-2)*2)
    X_LIM = np.linspace(-end_distance+1E-3,aircraft_range + end_distance+1E-3,3)          
    
    ti                = time.time() 
    vehicle           = vehicle_setup() 
    configs           = configs_setup(vehicle)  
    Q_idx             = 1

    for i in range(len(X_LIM)-1):
        for j in range(len(Y_LIM)-1): 
            print('Running Quardant:' + str(Q_idx))
            min_x = X_LIM[i]
            max_x = X_LIM[i+1]
            min_y = Y_LIM[j]
            max_y = Y_LIM[j+1]
            
            # ------------------------------------------------------------------------------------------------
            # Noise Mission 
            # ------------------------------------------------------------------------------------------------   
            configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
            noise_mission     = approach_departure_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
            missions_analyses = missions_setup(noise_mission) 
            analyses          = SUAVE.Analyses.Analysis.Container()
            analyses.configs  = configs_analyses
            analyses.missions = missions_analyses 
            configs.finalize()
            analyses.finalize()      
            noise_mission     = analyses.missions.base
            noise_results     = noise_mission.evaluate()   
            filename          = 'Stopped_Rotor_Full_Mission_Noise_Q' + str(Q_idx)+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
            save_results(noise_results,filename)  
            Q_idx += 1 
        
    if plot_mission: 
        plot_results(noise_results,run_noise_model)       
                
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = noise_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')
    
    return 


def run_approach_departure_noise_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      control_points,N_gm_x,N_gm_y): 

    Y_LIM = np.linspace(1E-6,0.5*Units.nmi,3)     
    X_LIM = np.linspace(0, 5.79*Units.nmi,3)            
    
    ti                = time.time() 
    vehicle           = vehicle_setup() 
    configs           = configs_setup(vehicle)  
    Q_idx             = 1

    for i in range(len(X_LIM)-1):
        for j in range(len(Y_LIM)-1): 
            print('Running Quardant:' + str(Q_idx))
            ti_quad  = time.time()
            min_x = X_LIM[i]
            max_x = X_LIM[i+1]
            min_y = Y_LIM[j]
            max_y = Y_LIM[j+1]
            
            # ------------------------------------------------------------------------------------------------
            # Noise Mission 
            # ------------------------------------------------------------------------------------------------   
            configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
            noise_mission     = approach_departure_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
            missions_analyses = missions_setup(noise_mission) 
            analyses          = SUAVE.Analyses.Analysis.Container()
            analyses.configs  = configs_analyses
            analyses.missions = missions_analyses 
            configs.finalize()
            analyses.finalize()      
            noise_mission     = analyses.missions.base
            noise_results     = noise_mission.evaluate()   
            filename          = 'Stopped_Rotor_Approach_Departure_Noise_Q' + str(Q_idx)+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
            
            save_results(noise_results,filename)  
            Q_idx += 1 
             
            tf_quad = time.time() 
            print ('time taken: '+ str(round(((tf_quad-ti_quad)/60),3)) + ' mins')              
        
    if plot_mission: 
        plot_results(noise_results,run_noise_model)       
                
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = noise_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')
    
    return 
  
  
    
def run_hover_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      control_points,N_gm_x,N_gm_y): 

    min_y = 1E-3 
    max_y = 0.25*Units.nmi
    min_x = -0.25*Units.nmi
    max_x = 0.25*Units.nmi
    
    ti                = time.time() 
    vehicle           = vehicle_setup() 
    configs           = configs_setup(vehicle) 
    configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test) 
    base_mission      = hover_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
    missions_analyses = missions_setup(base_mission) 
    analyses          = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses 
    configs.finalize()
    analyses.finalize()     
    mission           = analyses.missions.base
    hover_results     = mission.evaluate()   
    filename          = 'Stopped_Rotor_Hover_Mission'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    save_results(hover_results,filename)  
    
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = hover_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')   
    
    return 


# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup():
     
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle               = SUAVE.Vehicle()
    vehicle.tag           = 'Stopped_Rotor_CRM'
    vehicle.configuration = 'eVTOL'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff           = 2900
    vehicle.mass_properties.operating_empty   = 2900     
    vehicle.mass_properties.max_takeoff       = 2900 
    vehicle.envelope.ultimate_load            = 5.7   
    vehicle.envelope.limit_load               = 3.  
    vehicle.passengers                        = 6
    
    # ------------------------------------------------------------------    
    # WINGS                                    
    # ------------------------------------------------------------------    
    # WING PROPERTIES           
    wing                          = SUAVE.Components.Wings.Main_Wing()
    wing.tag                      = 'main_wing'  
    wing.aspect_ratio             = 12.27422  # will  be overwritten
    wing.sweeps.quarter_chord     = 0.0  
    wing.thickness_to_chord       = 0.14 
    wing.taper                    = 0.292
    wing.spans.projected          = 14
    wing.chords.root              = 1.75
    wing.total_length             = 1.75
    wing.chords.tip               = 0.525 
    wing.chords.mean_aerodynamic  = 0.8
    wing.dihedral                 = 0.0  
    wing.areas.reference          = 15.  
    wing.twists.root              = 0. * Units.degrees
    wing.twists.tip               = 0. 
    wing.origin                   = [[1.5, 0.,  1.1]]
    wing.aerodynamic_center       = [ 1.567, 0.,  1.1]    
    wing.winglet_fraction         = 0.0  
    wing.symmetric                = True
    wing.vertical                 = False
    
    # Segment                                  
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_1'   
    segment.percent_span_location = 0.  
    segment.twist                 = 4. * Units.degrees 
    segment.root_chord_percent    = 1. 
    segment.dihedral_outboard     = 0 * Units.degrees
    segment.sweeps.quarter_chord  = 3.18 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    wing.Segments.append(segment)               
    
    # Segment                                   
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_2'    
    segment.percent_span_location = 0.3
    segment.twist                 = 3. * Units.degrees 
    segment.root_chord_percent    = 0.8 # 0.7  
    segment.dihedral_outboard     = 0.0 * Units.degrees
    segment.sweeps.quarter_chord  = 0. 
    segment.thickness_to_chord    = 0.16  
    wing.Segments.append(segment)               
     
    # Segment                                  
    segment                       = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'Section_3'   
    segment.percent_span_location = 1.0
    segment.twist                 = 2.0 * Units.degrees 
    segment.root_chord_percent    = 0.4  # 0.5089086
    segment.dihedral_outboard     = 20   * Units.degrees 
    segment.sweeps.quarter_chord  = 26.45 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    wing.Segments.append(segment)                 
    

    # compute reference properties 
    wing_segmented_planform(wing, overwrite_reference = True ) 
    wing = segment_properties(wing)
    vehicle.reference_area        = wing.areas.reference  
    wing.areas.wetted             = wing.areas.reference  * 2 
    wing.areas.exposed            = wing.areas.reference  * 2  
        
    # add to vehicle 
    vehicle.append_component(wing)       
    
    # WING PROPERTIES 
    wing                          = SUAVE.Components.Wings.Wing()
    wing.tag                      = 'horizontal_tail'  
    wing.aspect_ratio             = 4.44533 
    wing.sweeps.quarter_chord     = 20. * Units.degrees
    wing.thickness_to_chord       = 0.12
    wing.taper                    = 0.5
    wing.spans.projected          = 2.52
    wing.chords.root              = 0.9494
    wing.total_length             = 0.9494
    wing.chords.tip               = 0.67
    wing.chords.mean_aerodynamic  = 0.809 
    wing.dihedral                 = 35.*Units.degrees
    wing.areas.reference          = 2.915
    wing.areas.wetted             = 2.915 * 2
    wing.areas.exposed            = 2.915 * 2
    wing.twists.root              = 0.0
    wing.twists.tip               = 0.0
    wing.origin                   = [[  5.7 ,0.0 , 0.27]]
    wing.aerodynamic_center       = [  5.7, 0.0, 0.27] 
    wing.winglet_fraction         = 0.0 
    wing.symmetric                = True    
    
    # add to vehicle 
    vehicle.append_component(wing)       
    
    # ---------------------------------------------------------------   
    # FUSELAGE                
    # ---------------------------------------------------------------   
    # FUSELAGE PROPERTIES
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5  
    fuselage.lengths.tail                       = 1.5
    fuselage.lengths.cabin                      = 4.46 
    fuselage.lengths.total                      = 6.46
    fuselage.width                              = 5.85 * Units.feet      # change 
    fuselage.heights.maximum                    = 4.65 * Units.feet      # change 
    fuselage.heights.at_quarter_length          = 3.75 * Units.feet      # change 
    fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet      # change 
    fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet      # change 
    fuselage.areas.wetted                       = 236. * Units.feet**2   # change 
    fuselage.areas.front_projected              = 0.14 * Units.feet**2   # change 
    fuselage.effective_diameter                 = 1.276     # change 
    fuselage.differential_pressure              = 0. 
    
    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0 
    segment.percent_z_location                  = 0.     # change  
    segment.height                              = 0.049 
    segment.width                               = 0.032 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.026  
    segment.percent_z_location                  = 0.00849
    segment.height                              = 0.481 
    segment.width                               = 0.553 
    fuselage.Segments.append(segment)           
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.074
    segment.percent_z_location                  = 0.02874
    segment.height                              = 1.00
    segment.width                               = 0.912 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                            
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.161  
    segment.percent_z_location                  = 0.04348  
    segment.height                              = 1.41
    segment.width                               = 1.174  
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.284 
    segment.percent_z_location                  = 0.05435 
    segment.height                              = 1.62
    segment.width                               = 1.276  
    fuselage.Segments.append(segment)              
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.531 
    segment.percent_z_location                  = 0.04669 
    segment.height                              = 1.409
    segment.width                               = 1.121 
    fuselage.Segments.append(segment)                     
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.651
    segment.percent_z_location                  = 0.03875 
    segment.height                              = 1.11
    segment.width                               = 0.833
    fuselage.Segments.append(segment)                  
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.773
    segment.percent_z_location                  = 0.03612 
    segment.height                              = 0.78
    segment.width                               = 0.512 
    fuselage.Segments.append(segment)                  
                                                
    # Segment                                             
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.03234 
    segment.height                              = 0.195  
    segment.width                               = 0.130 
    fuselage.Segments.append(segment)                   
                                                
    vehicle.append_component(fuselage) 
    
    #-------------------------------------------------------------------
    # INNER BOOMS   
    #-------------------------------------------------------------------   
    boom                                    = SUAVE.Components.Fuselages.Fuselage()
    boom.tag                                = 'boom_1r'
    boom.configuration                      = 'boom'  
    boom.origin                             = [[ 0.227,  1.413,   0.9]]  
    boom.seats_abreast                      = 0.  
    boom.seat_pitch                         = 0.0 
    boom.fineness.nose                      = 0.950   
    boom.fineness.tail                      = 1.029   
    boom.lengths.nose                       = 0.2 
    boom.lengths.tail                       = 0.2
    boom.lengths.cabin                      = 4.15
    boom.lengths.total                      = 4.55
    boom.width                              = 0.15 
    boom.heights.maximum                    = 0.15  
    boom.heights.at_quarter_length          = 0.15  
    boom.heights.at_three_quarters_length   = 0.15 
    boom.heights.at_wing_root_quarter_chord = 0.15 
    boom.areas.wetted                       = 0.018
    boom.areas.front_projected              = 0.018 
    boom.effective_diameter                 = 0.15  
    boom.differential_pressure              = 0.  
    boom.symmetric                          = True 
    boom.index                              = 1
    
    # Segment  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    boom.Segments.append(segment)           
    
    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.03
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    boom.Segments.append(segment) 
    
    # Segment                                   
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_3'    
    segment.percent_x_location        = 0.97
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    boom.Segments.append(segment)           
    
    # Segment                                  
    segment                           = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    boom.Segments.append(segment)           
    
    # add to vehicle
    vehicle.append_component(boom)   
    
    # add left long boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin[0][1] = -boom.origin[0][1]
    boom.tag          = 'boom_1l' 
    vehicle.append_component(boom)         
     
    # add left long boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       = [[ 0.409,  4.022,  1.0]] 
    boom.tag          = 'boom_2r' 
    boom.lengths.total                      = 4.16
    vehicle.append_component(boom)  
    
    # add outer right boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       = [[ 0.409, 6.631,   1.2 ]]    
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_3r' 
    vehicle.append_component(boom)  
    
    # add inner left boom 
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       = [[ 0.409, -4.022,  1.0 ]]   
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_2l' 
    vehicle.append_component(boom)     
    
    boom              = deepcopy(vehicle.fuselages.boom_1r)
    boom.origin       =  [[ 0.409, -6.631,  1.2 ]]    
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_3l' 
    vehicle.append_component(boom)  
    
 
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'rotor_nacelle'
    nacelle.length         = 0.45
    nacelle.diameter       = 0.3
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through   = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.2
    nac_segment.width              = 0.2
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.25  
    nac_segment.height             = 0.25
    nac_segment.width              = 0.25
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.75
    nac_segment.height             = 0.25
    nac_segment.width              = 0.25
    nacelle.append_segment(nac_segment)        

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 1.0
    nac_segment.height             = 0.2
    nac_segment.width              = 0.2
    nacelle.append_segment(nac_segment)      
 
    lift_rotor_nacelle_origins   = [[ 0.226, 1.413, 0.85] ,[ 0.226, -1.413 , 0.85],
                           [ 4.630 , 1.413 , 0.85] ,[ 4.630 , -1.413 , 0.85],
                           [ 0.409 , 4.022 , 0.95] ,[ 0.409 , -4.022 , 0.95],
                           [ 4.413 , 4.022 , 0.95] ,[ 4.413 , -4.022 , 0.95],
                           [ 0.409 , 6.630 , 1.050] , [0.409 , -6.630 ,1.050],
                           [ 4.413 , 6.630 , 1.050] ,[ 4.413 , -6.630 , 1.050]]
 
    for ii in range(12):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'rotor_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
    
    
    
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'rotor_nacelle'
    nacelle.length         = 1.0
    nacelle.diameter       = 0.4
    nacelle.orientation_euler_angles  = [0.,0.,0.]    
    nacelle.flow_through   = False  
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.10  
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    
    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.25  
    nac_segment.height             = 0.35
    nac_segment.width              = 0.35
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.4
    nac_segment.width              = 0.4
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.75
    nac_segment.height             = 0.35
    nac_segment.width              = 0.35
    nacelle.append_segment(nac_segment)        

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.9
    nac_segment.height             = 0.3
    nac_segment.width              = 0.3
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_7'
    nac_segment.percent_x_location = 1.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)     

    propeller_nacelle_origins   = [[  6.235, 1.3  ,  1.250] ,[  6.235, -1.3  ,  1.250]]

    for ii in range(2):
        propeller_nacelle          = deepcopy(nacelle)
        propeller_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        propeller_nacelle.origin   = [propeller_nacelle_origins[ii]]
        vehicle.append_component(propeller_nacelle)   
            
    #------------------------------------------------------------------
    # network
    #------------------------------------------------------------------
    net                              = Lift_Cruise()
    net.number_of_lift_rotor_engines = 12
    net.number_of_propeller_engines  = 2
    net.nacelle_diameter             = 0.6 * Units.feet
    net.engine_length                = 0.5 * Units.feet
    net.areas                        = Data()
    net.areas.wetted                 = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2 
    net.identical_propellers = True
    net.identical_rotors     = True


    #------------------------------------------------------------------
    # Design Electronic Speed Controller
    #------------------------------------------------------------------
    lift_rotor_esc              = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    lift_rotor_esc.efficiency   = 0.95
    net.lift_rotor_esc          = lift_rotor_esc

    propeller_esc            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    propeller_esc.efficiency = 0.95
    net.propeller_esc        = propeller_esc

    #------------------------------------------------------------------
    # Design Payload
    #------------------------------------------------------------------
    payload                        = SUAVE.Components.Energy.Peripherals.Avionics()
    payload.power_draw             = 10. # Watts 
    payload.mass_properties.mass   = 1.0 * Units.kg
    net.payload                    = payload

    #------------------------------------------------------------------
    # Design Avionics
    #------------------------------------------------------------------
    avionics                       = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw            = 20. # Watts  
    net.avionics                   = avionics  

    #------------------------------------------------------------------
    # Miscellaneous Systems 
    #------------------------------------------------------------------ 
    sys                            = SUAVE.Components.Systems.System()
    sys.mass_properties.mass       = 5 # kg      
    
    #------------------------------------------------------------------
    # Design Battery
    #------------------------------------------------------------------   
    total_cells                         = 150*100
    max_module_voltage                  = 50
    safety_factor                       = 1.5
   
    bat                                 = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
    bat.pack_config.series              = 140  # CHANGE IN OPTIMIZER 
    bat.pack_config.parallel            = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat)    
    net.voltage                         = bat.max_voltage  
    bat.module_config.number_of_modules = 16 # CHANGE IN OPTIMIZER 
    bat.module_config.total             = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage           = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor 
    bat.module_config.layout_ratio      = 0.5 # CHANGE IN OPTIMIZER 
    bat.module_config.normal_count      = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count    = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                         = bat    
    
    #------------------------------------------------------------------
    # Design Rotors and Propellers
    #------------------------------------------------------------------
    # atmosphere and flight conditions for propeller/rotor design  
    g              = 9.81                                   # gravitational acceleration 
    S              = vehicle.reference_area                 # reference area 
    speed_of_sound = 340                                    # speed of sound 
    rho            = 1.22                                   # reference density
    fligth_CL      = 0.75                                   # cruise target lift coefficient 
    AR             = vehicle.wings.main_wing.aspect_ratio   # aspect ratio 
    Cd0            = 0.06                                   # profile drag
    Cdi            = fligth_CL**2/(np.pi*AR*0.98)           # induced drag
    Cd             = Cd0 + Cdi                              # total drag
    V_inf          = 175.* Units['mph']                     # freestream velocity 
    Drag           = S * (0.5*rho*V_inf**2 )*Cd             # cruise drag
    Hover_Load     = vehicle.mass_properties.takeoff*g      # hover load  
    
    # Thrust Propeller        
    propeller                        = SUAVE.Components.Energy.Converters.Propeller()
    propeller.number_of_blades       = 3
    propeller.tag                    = 'propeller_1'
    propeller.freestream_velocity    = V_inf
    propeller.tip_radius             = 1.15
    propeller.hub_radius             = 0.1 * propeller.tip_radius  
    propeller.design_tip_mach        = 0.65
    propeller.angular_velocity       = propeller.design_tip_mach *speed_of_sound/propeller.tip_radius
    propeller.design_Cl              = 0.7
    propeller.design_altitude        = 2500 * Units.feet
    propeller.design_thrust          = 3500 #7000
    propeller.rotation               = 1
    propeller.variable_pitch         = True 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    propeller.airfoil_geometry       =  [rel_path + '../Airfoils/NACA_4412.txt']
    propeller.airfoil_polars         = [[rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]] 
    propeller.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    propeller                        = propeller_design(propeller)
    
    propeller_origins                = [[   7.126, 1.3  ,  1.250 ] ,[  7.126, -1.3  ,  1.250 ]]
    propeller.origin                 = [propeller_origins[0]]
    net.propellers.append(propeller)  

    propeller_2          = deepcopy(propeller)
    propeller_2.tag      = 'propeller_2'
    propeller_2.rotation = -1
    propeller_2.origin   = [propeller_origins[1]]
    net.propellers.append(propeller_2)  

    # Lift Rotors
    rotor                            = SUAVE.Components.Energy.Converters.Lift_Rotor() 
    rotor.tip_radius                 = 1.15
    rotor.hub_radius                 = 0.1 * rotor.tip_radius  
    rotor.number_of_blades           = 3
    rotor.design_tip_mach            = 0.65   
    rotor.inflow_ratio               = 0.06 
    rotor.angular_velocity           = rotor.design_tip_mach* 343 /rotor.tip_radius   
    rotor.freestream_velocity        = rotor.inflow_ratio*rotor.angular_velocity*rotor.tip_radius 
    rotor.design_Cl                  = 0.7
    rotor.design_altitude            = 20 * Units.feet                     
    rotor.design_thrust              = Hover_Load/(net.number_of_lift_rotor_engines) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    rotor.variable_pitch             = True 
    rotor.airfoil_geometry           =  [rel_path + '../Airfoils/NACA_4412.txt']
    rotor.airfoil_polars             = [[rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                         rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    rotor.airfoil_polar_stations     = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    rotor                            = propeller_design(rotor)


    lift_rotor_origins   = [[ 0.226, 1.413, 1.1] ,[  0.226, -1.413 , 1.1],
                           [ 4.630 , 1.413 , 1.1] ,[ 4.630 , -1.413 , 1.1],
                           [0.409 , 4.022 , 1.2] ,[ 0.409 , -4.022 , 1.2],
                           [ 4.413 , 4.022 , 1.2] ,[ 4.413 , -4.022 , 1.2],
                           [ 0.409 , 6.630 , 1.3] ,[ 0.409 , -6.630 , 1.3],
                           [ 4.413 , 6.630 , 1.3] ,[ 4.413 , -6.630 , 1.3]]
    
    # Appending rotors with different origins
    rotations = [-1,1,-1,1,-1,1,-1,1,-1,1,-1,1] 
    angle_offsets        = np.random.rand(12)*(np.pi)    
    for ii in range(12):
        lift_rotor                        = deepcopy(rotor)
        lift_rotor.tag                    = 'lift_rotor_' + str(ii+1)
        lift_rotor.rotation               = rotations[ii]
        lift_rotor.origin                 = [lift_rotor_origins[ii]]
        lift_rotor.azimuthal_offset_angle = angle_offsets[ii]
        net.lift_rotors.append(lift_rotor)   
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller (Thrust) motor
    propeller_motor                      = SUAVE.Components.Energy.Converters.Motor()
    propeller_motor.efficiency           = 0.80
    propeller_motor.origin               = propeller.origin
    propeller_motor.nominal_voltage      = bat.max_voltage 
    propeller_motor.origin               = propeller.origin
    propeller_motor.propeller_radius     = propeller.tip_radius
    propeller_motor.no_load_current      = 0.01
    propeller_motor                      = size_optimal_motor(propeller_motor,propeller)
    propeller_motor.mass_properties.mass = nasa_motor(propeller_motor.design_torque) 
    net.propeller_motors.append(propeller_motor) 

    propeller_motor_2          = deepcopy(propeller_motor)
    propeller_motor_2.tag      = 'propeller_motor_2' 
    propeller_motor_2.origin   = propeller_2.origin
    net.propeller_motors.append(propeller_motor)
    
        
    # Rotor (Lift) Motor     
    lift_rotor_motor                         = SUAVE.Components.Energy.Converters.Motor()
    lift_rotor_motor.efficiency              = 0.9
    lift_rotor_motor.nominal_voltage         = bat.max_voltage *0.75
    lift_rotor_motor.origin                  = rotor.origin 
    lift_rotor_motor.propeller_radius        = rotor.tip_radius   
    lift_rotor_motor.no_load_current         = 0.01 
    lift_rotor_motor                         = size_optimal_motor(lift_rotor_motor,rotor)
    lift_rotor_motor.mass_properties.mass    = nasa_motor(lift_rotor_motor.design_torque)    

    # Appending motors with different origins
    for _ in range(12):
        motor = deepcopy(lift_rotor_motor)
        motor.tag = 'motor_' + str(ii+1)
        net.lift_rotor_motors.append(motor) 

    # append motor origin spanwise locations onto wing data structure
    vehicle.append_component(net)

    # Add extra drag sources from motors, props, and landing gear. All of these hand measured
    motor_height                     = .25 * Units.feet
    motor_width                      = 1.6 * Units.feet
    propeller_width                  = 1. * Units.inches
    propeller_height                 = propeller_width *.12
    main_gear_width                  = 1.5 * Units.inches
    main_gear_length                 = 2.5 * Units.feet
    nose_gear_width                  = 2. * Units.inches
    nose_gear_length                 = 2. * Units.feet
    nose_tire_height                 = (0.7 + 0.4) * Units.feet
    nose_tire_width                  = 0.4 * Units.feet
    main_tire_height                 = (0.75 + 0.5) * Units.feet
    main_tire_width                  = 4. * Units.inches
    total_excrescence_area_spin      = 12.*motor_height*motor_width + 2.*main_gear_length*main_gear_width \
        + nose_gear_width*nose_gear_length + 2*main_tire_height*main_tire_width\
        + nose_tire_height*nose_tire_width
    total_excrescence_area_no_spin   = total_excrescence_area_spin + 12*propeller_height*propeller_width
    vehicle.excrescence_area_no_spin = total_excrescence_area_no_spin
    vehicle.excrescence_area_spin    = total_excrescence_area_spin
 
    rotor_motor_origins                   = np.array(lift_rotor_origins)
    propeller_motor_origins               = np.array(propeller_origins) 
    vehicle.wings['main_wing'].motor_spanwise_locations       = rotor_motor_origins[:,1]/vehicle.wings['main_wing'].spans.projected
    vehicle.wings['horizontal_tail'].motor_spanwise_locations = propeller_motor_origins[:,1]/vehicle.wings['horizontal_tail'].spans.projected 
     

    converge_evtol_weight(vehicle,contingency_factor = 1.0)

    breakdown = empty(vehicle,contingency_factor = 1.0 )
    print(breakdown)
    
    vehicle.weight_breakdown  = breakdown
    compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity()
    
    return vehicle 

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ---------------------------------------------------------------------- 
def analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test)
        analyses[tag] = analysis

    return analyses

# ------------------------------------------------------------------
# Base Analysis
# ------------------------------------------------------------------
def base_analysis(vehicle,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model,hover_noise_test):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle  
    aerodynamics.settings.model_fuselage = True 
    analyses.append(aerodynamics)  
        
    if run_noise_model:  
        # ------------------------------------------------------------------
        #  Noise Analysis
        noise = SUAVE.Analyses.Noise.Fidelity_One()   
        noise.geometry = vehicle
        noise.settings.level_ground_microphone_x_resolution = N_gm_x
        noise.settings.level_ground_microphone_y_resolution = N_gm_y
        noise.settings.level_ground_microphone_min_y        = min_y
        noise.settings.level_ground_microphone_max_y        = max_y
        noise.settings.level_ground_microphone_min_x        = min_x
        noise.settings.level_ground_microphone_max_x        = max_x
        analyses.append(noise)
    
    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses    

# ----------------------------------------------------------------------
#   Define the Configurations
# --------------------------------------------------------------------- 
def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    base_config.networks.lift_cruise.pitch_command = 0
    configs.append(base_config)


    config = SUAVE.Components.Configs.Config(vehicle)
    config.tag = 'transition' 
    configs.append(config)

    config = SUAVE.Components.Configs.Config(vehicle)
    config.tag = 'descent'
    config.networks.lift_cruise.propeller_pitch_command = -5 * Units.degrees
    configs.append(config)

    config = SUAVE.Components.Configs.Config(vehicle)
    config.tag = 'desceleration'
    config.networks.lift_cruise.propeller_pitch_command = -5 * Units.degrees
    configs.append(config)
        
    return configs

# ------------------------------------------------------------------
#   Full Mission Setup
# ------------------------------------------------------------------
def full_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test):


    starting_elevation = 0*Units.feet
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport           = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport      

    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)     

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment           
    base_segment                                             = Segments.Segment() 
    base_segment.battery_discharge                           = True  
    base_segment.state.numerics.number_control_points        = control_points
    ones_row                                                 = base_segment.state.ones_row
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.finalize.post_process.stability  = SUAVE.Methods.skip


    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))  
   

    for day in range(simulated_days):
        print(' ***********  Day ' + str(day) + ' ***********  ')
        for f_idx in range(flights_per_day):
        
            flight_no = f_idx + 1
            # ------------------------------------------------------------------
            #   First Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------
            segment     = Segments.Hover.Climb(base_segment)
            segment.tag = "Vertical_Climb"  + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base )  
            segment.altitude_start                          = 0.0  * Units.ft + starting_elevation 
            segment.altitude_end                            = 40.  * Units.ft + starting_elevation 
            segment.climb_rate                              = 500. * Units['ft/min'] 
            if day == 0:        
                segment.battery_energy                         = vehicle.networks.lift_cruise.battery.max_energy   
            segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
            segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability = SUAVE.Methods.skip
            segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                                    initial_lift_rotor_power_coefficient = 0.02,
                                                                                    initial_throttle_lift = 0.7)
            mission.append_segment(segment)
            
            # ------------------------------------------------------------------
            #   First Cruise Segment: Transition
            # ------------------------------------------------------------------
         
            segment                                    = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
            segment.tag                                = "Vertical_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.transition ) 
            segment.altitude                           = 40.  * Units.ft  + starting_elevation               
            segment.air_speed_start                    = 500. * Units['ft/min']
            segment.air_speed_end                      = 0.8 * Vstall
            segment.acceleration                       = 2.5
            segment.pitch_initial                      = 0.0 * Units.degrees
            segment.pitch_final                        = 2.  * Units.degrees  
            segment.state.unknowns.throttle            = 0.70    *  ones_row(1) 
            segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability = SUAVE.Methods.skip
            segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,  
                                                                 initial_throttle_lift = 0.9) 
            mission.append_segment(segment)
            
            # ------------------------------------------------------------------
            #   First Cruise Segment: Transition
            # ------------------------------------------------------------------ 
            segment                                         = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            segment.tag                                     = "Climb_Transition"+ "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base )    
            segment.altitude_start                          = 40.0 * Units.ft + starting_elevation 
            segment.altitude_end                            = 100.0 * Units.ft + starting_elevation 
            segment.air_speed                               = 0.8   * Vstall
            segment.climb_angle                             = 2     * Units.degrees   
            segment.acceleration                            = 0.25   * Units['m/s/s']    
            segment.pitch_initial                           = 7.    * Units.degrees  
            segment.pitch_final                             = 5.    * Units.degrees   
            segment.state.unknowns.throttle                 = 0.80   *  ones_row(1)
            segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
            segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment, 
                                                                 initial_throttle_lift = 0.8) 
            mission.append_segment(segment) 
          
            # ------------------------------------------------------------------
            #   Second Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
            segment                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                                      = "Climb_1"   + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base ) 
            segment.altitude_start                           = 100.0 * Units.ft + starting_elevation 
            segment.altitude_end                             = 500. * Units.ft + starting_elevation 
            segment.climb_rate                               = 600.  * Units['ft/min']
            segment.air_speed_start                          = 95.   * Units['mph']
            segment.air_speed_end                            = 125.  * Units['mph']  
            segment.state.unknowns.throttle                  = 0.9   *  ones_row(1)  
            segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)   
            mission.append_segment(segment)  
        
            # ------------------------------------------------------------------
            #   Third Climb Segment: Constant Acceleration, Constant Rate
            # ------------------------------------------------------------------ 
            segment                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                                      = "Climb_2" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base ) 
            segment.altitude_start                           = 500.0 * Units.ft + starting_elevation 
            segment.altitude_end                             = 2500. * Units.ft
            segment.climb_rate                               = 500.  * Units['ft/min']
            segment.air_speed_start                          = 125.  * Units['mph']  
            segment.air_speed_end                            = 175.  * Units['mph']    
            segment.state.unknowns.throttle                  = 0.90    *  ones_row(1)
            segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
            mission.append_segment(segment)  
        
            # ------------------------------------------------------------------
            #   Third Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
            segment.tag                                      = "Cruise" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base )                 
            segment.altitude                                 = 2500.0 * Units.ft
            segment.air_speed                                = 175.   * Units['mph']
            cruise_distance                                  = aircraft_range - 32.26 * Units.nmi    
            segment.distance                                 = cruise_distance  
            segment.state.unknowns.throttle                  = 0.8    *  ones_row(1)
            segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
            mission.append_segment(segment) 
           
            # ------------------------------------------------------------------
            #   First Descent Segment: Constant Acceleration, Constant Rate
            # ------------------------------------------------------------------
        
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag = "Descent_1"+ "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base)  
            segment.altitude_start                           = 2500.0 * Units.ft
            segment.altitude_end                             = 100. * Units.ft  + starting_elevation 
            segment.climb_rate                               = -300.  * Units['ft/min']
            segment.air_speed_start                          = 175.  * Units['mph']
            segment.air_speed_end                            = 1.1*Vstall    
            segment.state.unknowns.throttle                  = 0.8   *  ones_row(1)  
            segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
            mission.append_segment(segment) 
          
            if reserve_segment: 
                # ------------------------------------------------------------------
                #   Reserve Segment: Constant Acceleration, Constant Rate
                # ------------------------------------------------------------------
            
                segment     = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag = "Reserve_Climb" + "_F_" + str(flight_no) + "_D" + str (day) 
                segment.analyses.extend( analyses.base ) 
                segment.altitude_start           = 100.0 * Units.ft + starting_elevation 
                segment.altitude_end             = 1000. * Units.ft
                segment.climb_rate               = 500.  * Units['ft/min']
                segment.air_speed_start          = 1.1   * Vstall
                segment.air_speed_end            = 150.  * Units['mph']  
                segment.state.unknowns.throttle   = 0.9   *  ones_row(1)   
                segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment) 
                mission.append_segment(segment) 
              
            
                # ------------------------------------------------------------------
                #   Reserve Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------
            
                segment                                    = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
                segment.tag                                = "Reserve_Cruise" + "_F_" + str(flight_no) + "_D" + str (day) 
                segment.analyses.extend( analyses.base ) 
                segment.altitude                           = 1000.0 * Units.ft
                segment.air_speed                          = 150.   * Units['mph']
                segment.distance                           = cruise_distance*0.1  + 6.6*Units.nmi
                segment.state.unknowns.throttle            = 0.9    *  ones_row(1)
                segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)    
                mission.append_segment(segment)   
              
            
                # ------------------------------------------------------------------
                #   Reserve Descent Segment: Constant Acceleration, Constant Rate
                # ------------------------------------------------------------------ 
                segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                              = "Reserve_Descent" + "_F_" + str(flight_no) + "_D" + str (day) 
                segment.analyses.extend( analyses.base )  
                segment.altitude_start                   = 1000.0 * Units.ft
                segment.altitude_end                     = 100. * Units.ft + starting_elevation 
                segment.climb_rate                       = -300.  * Units['ft/min']
                segment.air_speed_start                  = 150.  * Units['mph']
                segment.air_speed_end                    = 1.1*Vstall    
                segment.state.unknowns.throttle          = 0.80    *  ones_row(1)
                segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment,initial_prop_power_coefficient = 0.016)   
                mission.append_segment(segment)        
             
            
            # ------------------------------------------------------------------
            #   First Cruise Segment: Transition
            # ------------------------------------------------------------------ 
            segment                                          = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
            segment.tag                                      = "Approach_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base ) 
            segment.altitude_start                           = 100.0 * Units.ft + starting_elevation 
            segment.altitude_end                             = 40.0 * Units.ft + starting_elevation 
            segment.air_speed                                = 1.1*Vstall 
            segment.climb_angle                              = 1 * Units.degrees
            segment.acceleration                             = -0.3 * Units['m/s/s']    
            segment.pitch_initial                            = 4. * Units.degrees     
            segment.pitch_final                              = 5. * Units.degrees    
            segment.state.unknowns.throttle                  = 0.70   *  ones_row(1) 
            segment.process.iterate.unknowns.mission         = SUAVE.Methods.skip
            segment.process.iterate.conditions.stability     = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability  = SUAVE.Methods.skip          
            segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,\
                                                                                            initial_throttle_lift = 0.7) 
            mission.append_segment(segment)  
        
            # ------------------------------------------------------------------
            #   Fifth Cuise Segment: Transition
            # ------------------------------------------------------------------ 
            segment                                         = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
            segment.tag                                     = "Descent_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.desceleration ) 
            segment.altitude                                = 40.  * Units.ft     + starting_elevation 
            segment.air_speed_start                         = 103.5  * Units['mph']
            segment.air_speed_end                           = 300. * Units['ft/min'] 
            segment.acceleration                            = -0.75              
            segment.pitch_initial                           =  5.  * Units.degrees  
            segment.pitch_final                             = 10. * Units.degrees  
            segment.state.unknowns.throttle                 = 0.8 *  ones_row(1)  
            segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability = SUAVE.Methods.skip   
            segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,\
                                                                                            initial_throttle_lift = 0.7) 
            mission.append_segment(segment)       
         
            # ------------------------------------------------------------------
            #   Third Descent Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
            segment                                         = Segments.Hover.Descent(base_segment)
            segment.tag                                     = "Vertical_Descent" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.base)     
            segment.altitude_start                          = 40.0  * Units.ft + starting_elevation 
            segment.altitude_end                            = 0.   * Units.ft + starting_elevation 
            segment.descent_rate                            = 300. * Units['ft/min']   
            segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
            segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
            segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
            segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                                    initial_lift_rotor_power_coefficient = 0.02,
                                                                                    initial_throttle_lift = 0.7)
            mission.append_segment(segment)
             
            if recharge_battery:
                # ------------------------------------------------------------------
                #  Charge Segment: 
                # ------------------------------------------------------------------  
                # Charge Model 
                segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                segment.tag                                              = 'Charge Day ' + "_F_" + str(flight_no) + "_D" + str (day)  
                segment.analyses.extend(analyses.base)           
                segment.battery_discharge                                = False   
                if flight_no  == flights_per_day:  
                    segment.increment_battery_cycle_day=True         
                segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment)    
                mission.append_segment(segment)       
            
  
    return mission 

# ------------------------------------------------------------------
#   Noise (Approach/Departure) Mission Setup
# ------------------------------------------------------------------
def approach_departure_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test):


    starting_elevation = 0*Units.feet
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport           = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport      

    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)     

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment           
    base_segment                                             = Segments.Segment() 
    base_segment.battery_discharge                           = True  
    base_segment.state.numerics.number_control_points        = control_points
    ones_row                                                 = base_segment.state.ones_row
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.finalize.post_process.stability  = SUAVE.Methods.skip


    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
     
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Descent_1" 
    segment.analyses.extend( analyses.descent)  
    segment.altitude_start                           = 500.0 * Units.ft
    segment.altitude_end                             = 100. * Units.ft  + starting_elevation 
    segment.climb_rate                               = -300.  * Units['ft/min']
    segment.air_speed_start                          = 175.  * Units['mph'] 
    segment.battery_energy                           = vehicle.networks.lift_cruise.battery.max_energy   
    segment.air_speed_end                            = 1.1*Vstall    
    segment.state.unknowns.throttle                  = 0.8   *  ones_row(1)  
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment,initial_prop_power_coefficient = 0.016)    
    mission.append_segment(segment)  
            
    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------ 
    segment                                          = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag                                      = "Approach_Transition"  
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                           = 100.0 * Units.ft + starting_elevation 
    segment.altitude_end                             = 40.0 * Units.ft + starting_elevation 
    segment.air_speed                                = 1.1*Vstall 
    segment.climb_angle                              = 1 * Units.degrees
    segment.acceleration                             = -0.3 * Units['m/s/s']    
    segment.pitch_initial                            = 4. * Units.degrees     
    segment.pitch_final                              = 1. * Units.degrees    
    segment.state.unknowns.throttle                  = 0.70   *  ones_row(1) 
    segment.process.iterate.unknowns.mission         = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability     = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability  = SUAVE.Methods.skip          
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #   Fifth Cuise Segment: Transition
    # ------------------------------------------------------------------ 
    segment                                         = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                     = "Descent_Transition"  
    segment.analyses.extend( analyses.desceleration ) 
    segment.altitude                                = 40.  * Units.ft     + starting_elevation 
    segment.air_speed_start                         = 103.5  * Units['mph']
    segment.air_speed_end                           = 300. * Units['ft/min'] 
    segment.acceleration                            = -0.75              
    segment.pitch_initial                           =  5.  * Units.degrees  
    segment.pitch_final                             = 20. * Units.degrees  
    segment.state.unknowns.throttle                 = 0.9 *  ones_row(1)  
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip   
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,\
                                                                                    initial_lift_rotor_power_coefficient = 0.02,
                                                                                    initial_throttle_lift = 0.9) 
    mission.append_segment(segment)       
 
    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                         = Segments.Hover.Descent(base_segment)
    segment.tag                                     = "Vertical_Descent" 
    segment.analyses.extend( analyses.base )     
    segment.altitude_start                          = 40.0  * Units.ft + starting_elevation 
    segment.altitude_end                            = 0.   * Units.ft + starting_elevation 
    segment.descent_rate                            = 300. * Units['ft/min']   
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                                    initial_lift_rotor_power_coefficient = 0.02,
                                                                                    initial_throttle_lift = 0.7)
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment     = Segments.Hover.Climb(base_segment)
    segment.tag = "Vertical_Climb"  
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                          = 0.0  * Units.ft + starting_elevation 
    segment.altitude_end                            = 40.  * Units.ft + starting_elevation 
    segment.climb_rate                              = 500. * Units['ft/min']   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                            initial_lift_rotor_power_coefficient = 0.02,
                                                                            initial_throttle_lift = 0.7)
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------
 
    segment                                    = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                = "Vertical_Transition"   
    segment.analyses.extend( analyses.transition ) 
    segment.altitude                           = 40.  * Units.ft  + starting_elevation            
    segment.air_speed_start                    = 500. * Units['ft/min']
    segment.air_speed_end                      = 0.8 * Vstall
    segment.acceleration                       = 1.5  
    segment.pitch_initial                      = 0.0 * Units.degrees
    segment.pitch_final                        = 7. * Units.degrees  
    segment.state.unknowns.throttle            = 0.70    *  ones_row(1) 
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,
                                                         initial_prop_power_coefficient = 0.1, 
                                                         initial_lift_rotor_power_coefficient = 0.016,
                                                         initial_throttle_lift = 0.9) 
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Transition
    # ------------------------------------------------------------------ 
    segment                                      = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
    segment.tag                                  = "Climb_Transition" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                       = 40.0 * Units.ft + starting_elevation 
    segment.altitude_end                         = 100.0 * Units.ft + starting_elevation 
    segment.air_speed                            = 0.8   * Vstall
    segment.climb_angle                          = 2     * Units.degrees
    segment.acceleration                         = 0.25   * Units['m/s/s']    
    segment.pitch_initial                        = 7.    * Units.degrees  
    segment.pitch_final                          = 5.    * Units.degrees   
    segment.state.unknowns.throttle              = 0.70   *  ones_row(1)
    segment.process.iterate.unknowns.mission        = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip 
    segment = vehicle.networks.lift_cruise.add_transition_unknowns_and_residuals_to_segment(segment,
                                                         initial_prop_power_coefficient = 0.1,
                                                         initial_lift_rotor_power_coefficient = 0.016,
                                                         initial_throttle_lift = 0.9) 
    mission.append_segment(segment) 
  
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                      = "Climb_1"   
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                           = 100.0 * Units.ft + starting_elevation 
    segment.altitude_end                             = 500. * Units.ft + starting_elevation 
    segment.climb_rate                               = 600.  * Units['ft/min']
    segment.air_speed_start                          = 95.   * Units['mph']
    segment.air_speed_end                            = Vstall*1.2  
    segment.state.unknowns.throttle                  = 0.9   *  ones_row(1)  
    segment = vehicle.networks.lift_cruise.add_cruise_unknowns_and_residuals_to_segment(segment)   
    mission.append_segment(segment)  

    return mission  
# ------------------------------------------------------------------
#  Hover Noise Mission Setup
# ------------------------------------------------------------------   
def hover_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test):
 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport           = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport      

    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)     

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment           
    base_segment                                             = Segments.Segment() 
    base_segment.battery_discharge                           = True  
    base_segment.state.numerics.number_control_points        = 3 
    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.finalize.post_process.stability  = SUAVE.Methods.skip
 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Hover" # very small climb so that broadband noise does not return 0's  
    segment.analyses.extend(analyses.base) 
    segment.altitude_start                             = 500.0  * Units.ft  
    segment.altitude_end                               = 500.1  * Units.ft 
    segment.climb_rate                                 = 100. * Units['ft/min']  
    segment.battery_energy                             = vehicle.networks.lift_cruise.battery.max_energy   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]  
    segment.process.iterate.unknowns.mission           = SUAVE.Methods.skip
    segment.process.iterate.conditions.stability       = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability    = SUAVE.Methods.skip
    segment = vehicle.networks.lift_cruise.add_lift_unknowns_and_residuals_to_segment(segment,\
                                                                            initial_lift_rotor_power_coefficient = 0.02,
                                                                            initial_throttle_lift = 0.7)
    mission.append_segment(segment)
  
    
    return mission

# ----------------------------------------------------------------------
#   Missions Setup
# ----------------------------------------------------------------------
def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions  



# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_results(results,run_noise_model,line_style='bo-'): 
    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 24,
                  'xtick.labelsize': 20,
                  'ytick.labelsize': 20,
                  'axes.titlesize': 24}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 3 
    plot_parameters.line_style       = '-' 
    plot_parameters.figure_width     = 12 
    plot_parameters.figure_height    = 6 
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 20 
    plot_parameters.plot_grid        = True   
    plot_parameters.markers          = ['o','v','s','P','p','^','D','X','*']
    plot_parameters.colors           = cm.viridis(np.linspace(0,1,5))     
    plot_parameters.lw               = 3                              # line_width               
    plot_parameters.m                = 14                             # markersize               
    plot_parameters.legend_font      = 20                             # legend_font_size         
    plot_parameters.Slc              = ['black','dimgray','silver' ]  # SUAVE_line_colors        
    plot_parameters.Slm              = '^'                            # SUAVE_line_markers       
    plot_parameters.Sls              = '-'                            # SUAVE_line_styles        
    plot_parameters.Elc              = ['firebrick','red','tomato']  # Experimental_line_colors 
    plot_parameters.Elm              = 's'                            # Experimental_line_markers
    plot_parameters.Els              = '-'                            # Experimental_line_styles 
    plot_parameters.Rlc              = ['mediumblue','blue','cyan']   # Ref_Code_line_colors     
    plot_parameters.Rlm              = 'o'                            # Ref_Code_line_markers    
    plot_parameters.Rls              = '--'                           # Ref_Code_line_styles     
    
    plot_paper_results(results,plot_parameters)
    
    ## Plot Flight Conditions 
    #plot_flight_conditions(results, line_style) 
    
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    ## Plot Aircraft Flight Speed
    #plot_aircraft_velocities(results, line_style) 

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)    
    
    # Plot Electric Motor and Propeller Efficiencies  of Lift Cruise Network
    plot_lift_cruise_network(results, line_style)   

    ## Plot Battery Degradation  
    #plot_battery_degradation(results, line_style)    

    #if run_noise_model:     
        ## Plot noise level
        #plot_ground_noise_levels(results)
        
        ## Plot noise contour
        #plot_flight_profile_noise_contours(results) 
    return     

def plot_paper_results(results,PP):  
    file_type = ".png"
    
    save_filename_1 = "SR_Altitude"
    save_filename_2 = "SR_CL"
    save_filename_3 = "SR_RPM"
    fig_1 = plt.figure(save_filename_1)
    fig_1.set_size_inches(PP.figure_width, PP.figure_height) 
    fig_2 = plt.figure(save_filename_2)
    fig_2.set_size_inches(PP.figure_width, PP.figure_height) 
    fig_3 = plt.figure(save_filename_3)
    fig_3.set_size_inches(PP.figure_width, PP.figure_height) 
    
    size = 12
    axes_1 = fig_1.add_subplot(1,1,1)
    axes_2 = fig_2.add_subplot(1,1,1)
    axes_3 = fig_3.add_subplot(1,1,1)
    i = 0
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min 
        altitude = segment.conditions.freestream.altitude[:,0]/Units.feet  
        cl       = segment.conditions.aerodynamics.lift_coefficient[:,0,None]
        prop_rpm     = segment.conditions.propulsion.propeller_rpm[:,0]  
        lift_rotor_rpm    = segment.conditions.propulsion.lift_rotor_rpm[:,0]  
            
                
        axes_1.plot(time, altitude, color = 'blue', marker = 'o', markersize = size, linewidth = PP.lw)
        axes_1.set_ylabel('Altitude (ft)')
        axes_1.set_xlabel('Time (min)')
        
        
        axes_2.plot( time , cl, color = 'blue', marker = 'o', markersize = size , linewidth = PP.lw)
        axes_2.set_ylabel('C$_{L}$' )
        axes_2.set_xlabel('Time (min)')
         
      
        axes_3.plot(time, prop_rpm , color = 'blue', marker = 'o', markersize = size , linewidth = PP.lw,label='Propeller')
        axes_3.plot(time, lift_rotor_rpm, color = 'red', marker = '^', markersize = size , linewidth = PP.lw ,label='Lift-Rotor')
        axes_3.set_ylabel('RPM')
        axes_3.set_xlabel('Time (min)')
        if i == 0:
            axes_3.legend(loc='upper right', prop={'size': PP.legend_font})  
        i+= 1
        
    axes_1.set_ylim([0,3000])
    axes_2.set_ylim([0,1.5])
    axes_3.set_ylim([0,2000])
    set_axes(axes_1) 
    set_axes(axes_2)   
    set_axes(axes_3)      
    
    fig_1.tight_layout()  
    fig_2.tight_layout()  
    fig_3.tight_layout()  
    fig_1.savefig(save_filename_1 + file_type)
    fig_2.savefig(save_filename_2 + file_type)
    fig_3.savefig(save_filename_3 + file_type)
        
    return

# ------------------------------------------------------------------
#   Set Axis Parameters 
# ------------------------------------------------------------------
## @ingroup Plots
def set_axes(axes):
    """This sets the axis parameters for all plots

    Assumptions:
    None

    Source:
    None

    Inputs
    axes
        
    Outputs: 
    axes

    Properties Used:
    N/A	
    """   
    
    axes.minorticks_on()
    axes.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axes.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')      
    axes.grid(True)   
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)        

    return  


# ----------------------------------------------------------------------
#   Save Results
# ----------------------------------------------------------------------
def save_results(results,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(results, file) 
    return   

# ------------------------------------------------------------------
#   Load Results
# ------------------------------------------------------------------   
def load_results(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file)
        
    return results

if __name__ == '__main__': 
    main()    
    plt.show()
     
