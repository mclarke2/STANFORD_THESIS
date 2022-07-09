# Multirotor.py 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data  
import pickle
import time 
import os
from copy import deepcopy
from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Plots.Geometry  import *  
from SUAVE.Analyses.Propulsion.Rotor_Wake_Fidelity_Zero import Rotor_Wake_Fidelity_Zero
from SUAVE.Analyses.Propulsion.Rotor_Wake_Fidelity_One import Rotor_Wake_Fidelity_One
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
from SUAVE.Methods.Power.Battery.Sizing                        import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion                                  import propeller_design 
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                import empty 
from SUAVE.Methods.Propulsion.electric_motor_sizing            import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion             import nasa_motor 
import numpy as np

from SUAVE.Methods.Weights.Buildups.eVTOL.converge_evtol_weight    import converge_evtol_weight  

try:
    import vsp 
    from SUAVE.Input_Output.OpenVSP.vsp_write import write 
except ImportError:
    # This allows SUAVE to build without OpenVSP
    pass 

def main():    
    simulated_days   = 1
    flights_per_day  = 1 
    aircraft_range   = 25 *Units.nmi
    reserve_segment  = False 
    plot_geometry    = False
    recharge_battery = False
    run_analysis     = True
    plot_mission     = True
    control_points   =  5 # 20
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
    #write(vehicle,"Multirotor") 
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
    filename          = 'Multirotor_Full_Mission'  
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

    Y_LIM = np.linspace(1E-1,5*Units.nmi,3)    
    end_distance = aircraft_range/((N_gm_x-2)*2)
    X_LIM = np.linspace(-end_distance+1E1,aircraft_range + end_distance+1E1,3)      
    
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
            noise_mission     = full_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
            missions_analyses = missions_setup(noise_mission) 
            analyses          = SUAVE.Analyses.Analysis.Container()
            analyses.configs  = configs_analyses
            analyses.missions = missions_analyses 
            configs.finalize()
            analyses.finalize()      
            noise_mission     = analyses.missions.base
            noise_results     = noise_mission.evaluate()   
            filename          = 'Multirotor_Full_Mission_Noise_Q' + str(Q_idx)+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
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

    Y_LIM        = np.linspace(1E-6,0.5*Units.nmi,3)    
    X_LIM        = np.linspace(1E-3, 2.45*Units.nmi,3)       
         
    
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
            filename          = 'Multirotor_Approach_Departure_Noise_Q' + str(Q_idx)+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
            
            tf_quad = time.time() 
            print ('time taken: '+ str(round(((tf_quad-ti_quad)/60),3)) + ' mins') 
            
            save_results(noise_results,filename)  
            Q_idx += 1 
        
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
    filename          = 'Multirotor_Hover_Mission'+ '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    save_results(hover_results,filename)  
    
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = hover_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')   
    
    return 


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------
def analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model, hover_noise_test ):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model, hover_noise_test )
        analyses[tag] = analysis 
        
    return analyses

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(): 
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Multirotor_CRM'
    vehicle.configuration                       = 'eVTOL'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff             = 3200 #3800   
    vehicle.mass_properties.operating_empty     = 3200 ##3800  
    vehicle.mass_properties.max_takeoff         = 3200 ##3800  
    vehicle.mass_properties.center_of_gravity   = [[2.6, 0., 0. ] ] 
                                                
    # This needs updating                       
    vehicle.passengers                          = 6
    vehicle.reference_area                      = 73  * Units.feet**2 
    vehicle.envelope.ultimate_load              = 5.7   
    vehicle.envelope.limit_load                 = 3.  
                                                
    wing                                        = SUAVE.Components.Wings.Main_Wing()  # this is the body of the vehicle 
    wing.tag                                    = 'main_wing'   
    wing.aspect_ratio                           = 0.5 
    wing.sweeps.quarter_chord                   = 0.  
    wing.thickness_to_chord                     = 0.01   
    wing.spans.projected                        = 0.01  
    wing.chords.root                            = 0.01
    wing.total_length                           = 0.01
    wing.chords.tip                             = 0.01
    wing.chords.mean_aerodynamic                = 0.01
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = 0.0001 
    wing.areas.wetted                           = 0.01
    wing.areas.exposed                          = 0.01  
    wing.symbolic                               = True 
    wing.symmetric                              = True 
    
    vehicle.append_component(wing)
    
    # ------------------------------------------------------    
    # FUSELAGE    
    # ------------------------------------------------------    
    # FUSELAGE PROPERTIES
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5 
    fuselage.lengths.tail                       = 0.5
    fuselage.lengths.cabin                      = 4.
    fuselage.lengths.total                      = 5.
    fuselage.width                              = 1.8
    fuselage.heights.maximum                    = 1.8
    fuselage.heights.at_quarter_length          = 1.8
    fuselage.heights.at_wing_root_quarter_chord = 1.8
    fuselage.heights.at_three_quarters_length   = 1.8
    fuselage.areas.wetted                       = 19.829265
    fuselage.areas.front_projected              = 1.4294246 
    fuselage.effective_diameter                 = 1.4
    fuselage.differential_pressure              = 1. 
    
    # Segment  
    segment                          = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                      = 'segment_0'   
    segment.percent_x_location       = 0.  
    segment.percent_z_location       = 0.0 
    segment.height                   = 0.1   
    segment.width                    = 0.1   
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_1'   
    segment.percent_x_location      = 0.200/4.
    segment.percent_z_location      = 0.1713/4.
    segment.height                  = 0.737
    segment.width                   = 1.2
    segment.vsp_data.top_angle      = 53.79 * Units.degrees 
    segment.vsp_data.bottom_angle   = 28.28 * Units.degrees     
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_2'   
    segment.percent_x_location      = 0.8251/4.
    segment.percent_z_location      = 0.2840/4.
    segment.height                  = 1.40 
    segment.width                   = 1.8
    segment.vsp_data.top_angle      = 0 * Units.degrees 
    segment.vsp_data.bottom_angle   = 0 * Units.degrees     
    fuselage.append_segment(segment)            
                                                
    # Segment                                  
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_3'   
    segment.percent_x_location      = 3.342/4.
    segment.percent_z_location      = 0.356/4.
    segment.height                  = 1.40
    segment.width                   = 1.8
    #segment.vsp_data.top_angle      = 0 * Units.degrees 
    #segment.vsp_data.bottom_angle   = 0 * Units.degrees     
    fuselage.append_segment(segment)  
                                                
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_4'   
    segment.percent_x_location      = 3.70004/4.
    segment.percent_z_location      = 0.4636/4.
    segment.height                  = 0.9444
    segment.width                   = 1.2
    segment.vsp_data.top_angle      = -36.59 * Units.degrees 
    segment.vsp_data.bottom_angle   = -57.94 * Units.degrees 
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                         = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                     = 'segment_5'   
    segment.percent_x_location      = 1.
    segment.percent_z_location      = 0.6320/4.
    segment.height                  = 0.1    
    segment.width                   = 0.1    
    fuselage.append_segment(segment)             
    
                                                 
    # add to vehicle
    vehicle.append_component(fuselage)   
    
    
    
    #------------------------------------------------------------------
    # Network
    #------------------------------------------------------------------
    net                      = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines    = 6
    net.nacelle_diameter     = 0.6 * Units.feet # need to check 
    net.engine_length        = 0.5 * Units.feet
    net.areas                = Data()
    net.areas.wetted         = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2 
    net.identical_propellers= True

    #------------------------------------------------------------------
    # Design Electronic Speed Controller 
    #------------------------------------------------------------------
    esc             = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency  = 0.95
    net.esc         = esc
    
    #------------------------------------------------------------------
    # Design Payload
    #------------------------------------------------------------------
    payload                       = SUAVE.Components.Energy.Peripherals.Avionics()
    payload.power_draw            = 0. 
    net.payload                   = payload

    #------------------------------------------------------------------
    # Design Avionics
    #------------------------------------------------------------------
    avionics            = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 200. * Units.watts
    net.avionics        = avionics
                                                
    #------------------------------------------------------------------
    # Design Battery
    #------------------------------------------------------------------  
    total_cells                           = 150*270
    max_module_voltage                    = 50
    safety_factor                         = 1.5
     
    bat                                   = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
    bat.pack_config.series                = 150  # CHANGE IN OPTIMIZER 
    bat.pack_config.parallel              = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat,module_weight_factor = 1.05)    
    net.voltage                           = bat.max_voltage  
    bat.module_config.number_of_modules   = 20 # CHANGE IN OPTIMIZER 
    bat.module_config.total               = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage             = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor 
    bat.module_config.layout_ratio        = 0.5 # CHANGE IN OPTIMIZER 
    bat.module_config.normal_count        = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count      = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                           = bat        
     
    #------------------------------------------------------------------
    # Design Rotors  
    #------------------------------------------------------------------ 
    # atmosphere and flight conditions for propeller/lift_rotor design
    g                            = 9.81                                   # gravitational acceleration  
    speed_of_sound               = 340                                    # speed of sound 
    Hover_Load                   = vehicle.mass_properties.takeoff*g      # hover load   
    design_tip_mach              = 0.7                                    # design tip mach number 
    
    rotor                        = SUAVE.Components.Energy.Converters.Lift_Rotor() 
    rotor.tip_radius             = 2.5
    rotor.hub_radius             = 0.1*rotor.tip_radius  
    rotor.disc_area              = np.pi*(rotor.tip_radius**2) 
    rotor.number_of_blades       = 3
    rotor.freestream_velocity    = 10.0
    rotor.angular_velocity       = (design_tip_mach*speed_of_sound)/rotor.tip_radius   
    rotor.design_Cl              = 0.7
    rotor.design_altitude        = 1000 * Units.feet                   
    rotor.design_thrust          = Hover_Load/(net.number_of_propeller_engines) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    rotor.airfoil_geometry         =  [ rel_path + '../Airfoils/NACA_4412.txt']
    rotor.airfoil_polars           = [[ rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                        rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                        rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                        rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                        rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]  
    rotor.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    rotor.variable_pitch         = True 
    
    #rotor.Wake                                          = Rotor_Wake_Fidelity_One()
    #rotor.use_2d_analysis                               = True  
    #rotor.sol_tolerance                                 = 1e-6
    #rotor.Wake.wake_settings.number_rotor_rotations     = 2 # 5    
    #rotor.number_azimuthal_stations                     = 24  
    #rotor.Wake.wake_settings.initial_timestep_offset    = 0    
    #rotor.Wake.maximum_convergence_iteration            = 10
    #rotor.Wake.axial_velocity_convergence_tolerance     = 1e-2  
    #rotor.Wake.wake_settings.number_steps_per_rotation  = rotor.number_azimuthal_stations
    
    rotor                        = propeller_design(rotor)     
    
    # Appending rotors with different origins
    origins                 = [[ -1.5,2.6,1.7],[ -1.5,-2.6,1.7],
                                [2.5,6.0,1.7] ,[2.5,-6.,1.7],
                                [6.5,2.6,1.7] ,[6.5,-2.6,1.7]]   
    
    for ii in range(6):
        lift_rotor          = deepcopy(rotor)
        lift_rotor.tag      = 'mr_lift_rotor_' + str(ii+1)
        lift_rotor.origin   = [origins[ii]]
        net.propellers.append(lift_rotor)
     
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'lift_rotor_nacelle'
    nacelle.length         = 0.5
    nacelle.diameter       = 0.8
    nacelle.inlet_diameter = 0.6
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through   = True 
 
    lift_rotor_nacelle_origins   =  [[ -1.5,2.6,1.9],[ -1.5,-2.6,1.9],
                                [2.5,6.0,1.9] ,[2.5,-6.,1.9],
                                [6.5,2.6,1.9] ,[6.5,-2.6,1.9]]   
 
    for ii in range(6):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'engine_nacelle_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
       

    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'cowling'
    nacelle.length         = 0.4
    nacelle.diameter       = 2.6*2
    nacelle.inlet_diameter = 2.55*2
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through   = True 
 
    lift_rotor_nacelle_origins   =  [[ -1.5,2.6,1.8],[ -1.5,-2.6,1.8],
                                [2.5,6.0,1.8] ,[2.5,-6.,1.8],
                                [6.5,2.6,1.8] ,[6.5,-2.6,1.8]]  
    
 
    for ii in range(6):
        rotor_nacelle          = deepcopy(nacelle)
        rotor_nacelle.tag      = 'cowling_' + str(ii+1) 
        rotor_nacelle.origin   = [lift_rotor_nacelle_origins[ii]]
        vehicle.append_component(rotor_nacelle)   
   
       
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Motor 
    lift_motor                      = SUAVE.Components.Energy.Converters.Motor() 
    lift_motor.efficiency           = 0.95
    lift_motor.nominal_voltage      = bat.max_voltage 
    lift_motor.mass_properties.mass = 3. * Units.kg  
    lift_motor.propeller_radius     = rotor.tip_radius   
    lift_motor.no_load_current      = 2.0     
    lift_motor                      = size_optimal_motor(lift_motor,rotor)
    lift_motor.mass_properties.mass   = nasa_motor(lift_motor.design_torque)   - 20 # NEED BETTER MOTOR MODEL
    net.lift_motor                    = lift_motor   
     
    # Appending motors with different origins    
    for ii in range(6):
        lift_rotor_motor = deepcopy(lift_motor)
        lift_rotor_motor.tag = 'motor_' + str(ii+1)
        lift_motor.origin   = [origins[ii]]
        net.propeller_motors.append(lift_rotor_motor)       
    
    vehicle.append_component(net)
    
    vehicle.wings['main_wing'].motor_spanwise_locations   = np.array(origins)[:,1]/ vehicle.wings['main_wing'].spans.projected
 
    
    converge_evtol_weight(vehicle,contingency_factor = 1.0) 

    breakdown = empty(vehicle,contingency_factor = 1.0 )
    print(breakdown)
    
    vehicle.weight_breakdown  = breakdown
    compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity() 
    
    return vehicle



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
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'hover' 
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 0.  * Units.degrees
    configs.append(config)
    
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'vertical_climb'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 0.  * Units.degrees
    configs.append(config)
    
  
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'vertical_transition'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 3.  * Units.degrees
    configs.append(config)
      
      
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'descent_transition'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 3.  * Units.degrees
    configs.append(config)
  
    
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'climb'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     =2.  * Units.degrees
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'   
    for rotor in config.networks.battery_propeller.propellers:  
        rotor.inputs.pitch_command                     = 5.  * Units.degrees
    configs.append(config)     
    
    return configs

# ------------------------------------------------------------------
#   Base Analysis
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


# ------------------------------------------------------------------
#   Full Mission Setup
# ------------------------------------------------------------------     
def full_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range, reserve_segment,control_points,recharge_battery, hover_noise_test ):
    
     
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    starting_elevation  = 0 * Units.ft
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
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
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip    
    
     
    for day in range(simulated_days): 
        print(' ***********  Day ' + str(day) + ' ***********  ')
        for f_idx in range(flights_per_day): 
            flight_no = f_idx + 1        
            
            # ------------------------------------------------------------------
            #   First Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
        
            segment                                            = Segments.Hover.Climb(base_segment)
            segment.tag                                        = "Vertical_Climb"+ "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.vertical_climb) 
            segment.altitude_start                             = 0.0  * Units.ft + starting_elevation
            segment.altitude_end                               = 40.  * Units.ft + starting_elevation
            segment.climb_rate                                 = 300. * Units['ft/min'] 
            if day == 0:        
                segment.battery_energy                         = vehicle.networks.battery_propeller.battery.max_energy   
            segment.battery_pack_temperature                   = atmo_data.temperature[0,0]
            segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
            mission.append_segment(segment)
            
            # ------------------------------------------------------------------
            #  First Transition Segment
            # ------------------------------------------------------------------  
        
            segment                                  = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
            segment.tag                              = "Vertical_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.vertical_transition) 
            segment.altitude                         = 40.  * Units.ft         + starting_elevation
            segment.air_speed_start                  = 300. * Units['ft/min'] 
            segment.air_speed_end                    = 35 * Units['mph']    
            segment.acceleration                     = 0.5
            segment.pitch_initial                    = 0. * Units.degrees
            segment.pitch_final                      = 0. * Units.degrees 
            segment.state.unknowns.throttle          = 0.8  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
            mission.append_segment(segment)
        
            
            # ------------------------------------------------------------------
            #   First Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------
            segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                              = "Climb_1" + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.climb) 
            segment.climb_rate                       = 600. * Units['ft/min']
            segment.air_speed_start                  = 35.   * Units['mph']
            segment.air_speed_end                    = 55.  * Units['mph']       
            segment.altitude_start                   = 40.0 * Units.ft    + starting_elevation
            segment.altitude_end                     = 500.0 * Units.ft   
            segment.state.unknowns.throttle          = 0.8  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
            mission.append_segment(segment)
            
            # ------------------------------------------------------------------
            #   First Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------
            segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                              = "Climb_2" + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.climb) 
            segment.climb_rate                       = 500. * Units['ft/min']
            segment.air_speed_start                  = 55.   * Units['mph']
            segment.air_speed_end                    = 75.  * Units['mph']       
            segment.altitude_start                   = 500.0 * Units.ft    + starting_elevation
            segment.altitude_end                     = 2500.0 * Units.ft   
            segment.state.unknowns.throttle          = 0.8  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
            mission.append_segment(segment)                
        
            # ------------------------------------------------------------------
            #   First Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                                  = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
            segment.tag                              = "Cruise" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend(analyses.cruise)  
            segment.altitude                         = 2500.0 * Units.ft      
            segment.air_speed                        = 75. * Units['mph']      
            cruise_distance                          = aircraft_range -14.29 * Units.nmi
            segment.distance                         = cruise_distance   
            segment.state.unknowns.throttle          = 0.8  * ones_row(1)   
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)   
            mission.append_segment(segment)      
            
                        
            # ------------------------------------------------------------------
            #   First Descent Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                              = "Descent_1" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend(analyses.cruise)
            segment.climb_rate                       = -200. * Units['ft/min']
            segment.air_speed_start                  = 75. * Units['mph']      
            segment.air_speed_end                    = 35. * Units['mph']      
            segment.altitude_start                   = 2500.0 * Units.ft 
            segment.altitude_end                     = 40.0 * Units.ft   + starting_elevation
            segment.state.unknowns.throttle          = 0.8  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
            mission.append_segment(segment)     
            
            if reserve_segment : 
                # ------------------------------------------------------------------
                #  Reserve Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------
                segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                              = "Reserve_Climb" + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.cruise) 
                segment.climb_rate                       = 600. * Units['ft/min']
                segment.air_speed_start                  = 35.   * Units['mph']
                segment.air_speed_end                    = 55.   * Units['mph']      
                segment.altitude_start                   = 100.0 * Units.ft  + starting_elevation
                segment.altitude_end                     = 1000.0 * Units.ft       
                segment.state.unknowns.throttle          = 0.8  * ones_row(1)  
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
                mission.append_segment(segment)    
            
                # ------------------------------------------------------------------
                #   Reserve Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                                  = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
                segment.tag                              = "Reserve_Cruise" + "_F_" + str(flight_no) + "_D" + str (day) 
                segment.analyses.extend(analyses.cruise) 
                segment.altitude                         = 1000.0 * Units.ft      
                segment.air_speed                        = 55.    * Units['mph']      
                segment.distance                         = cruise_distance*0.1 + 2.7 *Units.nmi 
                segment.state.unknowns.throttle          = 0.8  * ones_row(1)   
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
                mission.append_segment(segment)      
                
                            
                # ------------------------------------------------------------------
                #   Reserve Descent: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                                   = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                               = "Reserve_Descent" + "_F_" + str(flight_no) + "_D" + str (day) 
                segment.analyses.extend(analyses.cruise)
                segment.climb_rate                        = -200. * Units['ft/min']
                segment.air_speed_start                   = 55.  * Units['mph']      
                segment.air_speed_end                     = 35. * Units['mph']      
                segment.altitude_start                    = 1000.0 * Units.ft   
                segment.altitude_end                      = 40.0 * Units.ft      + starting_elevation 
                segment.state.unknowns.throttle          = 0.6  * ones_row(1)  
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
                mission.append_segment(segment)     
             
            # ------------------------------------------------------------------
            #  Third Transition Segment
            # ------------------------------------------------------------------
   
            segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
            segment.tag                       = "Decent_Transition" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.descent_transition) 
            segment.altitude                  = 40.  * Units.ft + starting_elevation
            segment.air_speed_start           = 35.  * Units['mph'] 
            segment.air_speed_end             = 300. * Units['ft/min']
            segment.acceleration              = -0.5307 
            segment.pitch_initial             = 1. * Units.degrees
            segment.pitch_final               = 2. * Units.degrees       
            segment.state.unknowns.throttle   = 0.5  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
            mission.append_segment(segment)
        
            
            # ------------------------------------------------------------------
            #   First Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
            segment                           = Segments.Hover.Descent(base_segment)
            segment.tag                       = "Vertical_Descent" + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend( analyses.vertical_climb) 
            segment.altitude_start            = 40.0  * Units.ft  + starting_elevation
            segment.altitude_end              = 0.  * Units.ft + starting_elevation
            segment.descent_rate              = 300. * Units['ft/min']  
            segment.state.unknowns.throttle   = 0.5  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
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
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    
                mission.append_segment(segment)  
             
            
    return mission
         
# ------------------------------------------------------------------
#   Noise (Approach/Departure) Mission Setup
# ------------------------------------------------------------------     
def approach_departure_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range, reserve_segment,control_points,recharge_battery, hover_noise_test ):
      
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    starting_elevation  = 0 * Units.ft
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
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
    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip  
     
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                              = "Descent_1"  
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate                       = -200. * Units['ft/min']
    segment.battery_energy                   = vehicle.networks.battery_propeller.battery.max_energy  
    segment.air_speed_start                  = 75. * Units['mph']      
    segment.air_speed_end                    = 35. * Units['mph']      
    segment.altitude_start                   = 500.0 * Units.ft 
    segment.altitude_end                     = 40.0 * Units.ft   + starting_elevation
    segment.state.unknowns.throttle          = 0.6  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)      
    # ------------------------------------------------------------------
    #  Third Transition Segment
    # ------------------------------------------------------------------

    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "Decent_Transition"  
    segment.analyses.extend( analyses.descent_transition) 
    segment.altitude                  = 40.  * Units.ft + starting_elevation
    segment.air_speed_start           = 35.  * Units['mph'] 
    segment.air_speed_end             = 300. * Units['ft/min']
    segment.acceleration              = -0.5307 
    segment.pitch_initial             = 1. * Units.degrees
    segment.pitch_final               = 2. * Units.degrees       
    segment.state.unknowns.throttle   = 0.5  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)

    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                           = Segments.Hover.Descent(base_segment)
    segment.tag                       = "Vertical_Descent"  
    segment.analyses.extend( analyses.vertical_climb) 
    segment.altitude_start            = 40.0  * Units.ft  + starting_elevation
    segment.altitude_end              = 0.  * Units.ft + starting_elevation
    segment.descent_rate              = 300. * Units['ft/min']  
    segment.state.unknowns.throttle   = 0.5  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)

    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 

    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb" 
    segment.analyses.extend( analyses.vertical_climb) 
    segment.altitude_start                             = 0.0  * Units.ft + starting_elevation
    segment.altitude_end                               = 40.  * Units.ft + starting_elevation
    segment.climb_rate                                 = 300. * Units['ft/min']  
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]
    segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------

    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "Decent_Transition"  
    segment.analyses.extend( analyses.descent_transition) 
    segment.altitude                  = 40.  * Units.ft + starting_elevation
    segment.air_speed_start           = 35.  * Units['mph'] 
    segment.air_speed_end             = 300. * Units['ft/min']
    segment.acceleration              = -0.5307 
    segment.pitch_initial             = 1. * Units.degrees
    segment.pitch_final               = 2. * Units.degrees       
    segment.state.unknowns.throttle   = 0.5  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)

    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------
    segment                                  = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                              = "Climb_1"  
    segment.analyses.extend(analyses.climb) 
    segment.climb_rate                       = 600. * Units['ft/min']
    segment.air_speed_start                  = 35.   * Units['mph']
    segment.air_speed_end                    = 55.  * Units['mph']       
    segment.altitude_start                   = 40.0 * Units.ft    + starting_elevation
    segment.altitude_end                     = 500.0 * Units.ft   
    segment.state.unknowns.throttle          = 0.8  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment) 
    mission.append_segment(segment)
         
    return mission


# ------------------------------------------------------------------
#  Hover Noise Mission Setup
# ------------------------------------------------------------------   
def hover_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range, reserve_segment,control_points,recharge_battery, hover_noise_test ):
    
     
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------ 
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
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
    ones_row                                                 = base_segment.state.ones_row    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip    
     
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Hover" # very small climb so that broadband noise does not return 0's  
    segment.analyses.extend(analyses.vertical_climb) 
    segment.altitude_start                             = 500.0  * Units.ft  
    segment.altitude_end                               = 500.1  * Units.ft 
    segment.climb_rate                                 = 100. * Units['ft/min']  
    segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]              
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]
    segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)  
    mission.append_segment(segment)
    
    return mission

# ------------------------------------------------------------------
#   Missions Setup
# ------------------------------------------------------------------   
def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission


    # done!
    return missions   

# ------------------------------------------------------------------
#   Plot Results
# ------------------------------------------------------------------   
def plot_results(results,run_noise_model,line_style='bo-'): 
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)
    
    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results, line_style)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)
    
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)    

    # Plot Battery Degradation  
    plot_battery_degradation(results, line_style)    
    

    if run_noise_model:     
        # Plot noise level
        plot_ground_noise_levels(results)
        
        # Plot noise contour
        plot_flight_profile_noise_contours(results) 
        
    return 

# ------------------------------------------------------------------
#   Save Results
# ------------------------------------------------------------------   
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
