# Tiltwing.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data

from SUAVE.Methods.Power.Battery.Sizing                                   import initialize_from_mass 
from SUAVE.Methods.Power.Battery.Sizing                                   import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing                       import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion                        import nasa_motor
from SUAVE.Methods.Propulsion                                             import propeller_design
from SUAVE.Plots.Geometry                                                 import *
from SUAVE.Plots.Performance.Mission_Plots                                import *
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                           import empty
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity import compute_component_centers_of_gravity
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.generate_microphone_points import generate_building_microphone_points
from copy import deepcopy
import pickle 
import time 
import os
import vsp 
from SUAVE.Input_Output.OpenVSP.vsp_write import write

import numpy as np
import pylab as plt 

import plotly.graph_objects as go  
 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 

    ti = time.time()
    run_analysis = True 
    
    if run_analysis:
        simulated_days  = 1
        flights_per_day = 1 
        aircraft_range  = 80 *Units.miles 
        reserve_segment = True 
        control_points  = 16  
        recharge_battery = False 
        # build the vehicle, configs, and analyses
        configs, analyses = full_setup(simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery )
    
        breakdown = empty(configs.base,contingency_factor=1.0)
        print(breakdown)    
        
        configs.finalize()
        analyses.finalize()    
    
        # mission analysis
        mission = analyses.missions.base
        results = mission.evaluate() 
    
        tf = time.time()
        print ('time taken: ' + str(round(((tf-ti)/60),3)) + ' mins')      
        
        # save results  
        save_results(results)
    
    else:
        results = load_results('Tiltwing_Building.pkl')
    # plot the results
    plot_results(results) 

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup(simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery  ):    

    # vehicle data
    vehicle  = vehicle_setup()
    #write(vehicle, "Tiltwing_VSP_NEW") 
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs )

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses


    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs ):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config )
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):

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

    # ------------------------------------------------------------------
    #  Noise Analysis
    noise = SUAVE.Analyses.Noise.Fidelity_One()   
    noise.geometry = vehicle 
    urban_canyon_microphone_array,building_locations,building_dimensions,N_x,N_y,N_z = urban_canyon_microphone_setup() 
    noise.settings.urban_canyon_microphone_locations    = urban_canyon_microphone_array
    noise.settings.urban_canyon_building_locations      = building_locations
    noise.settings.urban_canyon_building_dimensions     = building_dimensions
    noise.settings.urban_canyon_microphone_x_resolution = N_x 
    noise.settings.urban_canyon_microphone_y_resolution = N_y
    noise.settings.urban_canyon_microphone_z_resolution = N_z
    noise.settings.level_ground_microphone_x_resolution = 10 
    noise.settings.level_ground_microphone_y_resolution = 10  
      
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


def vehicle_setup(): 
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------     
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Tiltwing_CRM'
    vehicle.configuration                       = 'eVTOL'
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff             = 2200
    vehicle.mass_properties.operating_empty     = 2200
    vehicle.mass_properties.max_takeoff         = 2200
    vehicle.mass_properties.center_of_gravity   = [[ 2.0144,   0.  ,  0.]]
    
    vehicle.reference_area                      = 10.85*2 
    vehicle.passengers                          = 6
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.     
    
    # ------------------------------------------------------    
    # WINGS    
    # ------------------------------------------------------  
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'canard_wing'  
    wing.aspect_ratio                           = 8.51507 
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.16 
    wing.taper                                  = 1.  
    wing.spans.projected                        = 9.6
    wing.chords.root                            = 1.1
    wing.total_length                           = 1.1 
    wing.chords.tip                             = 1.1
    wing.chords.mean_aerodynamic                = 1.1
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = 10.85  
    wing.areas.wetted                           = 10.85*2
    wing.areas.exposed                          = 10.85*2
    wing.twists.root                            = 0.* Units.degrees  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[0.1,  0.0 , 0.0]]  
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True        
                                                
    # add to vehicle                                          
    vehicle.append_component(wing)                            
                                                
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'  
    wing.aspect_ratio                           = 8.51507 
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.16 
    wing.taper                                  = 1.  
    wing.spans.projected                        = 9.6
    wing.chords.root                            = 1.1
    wing.total_length                           = 1.1 
    wing.chords.tip                             = 1.1
    wing.chords.mean_aerodynamic                = 1.1
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = 10.85  
    wing.areas.wetted                           = 10.85*2
    wing.areas.exposed                          = 10.85*2
    wing.twists.root                            = 0. * Units.degrees 
    wing.twists.tip                             = 0.   
    wing.origin                                 = [[ 5.138, 0.0  ,  1.323 ]]  # for images 1.54
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True 
    
    # add to vehicle 
    vehicle.append_component(wing)   
    
    
    # ------------------------------------------------------    
    # FUSELAGE    
    # ------------------------------------------------------    
    # FUSELAGE PROPERTIES                       
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 0.  
    fuselage.seat_pitch                         = 1.  
    fuselage.fineness.nose                      = 1.5 
    fuselage.fineness.tail                      = 4.0 
    fuselage.lengths.nose                       = 1.7   
    fuselage.lengths.tail                       = 2.7 
    fuselage.lengths.cabin                      = 1.7  
    fuselage.lengths.total                      = 6.3  
    fuselage.width                              = 1.15  
    fuselage.heights.maximum                    = 1.7 
    fuselage.heights.at_quarter_length          = 1.2  
    fuselage.heights.at_wing_root_quarter_chord = 1.7  
    fuselage.heights.at_three_quarters_length   = 0.75 
    fuselage.areas.wetted                       = 12.97989862  
    fuselage.areas.front_projected              = 1.365211404  
    fuselage.effective_diameter                 = 1.318423736  
    fuselage.differential_pressure              = 0.  
    
    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'   
    segment.percent_x_location                  = 0.  
    segment.percent_z_location                  = 0.  
    segment.height                              = 0.09  
    segment.width                               = 0.23473  
    segment.length                              = 0.  
    segment.effective_diameter                  = 0. 
    fuselage.Segments.append(segment)             
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.97675/6.1 
    segment.percent_z_location                  = 0.21977/6.1
    segment.height                              = 0.9027  
    segment.width                               = 1.01709  
    fuselage.Segments.append(segment)             
    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 1.93556/6.1 
    segment.percent_z_location                  = 0.39371/6.1
    segment.height                              = 1.30558   
    segment.width                               = 1.38871  
    fuselage.Segments.append(segment)             
    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 3.44137/6.1 
    segment.percent_z_location                  = 0.57143/6.1
    segment.height                              = 1.52588 
    segment.width                               = 1.47074 
    fuselage.Segments.append(segment)             
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 4.61031/6.1
    segment.percent_z_location                  = 0.81577/6.1
    segment.height                              = 1.14788 
    segment.width                               = 1.11463  
    fuselage.Segments.append(segment)              
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.9827
    segment.percent_z_location                  = 0.180
    segment.height                              = 0.6145
    segment.width                               = 0.3838
    fuselage.Segments.append(segment)            
    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 1. 
    segment.percent_z_location                  = 0.2058
    segment.height                              = 0.4
    segment.width                               = 0.25
    fuselage.Segments.append(segment)                 
    
    # add to vehicle
    vehicle.append_component(fuselage)    
    
    
    #------------------------------------------------------------------
    # PROPULSOR
    #------------------------------------------------------------------
    net                            = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_lift_rotor_engines= 8
    net.thrust_angle               = 0.0   * Units.degrees #  conversion to radians, 
    net.nacelle_diameter           = 0.4   # https://www.magicall.biz/products/integrated-motor-controller-magidrive/
    net.engine_length              = 0.95 
    net.areas                      = Data()
    net.areas.wetted               = np.pi*net.nacelle_diameter*net.engine_length + 0.5*np.pi*net.nacelle_diameter**2    
    net.identical_lift_rotors      = True  
    
        # Component 1 the ESC
    esc                            = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency                 = 0.95
    net.esc                        = esc 
    
    # Component 2 the Payload
    payload                        = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw             = 10. # Watts 
    payload.mass_properties.mass   = 1.0 * Units.kg
    net.payload                    = payload
    
    # Component 3 the Avionics    
    avionics                       = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw            = 20. # Watts  
    net.avionics                   = avionics   
    
    # Component 4 Miscellaneous Systems 
    sys                            = SUAVE.Components.Systems.System()
    sys.mass_properties.mass       = 5 # kg      
    
    # Component 5 the Battery      
    bat                         = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()   
    bat.cell.surface_area       = (np.pi*bat.cell.height*bat.cell.diameter)  
    bat.pack_config.series      =  150 # 140  
    bat.pack_config.parallel    =  120 #60    
    initialize_from_circuit_configuration(bat)    
    net.voltage                 = bat.max_voltage
    
    # Here we, are going to assume a battery pack module shape. This step is optional but
    # required for thermal analysis of the pack. We will assume that all cells electrically connected 
    # in series wihtin the module are arranged in one row normal direction to the airflow. Likewise ,
    # all cells electrically in paralllel are arranged in the direction to the cooling fluid  
    
    number_of_modules                = 10
    bat.module_config.total          = int(np.ceil(bat.pack_config.total/number_of_modules))
    bat.module_config.normal_count   = int(np.ceil(bat.module_config.total/bat.pack_config.series))
    bat.module_config.parallel_count = int(np.ceil(bat.module_config.total/bat.pack_config.parallel))
    net.battery                      = bat  
     
    
    # Component 6 the Rotor 
    # Design Rotors
    #------------------------------------------------------------------
    # atmosphere conditions
    speed_of_sound                 = 340
    rho                            = 1.22   
    Lift                           = vehicle.mass_properties.takeoff*9.81

    # Create propeller geometry
    rotor                          = SUAVE.Components.Energy.Converters.Rotor()  
    rotor.tip_radius               = 1.2 
    rotor.hub_radius               = 0.15 * rotor.tip_radius
    rotor.disc_area                = np.pi*(rotor.tip_radius**2)   
    rotor.design_tip_mach          = 0.5
    rotor.number_of_blades         = 3  
    rotor.freestream_velocity      = 10     
    rotor.angular_velocity         = rotor.design_tip_mach*speed_of_sound/rotor.tip_radius      
    rotor.design_Cl                = 0.7
    rotor.design_altitude          = 500 * Units.feet                   
    rotor.design_thrust            = Lift/(net.number_of_lift_rotor_engines-1) # contingency for one-engine-inoperative condition 
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    rotor.airfoil_geometry         =  [ rel_path + '/Airfoils/NACA_4412.txt']
    rotor.airfoil_polars           = [[ rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                      rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                      rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                      rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                      rel_path + '/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]  
    rotor.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    rotor                          = propeller_design(rotor)   
    rotor.variable_pitch            = True 
    rotor.rotation                 = 1 

    # Front Rotors Locations 
    origins = [[-0.3, 1.8, 0.0], [-0.3, 4.8, 0.0],[-0.3, -1.8, 0.0], [-0.3, -4.8, 0.0],\
               [4.7, 1.8 ,1.4], [4.7, 4.8, 1.4],[4.7, -1.8, 1.4], [4.7, -4.8, 1.4]] 

    for ii in range(8):
        lift_rotor          = deepcopy(rotor)
        lift_rotor.tag      = 'rotor_' + str(ii+1)
        lift_rotor.origin   = [origins[ii]]
        net.lift_rotors.append(lift_rotor) 
     
    
    # Motor
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    motor                           = SUAVE.Components.Energy.Converters.Motor() 
    motor.origin                    = rotor.origin  
    motor.efficiency                = 0.9    
    motor.nominal_voltage           = bat.max_voltage *3/4  
    motor.propeller_radius          = rotor.tip_radius 
    motor.no_load_current           = 0.01 
    motor                           = size_optimal_motor(motor,rotor) 
    motor.mass_properties.mass      = nasa_motor(motor.design_torque)  - 20 # NEED BETTER MOTOR MODEL 

    for ii in range(8):
        rotor_motor = deepcopy(motor)
        rotor_motor.tag    = 'motor_' + str(ii+1)
        rotor_motor.origin = [origins[ii]]
        net.lift_rotor_motors.append(rotor_motor)  

    # Add extra drag sources from motors, props, and landing gear. All of these hand measured
    motor_height                     = .25 * Units.feet
    motor_width                      =  1.6 * Units.feet
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
    total_excrescence_area_spin      = 12.*motor_height*motor_width + 2.* main_gear_length*main_gear_width \
                                         + nose_gear_width*nose_gear_length + 2 * main_tire_height*main_tire_width\
                                         + nose_tire_height*nose_tire_width
    total_excrescence_area_no_spin   = total_excrescence_area_spin + 12*propeller_height*propeller_width
    vehicle.excrescence_area_no_spin = total_excrescence_area_no_spin
    vehicle.excrescence_area_spin    = total_excrescence_area_spin

    # append motor origin spanwise locations onto wing data structure
    motor_origins_front                                   = np.array(origins[:4])
    motor_origins_rear                                    = np.array(origins[5:])
    vehicle.wings['canard_wing'].motor_spanwise_locations = motor_origins_front[:,1]/ vehicle.wings['canard_wing'].spans.projected
    vehicle.wings['canard_wing'].motor_spanwise_locations = motor_origins_front[:,1]/ vehicle.wings['canard_wing'].spans.projected
    vehicle.wings['main_wing'].motor_spanwise_locations   = motor_origins_rear[:,1]/ vehicle.wings['main_wing'].spans.projected

    vehicle.append_component(net)

    vehicle.weight_breakdown  = empty(vehicle)
    compute_component_centers_of_gravity(vehicle)
    vehicle.center_of_gravity()
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------ 
    # plot vehicle 
    #plot_vehicle(vehicle,plot_control_points = False) 
    #plt.show()  

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one propeller operates at varying flight conditions, one must perscribe  the 
    pitch command of the propeller which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                     = SUAVE.Components.Configs.Config.Container() 
    base_config                                 = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                             = 'base'
    configs.append(base_config)
 
    # ------------------------------------------------------------------
    #   Hover Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle   
    config.networks.battery_propeller.pitch_command   = -5.  * Units.degrees   
    configs.append(config)

    # ------------------------------------------------------------------
    #    
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 45.0  * Units.degrees 
    config.tag                                        = 'vertical_transition'
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    config.networks.battery_propeller.pitch_command   = 3.  * Units.degrees  
    configs.append(config)
    

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb_transition'
    vector_angle                                      = 15.0  * Units.degrees  
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    config.networks.battery_propeller.pitch_command   = 5.  * Units.degrees     
    configs.append(config)
     
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb'   
    vector_angle                                      = 0.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle  
    config.networks.battery_propeller.pitch_command   = 20.  * Units.degrees   # 20 
    configs.append(config)    

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'   
    vector_angle                                      = 0.0 * Units.degrees
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle  
    config.networks.battery_propeller.pitch_command   = 16.  * Units.degrees   # 17
    configs.append(config)    
    
  
    # ------------------------------------------------------------------
    #   
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 25.0  * Units.degrees   
    config.tag                                        = 'descent_transition'  
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    config.networks.battery_propeller.pitch_command   = 10.  * Units.degrees   
    configs.append(config) 
        


    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees  
    config.networks.battery_propeller.thrust_angle    = vector_angle
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle     
    config.networks.battery_propeller.pitch_command   = -5.  * Units.degrees  
    configs.append(config)

    return configs 


def urban_canyon_microphone_setup():  
    
    # define building locations 
    building_locations  = [[100,0,0] ,[300,150,0]] #,[400,-200,0],[900,150,0],[500,100,0],[700,-125,0]] # [[x,y,z]]     
     
    # define building dimensions  
    building_dimensions = [[200,100,100*Units.feet],[200,200,55]] # ,[160,160,90],[200,200,50],[200,200,55],[200,200,80]] # [[length,width,height]]     
    
    N_X = 3
    N_Y = 3
    N_Z = 9 
    mic_locations  = generate_building_microphone_points(building_locations,building_dimensions,N_x = N_X ,N_y = N_Y ,N_z = N_Z ) 
     
    return mic_locations,building_locations ,building_dimensions,N_X ,N_Y ,N_Z 


def mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery):
    
    
    starting_elevation = 100*Units.feet
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                             = Segments.Segment() 
    base_segment.state.numerics.number_control_points        = control_points     
    ones_row                                                 = base_segment.state.ones_row
    base_segment.battery_discharge                           = True  
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  

  
    for day in range(simulated_days):
        print(' ***********  Day ' + str(day) + ' ***********  ')
        for f_idx in range(flights_per_day): 
            flight_no = f_idx + 1        
            # ------------------------------------------------------------------
            #   First Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------ 
            segment                                            = Segments.Hover.Climb(base_segment)
            segment.tag                                        = "Vertical_Climb"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.vertical_climb) 
            segment.altitude_start                             = 0.0  * Units.ft + starting_elevation 
            segment.altitude_end                               = 40.  * Units.ft + starting_elevation 
            segment.climb_rate                                 = 300. * Units['ft/min']
            segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
            segment.state.unknowns.throttle                    = 0.6  * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
            mission.append_segment(segment)  
            
            
            # ------------------------------------------------------------------
            #  First Transition Segment
            # ------------------------------------------------------------------ 
            segment                       = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
            segment.tag                   = "Vertical_Transition" + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.vertical_transition) 
            segment.altitude              = 40.  * Units.ft + starting_elevation 
            segment.air_speed_start       = 300. * Units['ft/min']     
            segment.air_speed_end         = 55 * Units['mph']     
            segment.acceleration          = 9.81/5
            segment.pitch_initial         = 1. * Units.degrees
            segment.pitch_final           = 2. * Units.degrees 
            segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
            segment                       = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06) 
            mission.append_segment(segment)
        
        
            # ------------------------------------------------------------------
            #   First Cruise Segment: Constant Acceleration, Constant Altitude
            # ------------------------------------------------------------------ 
            segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag                      = "Climb_Transition_1"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.climb_transition) 
            segment.climb_rate               = 500. * Units['ft/min']
            segment.air_speed_start          = 55.   * Units['mph']
            segment.air_speed_end            = 85.   * Units['mph'] 
            segment.altitude_start           = 40.0 * Units.ft   + starting_elevation 
            segment.altitude_end             = 100.0 * Units.ft  + starting_elevation 
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)       
            mission.append_segment(segment)   
            
             
            # ------------------------------------------------------------------
            #  Second Transition Segment
            # ------------------------------------------------------------------ 
            segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
            segment.tag                       = "Climb_Transition_2"  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.climb_transition) 
            segment.altitude                  = 100.  * Units.ft + starting_elevation 
            segment.air_speed_start           = 85.  * Units['mph'] 
            segment.air_speed_end             = 125.  * Units['mph']  
            segment.acceleration              = 9.81/5
            segment.pitch_initial             = 2. * Units.degrees
            segment.pitch_final               = 5. * Units.degrees
            segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)     
            mission.append_segment(segment)
        
            ## ------------------------------------------------------------------
            ##   First Cruise Segment: Constant Acceleration, Constant Altitude
            ## ------------------------------------------------------------------ 
            #segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag                       = "Climb" + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend(analyses.climb) 
            #segment.climb_rate                = 500. * Units['ft/min']
            #segment.air_speed_start           = 125.   * Units['mph']
            #segment.air_speed_end             = 175.   * Units['mph'] 
            #segment.altitude_start            = 100.0 * Units.ft  + starting_elevation 
            #segment.altitude_end              = 2500.0 * Units.ft                
            #segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03)     
            #mission.append_segment(segment)     
        
            ## ------------------------------------------------------------------
            ##   First Cruise Segment: Constant Acceleration, Constant Altitude
            ## ------------------------------------------------------------------ 
            #segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
            #segment.tag                      = "Cruise" + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend(analyses.cruise) 
            #segment.altitude                 = 2500.0 * Units.ft
            #segment.air_speed                = 175.   * Units['mph'] 
            #cruise_distance                  = 30.    * Units.miles + (aircraft_range - 65.05*Units.miles ) 
            #segment.distance                 = cruise_distance
            #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)     
            #mission.append_segment(segment)     
            
            ## ------------------------------------------------------------------
            ##    Descent Segment: Constant Acceleration, Constant Altitude
            ## ------------------------------------------------------------------ 
            #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag                      = "Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend(analyses.climb)
            #segment.climb_rate               = -300. * Units['ft/min']
            #segment.air_speed_start          = 175.   * Units['mph']
            #segment.air_speed_end            = 100.   * Units['mph'] 
            #segment.altitude_start           = 2500.0 * Units.ft
            #segment.altitude_end             = 100.0 * Units.ft       
            #segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
            #mission.append_segment(segment)     
            
            #if reserve_segment: 
                ## ------------------------------------------------------------------
                ##   Reserve Climb Segment 
                ## ------------------------------------------------------------------ 
                #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                #segment.tag                      = "Reserve_Climb"  + "_F_" + str(flight_no) + "_D" + str (day)
                #segment.analyses.extend(analyses.climb) 
                #segment.climb_rate               = 500. * Units['ft/min']
                #segment.air_speed_start          = 100.   * Units['mph'] 
                #segment.air_speed_end            = 150.   * Units['mph'] 
                #segment.altitude_start           = 100.0 * Units.ft+ starting_elevation 
                #segment.altitude_end             = 1000.0 * Units.ft              
                #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                #mission.append_segment(segment)      
            
                ## ------------------------------------------------------------------
                ##   First Cruise Segment: Constant Acceleration, Constant Altitude
                ## ------------------------------------------------------------------ 
                #segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
                #segment.tag                      = "Reserve_Cruise"  + "_F_" + str(flight_no) + "_D" + str (day)
                #segment.analyses.extend(analyses.cruise)  
                #segment.air_speed                = 150.   * Units['mph'] 
                #segment.distance                 = cruise_distance*0.1              
                #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)  
                #mission.append_segment(segment)     
            
                ## ------------------------------------------------------------------
                ##   Reserve Descent Segment: Constant Acceleration, Constant Altitude
                ## ------------------------------------------------------------------ 
                #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                #segment.tag                      = "Reserve_Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
                #segment.analyses.extend(analyses.climb)
                #segment.climb_rate               = -300. * Units['ft/min']
                #segment.air_speed_start          = 150.   * Units['mph']
                #segment.air_speed_end            = 100.   * Units['mph'] + starting_elevation 
                #segment.altitude_start           = 1000.0 * Units.ft
                #segment.altitude_end             = 100.0 * Units.ft                  
                #segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                #mission.append_segment(segment)        
            
            ## ------------------------------------------------------------------
            ##  Forth Transition Segment
            ## ------------------------------------------------------------------ 
            #segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            #segment.tag                      = "Approach_Transition"  + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend( analyses.climb)  
            #segment.climb_rate               = -200. * Units['ft/min']
            #segment.air_speed_start          = 100.   * Units['mph']+ starting_elevation 
            #segment.air_speed_end            = 55.   * Units['mph'] + starting_elevation 
            #segment.altitude_start           = 100.0 * Units.ft
            #segment.altitude_end             = 40.0 * Units.ft                  
            #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
            #mission.append_segment(segment)     
            
            ## ------------------------------------------------------------------
            ##  Forth Transition Segment
            ## ------------------------------------------------------------------ 
            #segment                          = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)  
            #segment.tag                      = "Descent_Transition"  + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend( analyses.descent_transition)   
            #segment.altitude                 = 40.  * Units.ft+ starting_elevation 
            #segment.air_speed_start          = 55 * Units['mph']    
            #segment.air_speed_end            = 300. * Units['ft/min'] 
            #segment.acceleration             = -0.1 * Units['m/s/s']   
            #segment.pitch_initial            = 1. * Units.degrees
            #segment.pitch_final              = 2. * Units.degrees               
            #segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
            #mission.append_segment(segment)     
        
            ## ------------------------------------------------------------------
            ##   Descent Segment: Constant Speed, Constant Rate
            ## ------------------------------------------------------------------ 
            #segment                          = Segments.Hover.Descent(base_segment)
            #segment.tag                      = "Vertical_Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
            #segment.analyses.extend( analyses.vertical_descent) 
            #segment.altitude_start           = 40.0  * Units.ft+ starting_elevation 
            #segment.altitude_end             = 0.  * Units.ft + starting_elevation 
            #segment.descent_rate             = 300. * Units['ft/min']  
            #segment.state.unknowns.throttle                    = 0.6  * ones_row(1)  
            #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06)  
            #mission.append_segment(segment)  
            
            #if recharge_battery:
                ## ------------------------------------------------------------------
                ##  Charge Segment: 
                ## ------------------------------------------------------------------  
                ## Charge Model 
                #segment                                                  = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                #segment.tag                                              = 'Charge Day ' + "_F_" + str(flight_no) + "_D" + str (day)  
                #segment.analyses.extend(analyses.base)           
                #segment.battery_discharge                                = False   
                #if flight_no  == flights_per_day:  
                    #segment.increment_battery_cycle_day=True                     
                #segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment )    
                #mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------   

    return mission


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

def plot_results(results,line_style = 'bo-'):  

    ## Plot Flight Conditions 
    #plot_flight_conditions(results, line_style) 

    ## Plot Aerodynamic  Forces
    #plot_aerodynamic_forces(results)
    
    ## Plot Aerodynamic Coefficents
    #plot_aerodynamic_coefficients(results)

    ## Plot Aircraft Flight Speed
    #plot_aircraft_velocities(results, line_style)

    ## Plot Aircraft Electronics
    #plot_battery_pack_conditions(results, line_style)

    ## Plot Propeller Conditions 
    #plot_propeller_conditions(results, line_style) 

    ## Plot Electric Motor and Propeller Efficiencies 
    #plot_eMotor_Prop_efficiencies(results, line_style)

    ## Plot propeller Disc and Power Loading
    #plot_disc_power_loading(results, line_style)   
    
    ## Plot noise level
    #plot_ground_noise_levelsresults)
    
    ## Plot noise contour    
    plot_flight_profile_noise_contours(results)


    return 

def save_results(results):

    # Store data (serialize)
    with open('Tiltwing_Building.pkl', 'wb') as file:
        pickle.dump(results, file)

    return  

def load_results(filename): 
    with open(filename, 'rb') as file_1:
        results = pickle.load(file_1)      
    
    return results 


if __name__ == '__main__': 
    main()    
    plt.show()