# Tiltwing.py 

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
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform
from copy import deepcopy
import numpy as np
import os
import pylab as plt 
import pickle 
import time  

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
    
    # 4 hours 
    
    simulated_days   = 1
    flights_per_day  = 1 
    aircraft_range   = 70 *Units.nmi
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

    run_noise_model   = True
    hover_noise_test  = False   
    run_approach_departure_noise_mission(simulated_days,flights_per_day,aircraft_range,reserve_segment,run_noise_model,
                      hover_noise_test,plot_geometry,recharge_battery,run_analysis,plot_mission,
                      control_points,N_gm_x,N_gm_y) 

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
    #write(vehicle,"Tiltwing") 
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
    filename          = 'Tiltwing_Full_Mission'   
    save_results(mission_results,filename)   
            
    # weight plot breakdown 
    print('WEIGHT BREAKDOWN')
    breakdown = empty(configs.base,contingency_factor=1.0)
    print(breakdown)
    
    # plot geoemtry 
    if plot_geometry: 
        plot_vehicle(configs.base, elevation_angle = 90,azimuthal_angle =  180,axis_limits =8,plot_control_points = False)       
        plt.show()
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
            noise_mission     = full_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test )
            missions_analyses = missions_setup(noise_mission) 
            analyses          = SUAVE.Analyses.Analysis.Container()
            analyses.configs  = configs_analyses
            analyses.missions = missions_analyses 
            configs.finalize()
            analyses.finalize()      
            noise_mission     = analyses.missions.base
            noise_results     = noise_mission.evaluate()   
            filename          = 'Tiltwing_Full_Mission_Noise_Q' + str(Q_idx) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
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
    X_LIM        = np.linspace(1E-6, 4.38*Units.nmi,3)       
    
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
            filename          = 'Tiltwing_Approach_Departure_Noise_Q' + str(Q_idx) + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
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
    filename          = 'Tiltwing_Hover_Mission' + '_Nx' + str(N_gm_x) + '_Ny' + str(N_gm_y)
    save_results(hover_results,filename)  
    
    tf = time.time() 
    print ('time taken: '+ str(round(((tf-ti)/60),3)) + ' mins')     
    elapsed_range = hover_results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
    print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')   
    
    return 


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

# ----------------------------------------------------------------------
#   Base Analysis
# ---------------------------------------------------------------------- 
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
    #aerodynamics.settings.model_fuselage = True 
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
#   Build the Vehicle
# ----------------------------------------------------------------------
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
    vehicle.mass_properties.takeoff             = 2300
    vehicle.mass_properties.operating_empty     = 2300
    vehicle.mass_properties.max_takeoff         = 2300
    vehicle.mass_properties.center_of_gravity   = [[ 2.0144,   0.  ,  0.]] 
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
    wing.spans.projected                        = 9.2
    chord                                       = 0.85
    wing.chords.root                            = chord
    wing.total_length                           = chord
    wing.chords.tip                             = chord
    wing.chords.mean_aerodynamic                = chord
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
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
    wing.spans.projected                        = 9.2
    wing.chords.root                            = chord
    wing.total_length                           = chord 
    wing.chords.tip                             = chord
    wing.chords.mean_aerodynamic                = chord
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0. * Units.degrees 
    wing.twists.tip                             = 0.   
    wing.origin                                 = [[ 5.138, 0.0  ,  1.323 ]]  # for images 1.54
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.symmetric                              = True 
    
    # compute reference properties  
    vehicle.reference_area                     = wing.areas.reference*2   
        
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
    net                              = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines  = 8
    net.thrust_angle                 = 0.0   * Units.degrees #  conversion to radians,  
    net.engine_length                = 0.95 
    net.areas                        = Data()
    net.identical_propellers         = True  
    
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
    total_cells                          = 140*80
    max_module_voltage                   = 50
    safety_factor                        = 1.5
     
    bat                                  = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650() 
    bat.pack_config.series               = 140  # CHANGE IN OPTIMIZER 
    bat.pack_config.parallel             = int(total_cells/bat.pack_config.series)
    initialize_from_circuit_configuration(bat)    
    net.voltage                          = bat.max_voltage  
    bat.module_config.number_of_modules  = 16 # CHANGE IN OPTIMIZER 
    bat.module_config.total              = int(np.ceil(bat.pack_config.total/bat.module_config.number_of_modules))
    bat.module_config.voltage            = net.voltage/bat.module_config.number_of_modules # must be less than max_module_voltage/safety_factor 
    bat.module_config.layout_ratio       = 0.5 # CHANGE IN OPTIMIZER 
    bat.module_config.normal_count       = int(bat.module_config.total**(bat.module_config.layout_ratio))
    bat.module_config.parallel_count     = int(bat.module_config.total**(1-bat.module_config.layout_ratio)) 
    net.battery                          = bat        
    
    # Component 6 the Rotor 
    # Design Rotors
    #------------------------------------------------------------------
    # atmosphere conditions
    speed_of_sound                 = 340
    rho                            = 1.22   
    Lift                           = vehicle.mass_properties.takeoff*9.81

    # Create propeller geometry
    prop                          = SUAVE.Components.Energy.Converters.Propeller()  
    prop.tip_radius               = 1.25  
    prop.hub_radius               = 0.15 * prop.tip_radius   
    prop.design_tip_mach          = 0.65 # gives better noise results and more realistic blade 
    prop.number_of_blades         = 3  
    prop.freestream_velocity      = 130 * Units.mph  # 10  
    prop.angular_velocity         = prop.design_tip_mach*speed_of_sound/prop.tip_radius      
    prop.design_Cl                = 0.7
    prop.design_altitude          = 500 * Units.feet                   
    prop.design_thrust            = Lift/(net.number_of_propeller_engines) # contingency for one-engine-inoperative condition and then turning off off-diagonal rotor
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    prop.airfoil_geometry         =  [ rel_path + '../Airfoils/NACA_4412.txt']
    prop.airfoil_polars           = [[ rel_path + '../Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                       rel_path +  '../Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]  
    prop.airfoil_polar_stations   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop                          = propeller_design(prop)   
    prop.variable_pitch           = True 
    prop.rotation                 = 1 

    # Rotors Locations  
    origins   = [[-0.3, 2.0, 0.0], [-0.3, 4.8, 0.0],[-0.3, -2.0, 0.0], [-0.3, -4.8, 0.0],\
               [4.7, 2.0 ,1.4], [4.7, 4.8, 1.4],[4.7, -2.0, 1.4], [4.7, -4.8, 1.4]]      

    for ii in range(8):
        lift_prop          = deepcopy(prop)
        lift_prop.tag      = 'prop_' + str(ii+1)
        lift_prop.origin   = [origins[ii]]
        net.propellers.append(lift_prop) 
        
    
    # Nacelles 
    nacelle                = SUAVE.Components.Nacelles.Nacelle()
    nacelle.tag            = 'prop_nacelle'
    nacelle.length         = 1.5
    nacelle.diameter       = 0.5
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
    nac_segment.height             = 0.45
    nac_segment.width              = 0.45
    nacelle.append_segment(nac_segment)    
    
    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5 
    nac_segment.height             = 0.5
    nac_segment.width              = 0.5
    nacelle.append_segment(nac_segment)    

    nac_segment                    = SUAVE.Components.Lofted_Body_Segment.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.75
    nac_segment.height             = 0.45
    nac_segment.width              = 0.45
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


    prop_nacelle_origins = [[-0.5, 2.0, 0.0], [-0.5, 4.8, 0.0],[-0.5, -2.0, 0.0], [-0.5, -4.8, 0.0],\
               [4.5, 2.0 ,1.4], [4.5, 4.8, 1.4],[4.5, -2.0, 1.4], [4.5, -4.8, 1.4]] 
    
    for ii in range(8):
        prop_nacelle          = deepcopy(nacelle)
        prop_nacelle.tag      = 'propeller_nacelle_' + str(ii+1) 
        prop_nacelle.origin   = [prop_nacelle_origins[ii]]
        vehicle.append_component(prop_nacelle)       
     
    
    # Motor
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    motor                           = SUAVE.Components.Energy.Converters.Motor() 
    motor.origin                    = prop.origin  
    motor.efficiency                = 0.9  
    motor.nominal_voltage           = bat.max_voltage *0.8  
    motor.propeller_radius          = prop.tip_radius 
    motor.no_load_current           = 0.01  
    motor                           = size_optimal_motor(motor,prop) 
    motor.mass_properties.mass      = nasa_motor(motor.design_torque)  

    for ii in range(8):
        prop_motor = deepcopy(motor)
        prop_motor.tag    = 'motor_' + str(ii+1)
        prop_motor.origin = [origins[ii]]
        net.propeller_motors.append(prop_motor)  

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
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_climb'
    vector_angle                                      = 90.0 * Units.degrees
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -10.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    configs.append(config) 

    # ------------------------------------------------------------------
    #  Vertical Transition 1 
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 35.0  * Units.degrees
    config.tag                                        = 'vertical_transition_1'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -8.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    


    # ------------------------------------------------------------------
    # Vertical Transition 2    
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 25.0  * Units.degrees
    config.tag                                        = 'vertical_transition_2'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -5.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    
    

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'climb_transition'
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -3.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

     
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'climb'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = 0.  * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)
   

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------      
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'cruise'
    vector_angle                                      = 0.0 * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = 0.  * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)
  

    # ------------------------------------------------------------------
    #   Approach Configuration
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 15.0  * Units.degrees
    config.tag                                        = 'approach'  
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = 0.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

    
    
    # ------------------------------------------------------------------
    #  Descent Transition
    # ------------------------------------------------------------------  
    config                                            = SUAVE.Components.Configs.Config(base_config)
    vector_angle                                      = 45.0  * Units.degrees
    config.tag                                        = 'descent_transition'  
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -5.  * Units.degrees
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    configs.append(config)    

    

    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------ 
    config                                            = SUAVE.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees 
    for prop in config.networks.battery_propeller.propellers: 
        prop.orientation_euler_angles                 = [0,vector_angle,0]
        prop.inputs.pitch_command                     = -10.  * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
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

# ------------------------------------------------------------------
#   Full Mission Setup
# ------------------------------------------------------------------  
def full_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test):
        
  
    starting_elevation = 0*Units.feet
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport    
    atmosphere         = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)    
    
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



    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))  



    if  hover_noise_test:   
        segment                                            = Segments.Hover.Climb(base_segment)
        segment.tag                                        = "Hover" # very small climb so that broadband noise does not return 0's  
        segment.analyses.extend(analyses.vertical_climb) 
        segment.altitude_start                             = 500.0  * Units.ft  
        segment.altitude_end                               = 500.1  * Units.ft 
        segment.climb_rate                                 = 100. * Units['ft/min']  
        segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
        segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
        segment.state.unknowns.throttle                    = 0.6  * ones_row(1)  
        segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
        mission.append_segment(segment)  

    
    else:     
  
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
                if day == 0:        
                    segment.battery_energy                         = vehicle.networks.battery_propeller.battery.max_energy   
                segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
                segment.state.unknowns.throttle                    = 0.8  * ones_row(1)  
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
                mission.append_segment(segment)  
                
                
                # ------------------------------------------------------------------
                #  First Transition Segment
                # ------------------------------------------------------------------ 
                segment                       = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
                segment.tag                   = "Vertical_Transition_1" + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend( analyses.vertical_transition_1) 
                segment.altitude              = 40.  * Units.ft + starting_elevation 
                segment.air_speed_start       = 300. * Units['ft/min']     
                segment.air_speed_end         = Vstall* 0.4 
                segment.acceleration          = 9.81/5
                segment.pitch_initial         = 20. * Units.degrees
                segment.pitch_final           = -20. * Units.degrees 
                segment.state.unknowns.throttle   = 0.80 * ones_row(1)  
                segment                       = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06) 
                mission.append_segment(segment)
                

                # ------------------------------------------------------------------
                #  First Transition Segment
                # ------------------------------------------------------------------ 
                segment                       = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
                segment.tag                   = "Vertical_Transition_2" + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend( analyses.vertical_transition_2) 
                segment.altitude              = 40.  * Units.ft + starting_elevation 
                segment.air_speed_start       = Vstall* 0.4  
                segment.air_speed_end         = Vstall* 0.8 
                segment.acceleration          = 9.81/5
                segment.pitch_initial         = 10. * Units.degrees
                segment.pitch_final           = -20. * Units.degrees 
                segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
                segment                       = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06) 
                mission.append_segment(segment) 
                
                # ------------------------------------------------------------------
                #   First Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                      = "Climb_Transition_1"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.climb_transition) 
                segment.climb_rate               = 600. * Units['ft/min']
                segment.air_speed_start          = Vstall* 0.8  
                segment.air_speed_end            = Vstall* 1.0 
                segment.altitude_start           = 40.0 * Units.ft   + starting_elevation 
                segment.altitude_end             = 500.0 * Units.ft   + starting_elevation 
                segment.state.unknowns.throttle   = 0.80 * ones_row(1)  
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)       
                mission.append_segment(segment)   
                
                 
                # ------------------------------------------------------------------
                #  Second Transition Segment
                # ------------------------------------------------------------------ 
                segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
                segment.tag                       = "Climb_Transition_2"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend( analyses.climb_transition) 
                segment.altitude                  = 500.0 * Units.ft   
                segment.air_speed_start           = Vstall* 1.0  
                segment.air_speed_end             = 125.  * Units['mph']  
                segment.acceleration              = 9.81/5
                segment.pitch_initial             = 2. * Units.degrees
                segment.pitch_final               = 5. * Units.degrees
                segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)     
                mission.append_segment(segment)
            
                # ------------------------------------------------------------------
                #   First Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                       = "Climb" + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.climb) 
                segment.climb_rate                = 500. * Units['ft/min']
                segment.air_speed_start           = 125.   * Units['mph']
                segment.air_speed_end             = 175.   * Units['mph'] 
                segment.altitude_start            = 500.0 * Units.ft  + starting_elevation                    
                segment.altitude_end              = 2500.0 * Units.ft                
                segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03)     
                mission.append_segment(segment)     
            
                # ------------------------------------------------------------------
                #   First Cruise Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
                segment.tag                      = "Cruise" + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.cruise) 
                segment.altitude                 = 2500.0 * Units.ft               
                segment.air_speed                = 175.   * Units['mph'] 
                cruise_distance                  = aircraft_range  - 26.49 * Units.nmi
                segment.distance                 = cruise_distance
                segment.state.unknowns.throttle  = 0.8 * ones_row(1)   
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)     
                mission.append_segment(segment)     
                
                # ------------------------------------------------------------------
                #    Descent Segment: Constant Acceleration, Constant Altitude
                # ------------------------------------------------------------------ 
                segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                      = "Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend(analyses.climb)
                segment.climb_rate               = -300. * Units['ft/min']
                segment.air_speed_start          = 175.   * Units['mph']
                segment.air_speed_end            = 100.   * Units['mph'] 
                segment.altitude_start           = 2500.0 * Units.ft 
                segment.altitude_end             = 100.0 * Units.ft + starting_elevation      
                segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                mission.append_segment(segment)     
                
                if reserve_segment: 
                    # ------------------------------------------------------------------
                    #   Reserve Climb Segment 
                    # ------------------------------------------------------------------ 
                    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                    segment.tag                      = "Reserve_Climb"  + "_F_" + str(flight_no) + "_D" + str (day)
                    segment.analyses.extend(analyses.climb) 
                    segment.climb_rate               = 500. * Units['ft/min']
                    segment.air_speed_start          = 100.   * Units['mph'] 
                    segment.air_speed_end            = 150.   * Units['mph'] 
                    segment.altitude_start           = 100.0 * Units.ft+ starting_elevation 
                    segment.altitude_end             = 1000.0 * Units.ft              
                    segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
                    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                    mission.append_segment(segment)      
                
                    # ------------------------------------------------------------------
                    #   First Cruise Segment: Constant Acceleration, Constant Altitude
                    # ------------------------------------------------------------------ 
                    segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
                    segment.tag                      = "Reserve_Cruise"  + "_F_" + str(flight_no) + "_D" + str (day)
                    segment.analyses.extend(analyses.cruise)  
                    segment.air_speed                = 150.   * Units['mph'] 
                    segment.distance                 = cruise_distance*0.1  - 4.*Units.nmi     
                    segment.state.unknowns.throttle  = 0.8 * ones_row(1) 
                    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.01)  
                    mission.append_segment(segment)     
                
                    # ------------------------------------------------------------------
                    #   Reserve Descent Segment: Constant Acceleration, Constant Altitude
                    # ------------------------------------------------------------------ 
                    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                    segment.tag                      = "Reserve_Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
                    segment.analyses.extend(analyses.climb)
                    segment.climb_rate               = -300. * Units['ft/min']
                    segment.air_speed_start          = 150.   * Units['mph']
                    segment.air_speed_end            = 100.   * Units['mph']
                    segment.altitude_start           = 1000.0 * Units.ft
                    segment.altitude_end             = 100.0 * Units.ft    + starting_elevation                
                    segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
                    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                    mission.append_segment(segment)        
                
                # ------------------------------------------------------------------
                #  Forth Transition Segment
                # ------------------------------------------------------------------ 
                segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment.tag                      = "Approach_Transition"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend( analyses.approach)  
                segment.climb_rate               = -300. * Units['ft/min']
                segment.air_speed_start          = 100.   * Units['mph'] 
                segment.air_speed_end            = 55.   * Units['mph'] 
                segment.altitude_start           = 100.0 * Units.ft     + starting_elevation
                segment.altitude_end             = 40.0 * Units.ft     + starting_elevation              
                segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
                mission.append_segment(segment)     
                
                # ------------------------------------------------------------------
                #  Forth Transition Segment
                # ------------------------------------------------------------------ 
                segment                          = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)  
                segment.tag                      = "Descent_Transition"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend( analyses.descent_transition)   
                segment.altitude                 = 40.  * Units.ft+ starting_elevation 
                segment.air_speed_start          = 55 * Units['mph']    
                segment.air_speed_end            = 300. * Units['ft/min'] 
                segment.acceleration             = -0.5 * Units['m/s/s']   
                segment.pitch_initial            = 1. * Units.degrees
                segment.pitch_final              = 2. * Units.degrees               
                segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
                segment.battery_energy                         = vehicle.networks.battery_propeller.battery.max_energy   
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment , initial_power_coefficient = 0.01) 
                mission.append_segment(segment)     
            
                # ------------------------------------------------------------------
                #   Descent Segment: Constant Speed, Constant Rate
                # ------------------------------------------------------------------ 
                segment                          = Segments.Hover.Descent(base_segment)
                segment.tag                      = "Vertical_Descent"  + "_F_" + str(flight_no) + "_D" + str (day)
                segment.analyses.extend( analyses.vertical_descent) 
                segment.altitude_start           = 40.0  * Units.ft + starting_elevation 
                segment.altitude_end             = 0.  * Units.ft + starting_elevation 
                segment.descent_rate             = 300. * Units['ft/min']  
                segment.state.unknowns.throttle  = 0.6  * ones_row(1)  
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06)  
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
                    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment )    
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

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport   

    atmosphere    = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)    
    
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
 

    # VSTALL Calculation
    m      = vehicle.mass_properties.max_takeoff
    g      = 9.81
    S      = vehicle.reference_area
    atmo   = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho    = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax  = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax)))   
    
    
    # ------------------------------------------------------------------
    #    Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Descent"   
    segment.analyses.extend(analyses.climb) 
    segment.battery_energy           = vehicle.networks.battery_propeller.battery.max_energy*0.6
    segment.climb_rate               = -300. * Units['ft/min']
    segment.air_speed_start          = 175.   * Units['mph']
    segment.air_speed_end            = 100.   * Units['mph'] 
    segment.altitude_start           = 500.0 * Units.ft 
    segment.altitude_end             = 100.0 * Units.ft + starting_elevation      
    segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #  Forth Transition Segment
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Approach_Transition"  
    segment.analyses.extend( analyses.approach)  
    segment.climb_rate               = -300. * Units['ft/min']
    segment.air_speed_start          = 100.   * Units['mph'] 
    segment.air_speed_end            = 55.   * Units['mph'] 
    segment.altitude_start           = 100.0 * Units.ft     + starting_elevation
    segment.altitude_end             = 40.0 * Units.ft     + starting_elevation              
    segment.state.unknowns.throttle  = 0.6 * ones_row(1) 
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.03) 
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #  Forth Transition Segment
    # ------------------------------------------------------------------ 
    segment                          = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)  
    segment.tag                      = "Descent_Transition"   
    segment.analyses.extend( analyses.descent_transition)    
    segment.altitude                 = 40.  * Units.ft+ starting_elevation 
    segment.air_speed_start          = 55 * Units['mph']    
    segment.air_speed_end            = 300. * Units['ft/min'] 
    segment.acceleration             = -0.5 * Units['m/s/s']   
    segment.pitch_initial            = 1. * Units.degrees
    segment.pitch_final              = 2. * Units.degrees               
    segment.state.unknowns.throttle  = 0.6 * ones_row(1)   
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment , initial_power_coefficient = 0.01) 
    mission.append_segment(segment)     

    # ------------------------------------------------------------------
    #   Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                          = Segments.Hover.Descent(base_segment)
    segment.tag                      = "Vertical_Descent"   
    segment.analyses.extend( analyses.vertical_descent) 
    segment.altitude_start           = 40.0  * Units.ft + starting_elevation  
    segment.altitude_end             = 0.  * Units.ft + starting_elevation 
    segment.descent_rate             = 300. * Units['ft/min']  
    segment.state.unknowns.throttle  = 0.6  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06)  
    mission.append_segment(segment)    
    
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"   
    segment.analyses.extend(analyses.vertical_climb) 
    segment.altitude_start                             = 0.0  * Units.ft + starting_elevation 
    segment.altitude_end                               = 40.  * Units.ft + starting_elevation 
    segment.climb_rate                                 = 300. * Units['ft/min']  
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
    segment.state.unknowns.throttle                    = 0.6  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
    mission.append_segment(segment)  
    
    
    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------ 
    segment                       = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                   = "Vertical_Transition_1"  
    segment.analyses.extend( analyses.vertical_transition_1) 
    segment.altitude              = 40.  * Units.ft + starting_elevation 
    segment.air_speed_start       = 300. * Units['ft/min']     
    segment.air_speed_end         = Vstall* 0.4 
    segment.acceleration          = 9.81/5
    segment.pitch_initial         = 20. * Units.degrees
    segment.pitch_final           = -20. * Units.degrees 
    segment.state.unknowns.throttle   = 0.80 * ones_row(1)  
    segment                       = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06) 
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------ 
    segment                       = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                   = "Vertical_Transition_2"  
    segment.analyses.extend( analyses.vertical_transition_2) 
    segment.altitude              = 40.  * Units.ft + starting_elevation 
    segment.air_speed_start       = Vstall* 0.4  
    segment.air_speed_end         = Vstall* 0.8 
    segment.acceleration          = 9.81/5
    segment.pitch_initial         = 10. * Units.degrees
    segment.pitch_final           = -20. * Units.degrees 
    segment.state.unknowns.throttle   = 0.80 * ones_row(1) 
    segment                       = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06) 
    mission.append_segment(segment) 
    
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Climb_Transition_1"   
    segment.analyses.extend(analyses.climb_transition) 
    segment.climb_rate               = 600. * Units['ft/min']
    segment.air_speed_start          = Vstall* 0.8  
    segment.air_speed_end            = Vstall* 1.0 
    segment.altitude_start           = 40.0 * Units.ft   + starting_elevation 
    segment.altitude_end             = 500.0 * Units.ft   + starting_elevation 
    segment.state.unknowns.throttle   = 0.80 * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment, initial_power_coefficient = 0.06)       
    mission.append_segment(segment)    
    
    return mission

# ------------------------------------------------------------------
#  Hover Noise Mission Setup
# ------------------------------------------------------------------   
def hover_mission_setup(analyses,vehicle,simulated_days,flights_per_day,aircraft_range,reserve_segment,control_points,recharge_battery,hover_noise_test):
         
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport            = SUAVE.Attributes.Airports.Airport() 
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport   

    atmosphere    = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data          = atmosphere.compute_values(altitude = airport.altitude,temperature_deviation= 1.)    
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                             = Segments.Segment() 
    base_segment.state.numerics.number_control_points        = 3     
    ones_row                                                 = base_segment.state.ones_row
    base_segment.battery_discharge                           = True  
    base_segment.process.iterate.conditions.stability        = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability     = SUAVE.Methods.skip    
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  

 
    segment                                            = Segments.Hover.Climb(base_segment)
    segment.tag                                        = "Hover" # very small climb so that broadband noise does not return 0's  
    segment.analyses.extend(analyses.vertical_climb) 
    segment.altitude_start                             = 500.0  * Units.ft  
    segment.altitude_end                               = 500.1  * Units.ft 
    segment.climb_rate                                 = 100. * Units['ft/min']  
    segment.battery_energy                             = vehicle.networks.battery_propeller.battery.max_energy   
    segment.battery_pack_temperature                   = atmo_data.temperature[0,0]             
    segment.state.unknowns.throttle                    = 0.6  * ones_row(1)  
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.06) 
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


# ----------------------------------------------------------------------
#   Plot Results
# ---------------------------------------------------------------------- 
def plot_results(results,run_noise_model,line_style = 'bo-'):  

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 

    # Plot Aerodynamic  Forces
    plot_aerodynamic_forces(results)
    
    # Plot Aerodynamic Coefficents
    plot_aerodynamic_coefficients(results)

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