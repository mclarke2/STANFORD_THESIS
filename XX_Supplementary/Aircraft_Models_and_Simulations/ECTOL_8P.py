# ECTOL_8P.py
#
# Created: Feb 2020, M. Clarke
#          Sep 2020, M. Clarke 

""" setup file for the X57-Maxwell Electric Aircraft 
"""
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units 
import numpy as np   
import matplotlib.pyplot        as plt
import os
import pickle
from SUAVE.Core                                         import Data 
from SUAVE.Plots.Mission_Plots                          import *  
from SUAVE.Plots.Geometry_Plots                         import * 
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion                           import propeller_design 
from SUAVE.Methods.Power.Battery.Sizing                 import initialize_from_mass
from SUAVE.Methods.Power.Battery.Sizing                 import initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing     import size_optimal_motor
from SUAVE.Methods.Weights.Correlations.Propulsion      import nasa_motor
from SUAVE.Methods.Weights.Buildups.eVTOL.empty                import empty

#import vsp 
#from SUAVE.Input_Output.OpenVSP.vsp_write import write 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    configs, analyses = full_setup() 

    configs.finalize()
    analyses.finalize()  

    # weight analysis
    #weight  = General_Aviation.empty(configs.base)	 
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate() 
    
    # save results  
    save_results(results)
    
    # plot the results
    plot_results(results) 
    
    #plot_paper_image(results)
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup() 
    #write(vehicle, "ECTOL_8P")  
    
    # Set up configs
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
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
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    aerodynamics.settings.model_fuselage             = True 
    aerodynamics.settings.use_surrogate              = False 
    aerodynamics.settings.propeller_wake_model       = True 
    aerodynamics.settings.number_spanwise_vortices   = 25
    aerodynamics.settings.number_chordwise_vortices  = 1          
    analyses.append(aerodynamics)

    ## ------------------------------------------------------------------
    ##  Noise Analysis
    #noise = SUAVE.Analyses.Noise.Fidelity_One()   
    #noise.geometry = vehicle
    #analyses.append(noise)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
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

    # done!
    return analyses    


# ----------------------------------------------------------------------
#   Define the Vehicle
# ---------------------------------------------------------------------- 

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle() 
    vehicle.tag = 'ECTOL_8P'    


    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff   = 2186  * Units.pounds
    vehicle.mass_properties.takeoff       = 2186  * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 1675  * Units.pounds
    vehicle.mass_properties.cargo         = 0.   
    vehicle.envelope.ultimate_load        = 5.7
    vehicle.envelope.limit_load           = 3.8 
    vehicle.reference_area                = 14.76
    vehicle.passengers                    = 4
    vehicle.systems.control               = "fully powered"
    vehicle.systems.accessories           = "commuter"    
    
    cruise_speed                          = 135.*Units['mph']    
    altitude                              = 2500. * Units.ft
    atmo                                  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    freestream                            = atmo.compute_values (0.)
    freestream0                           = atmo.compute_values (altitude)
    mach_number                           = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure       = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number            =  mach_number
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------    
    wing                                  = SUAVE.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 14.76
    wing.spans.projected                  = 11.4 
    wing.chords.root                      = 1.46
    wing.chords.tip                       = 0.92
    wing.chords.mean_aerodynamic          = 1.19
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[2.93, 0., 1.01]]
    wing.aerodynamic_center               = [3., 0., 1.01] 
    wing.vertical                         = False
    wing.symmetric                        = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    airfoil                               = SUAVE.Components.Airfoils.Airfoil()
    airfoil.coordinate_file               = 'NACA_63_412.txt'
    
    cg_x = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[cg_x,   0.  ,  cg_z ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft

    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 0. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.5438
    segment.twist                         = 0.* Units.degrees 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    # Wing Segments
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'winglet_1'
    segment.percent_span_location         = 0.98
    segment.twist                         = 0.  * Units.degrees 
    segment.root_chord_percent            = 0.630
    segment.dihedral_outboard             = 75. * Units.degrees 
    segment.sweeps.quarter_chord          = 82. * Units.degrees 
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    

    ## Wing Segments
    #segment                               = SUAVE.Components.Wings.Segment()
    #segment.tag                           = 'winglet_2'
    #segment.percent_span_location         = 0.99
    #segment.twist                         = 1.  * Units.degrees 
    #segment.root_chord_percent            = 0.25
    #segment.dihedral_outboard             = 75. * Units.degrees 
    #segment.sweeps.quarter_chord          = 65. * Units.degrees 
    #segment.thickness_to_chord            = 0.12 
    #segment.append_airfoil(airfoil)
    #wing.append_segment(segment) 

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees 
    segment.root_chord_percent            = 0.12
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)    
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------       
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 2.540 
    wing.spans.projected                  = 3.3  * Units.meter 
    wing.sweeps.quarter_chord             = 0 * Units.deg 
    wing.chords.root                      = 0.769 * Units.meter 
    wing.chords.tip                       = 0.769 * Units.meter 
    wing.chords.mean_aerodynamic          = 0.769 * Units.meter  
    wing.taper                            = 1. 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[7.7, 0., 0.25]]
    wing.aerodynamic_center               = [7.8, 0., 0.25] 
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.symmetric                        = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                  = SUAVE.Components.Wings.Wing()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.quarter_chord             = 25. * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 2.258 * Units['meters**2']  
    wing.spans.projected                  = 1.854   * Units.meter  
    wing.chords.root                      = 1.6764 * Units.meter 
    wing.chords.tip                       = 0.6858 * Units.meter 
    wing.chords.mean_aerodynamic          = 1.21 * Units.meter 
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[6.75 ,0,  0.623]]
    wing.aerodynamic_center               = [0.508 ,0,0]  
    wing.vertical                         = True 
    wing.symmetric                        = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------ 
    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2. 
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.lengths.tail                       = 161. * Units.inches
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.total                      = 332.2* Units.inches
    fuselage.lengths.fore_space                 = 0.
    fuselage.lengths.aft_space                  = 0.   
    fuselage.width                              = 42. * Units.inches 
    fuselage.heights.maximum                    = 62. * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches 
    fuselage.areas.side_projected               = 8000.  * Units.inches**2.
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.areas.front_projected              = 42.* 62. * Units.inches**2. 
    fuselage.effective_diameter                 = 50. * Units.inches


    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'  
    segment.percent_x_location                  = 0 
    segment.percent_z_location                  = 0 
    segment.height                              = 0.01 
    segment.width                               = 0.01 
    fuselage.Segments.append(segment)             
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.007279116466
    segment.percent_z_location                  = 0.002502014453
    segment.height                              = 0.1669064748
    segment.width                               = 0.2780205877
    fuselage.Segments.append(segment)    

    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.01941097724 
    segment.percent_z_location                  = 0.001216095397 
    segment.height                              = 0.3129496403 
    segment.width                               = 0.4365777215 
    fuselage.Segments.append(segment)         

    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.06308567604
    segment.percent_z_location                  = 0.007395489231
    segment.height                              = 0.5841726619
    segment.width                               = 0.6735119903
    fuselage.Segments.append(segment)      
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.1653761217 
    segment.percent_z_location                  = 0.02891281352 
    segment.height                              = 1.064028777 
    segment.width                               = 1.067200529 
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.2426372155 
    segment.percent_z_location                  = 0.04214148761 
    segment.height                              = 1.293766653 
    segment.width                               = 1.183058255 
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 0.2960174029  
    segment.percent_z_location                  = 0.04705241831  
    segment.height                              = 1.377026712  
    segment.width                               = 1.181540054  
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.3809404284 
    segment.percent_z_location                  = 0.05313580461 
    segment.height                              = 1.439568345 
    segment.width                               = 1.178218989 
    fuselage.Segments.append(segment)    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.5046854083 
    segment.percent_z_location                  = 0.04655492473 
    segment.height                              = 1.29352518 
    segment.width                               = 1.054390707 
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.6454149933 
    segment.percent_z_location                  = 0.03741966266 
    segment.height                              = 0.8971223022 
    segment.width                               = 0.8501926505   
    fuselage.Segments.append(segment)  
      
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_10'   
    segment.percent_x_location                  = 0.985107095 
    segment.percent_z_location                  = 0.04540283436 
    segment.height                              = 0.2920863309 
    segment.width                               = 0.2012565415  
    fuselage.Segments.append(segment)         
       
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_11'   
    segment.percent_x_location                  = 1 
    segment.percent_z_location                  = 0.04787575562  
    segment.height                              = 0.1251798561 
    segment.width                               = 0.1206021048 
    fuselage.Segments.append(segment)             
    
    # add to vehicle
    vehicle.append_component(fuselage)   
    
    # ---------------------------------------------------------------------------------------------
    #   Landing gear                                                   
    # ---------------------------------------------------------------------------------------------  
    landing_gear                                = SUAVE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = SUAVE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = SUAVE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches # guess based on picture
    nose_gear.strut_length                      = 6. * Units.inches  
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear 
    vehicle.landing_gear                        = landing_gear
    
    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net                         = Battery_Propeller() 
    net.number_of_engines       = 8.
    net.nacelle_diameter        = 1  * Units.feet  
    net.engine_length           = 3 * Units.feet   
    net.nacelle_start           = 0.2 
    net.nacelle_end             = 1.
    net.nacelle_offset          = [0.5,0.0,0.0] 
    net.areas                   = Data() 
    net.areas.wetted            = net.engine_length*(2*np.pi*net.nacelle_diameter/2)   

    # Component 1 the ESC
    esc                         = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency              = 0.95 # Gundlach for brushless motors
    net.esc                     = esc 

    # Component 2 the Propeller 
    propeller                        = SUAVE.Components.Energy.Converters.Propeller()  
    propeller.number_of_blades       = 5
    propeller.freestream_velocity    = 135.*Units['mph']    
    propeller.angular_velocity       = 1300.  * Units.rpm  
    propeller.tip_radius             = 0.58 
    propeller.hub_radius             = propeller.tip_radius*0.15
    propeller.design_Cl              = 0.8
    propeller.design_altitude        = 2500. * Units.feet 
    propeller.design_thrust          = 400.     
    propeller.pitch                  = 1.4
    propeller.origin                 = [[2.5, (1.05 + propeller.pitch*0), 1.01],[2.5, (1.05 + propeller.pitch*1), 1.01],
                                        [2.5, (1.05 + propeller.pitch*2) ,1.01], [2.5, (1.05 + propeller.pitch*3), 1.01],
                                        [2.5, -(1.05 + propeller.pitch*0), 1.01],[2.5, -(1.05 + propeller.pitch*1), 1.01], 
                                       [2.5, -(1.05 + propeller.pitch*2), 1.01],[2.5 ,-(1.05 + propeller.pitch*3), 1.01]]
    propeller.rotation               = [-1,1,-1,1,-1,1,-1,1,-1,1] 
    propeller.symmetry               = True  
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator 
    propeller.airfoil_geometry           =  [rel_path +'NACA_4412.txt']
    propeller.airfoil_polars             = [[rel_path +'NACA_4412_polar_Re_50000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_100000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_200000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_500000.txt' ,
                                         rel_path +'NACA_4412_polar_Re_1000000.txt' ]]
    propeller.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]        
    propeller                        = propeller_design(propeller)    
    net.propeller                    = propeller      
    
    # Component 3 the Battery
    bat                         = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()   
    bat.cell.surface_area       = (np.pi*bat.cell.height*bat.cell.diameter)  
    bat.pack_config.series      =  120  
    bat.pack_config.parallel    =  48    
    initialize_from_circuit_configuration(bat)       
    net.battery                 = bat
    net.voltage                 = bat.max_voltage    
    
    # Component 4 Miscellaneous Systems 
    sys                           = SUAVE.Components.Systems.System()
    sys.mass_properties.mass      = 5 # kg
     
    # Component 5 the Motor
    motor                         = SUAVE.Components.Energy.Converters.Motor() 
    motor.efficiency              = 0.9
    motor.gearbox_efficiency      = 1.0
    motor.nominal_voltage         = bat.max_voltage *0.75
    motor.propeller_radius        = propeller.tip_radius    
    motor.no_load_current         = 0.001
    motor                         = size_optimal_motor(motor,propeller) 
    motor.mass_properties.mass    = nasa_motor(motor.design_torque)
    net.motor                     = motor   

    # Component 6 the Payload
    payload                       = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw            = 10. # Watts 
    payload.mass_properties.mass  = 1.0 * Units.kg
    net.payload                   = payload
                                  
    # Component 7 the Avionics    
    avionics                      = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw           = 20. # Watts  
    net.avionics                  = avionics      

    # add the solar network to the vehicle
    vehicle.append_component(net)          
 
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    # plot vehicle 
    #plot_vehicle(vehicle,plot_control_points = False)
    #plt.show()
    
    return vehicle

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses 

# ---------------------------------------------------------------------
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
    config.tag = 'landing' 
    config.propulsors.battery_propeller.pitch_command      = 0.0 * Units.degrees 
    config.propulsors.battery_propeller.motor.gear_ratio   = 1.1
    configs.append(config)                  


    # done!
    return configs 
 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle): 
    
    Climb_Rate           = 500* Units['ft/min']
    Descent_Rate         = 300 * Units['ft/min']
    Climb_Speed          = 120 * Units['mph']  
    Descent_Speed        = 110 * Units['mph'] 
    Nominal_Range        = 56.5 * Units['miles']
    Nominal_Flight_Time  = 29.3*(Nominal_Range/ Units['miles']) + 38.1
    Cruise_Altitude      = 2500 * Units.feet
    
    # Determine Stall Speed 
    m     = vehicle.mass_properties.max_takeoff
    g     = 9.81
    S     = vehicle.reference_area
    atmo  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho   = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0. * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments 
    
    # base segment
    base_segment                                             = Segments.Segment()
    ones_row                                                 = base_segment.state.ones_row
    base_segment.use_Jacobian                                = False  
    base_segment.process.initialize.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = 4
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_propeller.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.battery_propeller.residuals
    base_segment.state.unknowns.propeller_power_coefficient  = 0.05 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.battery_propeller.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)       
 
    # Determine Stall Speed 
    m     = vehicle.mass_properties.max_takeoff
    g     = 9.81
    S     = vehicle.reference_area
    atmo  = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    rho   = atmo.compute_values(1000.*Units.feet,0.).density
    CLmax = 1.2 
    Vstall = float(np.sqrt(2.*m*g/(rho*S*CLmax))) 
    
    # Calculate Flight Parameters 
    ICA_Altitude        = 500 * Units.feet 
    Downleg_Altitude    = 1000 * Units.feet  
    Climb_Altitude      = Cruise_Altitude  - ICA_Altitude
    Descent_Altitude    = Cruise_Altitude - Downleg_Altitude  
     
    Climb_Range_Speed   = np.sqrt(Climb_Speed**2 - Climb_Rate**2)
    Climb_Time          = Climb_Altitude/Climb_Rate 
    Climb_Range         = Climb_Range_Speed * Climb_Time 
    Descent_Range_Speed = np.sqrt(Descent_Speed**2 - Descent_Rate**2)
    Descent_Time        = Descent_Altitude /(Descent_Rate )
    Descent_Range       = Descent_Range_Speed * Descent_Time 
    Cruise_Distance     = Nominal_Range - (Climb_Range + Descent_Range)
    Cruise_Time         = Nominal_Flight_Time - (Climb_Time+Descent_Time )
    Cruise_Speed        = Cruise_Distance/Cruise_Time  
    
    
    # Calculate Flight Parameters 
    Downleg_Distance        = 6000 * Units.feet # length or runway
    Downleg_Altitude        = 1000 * Units.feet 
  
  
    cl_throt_ukn  = 0.85
    cl_CP_ukn     = 0.3
    cr_throt_ukn  = 0.6
    cr_CP_ukn     = 0.4
    des_throt_ukn = 0.5
    des_CP_ukn    = 0.3
    
    # ------------------------------------------------------------------
    #   Departure End of Runway Segment Flight 1 : 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'DER'       
    segment.analyses.extend( analyses.base ) 
    segment.battery_energy                                   = vehicle.propulsors.battery_propeller.battery.max_energy  
    segment.state.unknowns.throttle                          = 0.95 * ones_row(1) 
    segment.altitude_start                                   = 0.0 * Units.feet
    segment.altitude_end                                     = 50.0 * Units.feet
    segment.air_speed_start                                  = Vstall  
    segment.air_speed_end                                    = 45                    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Initial Climb Area Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'ICA' 
    segment.analyses.extend( analyses.base )  
    segment.state.unknowns.throttle                          = 0.9  * ones_row(1)  
    segment.altitude_start                                   = 50.0 * Units.feet
    segment.altitude_end                                     = 500.0 * Units.feet
    segment.air_speed_start                                  = 45  * Units['m/s']   
    segment.air_speed_end                                    = 50 * Units['m/s']   
    segment.climb_rate                                       = 600 * Units['ft/min']    
    mission.append_segment(segment) 
             
    # ------------------------------------------------------------------
    #   Climb Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Climb'        
    segment.analyses.extend( analyses.base )     
    segment.state.unknowns.throttle                          = cl_throt_ukn * ones_row(1)
    segment.state.unknowns.propeller_power_coefficient       = cl_CP_ukn  * ones_row(1)   
    segment.altitude_start                                   = 500.0 * Units.feet
    segment.altitude_end                                     = Cruise_Altitude
    segment.air_speed                                        = Climb_Speed
    segment.climb_rate                                       = Climb_Rate  
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Cruise Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
    segment.tag = 'Cruise'  
    segment.analyses.extend(analyses.base) 
    segment.air_speed                                        = Cruise_Speed
    segment.distance                                         = Cruise_Distance 
    segment.state.unknowns.throttle                          = cr_throt_ukn * ones_row(1)    
    segment.state.unknowns.propeller_power_coefficient       = 0.12 * ones_row(1)   
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   Descent Segment Flight 1   
    # ------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Descent'
    segment.analyses.extend( analyses.base )    
    segment.altitude_end                                     = Downleg_Altitude
    segment.air_speed                                        = Descent_Speed
    segment.descent_rate                                     = Descent_Rate 
    segment.state.unknowns.throttle                          = des_throt_ukn  * ones_row(1)  
    segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn     * ones_row(1)   
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #  Downleg_Altitude Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment) 
    segment.tag = 'Downleg'
    segment.analyses.extend(analyses.base) 
    segment.air_speed_start                                  = Descent_Speed * Units['m/s']
    segment.air_speed_end                                    = 45.0 * Units['m/s']            
    segment.distance                                         = Downleg_Distance
    segment.acceleration                                     = -0.05307 * Units['m/s/s']  
    segment.air_speed                                        = 49.174
    segment.descent_rate                                     = Descent_Rate
    segment.state.unknowns.throttle                          = des_throt_ukn   * ones_row(1)  
    segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn      * ones_row(1)      
    
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #  Reserve Climb 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Reserve_Climb'        
    segment.analyses.extend( analyses.base )     
    segment.state.unknowns.throttle                          = cl_throt_ukn * ones_row(1)
    segment.state.unknowns.propeller_power_coefficient       = cl_CP_ukn  * ones_row(1)    
    segment.altitude_end                                     = 1500 * Units.feet
    segment.air_speed                                        = Climb_Speed
    segment.climb_rate                                       = Climb_Rate  
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #  Researve Cruise Segment 
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
    segment.tag = 'Reserve_Cruise'  
    segment.analyses.extend(analyses.base) 
    segment.air_speed                                        = Cruise_Speed
    segment.distance                                         = Cruise_Distance * 0.1
    segment.state.unknowns.throttle                          = cr_throt_ukn * ones_row(1)    
    segment.state.unknowns.propeller_power_coefficient       = cr_CP_ukn * ones_row(1)   
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #  Researve Descent
    # ------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Reserve_Descent'
    segment.analyses.extend( analyses.base )    
    segment.altitude_end                                     = Downleg_Altitude
    segment.air_speed                                        = Descent_Speed
    segment.descent_rate                                     = Descent_Rate 
    segment.state.unknowns.throttle                          = des_throt_ukn  * ones_row(1)  
    segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn     * ones_row(1)   
    mission.append_segment(segment)  

    
    # ------------------------------------------------------------------
    #  Baseleg Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = 'Baseleg'
    segment.analyses.extend( analyses.base)  
    segment.state.unknowns.throttle                          = 0.8 * ones_row(1)
    segment.state.unknowns.propeller_power_coefficient       = 0.3    * ones_row(1)      
    segment.altitude_start                                   = 1000 * Units.feet
    segment.altitude_end                                     = 500.0 * Units.feet
    segment.air_speed_start                                  = 45 
    segment.air_speed_end                                    = 40    
    segment.climb_rate                                       = -350 * Units['ft/min']
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #  Final Approach Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment_name = 'Final_Approach'
    segment.tag = segment_name          
    segment.analyses.extend( analyses.landing)     
    segment.state.unknowns.throttle                          = 0.8 * ones_row(1)
    segment.state.unknowns.propeller_power_coefficient       = 0.3 *  ones_row(1)      
    segment.altitude_start                                   = 500.0 * Units.feet
    segment.altitude_end                                     = 00.0 * Units.feet
    segment.air_speed_start                                  = 40 
    segment.air_speed_end                                    = 35   
    segment.climb_rate                                       = -300 * Units['ft/min']        
    mission.append_segment(segment)  
 
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


def plot_results(results,line_style = 'bo-'):  
    
    
    plot_flight_conditions(results, line_style) 
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results, line_style)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results, line_style)
    
    # Plot Aircraft Electronics
    plot_electronic_conditions(results, line_style)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results, line_style) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results, line_style)
    
    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results, line_style)   
    
    ## Plot noise level
    #plot_noise_level(results)
    
    ## Plot noise contour
    #plot_flight_profile_noise_contour(results)
    
    # plot static contour
    #plot_static_noise_contour(results)
                        
    return


def plot_paper_image(results): 
    
    axis_tick_size   = 18
    axis_font = {'size':'18'} 
    fig = plt.figure()
    plt.rcParams.update({'font.size': 18})
    plt.rcParams['axes.linewidth'] = 3.
    plt.title('ECTOL Propeller Tip Mach Number & Altitude')  
    fig.set_size_inches(8, 6)   
    axes1 = fig.add_subplot(1,1,1)     
    #fig.tight_layout( pad= 20, h_pad= 10., w_pad=20 ) 
    for segment in results.segments.values():  
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        #rpm      = segment.conditions.propulsion.rpm[:,0] 
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft 
        tm       = segment.conditions.propulsion.propeller_tip_mach[:,0]   
        c2   = 'blue'   
        axes1.set_xlabel('Time (mins)',axis_font)
        axes1.set_ylabel('Tip Mach',axis_font, color =  c2 )    
        
        c1   = 'grey' 
        axes2 = axes1.twinx()   
        axes2.plot(time, altitude , marker = 'o', linestyle = '-', color = c1 , linewidth= 4)

        axes1.plot(time, tm, marker = 'o', linestyle = '-', color = c2 , linewidth= 4)        
        axes2.set_ylabel('Altitude (ft)', axis_font, color = c1 )   

        axes1.tick_params(axis = 'x', labelsize = axis_tick_size ) 
        axes2.tick_params(axis = 'y', labelsize = axis_tick_size , labelcolor= c1) 
        axes1.tick_params(axis = 'y', labelsize = axis_tick_size , labelcolor= c2) 
        axes1.set_ylim(0.15, 0.55 )   
        axes2.set_ylim(0, 2600 ) 
    
    plt.tight_layout()      
    return

def save_results(results):

    # Store data (serialize)
    with open('DEP_ECTOL_8P.pkl', 'wb') as file:
        pickle.dump(results, file)
        
    return   

if __name__ == '__main__': 
    main()    
    plt.show()