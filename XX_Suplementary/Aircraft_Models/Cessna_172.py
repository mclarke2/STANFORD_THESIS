# Cessna_172.py
# 
# Created:  Sep 2016, E. Botero
# Modified: Sep 2016, C. Ilario 

""" setup file for a mission with a C172 SP NAV III
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units

import numpy as np
import pylab as plt

from SUAVE.Core import Data
from SUAVE.Methods.Propulsion import propeller_design

from SUAVE.Input_Output.OpenVSP.vsp_write import write
from SUAVE.Input_Output.OpenVSP.get_vsp_areas import get_vsp_areas
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
import cProfile, pstats, StringIO
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()  

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)


    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    #write(vehicle, 'Cessna_172') 
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

    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)


    return analyses
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
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()    
    #aerodynamics.process.compute.lift.inviscid.settings.spanwise_vortex_density    = 3
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()    
    #stability.settings.spanwise_vortex_density                  = 3
    stability.geometry = vehicle
    analyses.append(stability)

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
    vehicle.tag = 'Cessna_172_SP'    


    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff   = 2550. * Units.pounds
    vehicle.mass_properties.takeoff       = 2550. * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 2550. * Units.pounds
    vehicle.mass_properties.cargo         = 0. 

    # envelope properties
    vehicle.envelope.ultimate_load = 5.7
    vehicle.envelope.limit_load    = 3.8

    # basic parameters
    vehicle.reference_area         = 174. * Units.feet**2       
    vehicle.passengers             = 4

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 174. * Units.feet**2
    wing.spans.projected         = 36.  * Units.feet + 1. * Units.inches

    wing.chords.root             = 66. * Units.inches
    wing.chords.tip              = 45. * Units.inches
    wing.chords.mean_aerodynamic = 58. * Units.inches # Guess
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 1.5 * Units.degrees

    wing.origin                  = [80.* Units.inches,0,0] 
    wing.aerodynamic_center      = [22.* Units.inches,0,0]

    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.95
    wing.areas.reference         = 5800. * Units.inches**2
    wing.spans.projected         = 136.  * Units.inches

    wing.chords.root             = 55. * Units.inches
    wing.chords.tip              = 30. * Units.inches
    wing.chords.mean_aerodynamic = 43. * Units.inches # Guess
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [246.* Units.inches,0,0] 
    wing.aerodynamic_center      = [20.* Units.inches,0,0]
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = False

    wing.dynamic_pressure_ratio  = 0.9

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    

    wing.sweeps.quarter_chord    = 25. * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 3500. * Units.inches**2
    wing.spans.projected         = 73.   * Units.inches

    wing.chords.root             = 66. * Units.inches
    wing.chords.tip              = 27. * Units.inches
    wing.chords.mean_aerodynamic = 48. * Units.inches # Guess
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [237.* Units.inches,0,  0.623] 
    wing.aerodynamic_center      = [20.* Units.inches,0,0] 

    wing.vertical                = True 
    wing.symmetric               = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.seats_abreast         = 2.

    fuselage.fineness.nose         = 1.6
    fuselage.fineness.tail         = 2.

    fuselage.lengths.nose          = 60.  * Units.inches
    fuselage.lengths.tail          = 161. * Units.inches
    fuselage.lengths.cabin         = 105. * Units.inches
    fuselage.lengths.total         = 326. * Units.inches
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.    

    fuselage.width                 = 42. * Units.inches

    fuselage.heights.maximum       = 62. * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches

    fuselage.areas.side_projected  = 8000.  * Units.inches**2.
    fuselage.areas.wetted          = 30000. * Units.inches**2.
    fuselage.areas.front_projected = 42.* 62. * Units.inches**2.

    fuselage.effective_diameter    = 50. * Units.inches


    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------ 
    
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    landing_gear.main_tire_diameter = 0.5587 * Units.m
    landing_gear.nose_tire_diameter = 0.381 * Units.m
    landing_gear.main_strut_length = 0.721 * Units.m
    landing_gear.nose_strut_length = 0.839 * Units.m
    landing_gear.main_units = 2     #number of main landing gear units
    landing_gear.nose_units = 1     #number of nose landing gear
    landing_gear.main_wheels = 1    #number of wheels on the main landing gear
    landing_gear.nose_wheels = 1    #number of wheels on the nose landing gear      
    vehicle.landing_gear=landing_gear    

  
    ## ------------------------------------------------------------------
    ##   Piston Propeller Network
    ## ------------------------------------------------------------------    
    
    ## build network
    #net = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    #net.number_of_engines = 1.
    #net.nacelle_diameter  = 42 * Units.inches
    #net.engine_length     = 0.01 * Units.inches
    #net.areas             = Data()
    #net.rated_speed       = 2700. * Units.rpm
    #net.areas.wetted      = 0.01

    ## Component 1 the engine
    #net.engine = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    #net.engine.sea_level_power    = 180. * Units.horsepower
    #net.engine.flat_rate_altitude = 0.0
    #net.engine.speed              = 2700. * Units.rpm
    #net.engine.BSFC               = 0.52


    ## Design the Propeller    
    #prop_attributes = Data()
    #prop_attributes.number_of_blades      = 2.0
    #prop_attributes.freestream_velocity = 119.   * Units.knots
    #prop_attributes.angular_velocity    = 2650.  * Units.rpm
    #prop_attributes.tip_radius          = 76./2. * Units.inches
    #prop_attributes.hub_radius          = 8.     * Units.inches
    #prop_attributes.design_Cl           = 0.8
    #prop_attributes.design_altitude     = 12000. * Units.feet
    #prop_attributes.design_thrust       = 0.0
    #prop_attributes.design_power        = .64 * 180. * Units.horsepower
    #prop_attributes                     = propeller_design(prop_attributes)    

    #prop = SUAVE.Components.Energy.Converters.Propeller()
    #prop.prop_attributes = prop_attributes
    #net.propeller        = prop

    ## add the network to the vehicle
    #vehicle.append_component(net)  
    
    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------

    # build network    
    net = Battery_Propeller()
    net.number_of_engines = 2.
    net.nacelle_diameter  = 42 * Units.inches
    net.engine_length     = 0.01 * Units.inches


    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc


    # Component 2 the Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades    = 3
    prop.freestream_velocity = 119.   * Units.knots
    prop.angular_velocity    = 2650.  * Units.rpm
    prop.tip_radius          = 1. *     Units.meters
    prop.hub_radius          = 8.     * Units.inches
    prop.design_Cl           = 0.8
    prop.design_altitude     = 12000. * Units.feet
    prop.design_thrust       = 0.0
    prop.design_power        = .64 * 180. * Units.horsepower
    prop                     = propeller_design(prop)    

    prop.origin          = [[0.,3.,0.],[0.,8.,0.]] 
    prop.rotation        = [1,-1]
    net.propeller        = prop    

    # Component 4 the Motor
    motor = SUAVE.Components.Energy.Converters.Motor()
    kv    = 400. * Units['rpm'] # RPM/volt converted to (rad/s)/volt 
    motor = size_from_kv(motor, kv)
    motor.gear_ratio           = 1.  # Gear ratio
    motor.gearbox_efficiency   = 1.  # Gear box efficiency
    motor.expected_current     = 15. # Expected current
    motor.propeller_radius     = prop.tip_radius
    motor.propeller_Cp         = prop.Cp    
    net.motor                  = motor    

    
    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 0. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload
    
    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. #Watts  
    net.avionics        = avionics      

    # Component 8 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 50 * Units.kg
    bat.specific_energy      = 145.5 * Units.Wh/Units.kg
    bat.resistance           = 0.006
    bat.max_voltage          = 24.6
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat
    
     
    # add the solar network to the vehicle
    vehicle.append_component(net)          


    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

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
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    configs.append(config)


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings['main_wing'].flaps.angle = 0. * Units.deg

    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2.
    
    config.landing_gear.gear_condition = 'down'

    configs.append(config)


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps_angle = 30. * Units.deg

    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.

    configs.append(config)


    # done!
    return configs

# ----------------------------------------------------------------------
#   Sizing for the Vehicle Configs
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    axis_font = {'fontname':'Arial', 'size':'14'}    

    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------


    fig = plt.figure("Aerodynamic Forces",figsize=(8,6))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0] / Units.lbf
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0] / Units.lbf
        eta  = segment.conditions.propulsion.throttle[:,0]
        mdot   = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc    = 3600. * mdot / 0.1019715 / thrust	

        axes = fig.add_subplot(2,1,1)
        axes.plot( time , Thrust , line_style )
        axes.set_ylabel('Thrust (lbf)',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)             
        axes.grid(True)

        axes = fig.add_subplot(2,1,2)
        axes.plot( time , eta , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Throttle',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)             
        axes.grid(True)	



    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients",figsize=(8,10))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]
        aoa = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d = CLift/CDrag


        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , line_style )
        axes.set_ylabel('Lift Coefficient',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , l_d , line_style )
        axes.set_ylabel('L/D',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , aoa , 'ro-' )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('AOA (deg)',axis_font)
        axes.grid(True)

    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components",figsize=(8,10))
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]

        if line_style == 'bo-':
            axes.plot( time , cdp , 'ko-', label='CD parasite' )
            axes.plot( time , cdi , 'bo-', label='CD induced' )
            axes.plot( time , cdc , 'go-', label='CD compressibility' )
            axes.plot( time , cdm , 'yo-', label='CD miscellaneous' )
            axes.plot( time , cd  , 'ro-', label='CD total'   )
            if i == 0:
                axes.legend(loc='upper center')            
        else:
            axes.plot( time , cdp , line_style )
            axes.plot( time , cdi , line_style )
            axes.plot( time , cdc , line_style )
            axes.plot( time , cdm , line_style )
            axes.plot( time , cd  , line_style )            

    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)

    # ------------------------------------------------------------------
    #   Altitude, sfc, vehicle weight
    # ------------------------------------------------------------------

    fig = plt.figure("Altitude_sfc_weight",figsize=(8,10))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]
        aoa    = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d    = CLift/CDrag
        mass   = segment.conditions.weights.total_mass[:,0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft
        mdot   = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc    = 3600. * mdot / 0.1019715 / thrust	

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , altitude , line_style )
        axes.set_ylabel('Altitude (ft)',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , sfc , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('sfc (lb/lbf-hr)',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , mass , 'ro-' )
        axes.set_ylabel('Weight (lb)',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.grid(True)
        plt.ylim((1800,2550))

        
    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Velocities",figsize=(8,10))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0] / Units.lbf
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0] / Units.lbf
        eta  = segment.conditions.propulsion.throttle[:,0]
        mdot   = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc    = 3600. * mdot / 0.1019715 / thrust
        velocity  = segment.conditions.freestream.velocity[:,0]
        pressure  = segment.conditions.freestream.pressure[:,0]
        density  = segment.conditions.freestream.density[:,0]
        EAS = velocity * np.sqrt(density/1.225)
        mach = segment.conditions.freestream.mach_number[:,0]


        axes = fig.add_subplot(3,1,1)
        axes.plot( time , velocity / Units.kts, line_style )
        axes.set_ylabel('velocity (kts)',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , EAS / Units.kts, line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Equivalent Airspeed',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.grid(True)    
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Mach',axis_font)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.grid(True)          
    
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Propulsor")
    fig.set_size_inches(10, 8)
    for i in range(len(results.segments)):  

        time   = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        rpm    = results.segments[i].conditions.propulsion.rpm[:,0] 
        power  = results.segments[i].conditions.propulsion.power[:,0] / Units.horsepower
        torque = results.segments[i].conditions.propulsion.propeller_torque 
        

        axes = fig.add_subplot(3,1,1)
        axes.plot(time, rpm, 'bo-')
        axes.set_ylabel('RPM')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)       
        #plt.ylim((0,2700))

        axes = fig.add_subplot(3,1,2)
        axes.plot(time, power, 'bo-')
        axes.set_ylabel('Power (hp)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)      
        axes.grid(True)   
        plt.ylim((0,180))

        axes = fig.add_subplot(3,1,3)
        axes.plot(time, torque, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)     
        axes.grid(True)   
              
        
    return

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 1.75 * wing.areas.reference
        wing.areas.exposed  = 0.8  * wing.areas.wetted
        wing.areas.affected = 0.6  * wing.areas.wetted


    # diff the new data
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # diff the new data
    landing.store_diff()

    # done!
    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.network.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.network.residuals
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1) 
    base_segment.state.unknowns.battery_voltage_under_load   = vehicle.propulsors.network.battery.max_voltage * ones_row(1)  
    base_segment.state.residuals.network                     = 0. * ones_row(2)       
    
    ## ------------------------------------------------------------------
    ##   First Climb Segment: constant Speed, constant rate segment 
    ## ------------------------------------------------------------------

    #segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    #segment.tag = "climb_1"

    #segment.analyses.extend( analyses.takeoff )

    #segment.altitude_start = 0.0   * Units.km
    #segment.altitude_end   = 1000. * Units.feet
    #segment.air_speed      = 73.5  * Units['knots']
    #segment.climb_rate     = 695.  * Units['ft/min']
    
    ## add to misison
    #mission.append_segment(segment)
   
    # ------------------------------------------------------------------
    #   First Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 8000. * Units.feet
    segment.air_speed = 92.   * Units['knots']
    segment.distance  = 100.   * Units.nautical_mile

    # add to misison
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



if __name__ == '__main__': 
    main()    
    plt.show()