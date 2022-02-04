# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units

import numpy as np
import pylab as plt
import matplotlib
import copy, time
import pickle
from SUAVE.Core import Data

from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Power.Battery.Sizing import  initialize_from_circuit_configuration
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    '''This script allows you to choose compare different discharge methods of SUAVE 
    2 -> Thevevin Model with RC components LiNCA 
    3 -> LiNiMnCo '''
    battery_fidelity = 3
    
    # second order discharge
    configs, analyses = full_setup(battery_fidelity) 
    simple_sizing(configs) 
    configs.finalize()
    analyses.finalize()     
    mission      = analyses.missions.base
    results = mission.evaluate()
    
    #filename = 'test_vehicle.pkl'
    #save_results(results,filename)
    plot_mission(results, configs.base)


    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(battery_fidelity):

    # vehicle data
    vehicle  = vehicle_setup(battery_fidelity)
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
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()    
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

def vehicle_setup(battery_fidelity):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'X57_Maxwell'    


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
    vehicle.reference_area         = 15.45  
    vehicle.passengers             = 4

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.span_efficiency         = 0.9
    wing.areas.reference         = 15.45 * Units['meters**2']  
    wing.spans.projected         = 11. * Units.meter  

    wing.chords.root             = 1.67 * Units.meter  
    wing.chords.tip              = 1.14 * Units.meter  
    wing.chords.mean_aerodynamic = 1.47 * Units.meter   
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 1.5 * Units.degrees

    wing.origin                  = [[2.032, 0., 0.]] 
    wing.aerodynamic_center      = [0.558, 0., 0.]

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
    wing.areas.reference         = 3.74 * Units['meters**2']  
    wing.spans.projected         = 3.454  * Units.meter 
    wing.sweeps.quarter_chord    = 12.5 * Units.deg

    wing.chords.root             = 1.397 * Units.meter 
    wing.chords.tip              = 0.762 * Units.meter 
    wing.chords.mean_aerodynamic = 1.09 * Units.meter 
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[6.248, 0., 0.784]] 
    wing.aerodynamic_center      = [0.558, 0., 0.]
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
    wing.areas.reference         = 2.258 * Units['meters**2']  
    wing.spans.projected         = 1.854   * Units.meter 

    wing.chords.root             = 1.6764 * Units.meter 
    wing.chords.tip              = 0.6858 * Units.meter 
    wing.chords.mean_aerodynamic = 1.21 * Units.meter 
    wing.taper                   = wing.chords.root/wing.chords.tip

    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[6.01 ,0,  0.623]] 
    wing.aerodynamic_center      = [0.508 ,0,0] 

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
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2. 
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
    segment                                     = SUAVE.Components.Fuselages.Segment() 
    segment.tag                                 = 'segment_0'  
    segment.percent_x_location                  = 0 
    segment.percent_z_location                  = 0 
    segment.height                              = 0.01 
    segment.width                               = 0.01 
    fuselage.Segments.append(segment)             
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.007279116466
    segment.percent_z_location                  = 0.002502014453
    segment.height                              = 0.1669064748
    segment.width                               = 0.2780205877
    fuselage.Segments.append(segment)    

    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.01941097724 
    segment.percent_z_location                  = 0.001216095397 
    segment.height                              = 0.3129496403 
    segment.width                               = 0.4365777215 
    fuselage.Segments.append(segment)         

    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.06308567604
    segment.percent_z_location                  = 0.007395489231
    segment.height                              = 0.5841726619
    segment.width                               = 0.6735119903
    fuselage.Segments.append(segment)      
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.1653761217 
    segment.percent_z_location                  = 0.02891281352 
    segment.height                              = 1.064028777 
    segment.width                               = 1.067200529 
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.2426372155 
    segment.percent_z_location                  = 0.04214148761 
    segment.height                              = 1.293766653 
    segment.width                               = 1.183058255 
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 0.2960174029  
    segment.percent_z_location                  = 0.04705241831  
    segment.height                              = 1.377026712  
    segment.width                               = 1.181540054  
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.3809404284 
    segment.percent_z_location                  = 0.05313580461 
    segment.height                              = 1.439568345 
    segment.width                               = 1.178218989 
    fuselage.Segments.append(segment)    
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.5046854083 
    segment.percent_z_location                  = 0.04655492473 
    segment.height                              = 1.29352518 
    segment.width                               = 1.054390707 
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.6454149933 
    segment.percent_z_location                  = 0.03741966266 
    segment.height                              = 0.8971223022 
    segment.width                               = 0.8501926505   
    fuselage.Segments.append(segment)  
      
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_10'   
    segment.percent_x_location                  = 0.985107095 
    segment.percent_z_location                  = 0.04540283436 
    segment.height                              = 0.2920863309 
    segment.width                               = 0.2012565415  
    fuselage.Segments.append(segment)         
       
    # Segment                                   
    segment                                     = SUAVE.Components.Fuselages.Segment()
    segment.tag                                 = 'segment_11'   
    segment.percent_x_location                  = 1 
    segment.percent_z_location                  = 0.04787575562  
    segment.height                              = 0.1251798561 
    segment.width                               = 0.1206021048 
    fuselage.Segments.append(segment)             
    
    # add to vehicle
    vehicle.append_component(fuselage)   

    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net = Battery_Propeller()
    net.dischage_model_fidelity = battery_fidelity
    net.number_of_engines       = 2.
    net.nacelle_diameter        = 42 * Units.inches
    net.engine_length           = 0.01 * Units.inches
    net.areas                   = Data()
    net.areas.wetted            = 0.01*(2*np.pi*0.01/2)    


    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 the Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller() 

    prop.number_of_blades    = 3
    prop.freestream_velocity = 135.*Units['mph']    
    prop.angular_velocity    = 1300.  * Units.rpm # 2400
    prop.tip_radius          = 76./2. * Units.inches
    prop.hub_radius          = 8.     * Units.inches
    prop.design_Cl           = 0.8
    prop.design_altitude     = 12000. * Units.feet
    prop.design_altitude     = 12000. * Units.feet
    prop.design_thrust       = 800. 
    prop.origin              = [[2.,2.5,0.784],[2.,-2.5,0.784]]  #  prop influence      
    prop.rotation            = [1,-1]
    prop                     = propeller_design(prop)    
    net.propeller            = prop    

    # Component 8 the Battery
    if battery_fidelity == 2:
        bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNCA_18650()  
    elif battery_fidelity == 3: 
        bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()   
    bat.cell.mass                   = 0.048 * Units.kg
    bat.cell.nominal_capacity       = 3.   # [Ah]
    bat.cell.nominal_voltage        = 3.6  # [V]
    bat.cell.max_mission_discharge  = 9.   # Amps 
    bat.cell.max_discharge_rate     = 15.  # Amps 
    bat.cell.specific_heat_capacity = 0.83 # [J/gramC] 
    bat.cell.diameter               = 0.0018
    bat.cell.height                 = 0.06485
    bat.cell.surface_area           = (0.5*np.pi*bat.cell.diameter**2)  + (np.pi*bat.cell.height*bat.cell.diameter) 
    bat.cell.temperature            = 45. # currently set at 45 constant temp - this assumption holds if BMS has thermal management system 
    bat.pack_config.series          = 128 #[series,parallel]   
    bat.pack_config.parallel        = 48
    bat.heat_transfer_coefficient   = 10 # incorrect number  
    bat.coolant_temperature         = 20
    bat.temperature                 = 45. 
    initialize_from_circuit_configuration(bat)   
    net.battery                     = bat  
    net.voltage                     = bat.max_voltage
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller  motor
    # Component 4 the Motor
    motor                              = SUAVE.Components.Energy.Converters.Motor()
    etam                               = 0.95
    v                                  = bat.max_voltage *3/4
    omeg                               = prop.angular_velocity  
    io                                 = 4.0 
    start_kv                           = 1
    end_kv                             = 25
    # do optimization to find kv or just do a linspace then remove all negative values, take smallest one use 0.05 change
    # essentially you are sizing the motor for a particular rpm which is sized for a design tip mach 
    # this reduces the bookkeeping errors     
    possible_kv_vals                   = np.linspace(start_kv,end_kv,(end_kv-start_kv)*20 +1 , endpoint = True) * Units.rpm
    res_kv_vals                        = ((v-omeg/possible_kv_vals)*(1.-etam*v*possible_kv_vals/omeg))/io  
    positive_res_vals                  = np.extract(res_kv_vals > 0 ,res_kv_vals) 
    kv_idx                             = np.where(res_kv_vals == min(positive_res_vals))[0][0]   
    kv                                 = possible_kv_vals[kv_idx]  
    res                                = min(positive_res_vals) 

    motor.mass_properties.mass         = 10. * Units.kg
    motor.origin                       = prop.origin  
    motor.propeller_radius             = prop.tip_radius   
    motor.speed_constant               = 0.35 
    motor.resistance                   = res
    motor.no_load_current              = io 
    motor.gear_ratio                   = 1. 
    motor.gearbox_efficiency           = 1. # Gear box efficiency     
    net.motor                          = motor 


    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 10. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 20. #Watts  
    net.avionics        = avionics      

    # add the solar network to the vehicle
    vehicle.append_component(net)          

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

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
    base_config.propulsors.battery_propeller.pitch_command = 0 
    configs.append(base_config) 


    # done!
    return configs

# ----------------------------------------------------------------------
#   Sizing for the Vehicle Configs
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results ,vec_configs):
    label_size = 12
    legend_font_size = 14 
    line_width  = 3 
    axis_font        = {'size':'12'}
    taxi_takeoff_time = 335
    paper_results  = NASA_X57_data()
    prop_radius_ft = vec_configs.propulsors.battery_propeller.propeller.tip_radius*3.28084 # convert to ft 

    # Final Energy
    maxcharge         = vec_configs.propulsors.battery_propeller.battery.max_energy
    energy_constraint = (results.segments[-1].conditions.propulsion.battery_energy[-1,0] - maxcharge*0.1)    

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    fig.set_size_inches(16, 8)

    for i in range(len(results.segments)): 

        time       = taxi_takeoff_time +  results.segments[i].conditions.frames.inertial.time[:,0] 
        airspeed   = results.segments[i].conditions.freestream.velocity[:,0] 
        range_nm   = results.segments[i].conditions.frames.inertial.position_vector[:,0]  
        altitude   = results.segments[i].conditions.freestream.altitude[:,0]*3.28084  # convert to ft 

        axes = fig.add_subplot(2,1,1)
        axes.plot(time, altitude, 'bo-',label='SUAVE Model')
        axes.plot(paper_results.altitude[:,0],paper_results.altitude[:,1] , 'sg-',label='NASA Model')
        axes.set_ylabel('Altitude (ft)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')        
        if i == 0:
            axes.legend(loc='upper right')  
 
 
        range_credit  = 2 
        axes = fig.add_subplot(2,1,2)
        axes.plot( time ,range_credit +  range_nm*0.000539957 , 'bo-' ,label='SUAVE Model' )
        axes.plot(paper_results.vehicle_range[:,0],paper_results.vehicle_range[:,1] , 'sg-',label='NASA Model')
        axes.set_ylabel('Range (nm)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
        axes.set_xlabel('Time (s)', axis_font)
        if i == 0:
            axes.legend(loc='upper right')    
         

    # ------------------------------------------------------------------
    #   Aero Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Aero Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)): 

        time = taxi_takeoff_time +  results.segments[i].conditions.frames.inertial.time[:,0] 
        cl   = results.segments[i].conditions.aerodynamics.lift_coefficient[:,0,None] 
        cd   = results.segments[i].conditions.aerodynamics.drag_coefficient[:,0,None] 
        aoa  = results.segments[i].conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d  = cl/cd

        axes = fig.add_subplot(2,2,1)
        axes.plot( time , aoa , 'bo-' )
        axes.set_ylabel('Angle of Attack (deg)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , cl, 'bo-' )
        axes.set_ylabel('CL', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,2,3)
        axes.plot( time , cd, 'bo-' )
        axes.set_xlabel('Time (s)', axis_font)
        axes.set_ylabel('CD', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,2,4)
        axes.plot( time , l_d, 'bo-' )
        axes.set_xlabel('Time (s)', axis_font)
        axes.set_ylabel('L/D', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        

    #plt.savefig("X57_Maxwell_Aero.png")   


    # ------------------------------------------------------------------
    #   Electronic Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Pack Level Electronic Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):  

        time     = taxi_takeoff_time +  results.segments[i].conditions.frames.inertial.time[:,0] 
        eta       = results.segments[i].conditions.propulsion.throttle[:,0] 
        SOC       = results.segments[i].conditions.propulsion.state_of_charge[:,0]
        energy    = results.segments[i].conditions.propulsion.battery_energy[:,0] 
        volts   = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc  = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]     
        current  = results.segments[i].conditions.propulsion.current[:,0]      
        battery_amp_hr  = (energy *0.000277778)/volts 
        C_rating    = current /battery_amp_hr 
        
        axes = fig.add_subplot(3,2,1) 
        axes.plot(time, SOC , 'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)        
        axes.plot(paper_results.battery_SOC[:,0],paper_results.battery_SOC[:,1] , 'sg-',label='Chin T.E.C Model') 
        axes.set_ylabel('State of Charge', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,2) 
        axes.plot(time, energy *0.000277778 , 'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)        
        axes.set_ylabel('Battery Energy (W-hr)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,3) 
        axes.plot(time, volts   , 'rs-' ,label='SUAVE T.E.C Model ULV', linewidth= line_width) 
        axes.plot(time, volts_oc , color='darkred', linestyle = ':', marker = 's', label='SUAVE T.E.C Model OCV')
        axes.set_ylabel('Battery Voltage (V)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,4) 
        axes.plot(time, current , 'rs-' , label ='SUAVE T.E.C Model', linewidth= line_width)
        axes.set_ylabel('Current (Amp)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,5) 
        axes.plot(time, C_rating , 'rs-' , label ='SUAVE T.E.C Model', linewidth= line_width) 
        axes.set_ylabel('C-Rating (C)', axis_font)  
        axes.set_xlabel('Time (s)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,6) 
        axes.plot(time, eta ,  'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)
        axes.set_ylabel('Throttle ($\eta$)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.set_xlabel('Time (s)', axis_font)                 
        if i == 0:
            axes.legend(loc='upper right')          

    #plt.savefig("X57_Maxwell_Electronics.png")   
    
    
    # ------------------------------------------------------------------
    #   Cell Level Electronic Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Cell Level Electronic Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):  

        time                 = taxi_takeoff_time +  results.segments[i].conditions.frames.inertial.time[:,0] 
        eta            = results.segments[i].conditions.propulsion.throttle[:,0] 
        SOC            = results.segments[i].conditions.propulsion.state_of_charge[:,0]
        energy         = results.segments[i].conditions.propulsion.battery_energy[:,0] 
        volts          = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc       = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]     
        current        = results.segments[i].conditions.propulsion.current[:,0]      
        battery_amp_hr = (energy *0.000277778)/volts 
        C_rating       = current /battery_amp_hr 
        
        axes = fig.add_subplot(3,2,1) 
        axes.plot(time, SOC , 'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)        
        axes.plot(paper_results.battery_SOC[:,0],paper_results.battery_SOC[:,1] , 'sg-',label='Chin T.E.C Model') 
        axes.set_ylabel('State of Charge', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')             

        axes = fig.add_subplot(3,2,2) 
        axes.plot(time, energy *0.000277778 , 'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)        
        axes.set_ylabel('Cell Energy (W-hr)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,3) 
        axes.plot(time, volts/128   , 'rs-' ,label='SUAVE T.E.C Model ULV', linewidth= line_width) 
        axes.plot(time, volts_oc/128 , color='darkred', linestyle = ':', marker = 's',label='SUAVE T.E.C Model OCV')
        axes.set_ylabel('Cell Voltage (V)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')     
        if i == 0:
            axes.legend(loc='upper right')             

        axes = fig.add_subplot(3,2,4) 
        axes.plot(time, current/40 , 'rs-' , label ='SUAVE T.E.C Model', linewidth= line_width) 
        axes.set_ylabel('Current (Amp)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,5) 
        axes.plot(time, C_rating , 'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)
        axes.set_xlabel('Time (s)', axis_font)
        axes.set_ylabel('C-Rating (C)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,6) 
        axes.plot(time, eta ,  'rs-', label ='SUAVE T.E.C Model', linewidth= line_width)
        axes.set_ylabel('Throttle ($\eta$)', axis_font) 
        axes.set_xlabel('Time (s)', axis_font)        
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Propulsor")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):  

        time   = results.segments[i].conditions.frames.inertial.time[:,0]
        rpm    = results.segments[i].conditions.propulsion.rpm [:,0] 
        thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,0]
        torque = results.segments[i].conditions.propulsion.motor_torque
        effp   = results.segments[i].conditions.propulsion.etap[:,0]
        effm   = results.segments[i].conditions.propulsion.etam[:,0]
        prop_omega = results.segments[i].conditions.propulsion.rpm*0.104719755  
        ts = prop_omega*prop_radius_ft

        axes = fig.add_subplot(2,3,1)
        axes.plot(time, rpm, 'bo-')
        axes.set_ylabel('RPM', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, thrust, 'bo-')
        axes.set_ylabel('Thrust (N)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,3,3)
        axes.plot(time, torque, 'bo-' ) 
        axes.set_ylabel('Torque (N-m)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(2,3,4)
        axes.plot(time, effp, 'bo-' )
        axes.set_xlabel('Time (s)', axis_font)
        axes.set_ylabel('Propeller Efficiency ($\eta_{prop}$)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        plt.ylim((0,1))

        axes = fig.add_subplot(2,3,5)
        axes.plot(time, effm, 'bo-' )
        axes.set_xlabel('Time (s)', axis_font)
        axes.set_ylabel('Motor Efficiency ($\eta_{motor}$)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        plt.ylim((0,1))

        axes = fig.add_subplot(2,3,6)
        axes.plot(time, ts, 'bo-' )
        axes.set_xlabel('Time (s)', axis_font)
        axes.set_ylabel('Tip Speed (ft/s)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')     

    #plt.savefig("X57_Maxwell_Propulsion.png")   

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

    # done!
    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle,battery_fidelity):
    Kelvin_to_Celcius = -273.15
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
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip  
    base_segment.state.unknowns.propeller_power_coefficient  = 0.005 * ones_row(1)         
    base_segment.state.residuals.network                     = 0. * ones_row(3)        
    
    # Thevenin Discharge Model 
    if battery_fidelity == 2:
        segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)
        segment.tag = "Battery Discharge" 
        segment.analyses.extend(analyses.base)     
        segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_linca
        segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_linca    
        segment.state.unknowns.battery_state_of_charge      = 1  * ones_row(1) 
        segment.state.unknowns.battery_thevenin_voltage     = 0.01 * ones_row(1) 
        segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
        segment.state.residuals.network                     = 0.   * ones_row(3)   
        segment.time                                        = discharge_time
        segment.battery_discharge                           = True 
        segment.battery_cell_temperature                    = 20.  
        segment.ambient_temperature                         = 20    
        segment.battery_cumulative_charge_throughput         = 0
        segment.battery_energy                              = bat.max_energy * 1.
                
        
    elif battery_fidelity ==3 :
        segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)
        segment.tag = "Battery Discharge" 
        segment.analyses.extend(analyses.base)     
        segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_linmco
        segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_linmco    
        segment.state.unknowns.battery_state_of_charge      = 1  * ones_row(1)  
        segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
        segment.state.unknowns.battery_current              = 2  * ones_row(1) 
        segment.state.residuals.network                     = 0.   * ones_row(3)   
        segment.time                                        = discharge_time
        segment.battery_discharge                           = True 
        segment.battery_cell_temperature                    = 20. 
        segment.ambient_temperature                         = 20.  
        segment.battery_cumulative_charge_throughput        = 0
        segment.battery_energy                              = bat.max_energy * 1.
        

   
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

def save_results(results,filename):

    # Store data (serialize)
    with open(filename, 'wb') as file:
        pickle.dump(results, file)
    return  
def NASA_X57_data():
    X57_Maxwell_data = Data()
    X57_Maxwell_data.vehicle_range = np.array([[21.67182662538704, 0.5633802816901365  ]
                                               ,[111.0681114551083, 1.126760563380273   ]
                                               ,[235.68111455108357, 2.2535211267605604 ]
                                               ,[300.6965944272445, 2.816901408450704   ]
                                               ,[325.07739938080493, 2.816901408450704  ]
                                               ,[338.62229102167186, 2.816901408450704  ]
                                               ,[365.7120743034055, 3.3802816901408406  ]
                                               ,[493.0340557275542, 6.197183098591545   ]
                                               ,[609.5201238390093, 8.450704225352105   ]
                                               ,[707.0433436532508, 10.704225352112672  ]
                                               ,[801.8575851393186, 12.95774647887324   ]
                                               ,[850.6191950464395, 14.08450704225352   ]
                                               ,[967.1052631578946, 18.591549295774648  ]
                                               ,[1059.2105263157891, 21.408450704225345 ]
                                               ,[1137.7708978328171, 24.225352112676056 ]
                                               ,[1240.7120743034056, 27.605633802816904 ]
                                               ,[1308.4365325077397, 29.859154929577464 ]
                                               ,[1400.5417956656347, 33.239436619718305 ]
                                               ,[1452.0123839009286, 34.929577464788736 ]
                                               ,[1525.1547987616098, 37.183098591549296 ]
                                               ,[1601.0061919504642, 40                 ]
                                               ,[1668.7306501547987, 42.25352112676057  ]
                                               ,[1725.6191950464395, 44.507042253521135 ]
                                               ,[1779.798761609907, 45.07042253521127   ]])


    X57_Maxwell_data.altitude = np.array([[27.08978328173373, 2435.8974358974374]
                                          ,[295.27863777089783, 2500             ]
                                                  ,[327.7863777089783, 2435.8974358974374]
                                                  ,[346.74922600619186, 2500             ]
                                                  ,[837.0743034055728, 8076.923076923078 ]
                                                  ,[1408.6687306501549, 7948.717948717949]
                                                  ,[1957.2910216718265, 2371.794871794871]])


    X57_Maxwell_data.battery_SOC = np.array([[17.474707659965816, 0.9449152542372885]
                                             ,[294.4028379976349, 0.9110169491525428   ]
                                             ,[321.6725791617395, 0.90   ]
                                             ,[335.1924845618184, 0.89406779661017     ]
                                             ,[847.2014189988176, 0.5381355932203387   ]
                                             ,[1402.1613454211013, 0.26694915254237284 ]
                                             ,[1957.4372618578373, 0.21610169491525388 ]])

    X57_Maxwell_data.motor_temp = np.array([[22.105431164211836, 302.3822335719823 ]
                                            ,[38.70445528751878, 304.7407542523599  ]
                                                    ,[58.17638743216742, 308.27671120434127 ]
                                                    ,[93.84832869715905, 311.2298782434219  ]
                                                    ,[126.80696794199463, 314.1821332482101 ]
                                                    ,[167.66610424551965, 315.3723379999088 ]
                                                    ,[230.23165671029238, 316.5698390259474 ]
                                                    ,[262.8710839527568, 317.1690455561129  ]
                                                    ,[300.9371152355329, 317.77007615486343 ]
                                                    ,[306.76273427880886, 320.7132108167267 ]
                                                    ,[315.30165534224096, 323.6572575128825 ]
                                                    ,[329.4267864471705, 327.7796525149346  ]
                                                    ,[343.55191755209995, 331.9020475169866 ]
                                                    ,[366.0563637192758, 337.79196497788314 ]
                                                    ,[396.7007159469196, 343.68461854165713 ]
                                                    ,[429.89876419353385, 348.40165990241235]
                                                    ,[476.4239135391493, 351.35847507866293 ]
                                                    ,[514.649550823111, 353.13602991472476  ]
                                                    ,[590.7018103880707, 353.74982899357013 ]
                                                    ,[663.8013589310958, 352.59792968215606 ]
                                                    ,[728.8408044142461, 352.03155638652015 ]
                                                    ,[774.8073327557117, 350.8705367321811  ]
                                                    ,[809.9206530165534, 349.7058689406722  ]
                                                    ,[858.680286378768, 349.1340234392813   ]
                                                    ,[887.6487755939622, 342.67317251128645 ]
                                                    ,[914.1433717907794, 337.976195904966   ]
                                                    ,[965.0576861689999, 333.287427607278   ]
                                                    ,[1002.7247024488122, 330.94714761275026]
                                                    ,[1097.3710611518995, 328.6260203383647 ]
                                                    ,[1165.1238086552055, 328.06055907702125]
                                                    ,[1219.3898490583247, 328.0787997628711 ]
                                                    ,[1257.2962743399153, 327.50330612431026]
                                                    ,[1319.7022208035023, 327.5242829130375 ]
                                                    ,[1379.3948652469335, 327.54434766747227]
                                                    ,[1406.5278854484932, 327.5534680103972 ]
                                                    ,[1427.5958776049981, 322.8546673354918 ]
                                                    ,[1467.7367869031877, 318.75051301928954]
                                                    ,[1505.2441971818141, 315.2337087874504 ]
                                                    ,[1543.1506224634047, 314.6582151488896 ]
                                                    ,[1586.4836517853073, 314.0845455789138 ]
                                                    ,[1624.5496830680836, 314.6855761776643 ]
                                                    ,[1657.1891103105477, 315.28478270782983]
                                                    ,[1697.96844361348, 315.8867253408728   ]
                                                    ,[1730.6078708559444, 316.48593187103836]
                                                    ,[1784.8739112590636, 316.50417255688814]])

    return X57_Maxwell_data

if __name__ == '__main__': 
    main()    
    plt.show()