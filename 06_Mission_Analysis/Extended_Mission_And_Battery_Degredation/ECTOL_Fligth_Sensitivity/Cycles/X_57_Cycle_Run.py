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
    '''This scrpt runs a specified mission for n times, n = the number 
    of days'''
    
    days   = 1 # Days
    Rates   = [700,350] # [Ascent Rate , Descent Rate]
    time0  = time.time()
    configs, analyses = full_setup(days,Rates) 
    simple_sizing(configs) 
    configs.finalize()
    analyses.finalize()     
    mission  = analyses.missions.base
    results  = mission.evaluate()
    filename = 'test_mission.pkl'
    save_results(results,filename)
    time1 = time.time()
    
    print('The total elapsed time: '+ str((time1-time0)/3600) + '  Hrs')
    res_2 = plot_mission(results,configs.base)
    filename_2 = 'results_summary.pkl'
    save_results(res_2,filename_2) 
    
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(days,Rates ):

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,days,Rates )
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

def vehicle_setup():

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

    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net = Battery_Propeller()
    net.dischage_model_fidelity = 2
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
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()  
    bat.cell.mass                   = 0.048 * Units.kg
    bat.cell.nominal_capacity       = 3.   # [Ah]
    bat.cell.nominal_voltage        = 3.6  # [V]
    bat.cell.max_mission_discharge  = 9.   # Amps 
    bat.cell.max_discharge_rate     = 15.  # Amps 
    bat.cell.specific_heat_capacity = 837.4 
    bat.specific_heat_capacity      = 837.4      
    # [J/kgK] "Numerical investigation on cooling performance of Li-ion battery thermal management system at
    #high galvanostatic discharge" and "A review of lithium-ion battery thermal management system strategies 
    #and the evaluate criteria"
              
    bat.cell.diameter               = 0.0018
    bat.cell.height                 = 0.06485
    bat.cell.surface_area           =(np.pi*bat.cell.height*bat.cell.diameter) # + (0.5*np.pi*bat.cell.diameter**2)   
    bat.module_config               = [128,40] #[series,parallel]       
    bat.cell.surface_area           = 4.18E-3
    bat.charging_voltage            = 3.6
    bat.charging_SOC_cutoff         = 1.    
    bat.charging_current            = 3 #Amps    
    initialize_from_circuit_configuration(bat)   

    bat.initial_max_energy          = bat.max_energy 
    net.battery                     = bat
    net.voltage                     = bat.max_voltage
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller  motor
    # Component 4 the Motor
    motor                              = SUAVE.Components.Energy.Converters.Motor()
    etam                               = 0.95
    v                                  = bat.max_voltage  
    omeg                               = prop.angular_velocity  
    io                                 = 4.0 
    start_kv                           = 1
    end_kv                             = 50
    # do optimization to find kv or just do a linspace then remove all negative values, take smallest one use 0.05 change
    # essentially you are sizing the motor for a particular rpm which is sized for a design tip mach 
    # this reduces the bookkeeping errors     
    possible_kv_vals                   = np.linspace(start_kv,end_kv,(end_kv-start_kv)*20 +1 , endpoint = True) * Units.rpm
    res_kv_vals                        = ((v-omeg/possible_kv_vals)*(1.-etam*v*possible_kv_vals/omeg))/io  
    positive_res_vals                  = np.extract(res_kv_vals > 0 ,res_kv_vals) 
    kv_idx                             = np.where(res_kv_vals == min(positive_res_vals))[0][0]   
    kv                                 = possible_kv_vals[kv_idx]  
    res                                = max(positive_res_vals) 

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
    base_config.propulsors.propulsor.pitch_command = 0 
    configs.append(base_config) 


    # done!
    return configs

# ----------------------------------------------------------------------
#   Sizing for the Vehicle Configs
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,vec_configs):
    label_size = 12
    legend_font_size = 14 
    line_width  = 3 
    axis_font        = {'size':'12'}  
    prop_radius_ft = vec_configs.propulsors.propulsor.propeller.tip_radius*3.28084 # convert to ft 
    module_config  = vec_configs.propulsors.propulsor.battery.module_config 
    
    # Final Energy
    maxcharge         = vec_configs.propulsors.propulsor.battery.max_energy
    energy_constraint = (results.segments[-1].conditions.propulsion.battery_energy[-1,0] - maxcharge*0.1)    

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    fig.set_size_inches(16, 8)

    for i in range(len(results.segments)): 

        time       = results.segments[i].conditions.frames.inertial.time[:,0]/3600 
        airspeed   = results.segments[i].conditions.freestream.velocity[:,0] 
        range_nm   = results.segments[i].conditions.frames.inertial.position_vector[:,0]  
        altitude   = results.segments[i].conditions.freestream.altitude[:,0]*3.28084  # convert to ft 

        axes = fig.add_subplot(2,2,1)
        axes.plot(time, altitude, 'b-', linewidth=line_width) 
        axes.set_ylabel('Altitude (ft)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
  
        axes = fig.add_subplot(2,2,2)
        axes.plot( time , range_nm*0.000539957 , 'b-', linewidth= line_width) 
        axes.set_ylabel('Range (nm)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')    
         
        axes = fig.add_subplot(2,2,3)
        axes.plot( time ,airspeed, 'b-', linewidth= line_width) 
        axes.set_ylabel('Airspeed (m/s)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
        axes.set_xlabel('Time (hr)', axis_font)
    # ------------------------------------------------------------------
    #   Aero Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Aero Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)): 

        time = results.segments[i].conditions.frames.inertial.time[:,0] /3600
        cl   = results.segments[i].conditions.aerodynamics.lift_coefficient[:,0,None] 
        cd   = results.segments[i].conditions.aerodynamics.drag_coefficient[:,0,None] 
        aoa  = results.segments[i].conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d  = cl/cd

        axes = fig.add_subplot(2,2,1)
        axes.plot( time , aoa , 'b-' , linewidth= line_width )
        axes.set_ylabel('Angle of Attack (deg)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , cl, 'b-' , linewidth= line_width )
        axes.set_ylabel('CL', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,2,3)
        axes.plot( time , cd, 'b-' , linewidth= line_width )
        axes.set_xlabel('Time (hr)', axis_font)
        axes.set_ylabel('CD', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,2,4)
        axes.plot( time , l_d, 'b-' , linewidth= line_width )
        axes.set_xlabel('Time (hr)', axis_font)
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
        time = results.segments[i].conditions.frames.inertial.time[:,0]/3600
        eta       = results.segments[i].conditions.propulsion.throttle[:,0] 
        SOC       = results.segments[i].conditions.propulsion.state_of_charge[:,0]
        energy    = results.segments[i].conditions.propulsion.battery_energy[:,0] 
        volts     = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc  = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]     
        current   = results.segments[i].conditions.propulsion.current[:,0]      
        battery_amp_hr  = (energy *0.000277778)/volts 
        C_rating   = current /battery_amp_hr 
        
        axes = fig.add_subplot(3,2,1) 
        axes.plot(time, SOC , 'b-', linewidth= line_width)         
        axes.set_ylabel('State of Charge', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
        
        axes = fig.add_subplot(3,2,2) 
        axes.plot(time, energy *0.000277778 , 'b-' , linewidth= line_width)        
        axes.set_ylabel('Battery Energy (W-hr)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(3,2,3) 
        axes.plot(time, volts   , 'b-',label='ULV' , linewidth = line_width) 
        axes.plot(time, volts_oc , color='darkred', linewidth = line_width , linestyle = '--' ,label='OCV')
        axes.set_ylabel('Battery Voltage (V)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,4)
        axes.plot(time, current , 'b-' ,linewidth= line_width)
        axes.set_ylabel('Current (Amp)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   

        axes = fig.add_subplot(3,2,5)
        axes.plot(time, C_rating , 'b-' , linewidth= line_width) 
        axes.set_ylabel('C-Rating (C)', axis_font)  
        axes.set_xlabel('Time (hr)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')    

        axes = fig.add_subplot(3,2,6)
        axes.plot(time, eta , 'b-' ,   linewidth= line_width)  
        axes.set_ylabel('Throttle ($\eta$)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        axes.set_xlabel('Time (hr)', axis_font)                  

    #plt.savefig("X57_Maxwell_Electronics.png")   
    
    
    # ------------------------------------------------------------------
    #   Cell Level Electronic Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Cell Level Electronic Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):  
        time      = results.segments[i].conditions.frames.inertial.time[:,0]/3600
        eta       = results.segments[i].conditions.propulsion.throttle[:,0] 
        SOC       = results.segments[i].conditions.propulsion.state_of_charge[:,0]
        energy    = results.segments[i].conditions.propulsion.battery_energy[:,0]/(module_config[0]*module_config[1])
        volts     = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc  = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]     
        current   = results.segments[i].conditions.propulsion.current[:,0]      
        battery_amp_hr  = (energy *0.000277778)/(volts/128) 
        C_rating   = (current/40)  /battery_amp_hr 
        
        axes = fig.add_subplot(3,2,1)
        axes.plot(time, SOC , 'b-', linewidth= line_width)        
        axes.set_ylabel('State of Charge', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(3,2,2)
        axes.plot(time, energy *0.000277778 , 'b-', linewidth= line_width)        
        axes.set_ylabel('Cell Energy (W-hr)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        
        axes = fig.add_subplot(3,2,3)
        axes.plot(time, volts/128   , 'b-' ,label='ULV', linewidth= line_width) 
        axes.plot(time, volts_oc/128 , color='darkred', linewidth = line_width , linestyle = '--',label='OCV')
        axes.set_ylabel('Cell Voltage (V)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')     
        if i == 0:
            axes.legend(loc='upper right')             

        axes = fig.add_subplot(3,2,4)
        axes.plot(time, current/40 , 'b-' , linewidth= line_width) 
        axes.set_ylabel('Current (Amp)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        

        axes = fig.add_subplot(3,2,5) 
        axes.plot(time, C_rating , 'b-', linewidth= line_width)
        axes.set_xlabel('Time (hr)', axis_font)
        axes.set_ylabel('C-Rating (C)', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(3,2,6)
        axes.plot(time, eta ,  'b-', linewidth= line_width)
        axes.set_ylabel('Throttle ($\eta$)', axis_font) 
        axes.set_xlabel('Time (hr)', axis_font)        
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Propulsor")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):  
        time   = results.segments[i].conditions.frames.inertial.time[:,0]/3600 
        rpm    = results.segments[i].conditions.propulsion.rpm [:,0] 
        thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,0]
        torque = results.segments[i].conditions.propulsion.motor_torque
        Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient
        effp   = results.segments[i].conditions.propulsion.etap[:,0]
        effm   = results.segments[i].conditions.propulsion.etam[:,0]
        prop_omega = results.segments[i].conditions.propulsion.rpm*0.104719755  
        ts = prop_omega*prop_radius_ft

        axes = fig.add_subplot(2,3,1)
        axes.plot(time, rpm, 'b-' , linewidth= line_width)
        axes.set_ylabel('RPM', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, thrust, 'b-' , linewidth= line_width)
        axes.set_ylabel('Thrust (N)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 

        axes = fig.add_subplot(2,3,3)
        axes.plot(time, torque, 'b-' , linewidth= line_width ) 
        axes.set_ylabel('Torque (N-m)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(2,3,4)
        axes.plot(time, effp, 'b-' , linewidth= line_width )
        axes.set_xlabel('Time (hr)', axis_font)
        axes.set_ylabel('Propeller Efficiency ($\eta_{prop}$)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')       
        plt.ylim((0,1))

        axes = fig.add_subplot(2,3,5)
        axes.plot(time, effm, 'b-' , linewidth= line_width )
        axes.set_xlabel('Time (hr)', axis_font)
        axes.set_ylabel('Motor Efficiency ($\eta_{motor}$)', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        plt.ylim((0,1))

        axes = fig.add_subplot(2,3,6)
        axes.plot(time, Cp, 'b-' , linewidth= line_width )
        axes.set_xlabel('Time (hr)', axis_font)
        axes.set_ylabel('Power Coefficient', axis_font)
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')     

    # ------------------------------------------------------------------
    #   Capacitance and Resisitive Growth vs Time in Days
    # ------------------------------------------------------------------
    fig = plt.figure("CRT")
    fig.set_size_inches(16, 8)
    Aging_time  = []
    C_Fade      = []
    R_Growth    = []
    for i in range(len(results.segments)):   
        Aging_time.append(results.segments[i].conditions.frames.inertial.time[-1,0]/3600)
        C_Fade.append(results.segments[i].conditions.propulsion.battery_capacity_fade_factor)  
        R_Growth.append(results.segments[i].conditions.propulsion.battery_resistance_growth_factor)  
    axes = fig.add_subplot(1,1,1)       
    axes.plot(Aging_time, C_Fade, 'ro' ,label = 'Capacity fade') 
    axes.plot(Aging_time, R_Growth, 'bo' , linewidth= line_width,label = 'Resistive growth')           
    axes.set_ylim([0.8,1.3]) 
    axes.set_ylabel(r'$C_{act}/C_{init}$   and   $R_{act}/R_{init}$', axis_font)    
    axes.set_xlabel('Time (hr)', axis_font)  
    axes.minorticks_on()
    axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
    axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')
    axes.legend(loc='upper right')    
 
    res_2             = Data()
    res_2.C_Fade      = C_Fade  
    res_2.R_Growth    = R_Growth 
    res_2.Aging_time  = Aging_time  
    
    # ------------------------------------------------------------------
    #   Temperature vs Time in Days
    # ------------------------------------------------------------------
    fig = plt.figure("Temperature")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)): 
        time = results.segments[i].conditions.frames.inertial.time[:,0]/3600
        temp = results.segments[i].conditions.propulsion.battery_cell_temperature[:,0]  
        axes = fig.add_subplot(1,1,1) 
        axes.plot(time, temp, 'bo' , linewidth= line_width )           
        axes.set_ylim([15,40]) 
        axes.set_ylabel(r'Temperature $\degree C$', axis_font)    
        axes.set_xlabel('Cycle', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        
    return res_2

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

def mission_setup(analyses,vehicle,days,Rates): 
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
    base_segment.state.numerics.number_control_points             = 20
    base_segment.process.iterate.initials.initialize_battery      = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery    
    base_segment.process.finalize.post_process.update_battery_age = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age           
    base_segment.process.iterate.conditions.planet_position       = SUAVE.Methods.skip   
    base_segment.state.unknowns.battery_thevenin_voltage          = 0.02   * ones_row(1)    
    base_segment.state.residuals.network                          = 0.    * ones_row(4)    
    bat = vehicle.propulsors.propulsor.battery 
    base_segment.charging_SOC_cutoff                              = bat.charging_SOC_cutoff  
    base_segment.charging_voltage                                 = bat.charging_voltage  * bat.pack_config.series
    base_segment.charging_current                                 = bat.charging_current  * bat.pack_config.parallel
    base_segment.max_energy                                       = bat.max_energy  
    
    # Calculate Flight Parameters 
    
    
   
    print('Climb Rate: ' + str(Rates[0]) + ' ft/min')
    print('Descent Rate: ' + str(Rates[1]) + ' ft/min')
    
    Cruise_Altitude     = 5500    * Units.feet
    Total_Range         = 80000 # distance from SFO to Santa Cruz
    Total_Time          = 1805.274 
    
    Climb_Speed         = 96.4260 * Units['mph'] 
    Climb_Rate          = Rates[0]* Units['ft/min']    
    Climb_Range_Speed   = np.sqrt(Climb_Speed**2 - Climb_Rate**2)
    Climb_Time          = Cruise_Altitude/Climb_Rate 
    Climb_Range         = Climb_Range_Speed * Climb_Time
    
    Descent_Speed       = 110.0 * Units['mph']   
    Descent_Rate        = Rates[1]* Units['ft/min']    
    Descent_Range_Speed = np.sqrt(Descent_Speed**2 - Descent_Rate**2)
    Descent_Time        = Cruise_Altitude/(Descent_Rate )
    Descent_Range       = Descent_Range_Speed * Descent_Time 
    
    Cruise_Distance     = Total_Range - (Climb_Range + Descent_Range)
    Cruise_Time         = Total_Time - (Climb_Time+Descent_Time )
    Cruise_Speed        = Cruise_Distance/Cruise_Time
    
    for day in range(days):
        # compute daily temperature in san francisco: link: https://www.usclimatedata.com/climate/san-francisco/california/united-states/usca0987/2019/1
        #daily_temp = 13.5 + (day)*(-0.00882) + (day**2)*(0.00221) + (day**3)*(-0.0000314) + (day**4)*(0.000000185)  + (day**5)*(-0.000000000483)  + (day**6)*(4.57E-13)
        
        # ------------------------------------------------------------------
        #   Climb 1 : constant Speed, constant rate segment 
        # ------------------------------------------------------------------ 
        segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
        segment_name = 'Climb Day ' + str (day)
        segment.tag = segment_name          
        segment.analyses.extend( analyses.base ) 
        segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linca
        segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linca 
        segment.process.finalize.post_process.update_battery_age = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age          
        segment.altitude_end                                     = Cruise_Altitude
        segment.air_speed                                        = Climb_Speed
        segment.climb_rate                                       = Climb_Rate  
        segment.state.unknowns.throttle                          = 0.9   * ones_row(1)      
        segment.state.unknowns.propeller_power_coefficient       = 0.12 * ones_row(1) 
        segment.state.unknowns.battery_state_of_charge           = 0.9   * ones_row(1)         
        segment.state.unknowns.battery_cell_temperature          = 20.5 * ones_row(1)  
        segment.battery_cell_temperature                         = 20   
        segment.ambient_temperature                              = 20   
        segment.battery_age_in_days                              = day   
        segment.battery_discharge                                = True
        if day == 0:
            segment.battery_cumulative_charge_throughput         = 0
            segment.altitude_start                               = 0.0  * Units.feet  
            segment.battery_energy                               = bat.max_energy  
            segment.battery_resistance_growth_factor             = 1 
            segment.battery_capacity_fade_factor                 = 1           
        
        # add to misison
        mission.append_segment(segment)
        
        # ------------------------------------------------------------------
        #   Cruise Segment: constant Speed, constant altitude
        # ------------------------------------------------------------------ 
        segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment_name = 'Cruise Day ' + str (day)
        segment.tag = segment_name       
        segment.analyses.extend(analyses.base)
        segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linca
        segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linca 
        segment.process.finalize.post_process.update_battery_age = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age 
        segment.air_speed                                        = Cruise_Speed
        segment.distance                                         = Cruise_Distance 
        segment.state.unknowns.throttle                          = 0.8 * ones_row(1)   
        segment.state.unknowns.battery_cell_temperature          = 30  * ones_row(1) 
        segment.state.unknowns.propeller_power_coefficient       = 0.1 * ones_row(1)  
        segment.state.unknowns.battery_state_of_charge           = 0.6 * ones_row(1)           
        segment.battery_discharge                                = True  
        segment.battery_age_in_days                              = day          
        
        # add to misison
        mission.append_segment(segment)    
        
        # ------------------------------------------------------------------
        #   Descent Segment: constant Speed, constant rate segment 
        # ------------------------------------------------------------------ 
        segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        segment_name = 'Descent Day ' + str (day) 
        segment.tag = segment_name  
        segment.analyses.extend( analyses.base )
        segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linca
        segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linca  
        segment.process.finalize.post_process.update_battery_age = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age    
        segment.altitude_end                                     = 0  * Units.feet
        segment.air_speed                                        = Descent_Speed
        segment.descent_rate                                     = Descent_Rate 
        segment.state.unknowns.throttle                          = 0.75 * ones_row(1)   
        segment.state.unknowns.battery_cell_temperature          = 35   * ones_row(1)  
        segment.state.unknowns.battery_state_of_charge           = 0.4  * ones_row(1)         
        segment.state.unknowns.propeller_power_coefficient       = 0.1 * ones_row(1) 
        segment.battery_discharge                                = True  
        segment.battery_age_in_days                              = day        
        
        # add to misison
        mission.append_segment(segment)
        
        # Thevenin Discharge Model 
        segment      = Segments.Ground.Battery_Charge_Discharge(base_segment) 
        segment_name = 'Charge Day ' + str (day)         
        segment.tag  = segment_name 
        segment.analyses.extend(analyses.base)     
        segment.process.iterate.unknowns.network                      = vehicle.propulsors.propulsor.unpack_unknowns_linca_charge
        segment.process.iterate.residuals.network                     = vehicle.propulsors.propulsor.residuals_linca_charge
        segment.process.finalize.post_process.update_battery_age      = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age           
        segment.state.unknowns.battery_state_of_charge                = 0.4  * ones_row(1)  
        segment.state.unknowns.battery_thevenin_voltage               = 0.1 * ones_row(1) 
        segment.state.unknowns.battery_cell_temperature               = 35 * ones_row(1)           
        segment.state.residuals.network                               = 0. * ones_row(3)    
        segment.battery_discharge                                     = False
        segment.battery_temperature                                   = 20
        segment.battery_age_in_days                                   = day  
        
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

def save_results(results,filename):

    # Store data (serialize)
    with open(filename, 'wb') as file:
        pickle.dump(results, file)
        
    return   

if __name__ == '__main__': 
    main()    
    #plt.show()