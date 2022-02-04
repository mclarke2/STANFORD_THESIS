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
from SUAVE.Plots.Mission_Plots import *
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Power.Battery.Sizing import  initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_mass , compute_optimal_motor_parameters
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():      
    num_days             = 5
    num_flights          = 4 
    num_segments         = 15
    num_segs_per_day     = num_flights*num_segments 
    Climb_Rates          = np.array([500])* Units['ft/min'] 
    Descent_Rates        = np.array([300])* Units['ft/min']    
    Climb_Speeds         = np.array([120]) * Units['mph']     
    Descent_Speeds       = np.array([110]) * Units['mph']  
    battery_fidelity     = 3 
    Nominal_Ranges       = np.array([55,60,65,70,75])* Units['miles']
    Nominal_Flight_Times = 29.3*(Nominal_Ranges/ Units['miles']) + 38.1
    Cruise_Altitude      = 2500 * Units.feet  
    unknowns            = np.array([0.85, 0.3, 0.6, 0.4, 0.5, 0.3])

    # start stopclock 
    time0        = time.time() 
    Aging_time   = np.zeros((len(Nominal_Ranges),len(Descent_Rates),num_days,num_segs_per_day))   
    Mission_time = np.zeros_like(Aging_time)
    E_Fade       = np.zeros_like(Aging_time)
    Charge       = np.zeros_like(Aging_time)
    R_Growth     = np.zeros_like(Aging_time)
    Max_Temp     = np.zeros_like(Aging_time)
    Avg_Temp     = np.zeros_like(Aging_time)
    Amb_Temp     = np.zeros_like(Aging_time)
    Max_C_rate   = np.zeros_like(Aging_time)
    Avg_C_rate   = np.zeros_like(Aging_time)
    Mission_SOC  = np.zeros_like(Aging_time)
    Heat_Gen     = np.zeros_like(Aging_time)
    Segment_Ah   = np.zeros_like(Aging_time)    
    
    run = 1
    for i in range(len(Nominal_Ranges)):
        for j in range(len(Descent_Rates)):
            time0  = time.time()
            print('Run: ' +  str(run))
                         
            configs, analyses = full_setup(num_days,num_flights,Climb_Rates[0],Descent_Rates[0] , Climb_Speeds[0]\
                                           , Descent_Speeds[0],Cruise_Altitude, Nominal_Ranges[i] , Nominal_Flight_Times[i] \
                                           ,battery_fidelity,unknowns)
            
            simple_sizing(configs) 
            configs.finalize()
            analyses.finalize()     
            mission = analyses.missions.base
            results = mission.evaluate() 
             
            for day in range(num_days): 
                for k in range(num_segs_per_day): 
                    # Obtain results from the end of the day 
                    R               = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_resistance_growth_factor
                    E               = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_capacity_fade_factor   
                    Q               = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_cumulative_charge_throughput[-1,0] 
                    t               = results.segments[day*num_segs_per_day + k].conditions.frames.inertial.time[-1,0]  
                    temp            = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_cell_temperature[:,0]
                    amb_temp        = results.segments[day*num_segs_per_day + k].ambient_temperature
                    seg_SOC         = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_state_of_charge[-1,0]  
                    energy          = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_energy[:,0] 
                    volts           = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_voltage_under_load[:,0]
                    current         = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_current[:,0]
                    heat_gen        = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_cell_heat_energy_generated[:,0]
                    seg_Ah          = results.segments[day*num_segs_per_day + k].conditions.propulsion.battery_charge_throughput[-1,0] 
                    
                    battery_amp_hr  = (energy / Units.Wh )/volts   
                    C_rating        = current /battery_amp_hr
                     
                    # Store Results in matrix 
                    Aging_time[i,j,day,k]         = day  
                    Mission_time[i,j,day,k]       = t
                    E_Fade[i,j,day,k]             = E
                    Charge[i,j,day,k]             = Q
                    R_Growth[i,j,day,k]           = R       
                    Max_Temp[i,j,day,k]           = max(temp)
                    Avg_Temp[i,j,day,k]           = np.mean(temp)
                    Amb_Temp[i,j,day,k]           = amb_temp
                    Max_C_rate[i,j,day,k]         = max(C_rating)
                    Avg_C_rate[i,j,day,k]         = np.mean(C_rating) 
                    Mission_SOC[i,j,day,k]        = seg_SOC
                    Heat_Gen[i,j,day,k]           = np.mean(heat_gen)
                    Segment_Ah[i,j,day,k]         = seg_Ah 
                                    
            time1 = time.time() 
            print('The total elapsed time of run ' + str(run) + ' : '+ str( round((time1-time0)/3600 , 3 )) + ' Hrs \n \n ') 
            run += 1
            
    # Compile Results 
    results                  = Data()
    results.num_cl_rates     = len(Climb_Rates)
    results.num_des_rates    = len(Descent_Rates)
    results.num_days         = num_days 
    results.num_flights      = num_flights
    results.num_segments     = num_segments 
    results.Mission_time     = Aging_time  
    results.E_Fade           = E_Fade      
    results.Charge           = Charge       
    results.R_Growth         = R_Growth     
    results.Max_Temp         = Max_Temp      
    results.Avg_Temp         = Avg_Temp     
    results.Amb_Temp         = Amb_Temp     
    results.Max_C_rate       = Max_C_rate   
    results.Avg_C_rate       = Avg_C_rate  
    results.SOC              = Mission_SOC
    results.Avg_Heat_Gen     = Heat_Gen
    results.Segment_Charge   = Segment_Ah
    
    # Store Results 
    filename = 'range_comparison_mission_' + str(num_days) + '_days.pkl'
    save_results(results,filename) 
    
    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(days,num_flights,Climb_Rate,Descent_Rate , Climb_Speed , Descent_Speed,\
                Cruise_Altitude, Nominal_Range , Nominal_Flight_Time ,battery_fidelity,unknowns):

    # vehicle data
    vehicle  = vehicle_setup(battery_fidelity,days)
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    # mission analyses
    mission  = mission_setup(configs_analyses, days,num_flights, vehicle,Climb_Rate, Descent_Rate , Climb_Speed,Descent_Speed,\
                              Cruise_Altitude, Nominal_Range , Nominal_Flight_Time ,battery_fidelity, unknowns) 

        
    
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

def vehicle_setup(battery_fidelity,days):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'X57_Maxwell' 

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff   = 3000. * Units.pounds
    vehicle.mass_properties.takeoff       = 3000. * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 3000. * Units.pounds
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

    fuselage.seats_abreast                      = 2. 
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.lengths.tail                       = 161. * Units.inches
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.total                      = 326. * Units.inches
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


    # add to vehicle
    vehicle.append_component(fuselage)

    #---------------------------------------------------------------------------------------------
    # DEFINE PROPELLER
    #---------------------------------------------------------------------------------------------
    # build network    
    net = Battery_Propeller() 
    net.number_of_engines       = 2.
    net.nacelle_diameter        = 14 * Units.inches
    net.engine_length           = 3  * Units.feet
    net.areas                   = Data() 
    net.areas.wetted            = net.engine_length*(2*np.pi*net.nacelle_diameter/2)  

    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 the Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller() 
    
    prop.number_of_blades    = 3
    prop.freestream_velocity = 135.*Units['mph']    
    prop.angular_velocity    = 1400.  * Units.rpm
    prop.tip_radius          = 2.75* Units.feet
    prop.hub_radius          = 0.4* Units.feet
    prop.design_Cl           = 0.8
    prop.design_altitude     = 8000. * Units.feet
    prop.design_altitude     = 8000. * Units.feet
    prop.design_thrust       = 1000.   
    prop.origin              = [[2.,2.5,0.784],[2.,-2.5,0.784]]  #  prop influence            
    prop.rotation            = [1,-1]
    prop                     = propeller_design(prop)    
    net.propeller            = prop    

    # Component 8 the Battery  
    bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()  
    bat.cell.max_mission_discharge     = 9.        # Amps  
    bat.cell.max_discharge_rate        = 15.       # Amps   
    bat.cell.surface_area              = (np.pi*bat.cell.height*bat.cell.diameter)  
    bat.pack_config.series             = 140   
    bat.pack_config.parallel           = 48     
    bat.module_config.normal_count     = 16     
    bat.module_config.parallel_count   = 30      
    bat.age_in_days                    = days  
    initialize_from_circuit_configuration(bat)       
    net.battery                        = bat
    net.voltage                        = bat.max_voltage
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------
    # Propeller  motor
    # Component 4 the Motor
    motor                      = SUAVE.Components.Energy.Converters.Motor()
    motor.mass_properties.mass = 10. * Units.kg
    motor.origin               = prop.origin  
    motor.efficiency           = 0.95
    motor.gear_ratio           = 1. 
    motor.gearbox_efficiency   = 1. # Gear box efficiency        
    motor.nominal_voltage      = bat.max_voltage *3/4  
    motor.propeller_radius     = prop.tip_radius  
    motor.no_load_current      = 4.0 
    motor                      = compute_optimal_motor_parameters(motor,prop) 
    net.motor                  = motor 
    
    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 5. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 5. #Watts  
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

def plot_mission(results): 
    
    line_width = 1
    lin_style  = 'bo-'
    axis_font = {'size':'14'}
    
    plot_aircraft_velocities(results)
    plot_aerodynamic_coefficients(results)
    plot_aerodynamic_forces(results)  
    plot_flight_conditions(results)     
    plot_propeller_conditions(results)
    plot_battery_pack_conditions(results) 
     

    # ------------------------------------------------------------------
    #   Capacitance and Resisitive Growth vs Time in Days
    # ------------------------------------------------------------------
    Aging_time  = []
    C_Fade      = []
    R_Growth    = []
    Charge      = []
    for i in range(len(results.segments)):
        if results.segments[i].battery_discharge == False: 
            Aging_time.append(results.segments[i].conditions.frames.inertial.time[-1,0] /Units.hr)
            C_Fade.append(results.segments[i].conditions.propulsion.battery_capacity_fade_factor) 
            Charge.append(results.segments[i].conditions.propulsion.battery_cumulative_charge_throughput[:,0][-1])
            R_Growth.append(results.segments[i].conditions.propulsion.battery_resistance_growth_factor)   
    res_2             = Data()
    res_2.C_Fade      = C_Fade  
    res_2.R_Growth    = R_Growth 
    res_2.Aging_time  = Aging_time 
    res_2.Charge      = Charge 
     
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
 

def mission_setup(analyses, days, num_flights, vehicle, Climb_Rate, Descent_Rate , Climb_Speed,\
                  Descent_Speed, Cruise_Altitude, Nominal_Range , Nominal_Flight_Time , battery_fidelity, ukns):
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
    mission.airport    = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments 

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.state.numerics.number_control_points             = 10 
    base_segment.use_Jacobian                                     = False
    base_segment.state.numerics.jacobian_evaluations              = 0 
    base_segment.state.numerics.iterations                        = 0      
    base_segment.state.reverse_thrust                             = False 
    base_segment.process.iterate.initials.initialize_battery      = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery    
    base_segment.process.finalize.post_process.update_battery_age = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age         
    base_segment.process.iterate.conditions.planet_position       = SUAVE.Methods.skip       
    base_segment.state.residuals.network                          = 0.    * ones_row(4) 
    bat                                                           = vehicle.propulsors.propulsor.battery  
    base_segment.initial_mission_battery_energy                   = bat.initial_max_energy    
    base_segment.charging_SOC_cutoff                              = bat.cell.charging_SOC_cutoff  
    base_segment.charging_voltage                                 = bat.cell.charging_voltage  * bat.pack_config.series
    base_segment.charging_current                                 = bat.cell.charging_current  * bat.pack_config.parallel
    base_segment.battery_configuration                            = bat.pack_config
    base_segment.max_energy                                       = bat.max_energy  
    
    
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
    Downleg_Distance        = 6,000 * Units.feet # length or runway
    Downleg_Altitude        = 1000 * Units.feet
    Res_Cruise_Altitude     = 1500 * Units.feet  
    Res_Climb_Altitude      = Res_Cruise_Altitude - Downleg_Altitude 
    Res_Descent_Altitude    = Res_Cruise_Altitude - Downleg_Altitude
    Res_Nominal_Range       = 0.1 * Nominal_Range  
     
    Res_Climb_Range_Speed   = np.sqrt(Climb_Speed**2 - Climb_Rate**2)
    Res_Climb_Time          = Res_Climb_Altitude/Climb_Rate 
    Res_Climb_Range         = Res_Climb_Range_Speed * Res_Climb_Time 
    Res_Descent_Range_Speed = np.sqrt(Descent_Speed**2 - Descent_Rate**2)
    Res_Descent_Time        = Res_Descent_Altitude/(Descent_Rate )
    Res_Descent_Range       = Res_Descent_Range_Speed * Res_Descent_Time  
    Res_Cruise_Distance     = Res_Nominal_Range - (Res_Climb_Range + Res_Descent_Range) 
    Res_Cruise_Speed        = Cruise_Speed 
                        
    # Calculate Flight Pameters 
    print('\n\n Climb Rate: ' + str(round(Climb_Rate / Units['ft/min'],4)) + ' ft/min')
    print('Descent Rate: ' + str(round(Descent_Rate / Units['ft/min'],4)) + ' ft/min \n') 
    print('Cruise Speed : ' + str(round(Cruise_Speed,4)) + ' m/s')
    print('Cruise Distance : ' + str(round(Cruise_Distance,4))+ ' m \n ')
    print('Reserve Cruise Speed : ' + str(round(Res_Cruise_Speed,4)) + ' m/s')
    print('Reserve Cruise Distance : ' + str(round(Res_Cruise_Distance,4))+ ' m \n \n')
    

    # Unpack 
    cl_throt_ukn  = ukns[0]
    cl_CP_ukn     = ukns[1]
    cr_throt_ukn  = ukns[2]
    cr_CP_ukn     = ukns[3]
    des_throt_ukn = ukns[4]
    des_CP_ukn    = ukns[5]     
         
    for day in range(days):
        # compute daily temperature in san francisco: link: https://www.usclimatedata.com/climate/san-francisco/california/united-states/usca0987/2019/1
        # NY
        daily_temp = 6 -0.191*day + 4.59E-3*(day**2) - 2.02E-5*(day**3) + 2.48E-8*(day**4)
        
        # LA 
        daily_temp = 18.2 - 9.7E-3*(day) + 2.41E-4*(day**2) -7.74E-6*(day**3) \
                    + 1.38E-7*(day**4) - 1.01E-9*(day**5) + 3.67E-12*(day**6) \
                    - 6.78E-15*(day**7) + 5.1E-18*(day**8)
        
        # HOU 
        daily_temp = 17.1 - 0.0435*(day) + 2.77E-3*(day**2) -2.2E-5*(day**3) \
                    + 9.72E-8*(day**4) - 2.53E-10*(day**5) + 2.67E-13*(day**6)  
        
        # SF 
        daily_temp = 13.5 + (day)*(-0.00882) + (day**2)*(0.00221) + \
                    (day**3)*(-0.0000314) + (day**4)*(0.000000185)  +\
                    (day**5)*(-0.000000000483)  + (day**6)*(4.57E-13)
        
        # CHI
        daily_temp = -0.145 + (day)*(-0.11) + (day**2)*(4) + \
                    (day**3)*(-2.71E-5) + (day**4)*(7.92E-8)  +\
                    (day**5)*(-1.66E-10)  + (day**6)*(1.76E-13)
        
        daily_temp = 0
        for nf in range(num_flights):
            
            # Thevenin Discharge Model 
            
            # Thevenin Discharge Model 
            if battery_fidelity == 3:        
                # ------------------------------------------------------------------
                #   Takeoff Segment  Flight 1   : 
                # ------------------------------------------------------------------             
                segment = Segments.Ground.Takeoff(base_segment)
                segment_name = 'Takeoff Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base )  
                segment.state.numerics.number_control_points             = 40 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = 1.0 * ones_row(1)   
                segment.state.unknowns.propeller_power_coefficient       = 0.15 * ones_row(1)   
                segment.state.unknowns.battery_state_of_charge           = 0.95  * ones_row(1) 
                segment.state.unknowns.battery_current                   = 700 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp + 1) * ones_row(1)  
                segment.velocity_start                                   = Vstall*0.5
                segment.velocity_end                                     = Vstall
                segment.friction_coefficient                             = 0.3
                segment.time                                             = 35.0
                segment.battery_thevenin_voltage                         = 0 
                segment.ambient_temperature                              = daily_temp  
                segment.battery_age_in_days                              = day   
                segment.battery_discharge                                = True 
                segment.battery_cell_temperature                         = daily_temp + 1
                segment.battery_pack_temperature                         = daily_temp + 1                   
                if day == 0: 
                    if nf == 0:                         
                        segment.battery_resistance_growth_factor             = 1 
                        segment.battery_capacity_fade_factor                 = 1    
                        segment.battery_energy                               = bat.initial_max_energy    
                        segment.battery_cumulative_charge_throughput         = 0                          
                    
                mission.append_segment(segment)
                
                
                # ------------------------------------------------------------------
                #   Departure End of Runway Segment Flight 1 : 
                # ------------------------------------------------------------------ 
                segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment_name = 'DER Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = 1.0 * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = 0.2  * ones_row(1)  
                segment.state.unknowns.battery_state_of_charge           = 0.95 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)
                segment.altitude_start                                   = 0.0 * Units.feet
                segment.altitude_end                                     = 50.0 * Units.feet
                segment.air_speed_start                                  = Vstall  
                segment.air_speed_end                                    = 45      
                segment.ambient_temperature                              = daily_temp                  
                segment.climb_rate                                       = 700 * Units['ft/min']
                segment.battery_age_in_days                              = day   
                segment.battery_discharge                                = True                
                mission.append_segment(segment)
                
                # ------------------------------------------------------------------
                #   Initial Climb Area Segment Flight 1  
                # ------------------------------------------------------------------ 
                segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment_name = 'ICA_AltitudeDay ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = cl_throt_ukn * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = cl_CP_ukn  * ones_row(1)  
                segment.state.unknowns.battery_state_of_charge           = 0.8 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)
                segment.altitude_start                                   = 50.0 * Units.feet
                segment.altitude_end                                     = 500.0 * Units.feet
                segment.air_speed_start                                  = 45  
                segment.air_speed_end                                    = Climb_Speed           
                segment.climb_rate                                       = 600 * Units['ft/min']
                segment.battery_age_in_days                              = day   
                segment.ambient_temperature                              = daily_temp                  
                segment.battery_discharge                                = True                
                mission.append_segment(segment)
             
                         
                # ------------------------------------------------------------------
                #   Climb Segment Flight 1 
                # ------------------------------------------------------------------ 
                segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
                segment_name = 'Climb Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = cl_throt_ukn * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = cl_CP_ukn  * ones_row(1)  
                segment.state.unknowns.battery_state_of_charge           = 0.8 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 200 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)
                segment.altitude_start                                   = 500.0 * Units.feet
                segment.altitude_end                                     = Cruise_Altitude
                segment.air_speed                                        = Climb_Speed
                segment.climb_rate                                       = Climb_Rate   
                segment.ambient_temperature                              = daily_temp                  
                segment.battery_age_in_days                              = day   
                segment.battery_discharge                                = True                
                mission.append_segment(segment)
                
                # ------------------------------------------------------------------
                #   Cruise Segment Flight 1 
                # ------------------------------------------------------------------ 
                segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
                segment_name = 'Cruise Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name       
                segment.analyses.extend(analyses.base)
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco   
                segment.air_speed                                        = Cruise_Speed
                segment.distance                                         = Cruise_Distance 
                segment.state.unknowns.throttle                          = cr_throt_ukn * ones_row(1)    
                segment.state.unknowns.propeller_power_coefficient       = cr_CP_ukn * ones_row(1) 
                segment.state.unknowns.battery_state_of_charge           = 0.6 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 200  * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp ) * ones_row(1)               
                segment.battery_discharge                                = True   
                segment.ambient_temperature                              = daily_temp                  
                segment.battery_age_in_days                              = day         
                mission.append_segment(segment)    
                
                # ------------------------------------------------------------------
                #   Descent Segment Flight 1   
                # ------------------------------------------------------------------ 
                segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
                segment_name = 'Descent Day ' + str (day) + ' Flight ' + str(nf+1)
                segment.tag = segment_name  
                segment.analyses.extend( analyses.base )
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco      
                segment.altitude_end                                     = Downleg_Altitude
                segment.air_speed                                        = Descent_Speed
                segment.descent_rate                                     = Descent_Rate 
                segment.state.unknowns.throttle                          = des_throt_ukn  * ones_row(1)  
                segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn  * ones_row(1) 
                segment.state.unknowns.battery_state_of_charge           = 0.5 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 200  * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)  	            
                segment.battery_discharge                                = True  
                segment.battery_age_in_days                              = day    
                segment.ambient_temperature                              = daily_temp                  
                mission.append_segment(segment)  
            
                # ------------------------------------------------------------------
                #  Downleg_Altitude Segment Flight 1 
                # ------------------------------------------------------------------ 
                segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
                segment_name = 'Downleg_Altitude Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name       
                segment.analyses.extend(analyses.base)
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco  
                segment.acceleration                                     = -0.05307 * Units['m/s/s']
                segment.air_speed_start                                  = Descent_Speed * Units['m/s']
                segment.air_speed_end                                    = 45.0 * Units['m/s']            
                segment.distance                                         = Downleg_Distance
                segment.state.unknowns.throttle                          = 0.5 * ones_row(1)    
                segment.state.unknowns.propeller_power_coefficient       = 0.3 * ones_row(1) 
                segment.state.unknowns.battery_state_of_charge           = 0.4 * ones_row(1)  
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp ) * ones_row(1)               
                segment.battery_discharge                                = True  
                segment.battery_age_in_days                              = day   
                segment.ambient_temperature                              = daily_temp                  
                mission.append_segment(segment)     
                
                # ------------------------------------------------------------------
                #   Reserve Climb Segment Flight 1 
                # ------------------------------------------------------------------ 
                segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
                segment_name = 'Reserve Reserve Climb Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = cl_throt_ukn * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = cl_CP_ukn  * ones_row(1)  
                segment.state.unknowns.battery_state_of_charge           = 0.4 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)
                segment.altitude_start                                   = Downleg_Altitude
                segment.altitude_end                                     = Res_Cruise_Altitude
                segment.air_speed                                        = Climb_Speed
                segment.climb_rate                                       = Climb_Rate  
                segment.battery_age_in_days                              = day    
                segment.ambient_temperature                              = daily_temp                  
                segment.battery_discharge                                = True                
                mission.append_segment(segment)
                
                # ------------------------------------------------------------------
                #   Reserve Cruise Segment Flight 1 
                # ------------------------------------------------------------------ 
                segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
                segment_name = 'Reserve Cruise Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name       
                segment.analyses.extend(analyses.base)
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco   
                segment.air_speed                                        = Res_Cruise_Speed
                segment.distance                                         = Res_Cruise_Distance
                segment.state.unknowns.throttle                          = cr_throt_ukn * ones_row(1)    
                segment.state.unknowns.propeller_power_coefficient       = cr_CP_ukn * ones_row(1) 
                segment.state.unknowns.battery_state_of_charge           = 0.4 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp ) * ones_row(1)               
                segment.battery_discharge                                = True   
                segment.ambient_temperature                              = daily_temp                  
                segment.battery_age_in_days                              = day         
                mission.append_segment(segment)    
                
                # ------------------------------------------------------------------
                #    Reserve Descent Segment Flight 1  
                # ------------------------------------------------------------------ 
                segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
                segment_name = 'Reserve Descent Day ' + str (day) + ' Flight ' + str(nf+1)
                segment.tag = segment_name  
                segment.analyses.extend( analyses.base )
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco      
                segment.altitude_end                                     = Downleg_Altitude
                segment.air_speed                                        = Descent_Speed
                segment.descent_rate                                     = Descent_Rate 
                segment.state.unknowns.throttle                          = des_throt_ukn  * ones_row(1)  
                segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn  * ones_row(1) 
                segment.state.unknowns.battery_state_of_charge           = 0.4 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)  	            
                segment.battery_discharge                                = True  
                segment.ambient_temperature                              = daily_temp  
                segment.battery_age_in_days                              = day   
                mission.append_segment(segment)  
            
                # ------------------------------------------------------------------
                #  Baseleg Segment Flight 1  
                # ------------------------------------------------------------------ 
                segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment_name = 'Baseleg Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco
                segment.state.unknowns.throttle                          = des_throt_ukn * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn  * ones_row(1)  
                segment.state.unknowns.battery_state_of_charge           = 0.3 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)
                segment.altitude_start                                   = Downleg_Altitude
                segment.altitude_end                                     = 500.0 * Units.feet
                segment.air_speed_start                                  = 45 
                segment.air_speed_end                                    = 40    
                segment.climb_rate                                       = -350 * Units['ft/min']
                segment.battery_age_in_days                              = day  
                segment.ambient_temperature                              = daily_temp   
                segment.battery_discharge                                = True                
                mission.append_segment(segment) 
            
                # ------------------------------------------------------------------
                #  Final Approach Segment Flight 1  
                # ------------------------------------------------------------------ 
                segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
                segment_name = 'Final Approach Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = des_throt_ukn * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn * ones_row(1)  
                segment.state.unknowns.battery_state_of_charge           = 0.4 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1)
                segment.altitude_start                                   = 500.0 * Units.feet
                segment.altitude_end                                     = 00.0 * Units.feet
                segment.air_speed_start                                  = 40 
                segment.air_speed_end                                    = 35   
                segment.climb_rate                                       = -300 * Units['ft/min']
                segment.battery_age_in_days                              = day   
                segment.ambient_temperature                              = daily_temp  
                segment.battery_discharge                                = True                
                mission.append_segment(segment)  
            
                # ------------------------------------------------------------------
                #   Landing Segment Flight 1  
                # ------------------------------------------------------------------             
                segment = Segments.Ground.Landing(base_segment)
                segment_name = 'Landing Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = des_throt_ukn * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = des_CP_ukn  * ones_row(1)    
                segment.state.unknowns.battery_state_of_charge           = 0.3 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 550 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1) 
                segment.velocity_start                                   = Vstall
                segment.velocity_end                                     = Vstall 
                segment.friction_coefficient                             = 0.3
                segment.throttle                                         = 0.1
                segment.time                                             = 5.0 
                segment.battery_age_in_days                              = day   
                segment.ambient_temperature                              = daily_temp  
                segment.battery_discharge                                = True 
                mission.append_segment(segment)
                
                # ------------------------------------------------------------------
                #   Reverse Thrust Flight 1   
                # ------------------------------------------------------------------             
                segment = Segments.Ground.Landing(base_segment)
                segment_name = 'Reverse Thrust Day ' + str (day)+ ' Flight ' + str(nf+1)
                segment.tag = segment_name          
                segment.analyses.extend( analyses.base ) 
                segment.process.iterate.unknowns.network                 = vehicle.propulsors.propulsor.unpack_unknowns_linmco
                segment.process.iterate.residuals.network                = vehicle.propulsors.propulsor.residuals_linmco 
                segment.state.unknowns.throttle                          = 1.5 * ones_row(1)
                segment.state.unknowns.propeller_power_coefficient       = 0.1* ones_row(1)    
                segment.state.unknowns.battery_state_of_charge           = 0.3 * ones_row(1) 
                segment.state.unknowns.battery_current                   = 200 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature          = (daily_temp) * ones_row(1) 
                segment.velocity_start                                   = Vstall
                segment.velocity_end                                     = Vstall*0.5
                segment.state.reverse_thrust                             = True             
                segment.friction_coefficient                             = 0.3
                segment.throttle                                         = 0.8
                segment.time                                             = 40.0 
                segment.battery_age_in_days                              = day   
                segment.ambient_temperature                              = daily_temp  
                segment.battery_discharge                                = True 
                mission.append_segment(segment)   
            
                # ------------------------------------------------------------------
                #  Battery Charge 1
                # ------------------------------------------------------------------ 
                segment      = Segments.Ground.Battery_Charge_Discharge(base_segment) 
                segment_name = 'Charge Day ' + str (day)+ ' Flight ' + str(nf+1)          
                segment.tag  = segment_name 
                segment.analyses.extend(analyses.base)     
                segment.process.iterate.unknowns.network           = vehicle.propulsors.propulsor.unpack_unknowns_linmco_charge
                segment.process.iterate.residuals.network          = vehicle.propulsors.propulsor.residuals_linmco_charge  
                segment.state.unknowns.battery_state_of_charge     = 0.5 * ones_row(1) 
                segment.state.unknowns.battery_current             = 200 * ones_row(1)  
                segment.state.unknowns.battery_cell_temperature    = (daily_temp) * ones_row(1)  
                segment.state.residuals.network                    = 0. * ones_row(3)    
                segment.battery_discharge                          = False
                segment.battery_age_in_days                        = day 
                segment.ambient_temperature                        = daily_temp                  
                segment.battery_cell_temperature                   = daily_temp + 1
                segment.battery_pack_temperature                   = daily_temp + 1 
                mission.append_segment(segment)        
                
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
    plt.show()
