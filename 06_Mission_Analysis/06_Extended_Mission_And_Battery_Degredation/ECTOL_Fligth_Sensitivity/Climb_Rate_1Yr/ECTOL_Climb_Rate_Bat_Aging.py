# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Units 
import numpy as np   
import matplotlib.pyplot        as plt  
import pickle 
import time 
from SUAVE.Core                                                  import Data 
from SUAVE.Plots.Performance.Mission_Plots                       import *  
from SUAVE.Plots.Geometry                                        import *   

import sys 
sys.path.append('../../../../XX_Supplementary/Aircraft_Models_and_Simulations')  
from ECTOL_2P import  missions_setup , base_analysis , vehicle_setup , analyses_setup ,  configs_setup , missions_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    time0            = time.time()   
    simulated_days   = 8
    flights_per_day  = 365 
    aircraft_range   = 70 *Units.nmi
    flight_segments  = 10
    reserve_segment  = False  
    recharge_battery = False 
    control_points   = 10
    N_gm_x           = 1
    N_gm_y           = 1 
    run_noise_model  = False  
    
    ############### 
    run_no = 1
    ###############
    
    if run_no == 1: 
        climb_rates       = np.array([200])* Units['ft/min']  
        cruise_ranges     = np.array([[19.123,25.17,28.79,31.21,32.93]])*Units.nmi 
    if run_no == 2: 
        climb_rates       = np.array([300])* Units['ft/min']  
        cruise_ranges     = np.array([[26.53,32.57,36.2,38.62,40.34]])*Units.nmi
    if run_no == 3: 
        climb_rates       = np.array([400])* Units['ft/min'] 
        cruise_ranges     = np.array([[30.24,36.28,39.9,42.32,44.05]])*Units.nmi 
    if run_no == 4: 
        climb_rates       = np.array([500])* Units['ft/min']  
        cruise_ranges     = np.array([[32.46,38.5,42.13,44.54,46.27]])*Units.nmi
    if run_no == 5: 
        climb_rates       = np.array([600])* Units['ft/min']  
        cruise_ranges     = np.array([[33.95,39.99,43.61,46.03,47.75]])*Units.nmi 
        
    descent_rates     = np.array([150,200,250,300,350])* Units['ft/min']   
    
    for i in range(len(climb_rates)):
        for j in range(len(descent_rates)):
            min_y             = 1E-1
            max_y             = 0.25*Units.nmi
            min_x             = 1E-1
            max_x             = 70*Units.nmi 
            vehicle           = vehicle_setup() 
            configs           = configs_setup(vehicle) 
            configs_analyses  = analyses_setup(configs,N_gm_x,N_gm_y,min_y,max_y,min_x,max_x,aircraft_range,run_noise_model) 
            base_mission      = full_mission_setup(configs_analyses,vehicle,simulated_days,flights_per_day,cruise_ranges[i,j],
                                                   reserve_segment,control_points,recharge_battery,climb_rates[i],descent_rates[j])
            missions_analyses = missions_setup(base_mission) 
            analyses          = SUAVE.Analyses.Analysis.Container()
            analyses.configs  = configs_analyses
            analyses.missions = missions_analyses 
            configs.finalize()
            analyses.finalize()      
            mission           = analyses.missions.base
            results           = mission.evaluate()    
         
            
            elapsed_range     =  results.segments[-1].conditions.frames.inertial.position_vector[-1,0] 
            print('Range : ' + str(round(elapsed_range,2)) + ' m  or ' + str(round(elapsed_range/Units.nmi,2)) + ' nmi')   
            time1 = time.time() 
            print('The total elapsed time: ' + str(round((time1-time0)/3600 , 3 )) + ' Hrs')   
            
            results_prossessed = process_results(results,simulated_days,flights_per_day,flight_segments) 
            filename           = 'ECTOL_Climb_Rate_1Yr' + '_CR'+ str(int(np.ceil(climb_rates[i]/Units['ft/min']))) + '_DR'+ str(int(np.ceil(descent_rates[j]/Units['ft/min'])))
            save_results(results_prossessed,filename)  
    
    return
  
 
# ----------------------------------------------------------------------
#   Full Mission Setup
# ---------------------------------------------------------------------- 
def full_mission_setup(analyses,vehicle,simulated_days,flights_per_day,cruise_range,reserve_segment,
                       control_points,recharge_battery,climb_rate,descent_rate):   
     
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
    airport            = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  100. * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976() 
    mission.airport    = airport     
    
    atmosphere    = SUAVE.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data     = atmosphere.compute_values(altitude = 0,temperature_deviation= 0.)    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments 
    
    # base segment
    base_segment                                             = Segments.Segment()
    ones_row                                                 = base_segment.state.ones_row
    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery 
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = control_points
    bat                                                      = vehicle.networks.battery_propeller.battery
    base_segment.charging_SOC_cutoff                         = bat.cell.charging_SOC_cutoff 
    base_segment.charging_current                            = bat.charging_current
    base_segment.charging_voltage                            = bat.charging_voltage 
    base_segment.battery_discharge                           = True   
    
    for day in range(simulated_days):
        
        # compute daily temperature in san francisco: link: https://www.usclimatedata.com/climate/san-francisco/california/united-states/usca0987/2019/1
        daily_temp = (13.5 + (day)*(-0.00882) + (day**2)*(0.00221) + (day**3)*(-0.0000314) + (day**4)*(0.000000185)  + \
                      (day**5)*(-0.000000000483)  + (day**6)*(4.57E-13)) + 273.2
        
        base_segment.temperature_deviation = daily_temp - atmo_data.temperature[0][0]
        
        print(' ***********  Day ' + str(day) + ' ***********  ')
        for f_idx in range(flights_per_day): 
            flight_no = f_idx + 1     
            # ------------------------------------------------------------------
            #   Takeoff Roll
            # ------------------------------------------------------------------
        
            segment = Segments.Ground.Takeoff(base_segment)
            segment.tag = "Takeoff"  + "_F_" + str(flight_no) + "_D_" + str (day)  
            segment.analyses.extend( analyses.base )
            segment.velocity_start            = Vstall*0.1  
            segment.velocity_end              = Vstall  
            segment.friction_coefficient      = 0.04 
            segment.state.unknowns.time       = 10.            
            segment.altitude                  = 0.0 
            segment.state.unknowns.velocity_x = 0.5* Vstall * ones_row(1)  
            if (day == 0) and (f_idx == 0):        
                segment.battery_energy                               = vehicle.networks.battery_propeller.battery.max_energy   
                segment.initial_battery_resistance_growth_factor     = 1
                segment.initial_battery_capacity_fade_factor         = 1
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.005)          
            # add to misison
            mission.append_segment(segment) 
             
            # ------------------------------------------------------------------
            #   Departure End of Runway Segment Flight 1 : 
            # ------------------------------------------------------------------ 
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            segment.tag = 'Departure_End_of_Runway'   + "_F_" + str(flight_no) + "_D_" + str (day)     
            segment.analyses.extend( analyses.base )           
            segment.altitude_start                                   = 0.0 * Units.feet
            segment.altitude_end                                     = 50.0 * Units.feet
            segment.air_speed_start                                  = Vstall  
            segment.air_speed_end                                    = Vstall*1.1        
            segment.climb_rate                                       = 600 * Units['ft/min']  
            segment.state.unknowns.throttle                          = 0.9 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.005)  
            mission.append_segment(segment)  
                        
            # ------------------------------------------------------------------
            #   Initial Climb Area Segment Flight 1  
            # ------------------------------------------------------------------ 
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            segment.tag = 'Initial_CLimb_Area'  + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.base )   
            segment.altitude_start                                   = 50.0 * Units.feet
            segment.altitude_end                                     = 500.0 * Units.feet
            segment.air_speed_start                                  = Vstall*1.1     
            segment.air_speed_end                                    = Vstall*1.2  
            segment.climb_rate                                       = climb_rate
            segment.state.unknowns.throttle                          = 0.85 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.05)  
            mission.append_segment(segment) 
                    
            # ------------------------------------------------------------------
            #   Climb Segment Flight 1 
            # ------------------------------------------------------------------ 
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            segment.tag = 'Climb'   + "_F_" + str(flight_no) + "_D" + str (day)      
            segment.analyses.extend( analyses.base )         
            segment.altitude_start                                   = 500.0 * Units.feet
            segment.altitude_end                                     = 2500 * Units.feet 
            segment.air_speed_start                                  = Vstall*1.2  
            segment.air_speed_end                                    = 175.* Units['mph']    
            segment.climb_rate                                       = climb_rate  
            segment.state.unknowns.throttle                          = 0.85 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.005)  
            mission.append_segment(segment)  
            
            # ------------------------------------------------------------------
            #   Cruise Segment Flight 1 
            # ------------------------------------------------------------------ 
            segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
            segment.tag = 'Cruise'  + "_F_" + str(flight_no) + "_D" + str (day) 
            segment.analyses.extend(analyses.base) 
            segment.air_speed                                        = 175.* Units['mph']    
            segment.distance                                         = cruise_range
            segment.state.unknowns.throttle                          = 0.8 * ones_row(1)              
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.005)  
            mission.append_segment(segment)  
            
            # ------------------------------------------------------------------
            #   Descent Segment Flight 1   
            # ------------------------------------------------------------------ 
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
            segment.tag = 'Descent'   + "_F_" + str(flight_no) + "_D" + str (day)      
            segment.analyses.extend( analyses.base )       
            segment.altitude_start                                   = 2500 * Units.feet  
            segment.altitude_end                                     = 1000.0 * Units.feet
            segment.air_speed_start                                  = 175.* Units['mph']    
            segment.air_speed_end                                    = Vstall*1.3
            segment.climb_rate                                       = -descent_rate 
            segment.state.unknowns.throttle                          = 0.8 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.1)  
            mission.append_segment(segment)  
             
        
            # ------------------------------------------------------------------
            #  Downleg_Altitude Segment Flight 1 
            # ------------------------------------------------------------------ 
            segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment) 
            segment.tag = 'Downleg' + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend(analyses.base) 
            segment.air_speed_start                                  = Vstall*1.3
            segment.air_speed_end                                    = Vstall*1.2             
            segment.distance                                         =  6000 * Units.feet
            segment.acceleration                                     = -0.05 * Units['m/s/s']   
            segment.state.unknowns.throttle                          = 0.7 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.01)  
            mission.append_segment(segment)        
            
            # ------------------------------------------------------------------
            #  Baseleg Segment Flight 1  
            # ------------------------------------------------------------------ 
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment.tag = 'Baseleg' + "_F_" + str(flight_no) + "_D" + str (day)
            segment.analyses.extend( analyses.base)   
            segment.altitude_start                                   = 1000 * Units.feet
            segment.altitude_end                                     = 500.0 * Units.feet
            segment.air_speed_start                                  = Vstall*1.2  
            segment.air_speed_end                                    = Vstall*1.1  
            segment.climb_rate                                       = -descent_rate  
            segment.state.unknowns.throttle                          =  0.7 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.3  )  
            mission.append_segment(segment)   
        
            # ------------------------------------------------------------------
            #  Final Approach Segment Flight 1  
            # ------------------------------------------------------------------ 
            segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
            segment_name = 'Final_Approach' + "_F_" + str(flight_no) + "_D" + str (day)
            segment.tag = segment_name          
            segment.analyses.extend( analyses.base)     
            segment.state.unknowns.throttle                          = 0.8 * ones_row(1)
            segment.state.unknowns.propeller_power_coefficient       = 0.3 *  ones_row(1)      
            segment.altitude_start                                   = 500.0 * Units.feet
            segment.altitude_end                                     = 00.0 * Units.feet
            segment.air_speed_start                                  = Vstall*1.1  
            segment.air_speed_end                                    = Vstall 
            segment.climb_rate                                       = -300 * Units['ft/min'] 
            segment.state.unknowns.throttle                          =  0.8 * ones_row(1)  
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.3  )  
            mission.append_segment(segment)   
            
        
            # ------------------------------------------------------------------
            #   Landing  
            # ------------------------------------------------------------------ 
            segment = Segments.Ground.Landing(base_segment)
            segment.tag = "Landing"  + "_F_" + str(flight_no) + "_D_" + str (day)  
            segment.analyses.extend( analyses.base)
            segment.velocity_start            = Vstall  
            segment.velocity_end              = Vstall*0.1  
            segment.friction_coefficient      = 0.04 
            segment.state.unknowns.time       = 30.            
            segment.altitude                  = 0.0 
            segment.state.unknowns.velocity_x = 0.1* Vstall * ones_row(1)   
            segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,  initial_power_coefficient = 0.005)          
            # add to misison
            mission.append_segment(segment)   

                        
            if recharge_battery:
                # ------------------------------------------------------------------
                #  Charge Segment: 
                # ------------------------------------------------------------------     
                # Charge Model 
                segment                                             = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                segment.tag  = 'Charge_Day' + "_F_" + str(flight_no) + "_D" + str (day)  
                segment.analyses.extend(analyses.base)           
                segment.battery_discharge                                = False     
                if flight_no  == flights_per_day:  
                    segment.increment_battery_cycle_day=True            
                segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment)    
                mission.append_segment(segment)   
    
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------ 
    
    return mission
 
def process_results(results,num_simulated_days,num_flights_per_day,num_flight_segments):  
    
    num_segs_per_day  = num_flights_per_day*num_flight_segments
  
    
    Max_Temp           = np.zeros((num_simulated_days,num_flight_segments))  
    Aging_time         = np.zeros_like(Max_Temp)
    Mission_time       = np.zeros_like(Max_Temp)
    Avg_Temp           = np.zeros_like(Max_Temp)
    Amb_Temp           = np.zeros_like(Max_Temp)
    Max_C_rate         = np.zeros_like(Max_Temp)
    Bat_Final_Energy   = np.zeros_like(Max_Temp)
    Bat_Start_Energy   = np.zeros_like(Max_Temp)
    Avg_Current        = np.zeros_like(Max_Temp)
    Avg_Power          = np.zeros_like(Max_Temp)
    E_Fade             = np.zeros_like(Max_Temp)
    Avg_C_rate         = np.zeros_like(Max_Temp)  
    Charge             = np.zeros_like(Max_Temp)  
    R_Growth           = np.zeros_like(Max_Temp)  
     
    for day in range(num_simulated_days):    
        Aging_time[day]         = day  
        
        # find maximum temperature in day
        for day_seg in range(num_flight_segments):
            t               = results.segments[day*num_segs_per_day + day_seg].conditions.frames.inertial.time[-1,0]   
            Q               = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_cell_charge_throughput[-1,0] 
            R               = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_resistance_growth_factor
            E               = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_capacity_fade_factor   
            temp            = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_cell_temperature[:,0]
            amb_temp        = results.segments[day*num_segs_per_day + day_seg].conditions.freestream.temperature[:,0]   
            energy          = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_energy[:,0] 
            volts           = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_voltage_under_load[:,0]
            current         = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_current[:,0]  
            power           = -results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_power_draw[:,0]  
            day             = results.segments[day*num_segs_per_day + day_seg].conditions.propulsion.battery_cycle_day
            battery_amp_hr  = (energy / Units.Wh )/volts   
            C_rating        = current /battery_amp_hr
         
            # Store Results in matrix     
            Bat_Start_Energy[day,day_seg]   = energy[0]
            Bat_Final_Energy[day,day_seg]   = energy[-1]
            Max_Temp[day,day_seg]           = max(temp)
            Avg_Temp[day,day_seg]           = np.mean(temp)
            Amb_Temp[day,day_seg]           = np.mean(amb_temp)
            Max_C_rate[day,day_seg]         = max(C_rating)
            Avg_C_rate[day,day_seg]         = np.mean(C_rating)  
            Mission_time[day,day_seg]       = t
            E_Fade[day,day_seg]             = E
            Aging_time[day,day_seg]         = day
            Charge[day,day_seg]             = Q
            R_Growth[day,day_seg]           = R
            Avg_Current[day,day_seg]        = np.mean(current)
            Avg_Power[day,day_seg]          = np.mean(power)
                            
            
    # Compile Results 
    RES                        = Data()
    RES.num_simulated_days     = num_simulated_days
    RES.num_flights_per_day    = num_flights_per_day
    RES.num_flight_segments    = num_flight_segments
    RES.Mission_time           = Aging_time  
    RES.E_Fade                 = E_Fade      
    RES.Charge                 = Charge       
    RES.R_Growth               = R_Growth     
    RES.Max_Seg_Temp           = Max_Temp      
    RES.Avg_Seg_Temp           = Avg_Temp     
    RES.Amb_Seg_Temp           = Amb_Temp     
    RES.Max_Seg_C_rate         = Max_C_rate   
    RES.Avg_Seg_C_rate         = Avg_C_rate   
    RES.Avg_Power              = Avg_Power 
    RES.Avg_Current            = Avg_Current   
    RES.Bat_Start_Energy       = Bat_Start_Energy 
    RES.Bat_Final_Energy       = Bat_Final_Energy
                        
    return RES
  

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

