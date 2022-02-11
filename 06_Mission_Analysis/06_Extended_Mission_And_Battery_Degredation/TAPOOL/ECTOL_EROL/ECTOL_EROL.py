# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
import copy, time
from SUAVE.Core import Data , Units
import numpy as np   
import pickle 

import sys 
sys.path.append('../Aircraft_Models_With_Batteries') 


# aircraft   
from ECTOL_2P import full_setup   as   full_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    time0               = time.time()  
    
    num_simulated_days  = 365
    num_flights_per_day = 8 
    aircraft_ranges     = 65 *Units.nmi # [60,70,80,90,100] 
    reserve_segment     = False 
    control_points      = 8
    recharge_battery    = True
    N_gm_x              = 10
    N_gm_y              = 10
    
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup(num_simulated_days,num_flights_per_day,aircraft_ranges,reserve_segment,control_points,recharge_battery,N_gm_x,N_gm_y )
    
    configs.finalize()
    analyses.finalize()     
    mission            = analyses.missions.base
    results_raw        = mission.evaluate()  
    vehicle_name       = 'ECTOL'
    filename           = vehicle_name + '_' + str(num_simulated_days) + '_Days_' + str(round(aircraft_ranges/Units.nmi,0)) + '_Nmi'
    
    if vehicle_name =='ECTOL': 
        num_flight_segments = 9
    elif vehicle_name =='TW': 
        num_flight_segments = 11
    elif vehicle_name =='SR': 
        num_flight_segments = 11
    elif vehicle_name =='MR': 
        num_flight_segments = 8 
        
    results_prossessed = process_results(results_raw,num_simulated_days,num_flights_per_day,num_flight_segments) 
    save_results(results_prossessed,filename) 
            
    time1 = time.time() 
    print('The total elapsed time: ' + str(round((time1-time0)/3600 , 3 )) + ' Hrs')  
    
    return

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
  

def save_results(results,filename):
   
    save_file = filename + '.pkl' 
    with open(save_file, 'wb') as file:
        pickle.dump(results, file)
        
    return  

if __name__ == '__main__': 
    main()    

