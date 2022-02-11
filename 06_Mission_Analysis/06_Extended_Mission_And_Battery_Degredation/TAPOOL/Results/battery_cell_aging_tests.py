# battery_cell_discharge_tests.py 
# 
# Created: Sep 2021, M. Clarke 

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data 
from SUAVE.Methods.Power.Battery.Sizing import  initialize_from_circuit_configuration 
import numpy as np
import pylab as plt  
import time
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # Calendar and cycle life study of Li(NiMnCo)O2-based 18650 lithium-ion batteries
    ti                    = time.time() 
    battery_chemistry     = 'NMC'  
    days                  = 20 
    ctrl_pts              = 10 
    curr                  = np.array([2.05])
    SOC_cutoffs           = np.array([[1.0,0.]])
    cycles_per_day        = 12
    temperature           = 35
    Ah                    = np.array([ 2.05])    
    
    num_segments = days*2*cycles_per_day
    
    Results                            = Data()
    Results.Time                       = np.zeros((1,num_segments))
    Results.energy                     = np.zeros_like(Results.Time)  
    Results.current                    = np.zeros_like(Results.Time)
    Results.specific_power             = np.zeros_like(Results.Time) 
    Results.SOC                        = np.zeros_like(Results.Time)
    Results.volts                      = np.zeros_like(Results.Time)
    Results.volts_oc                   = np.zeros_like(Results.Time) 
    Results.battery_amp_hr             = np.zeros_like(Results.Time)
    Results.C_rate                     = np.zeros_like(Results.Time)
    Results.temp                       = np.zeros_like(Results.Time)   
    Results.charge_throughput          = np.zeros_like(Results.Time)  
    Results.normalized_max_energy      = np.zeros((1, num_segments))
    Results.bat_age                    = np.zeros_like(Results.normalized_max_energy ) 
    Results.normalized_R_0             = np.zeros_like(Results.normalized_max_energy )   
    Results.equivalent_cylces          = np.zeros_like(Results.normalized_max_energy)
    
    Results.normalized_max_energy[:,0] = 1.
    Results.normalized_R_0[:,0]        = 1.  
 
    for j in range(len(curr)):       
        configs, analyses = full_setup(curr[j],temperature,battery_chemistry,days,Ah[j],ctrl_pts,SOC_cutoffs[j],cycles_per_day)
        analyses.finalize()     
        mission = analyses.missions.base
        results = mission.evaluate()   
        for i , segment in enumerate(results.segments.values()):  
            Results.Time[j,i]                  = segment.conditions.frames.inertial.time[0,0] / Units.hr
            Results.energy[j,i]                = segment.conditions.propulsion.battery_energy[0,0] / Units.hr
            Results.current[j,i]               = segment.conditions.propulsion.battery_current[0,0] 
            Results.specific_power[j,i]        = segment.conditions.propulsion.battery_specfic_power[0,0] 
            Results.SOC[j,i]                   = segment.conditions.propulsion.battery_state_of_charge[0,0]
            Results.volts[j,i]                 = segment.conditions.propulsion.battery_voltage_under_load[0,0] 
            Results.volts_oc[j,i]              = segment.conditions.propulsion.battery_voltage_open_circuit[0,0]  
            Results.temp[j,i]                  = segment.conditions.propulsion.battery_cell_temperature[0,0]     
            Results.charge_throughput[j,i]     = segment.conditions.propulsion.battery_cell_charge_throughput[0,0]   
            Results.normalized_max_energy[j,i] = segment.conditions.propulsion.battery_capacity_fade_factor 
            Results.bat_age[j,i]               = segment.conditions.propulsion.battery_cell_charge_throughput[0,0]
            Results.normalized_R_0[j,i]        = segment.conditions.propulsion.battery_resistance_growth_factor 
                 

    Results.equivalent_cylces   = Results.Time       
    plot_results(Results)      
    tf = time.time()
    dt = tf-ti
    print('Time Elapsed ' + str(dt/60) + ' min')
    
    return

def plot_results(Results): 
    # plot settings 
    ls               = ['-' ,'--'] 
    lc               = ['black','darkblue']  
    lw               = 4 
    axis_tick_size   = 18   
    plt.rcParams.update({'font.size': 22})
    plt.rcParams['axes.linewidth'] = 3. 
    width            = 9
    height           = 7     
    fig              = plt.figure("Cell_Cap_Res")
    fig.set_size_inches(width,height)  
    axes             = fig.add_subplot(1,1,1)  
     
    # Plot Results 
    eq_cycles = Results.equivalent_cylces[0]  
    C_1       = Results.normalized_max_energy[0] 
    R_1       = Results.normalized_R_0[0]
    
     # Figure 12 Calendar and cycle life study of Li(NiMnCo)O2-based 18650 lithium-ion batteries
    Experimental_Data               = Data()
    Experimental_Data.exp_eq_cycles = np.array([0, 248.87, 500.98, 749.77, 995.23])  
    Experimental_Data.Cap_1         = np.array([1.0 , 0.9502 , 0.9201 , 0.8846 , 0.8509])   
    Experimental_Data.Res_1         = np.array([1. ,1.0928,1.2276,1.20810,1.3790])  

    # Plot Results     
    axes.set_xlabel('Equivalent Full Cycles')  
    axes.set_ylabel(r'$\%$ $E_{Fade}$  and  $R_{Growth}$')   
    axes.plot(eq_cycles,C_1*100, linewidth= lw, linestyle = ls[0],  color= lc[0]  ,label = r'$E_{Fade}$ $100-0\%$'   )    
    axes.plot(eq_cycles,R_1*100, linewidth= lw, linestyle = ls[1],  color= lc[0]  ,label = r'$R_{Growth}$ $100-0\%$' )   
    axes.scatter(Experimental_Data.exp_eq_cycles,  Experimental_Data.Cap_1*100,c=lc[0],s=80, marker = 'P',label= r'$E_{Fade}$ $100-0\%$ Exp.'   )   
    axes.scatter(Experimental_Data.exp_eq_cycles,  Experimental_Data.Res_1*100,c=lc[0],s=80, marker = 's',label= r'$R_{Growth}$ $100-0\%$ Exp.' )   
   
    axes.set_ylim([80,180]) 
    axes.set_xlim([0,1200])   
    axes.legend(loc='upper center', prop={'size': 16}, ncol=2)    
    axes.tick_params(axis = 'x', labelsize = axis_tick_size ) 
    axes.tick_params(axis = 'y', labelsize = axis_tick_size )
     
    return 
# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup(current,experiment_temp,battery_chemistry,days,Ah,time_steps,SOC_cutoffs,cycles_per_day):
    
    # vehicle data
    vehicle  = battery_cell_setup(current,battery_chemistry,Ah,SOC_cutoffs)
    configs  = configs_setup(vehicle)
    
    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,battery_chemistry,current,days,Ah,time_steps,SOC_cutoffs,cycles_per_day,experiment_temp)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses 

    return vehicle, analyses


# ----------------------------------------------------------------------
#   Build Battery
# ----------------------------------------------------------------------
def battery_cell_setup(current,battery_chemistry,Ah,SOC_cutoffs):


    vehicle                       = SUAVE.Vehicle() 
    vehicle.tag                   = 'battery'  

    net                           = SUAVE.Components.Energy.Networks.Battery_Cell_Cycler()
    net.tag                       ='battery_cell'   
    
    if battery_chemistry == 'NMC':
        bat                             = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()      
        bat.cell.nominal_capacity                = Ah                                                     
        bat.charging_current                     = 2.05 
        bat.convective_heat_transfer_coefficient = 7.17 
        bat.cell.charging_SOC_cutoff             = SOC_cutoffs[0]  
        initialize_from_circuit_configuration(bat) 
        net.voltage                     = bat.max_voltage
    
    net.battery                     = bat  
    avionics                        = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.current                = current
    net.avionics                    = avionics 
    
    vehicle.mass_properties.takeoff = bat.mass_properties.mass 
                                 
    vehicle.append_component(net)
    
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

    #return analyses     
def base_analysis(vehicle):   
    analyses = SUAVE.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy) 

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_eVTOL()
    weights.vehicle = vehicle
    analyses.append(weights)    
    
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


    # done!
    return configs

def mission_setup(analyses,vehicle,battery_chemistry,current,days,Ah,ctrl_pts,SOC_cutoffs,cycles_per_day,experiment_temp ):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission' 

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments


    # base segment
    base_segment                                                        = Segments.Segment() 
    base_segment.process.initialize.initialize_battery                  = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery   
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health  
    base_segment.process.iterate.conditions.stability                   = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.stability                = SUAVE.Methods.skip 
    base_segment.process.iterate.conditions.aerodynamics                = SUAVE.Methods.skip
    base_segment.process.finalize.post_process.aerodynamics             = SUAVE.Methods.skip     
    base_segment.process.iterate.conditions.planet_position             = SUAVE.Methods.skip
    
   
    bat                                                      = vehicle.networks.battery_cell.battery    
    base_segment.max_energy                                  = bat.max_energy
    base_segment.charging_SOC_cutoff                         = bat.cell.charging_SOC_cutoff 
    base_segment.charging_current                            = bat.charging_current
    base_segment.charging_voltage                            = bat.charging_voltage   
    discharge_time                                           = (SOC_cutoffs[0]-SOC_cutoffs[1]) * (Ah)/current * Units.hrs 
    
     
    temp_dev                                                 = experiment_temp - 15 
    base_segment.temperature_deviation                       = temp_dev 
    atmo                                                     = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    alt                                                      = 0
    temp_guess                                               = atmo.compute_values(alt,temperature_deviation=temp_dev).temperature   
    
    for day in range(days):   
        for cycle in range(cycles_per_day): 
            if battery_chemistry == 'NMC': 
        
                segment                                             = Segments.Ground.Battery_Charge_Discharge(base_segment)
                segment.tag                                         = "NMC_Discharge_Day_" + str (day) + '_Cycle_' + str (cycle)
                segment.analyses.extend(analyses.base)       
                segment.time                                        = discharge_time  
                if day == 0: 
                    segment.battery_energy                          = bat.max_energy * SOC_cutoffs[0]
                segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health                    
                segment = vehicle.networks.battery_cell.add_unknowns_and_residuals_to_segment(segment,initial_battery_cell_temperature =temp_guess)    
                mission.append_segment(segment)          
            
                # Charge Model 
                segment                                             = Segments.Ground.Battery_Charge_Discharge(base_segment)     
                segment.tag                                         = "NMC_Charge_Day_" + str (day) + '_Cycle_' + str (cycle)
                segment.battery_discharge                           = False 
                segment.charging_SOC_cutoff                         = SOC_cutoffs[0]
                segment.analyses.extend(analyses.base)            
                if cycle  == (cycles_per_day-1):  
                    segment.increment_battery_cycle_day=True
                segment = vehicle.networks.battery_cell.add_unknowns_and_residuals_to_segment(segment,initial_battery_cell_temperature =temp_guess)  
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
    
if __name__ == '__main__': 
    main()    
    plt.show()
    
    