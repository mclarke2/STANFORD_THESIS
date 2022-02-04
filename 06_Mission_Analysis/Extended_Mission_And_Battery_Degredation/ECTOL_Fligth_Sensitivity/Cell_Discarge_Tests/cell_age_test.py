# cell test 
# Created:  Oct 2018, M. Clarke  

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Components.Energy.Networks.Battery_Cell_Cycler import Battery_Cell_Cycler
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_mass 
from SUAVE.Methods.Weights.Buildups.eVTOL.empty import empty
from scipy.interpolate import interp1d, interp2d, RectBivariateSpline
from scipy.integrate import odeint
import numpy as np
import pylab as plt 
import pickle
import time
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    ti                    = time.time() 
    # Calendar and cycle life study of Li(NiMnCo)O2-based 18650 lithium-ion batteries
    battery_chemistry     = 'NMC'  
    days                  = 150
    ctrl_pts              = 10
    
    Run_Analysis          = False 
    curr                  = np.array([2.05,2.05])
    SOC_cutoffs           = np.array([[0.75,0.25],[0.60,0.40]])
    cycles_per_day        = np.array([7,14,21])
    temperature           = 35
    mAh                   = np.array([ 2050  , 2050 , 2050])    
    
    Results                            = Data()
    Results.Time                       = np.zeros((3, days))
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
    Results.normalized_max_energy      = np.zeros((3, days+1))
    Results.bat_age                    = np.zeros_like(Results.normalized_max_energy ) 
    Results.normalized_R_0             = np.zeros_like(Results.normalized_max_energy )   
    Results.equivalent_cylces          = np.zeros_like(Results.normalized_max_energy)
    
    Results.normalized_max_energy[:,0] = 1.
    Results.normalized_R_0[:,0]        = 1.  

    if Run_Analysis:
            
        for j in range(len(curr)):       
            configs, analyses = full_setup(curr[j],temperature,battery_chemistry,days,mAh[j],ctrl_pts,SOC_cutoffs[j],cycles_per_day[j])
            analyses.finalize()     
            mission = analyses.missions.base
            results = mission.evaluate()  
            k = 0
            i = 0
            
            for segment in results.segments.values(): 
                if k  % (cycles_per_day[j]*2)  == 0:
                    Results.Time[j,i]                    = segment.conditions.frames.inertial.time[0,0] / Units.hr
                    Results.energy[j,i]                  = segment.conditions.propulsion.battery_energy[0,0] / Units.hr
                    Results.current[j,i]                 = segment.conditions.propulsion.battery_current[0,0] 
                    Results.specific_power[j,i]          = segment.conditions.propulsion.battery_specfic_power[0,0] 
                    Results.SOC[j,i]                     = segment.conditions.propulsion.battery_state_of_charge[0,0]
                    Results.volts[j,i]                   = segment.conditions.propulsion.battery_voltage_under_load[0,0] 
                    Results.volts_oc[j,i]                = segment.conditions.propulsion.battery_voltage_open_circuit[0,0]  
                    Results.temp[j,i]                    = segment.conditions.propulsion.battery_cell_temperature[0,0]     
                    Results.equivalent_cylces[j,i+1]     = 12*(i+1)           
                    Results.charge_throughput[j,i]       = segment.conditions.propulsion.battery_cumulative_charge_throughput[0,0]   
                    Results.normalized_max_energy[j,i+1] = segment.conditions.propulsion.battery_capacity_fade_factor 
                    Results.bat_age[j,i+1]               = segment.conditions.propulsion.battery_cumulative_charge_throughput[0,0]
                    Results.normalized_R_0[j,i+1]        = segment.conditions.propulsion.battery_resistance_growth_factor 
                    
                    i += 1
                k += 1 
        
        tf = time.time()
        dt = tf-ti
        print('Time Elapsed ' + str(dt/60) + ' min')    
        filename   =  str(battery_chemistry) + '_' + str(days) + '_days.pkl'
        save_results(Results,filename)    
        filename   =  str(battery_chemistry) + '_' + str(days) + '_days.pkl'
        Loaded_Results = load_results(filename) 
        plot_results(Loaded_Results)     
    else:
        
        filename   =  str(battery_chemistry) + '_' + str(days) + '_days.pkl'
        Loaded_Results = load_results(filename) 
        plot_results(Loaded_Results) 
    
    return

def plot_results(Results): 
     
    ls                = ['-' ,'--'] 
    lc                = ['darkgreen','darkblue' , 'black', 'darkorange']  
    lw               = 4 
    axis_tick_size   = 18   
    plt.rcParams.update({'font.size': 22})
    plt.rcParams['axes.linewidth'] = 3. 
    width  = 8
    height = 6    
    
    fig = plt.figure("Cell_Cap_Res")
    fig.set_size_inches(width,height)  
    axes = fig.add_subplot(1,1,1) 
    
    
    
    
    exp_eq_cycles = np.array([0, 248.87, 500.98, 749.77, 995.23]) 
    Cap_2 = np.array([1.0 , 0.9502 , 0.9201 , 0.8846 , 0.8509])  
    Cap_3 = np.array([1.0, 0.9675 , 0.9515 , 0.9410 , 0.9321 ]) 
    Res_2 = np.array([1. ,1.0928,1.2276,1.20810,1.3790])
    Res_3 = np.array([1.,1.030, 1.090, 1.112, 1.121 ]) 

    
    # ------------------------------------------------------------
    # 2D PLOT RESULTS
    # ------------------------------------------------------------  
    mult = 1 
    
    eq_cycles = (Results.equivalent_cylces[0,::mult]/12)*18
    
    #C_1 = Results.normalized_max_energy[0,::mult]
    C_2 = Results.normalized_max_energy[1,::mult]
    C_3 = Results.normalized_max_energy[2,::mult] 
    
    #R_1 = Results.normalized_R_0[0,::mult]       
    R_2 = Results.normalized_R_0[1,::mult]       
    R_3 = Results.normalized_R_0[2,::mult]    
     
    axes.set_xlabel('Equivalent Full Cycles')  
    axes.set_ylabel(r'$\%$ $E_{Fade}$  and  $R_{Growth}$')  

    #axes.plot(eq_cycles,C_1, linewidth= lw, linestyle = ls[0],  color= lc[0]  ,label = '$E_{Fade} 90-10%$')   
    axes.plot(eq_cycles,C_2*100, linewidth= lw, linestyle = ls[0],  color= lc[1]  ,label = r'$E_{Fade}$ $75-25\%$'   )  
    axes.plot(eq_cycles,C_3*100, linewidth= lw, linestyle = ls[0],  color= lc[2]  ,label = r'$E_{Fade}$ $60-40\%$'   )  
    axes.plot(eq_cycles,R_2*100, linewidth= lw, linestyle = ls[1],  color= lc[1]  ,label = r'$R_{Growth}$ $75-25\%$' )   
    axes.plot(eq_cycles,R_3*100, linewidth= lw, linestyle = ls[1],  color= lc[2]  ,label = r'$R_{Growth}$ $60-40\%$' )  
    axes.scatter(exp_eq_cycles,  Cap_2*100,c=lc[1],s=80, marker = 'P',label= r'$E_{Fade}$ $75-25\%$ Exp.'   )  
    axes.scatter(exp_eq_cycles,  Cap_3*100,c=lc[2],s=80, marker = 'X',label= r'$E_{Fade}$ $60-40\%$ Exp.'   )      
    axes.scatter(exp_eq_cycles,  Res_2*100,c=lc[1],s=80, marker = 's',label= r'$R_{Growth}$ $75-25\%$ Exp.' )  
    axes.scatter(exp_eq_cycles,  Res_3*100,c=lc[2],s=80, marker = 'o',label= r'$R_{Growth}$ $60-40\%$ Exp.' )  
   
    axes.set_ylim([80,180]) 
    axes.set_xlim([0,1200])   
    axes.legend(loc='upper left', prop={'size': 16}, ncol=2)    
    axes.tick_params(axis = 'x', labelsize = axis_tick_size ) 
    axes.tick_params(axis = 'y', labelsize = axis_tick_size )
     
    return 
# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup(current,experiment_temp,battery_chemistry,days,mAh,time_steps,SOC_cutoffs,cycles_per_day):
    
    # vehicle data
    vehicle  = battery_cell_setup(current,battery_chemistry,mAh,SOC_cutoffs)
    configs  = configs_setup(vehicle)
    
    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,battery_chemistry,current,days,mAh,time_steps,SOC_cutoffs,cycles_per_day,experiment_temp)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses
    

    return vehicle, analyses


# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def battery_cell_setup(current,battery_chemistry,mAh,SOC_cutoffs):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle() 
    vehicle.tag = 'Battery'
    vehicle.mass_properties.takeoff = 3. * Units.kg
    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------

    net = Battery_Cell_Cycler()
    net.voltage                     = 4.2  
    if battery_chemistry == 'NMC':
        bat                             = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()   
    amp_hour_rating                 = 2.05
    bat.nominal_voltage             = 3.6
    watt_hour_rating                = amp_hour_rating * bat.nominal_voltage 
    bat.specific_energy             = watt_hour_rating*Units.Wh/bat.mass_properties.mass   
    bat.charging_voltage            = net.voltage   
    bat.cell.charging_SOC_cutoff    = SOC_cutoffs[0]  
    bat.charging_current            = 2.05
    bat.max_voltage                 = net.voltage  
    initialize_from_mass(bat, bat.mass_properties.mass)
    
    net.battery                     = bat  
    avionics                        = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.current                = current
    net.avionics                    = avionics 
                                 
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

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
    analyses.append(energy)
    
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

def mission_setup(analyses,vehicle,battery_chemistry,current,days,mAh,ctrl_pts,SOC_cutoffs,cycles_per_day,experiment_temp ):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission' 

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                                  = Segments.Segment()
    ones_row                                                      = base_segment.state.ones_row
    base_segment.state.numerics.number_control_points             = ctrl_pts 
    base_segment.process.iterate.initials.initialize_battery      = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.finalize.post_process.update_battery_age = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_age    
    
    bat                                              = vehicle.propulsors.battery_test.battery 
    base_segment.use_Jacobian                        =  True
    base_segment.state.numerics.jacobian_evaluations = 0 
    base_seg:ment.state.numerics.iterations           = 0    
    base_segment.max_energy                          = bat.max_energy
    base_segment.charging_SOC_cutoff                 = bat.cell.charging_SOC_cutoff 
    base_segment.charging_current                    = bat.charging_current
    base_segment.charging_voltage                    = bat.charging_voltage 
    base_segment.battery_resistance_growth_factor    = 1
    base_segment.battery_capacity_fade_factor        = 1     
    base_segment.battery_configuration               = bat.pack_config  
    discharge_time                                   = 0.7*(SOC_cutoffs[0]-SOC_cutoffs[1]) * (mAh/1000)/current * Units.hrs
    
    temp_dev                                         = experiment_temp - 15 
    base_segment.temperature_deviation               = temp_dev
    
    atmo        = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    alt         = 0
    temp_guess  = atmo.compute_values(alt,temperature_deviation=temp_dev).temperature   
    
    for day in range(days):   
        for cycle in range(cycles_per_day): 
            if battery_chemistry == 'NMC':
                segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)
                segment.tag = 'Bat_Discharge_D_' + str (day) + '_C_' + str (cycle)
                segment.analyses.extend(analyses.base)     
                segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_test.unpack_unknowns_linmco
                segment.process.iterate.residuals.network           = vehicle.propulsors.battery_test.residuals_linmco         
                segment.state.unknowns.battery_state_of_charge      = 0.5 * ones_row(1) 
                segment.state.unknowns.battery_current              = 5 * ones_row(1) 
                segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
                segment.state.residuals.network                     = 0.* ones_row(3)  
                segment.time                                        = discharge_time
                segment.battery_discharge                           = True   
                segment.battery_age_in_days                         = day
                mission.append_segment(segment)       
                if day == 0 and cycle == 0:   
                    segment.battery_energy                          = bat.max_energy  * SOC_cutoffs[0]
                
                # Charge Model 
                segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)     
                segment.tag = 'Bat_Charge_D_' + str (day) + '_C_' + str (cycle)
                segment.analyses.extend(analyses.base)     
                segment.process.iterate.unknowns.network            = vehicle.propulsors.battery_test.unpack_unknowns_linmco
                segment.process.iterate.residuals.network           = vehicle.propulsors.battery_test.residuals_linmco 
                segment.state.unknowns.battery_state_of_charge      = 0.5 * ones_row(1) 
                segment.state.unknowns.battery_current              = 5 * ones_row(1) 
                segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
                segment.state.residuals.network                     = 0.* ones_row(3)       
                segment.battery_discharge                           = False
                segment.battery_age_in_days                         = day 
                segment.charging_SOC_cutoff                         = SOC_cutoffs[0]
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


def load_results(filename):
    
    with open(filename, 'rb') as file:
        Results = pickle.load(file)   
        
    return Results

    
if __name__ == '__main__': 
    main()    
    plt.show()