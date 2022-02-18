# cell discharge and age test

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Components.Energy.Networks.Battery_Cell_Cycler import Battery_Cell_Cycler
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
import numpy as np
import pylab as plt 
import pickle
import time
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 

    # Universal Plot Settings 
    plt.rcParams['axes.linewidth'] = 2.
    plt.rcParams["font.family"] = "Times New Roman"
    parameters = {'axes.labelsize': 32,
                  'xtick.labelsize': 28,
                  'ytick.labelsize': 28,
                  'axes.titlesize': 32}
    plt.rcParams.update(parameters)
    plot_parameters                  = Data()
    plot_parameters.line_width       = 3
    plot_parameters.line_style       = ['-' ,'--'] 
    plot_parameters.figure_width     = 10 
    plot_parameters.figure_height    = 8 
    plot_parameters.marker_size      = 10 
    plot_parameters.legend_font_size = 20 
    plot_parameters.plot_grid        = True 
    plot_parameters.markers          = ['s' ,'o' ,'P' ,'X']  
    plot_parameters.marker_size      = 14        
    plot_parameters.line_colors      = [ 'black','darkblue','darkgreen','firebrick']
    plot_parameters.legend_font      = 20                             # legend_font_size   
     
    Run_Analysis                     = False 
    
    ti                                 = time.time() 
    battery_chemistry                  = 'NMC'  
    days                               = 150 # Calendar and cycle life study of Li(NiMnCo)O2-based 18650 lithium-ion batteries
    ctrl_pts                           = 10 
    curr                               = np.array([2.05,2.05])
    SOC_cutoffs                        = np.array([[0.75,0.25],[0.60,0.40]])
    cycles_per_day                     = np.array([7,14,21])
    temperature                        = 35
    mAh                                = np.array([ 2050  , 2050 , 2050])    
    
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
        filename       =  str(battery_chemistry) + '_' + str(days) + '_days.pkl'
        save_results(Results,filename)    
        filename       =  str(battery_chemistry) + '_' + str(days) + '_days.pkl'
        Loaded_Results = load_results(filename) 
        plot_cell_degradation_results(Loaded_Results,plot_parameters)     
    else:
        
        filename       = str(battery_chemistry) + '_' + str(days) + '_days.pkl'
        Loaded_Results = load_results(filename) 
        plot_cell_degradation_results(Loaded_Results,plot_parameters) 
    
    return 

def plot_cell_degradation_results(Results,plot_parameters): 
      
    
    fig = plt.figure("Cell_Cap_Res")
    fig.set_size_inches(plot_parameters.figure_width,plot_parameters.figure_height)  
    axes = fig.add_subplot(1,1,1) 
    
 
 
 
 
 
 
    
    # ------------------------------------------------------------
    # 2D PLOT RESULTS
    # ------------------------------------------------------------  
    mult = 1 
    
    eq_cycles = (Results.equivalent_cylces[0,::mult]/12)*18
     
    C_2 = Results.normalized_max_energy[1,::mult]
    C_3 = Results.normalized_max_energy[2,::mult]   
    R_2 = Results.normalized_R_0[1,::mult]       
    R_3 = Results.normalized_R_0[2,::mult]    
     
    axes.set_xlabel('Equivalent Full Cycles')  
    axes.set_ylabel(r'$\%$ $E_{Fade}$  and  $R_{Growth}$')  
 
    axes.plot(eq_cycles,C_2*100, linewidth= plot_parameters.line_width, linestyle = plot_parameters.line_style[0],  color= plot_parameters.line_colors[0]  ,label = r'$E_{Fade}$ $75-25\%$'   )  
    axes.plot(eq_cycles,C_3*100, linewidth= plot_parameters.line_width, linestyle = plot_parameters.line_style[0],  color= plot_parameters.line_colors[3]  ,label = r'$E_{Fade}$ $60-40\%$'   )  
    axes.plot(eq_cycles,R_2*100, linewidth= plot_parameters.line_width, linestyle = plot_parameters.line_style[1],  color= plot_parameters.line_colors[0]  ,label = r'$R_{Growth}$ $75-25\%$' )   
    axes.plot(eq_cycles,R_3*100, linewidth= plot_parameters.line_width, linestyle = plot_parameters.line_style[1],  color= plot_parameters.line_colors[3]  ,label = r'$R_{Growth}$ $60-40\%$' )  
  
    axes.set_ylim([80,160]) 
    axes.set_xlim([0,1200])   
    axes.legend(loc='upper center', prop={'size': 16}, ncol=2)  
    fig.tight_layout()
     
    fig_name = 'NMC_Cell_Aging_Validation'   
    fig.savefig(fig_name  + '.pdf')                
    
     
    return 
# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup(current,experiment_temp,battery_chemistry,days,mAh,time_steps,SOC_cutoffs,cycles_per_day):  

    # vehicle data
    vehicle  = vehicle_setup(current,battery_chemistry,mAh,SOC_cutoffs)
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
def vehicle_setup(current,battery_chemistry,mAh,SOC_cutoffs):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle() 
    vehicle.tag = 'Battery'
    vehicle.mass_properties.takeoff         = 0.048 * Units.kg 
    vehicle.mass_properties.max_takeoff     = 0.048 * Units.kg 
    

    # basic parameters
    vehicle.reference_area      = 1.    
    

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   
    wing                         = SUAVE.Components.Wings.Wing()
    wing.tag                     = 'main_wing' 
    wing.areas.reference         = 1.
    wing.spans.projected         = 1.
    wing.aspect_ratio            = 1.
    wing.symmetric               = True
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.
    wing.dynamic_pressure_ratio  = 1.
    wing.chords.mean_aerodynamic = 1.
    wing.chords.root             = 1.
    wing.chords.tip              = 1.
    wing.origin                  = [[0.0,0.0,0.0]] # meters
    wing.aerodynamic_center      = [0.0,0.0,0.0] # meters
    
    # add to vehicle
    vehicle.append_component(wing)
     

    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------
    net                           = SUAVE.Components.Energy.Networks.Battery_Cell_Cycler()
    net.tag                       ='battery_cell'   
    net.dischage_model_fidelity   = battery_chemistry  
    net.voltage                   = 4.2  

    if battery_chemistry == 'NMC': 
        bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()
    elif battery_chemistry == 'LFP': 
        bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiFePO4_18650()  
        
    amp_hour_rating                 = 3.550
    bat.nominal_voltage             = 3.6
    bat.mass_properties.mass        = 0.048 * Units.kg 
    watt_hour_rating                = amp_hour_rating * bat.nominal_voltage 
    bat.specific_energy             = watt_hour_rating*Units.Wh/bat.cell.mass   
    bat.charging_voltage            = net.voltage   
    bat.cell.charging_SOC_cutoff    = SOC_cutoffs[0]  
    bat.charging_current            = 3.5
    bat.max_voltage                 = net.voltage  
    #bat.convective_heat_transfer_coefficient = 7.17
    initialize_from_mass(bat,module_weight_factor = 1) 
    
    net.battery                     = bat   
    vehicle.mass_properties.takeoff = bat.mass_properties.mass 

    avionics                      = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.current              = current 
    net.avionics                  = avionics  

    vehicle.append_component(net)    
    
    return vehicle  
    

# ------------------------------------------------------------------
#   Analyses Setup 
# ------------------------------------------------------------------  
def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


# ------------------------------------------------------------------
#   Base Analysis
# ------------------------------------------------------------------    
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
    analyses.append(aerodynamics)  

    # ------------------------------------------------------------------	
    #  Stability Analysis	
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()    	
    stability.geometry = vehicle	
    analyses.append(stability) 

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

    # done!
    return analyses   

# ------------------------------------------------------------------
#   Configs Setup 
# ------------------------------------------------------------------ 
def configs_setup(vehicle): 
    configs         = SUAVE.Components.Configs.Config.Container()  
    base_config     = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base' 
    configs.append(base_config)   
    return configs 


# ------------------------------------------------------------------
#  Mission Setup 
# ------------------------------------------------------------------ 
def mission_setup(analyses,vehicle,battery_chemistry,current,days,mAh,ctrl_pts,SOC_cutoffs,cycles_per_day,experiment_temp ): 

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'  
       
    # unpack Segments module
    Segments     = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment                                                              = Segments.Segment() 
    base_segment.process.initialize.initialize_battery                        = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery 
    base_segment.process.finalize.post_process.update_battery_state_of_health = SUAVE.Methods.Missions.Segments.Common.Energy.update_battery_state_of_health     
    
   
    bat                                                      = vehicle.networks.battery_cell.battery    
    base_segment.max_energy                                  = bat.max_energy
    base_segment.charging_SOC_cutoff                         = bat.cell.charging_SOC_cutoff 
    base_segment.charging_current                            = bat.charging_current
    base_segment.charging_voltage                            = bat.charging_voltage  
    #discharge_time                                           = 0.7*(SOC_cutoffs[0]-SOC_cutoffs[1]) * (mAh/1000)/current * Units.hrs
    discharge_time                                           = 0.95*(SOC_cutoffs[0]-SOC_cutoffs[1]) * (mAh/1000)/current * Units.hrs
    
    temp_dev                                         = experiment_temp - 15 
    base_segment.temperature_deviation               = temp_dev
     
    for day in range(days):   
        for cycle in range(cycles_per_day): 
            if battery_chemistry == 'NMC':
                segment     = Segments.Ground.Battery_Charge_Discharge(base_segment) 
                segment.tag = 'Bat_Discharge_D_' + str (day) + '_C_' + str (cycle)
                segment.analyses.extend(analyses.base)               
                segment.time                                        = discharge_time
                if  (day == 0) and (cycle== 0):
                    segment.battery_energy                               = bat.max_energy  * SOC_cutoffs[0]
                    segment.initial_battery_resistance_growth_factor     = 1
                    segment.initial_battery_capacity_fade_factor         = 1 
                segment = vehicle.networks.battery_cell.add_unknowns_and_residuals_to_segment(segment,initial_battery_cell_temperature = 273 + temp_dev)    
                mission.append_segment(segment) 
                    
                # Charge Model 
                segment     =  Segments.Ground.Battery_Charge_Discharge(base_segment)  
                segment.tag = 'Bat_Charge_D_' + str (day) + '_C_' + str (cycle)
                segment.analyses.extend(analyses.base)        
                segment.battery_discharge                           = False  
                if cycle  == cycles_per_day:  
                    segment.increment_battery_cycle_day=True                 
                segment.charging_SOC_cutoff                         = SOC_cutoffs[0]
                segment = vehicle.networks.battery_cell.add_unknowns_and_residuals_to_segment(segment,initial_battery_cell_temperature = 273 + temp_dev)    
                mission.append_segment(segment)  
                
    return mission  

# ------------------------------------------------------------------
#   Mission Setup 
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



# ------------------------------------------------------------------
#   Save Results
# ------------------------------------------------------------------ 
def save_results(results,filename):

    # Store data (serialize)
    with open(filename, 'wb') as file:
        pickle.dump(results, file)
        
    return  


# ------------------------------------------------------------------
#   Load Results 
# ------------------------------------------------------------------  
def load_results(filename):
    
    with open(filename, 'rb') as file:
        Results = pickle.load(file)   
        
    return Results

    
if __name__ == '__main__': 
    main()    
    plt.show()