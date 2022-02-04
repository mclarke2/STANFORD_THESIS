# cell test 
# Created:  Oct 2018, M. Clarke  

#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Components.Energy.Networks.Battery_Test import Battery_Test
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass, initialize_from_circuit_configuration 
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_mass 
from SUAVE.Methods.Weights.Buildups.eVTOL.empty import empty
from scipy.interpolate import interp1d, interp2d, RectBivariateSpline
from scipy.integrate import odeint
import numpy as np
import pylab as plt
from copy import deepcopy

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():    

    days                  = 1
    battery_fidelity      = [3]
    curr                  = np.array([1.5, 3, 6, 9 ])*40
    mAh                   = np.array([3300, 3300  , 3300 , 3500 ])*1520
    temperature           = [ 27 , 27 , 27, 27 ]
    temp_guess            = [32 , 37. , 42 , 39.]        
    time_steps            = 20 

    # voltage plots 
    fig1 = plt.figure(1)
    fig1.set_size_inches(10, 10) 
    axes1 = fig1.add_subplot(1,1,1)
    
    # temperature plots 
    fig2 = plt.figure(2)
    fig2.set_size_inches(10 , 10) 
    axes2 = fig2.add_subplot(1,1,1)
    
    # C rate plots 
    fig3 = plt.figure(3)
    fig3.set_size_inches(10 , 10) 
    axes3 = fig3.add_subplot(1,1,1)
    
    # joule heating  plots 
    fig4 = plt.figure(4)
    fig4.set_size_inches(10 , 10) 
    axes4 = fig4.add_subplot(1,1,1)    
    
    # current heating plots 
    fig5 = plt.figure(5)
    fig5.set_size_inches(10 , 10) 
    axes5 = fig5.add_subplot(1,1,1)   
    
    for j in range(len(curr)):      
        for i in range(len(battery_fidelity)):    
            configs, analyses = full_setup(curr[j],temperature[j],battery_fidelity[i],temp_guess[j],days,mAh[j],time_steps)
            analyses.finalize()     
            mission = analyses.missions.base
            results = mission.evaluate() 

            plot_results(results,i,j,battery_fidelity[i],axes1 ,axes2  ,axes3  ,axes4 ,axes5) 
 
    return 

def plot_results(results,i,j,bf,axes1 ,axes2  ,axes3  ,axes4,axes5) :

    curr   = ['1.5' ,'3' ,'6', '9']  
    #curr   = ['2' ,'4' ,'6', '8']  
    mark_1 = ['.' ,'.' ,'.' ,'.','.']
    mark_2 = ['s' ,'s' ,'s' ,'s','s']
    ls_1   = [':' ,':' ,':' ,':',':']
    ls_2   = ['-' ,'-' ,'-' ,'-','-']
    lc_1   = ['blue' , 'red' , 'green' , 'grey', 'orange' ]
    lc_2   = ['darkblue' , 'darkred' , 'darkgreen' , 'black', 'darkorange']
    lw     = 2    
    lbs    = 12    
    axis_font  = {'size':'14'}         

    for segment in results.segments.values():
        time          = segment.conditions.frames.inertial.time[:,0]/60
        energy        = segment.conditions.propulsion.battery_energy[:,0]
        max_energy    = segment.conditions.propulsion.battery_capacity_fade_factor 
        bat_age       = segment.conditions.propulsion.battery_age_in_days
        R_0           = segment.conditions.propulsion.battery_internal_resistance[:,0]  
        current       = segment.conditions.propulsion.battery_current[:,0] 
        specific_power= segment.conditions.propulsion.battery_specfic_power[:,0] 
        SOC           = segment.conditions.propulsion.battery_state_of_charge[:,0]
        volts         = segment.conditions.propulsion.battery_voltage_under_load[:,0] 
        volts_oc      = segment.conditions.propulsion.battery_voltage_open_circuit[:,0]  
        cell_temp     = segment.conditions.propulsion.battery_cell_temperature[:,0] 
        pack_temp     = segment.conditions.propulsion.battery_pack_temperature[:,0]   
        Amp_Hrs       = segment.conditions.propulsion.battery_cumulative_charge_throughput[:,0]   
        jhf           = segment.conditions.propulsion.battery_cell_joule_heat_fraction[:,0]   
        ehf           = segment.conditions.propulsion.battery_cell_entropy_heat_fraction[:,0]  
        battery_amp_hr = (energy/ Units.Wh )/volts  
        C_rating       = current/battery_amp_hr 
        
        
        use_amp_hrs = False
        
        if use_amp_hrs == True:
            x_vals = Amp_Hrs
        else:
            x_vals = time 
            
        # Voltage plot 
        if segment.tag == 'Battery Charge':
            if bf == 2:
                axes1.plot(x_vals , volts , marker= mark_1[j], linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw )  
                axes2.plot(x_vals , cell_temp+273, marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw ) 
                           
            elif bf == 3:  
                axes1.plot(x_vals , volts , marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw)  
                axes1.plot(x_vals, volts_oc,  marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw )    
                axes2.plot(x_vals , cell_temp+273, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw )  
                axes2.plot(x_vals, cell_temp+273, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw)    
                axes3.plot(x_vals, C_rating, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw)    
                axes4.plot(x_vals, jhf, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw     )    
                axes5.plot(x_vals, ehf , marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw    )                   
        else:              
            if bf == 2:    
                axes1.plot(x_vals , volts , marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw   ,label = 'NCA Model @ '+ curr[j] + ' A') 
                axes2.plot(x_vals , cell_temp+273, marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw,label = 'NCA Model @ '+ curr[j] + ' A')    
            elif bf == 3:                                                   
                axes1.plot(x_vals, volts,  marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw   ,label = 'NMC ULV Model @ '+ curr[j] + ' A') 
                axes1.plot(x_vals, volts_oc,  marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw   ,label = 'NMC OCV Model @ '+ curr[j] + ' A')    
                axes2.plot(x_vals, cell_temp+273, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw,label = 'cell temp NMC Model @ '+ curr[j] + ' A')   
                axes2.plot(x_vals, pack_temp+273, marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw,label = 'pack temp NMC Model @ '+ curr[j] + ' A')    
                axes3.plot(x_vals, C_rating, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw,label = 'NMC Model @ '+ curr[j] + ' A')    
                axes4.plot(x_vals, jhf, marker= mark_2[j] , linestyle = ls_2[j],  color= lc_2[j] , linewidth = lw,label = ' Joule Heat NMC Model @ '+ curr[j] + ' A')    
                axes4.plot(x_vals, ehf , marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw,label = 'Entropy Heat NMC Model @ '+ curr[j] + ' A')
                axes5.plot(x_vals, current, marker= mark_1[j] , linestyle = ls_1[j],  color= lc_1[j] , linewidth = lw,label = 'Current NMC Model @ '+ curr[j] + ' A')
              
        
                
        axes1.set_ylabel('Voltage $(V_{UL}$)', axis_font)    
        axes1.set_xlabel('Amp-Hours (Ah)', axis_font) 
        #axes1.set_ylim([2.8,5])
        #axes1.set_xlim([0,7])
        axes1.tick_params(axis='both', which='major', labelsize=lbs)          
       
        #axes2.set_ylim([15,50])
       # axes2.set_xlim([0,7])
        axes2.set_ylabel(r'Temperature ($\degree$C)', axis_font)    
        axes2.set_xlabel('Amp-Hours (Ah)', axis_font)  
        axes2.tick_params(axis='both', which='major', labelsize=lbs)  
          
        axes3.set_ylabel('C-Rating', axis_font)    
        axes3.set_xlabel('Amp-Hours (Ah)', axis_font) 
        #axes3.set_ylim([0,20])
        #axes3.set_xlim([0,7])   
        axes3.tick_params(axis='both', which='major', labelsize=lbs)     
        
        axes4.set_ylabel('Heating', axis_font)    
        axes4.set_xlabel('Amp-Hours (Ah)', axis_font)  
        #axes4.set_xlim([0,7])   
        axes4.tick_params(axis='both', which='major', labelsize=lbs)      
        
        axes5.set_ylabel('Current', axis_font)    
        axes5.set_xlabel('Amp-Hours (Ah)', axis_font) 
        #axes5.set_ylim([0,20])
        #axes5.set_xlim([0,7])   
        axes5.tick_params(axis='both', which='major', labelsize=lbs)     
        
        if segment.tag != 'Battery Charge':
            axes1.legend(loc='upper center') 
            axes2.legend(loc='upper center')           
    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------
def full_setup(current,temperature,battery_fidelity,temp_guess,days,mAh,time_steps):

    # vehicle data
    vehicle  = vehicle_setup(current,temperature,battery_fidelity)
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle,battery_fidelity,current,temp_guess,days,mAh,time_steps)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses


    return vehicle, analyses


# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(current,temperature,battery_fidelity):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle() 
    vehicle.tag = 'Battery'
    vehicle.mass_properties.takeoff = 3. * Units.kg
    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------

    net = Battery_Test()
    net.voltage                     = 4.1   
    net.dischage_model_fidelity     = battery_fidelity

    # Component 8 the Battery
    if battery_fidelity == 2:
        bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNCA_18650()  
    elif battery_fidelity == 3: 
        bat= SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion_LiNiMnCoO2_18650()  
    bat.cell.max_mission_discharge      = 9.        # Amps  
    bat.cell.max_discharge_rate         = 15.       # Amps   
    bat.cell.surface_area               = (np.pi*bat.cell.height*bat.cell.diameter)  
    
    # electrical characteristics (for energy model)
    bat.pack_config.series              = 128
    bat.pack_config.parallel            = 40  
                                        
    bat.cell.charging_voltage           = 3.6 
    bat.cell.charging_current           = 5.0  
    bat.age_in_days                     = 0  
    initialize_from_circuit_configuration(bat)   
    net.battery                         = bat 

    avionics                            = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.current                    = current 
    net.avionics                        = avionics 

    vehicle.append_component(net)

    return vehicle

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

def mission_setup(analyses,vehicle,battery_fidelity,current,temp_guess,days,mAh,ctrl_pts ):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission' 

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row     = base_segment.state.ones_row
    base_segment.state.numerics.number_control_points        = ctrl_pts 
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery

    bat = vehicle.propulsors.propulsor.battery 
    base_segment.use_Jacobian =  True
    base_segment.state.numerics.jacobian_evaluations = 0 
    base_segment.state.numerics.iterations           = 0    
    base_segment.max_energy                       = bat.max_energy
    base_segment.charging_SOC_cutoff              = bat.cell.charging_SOC_cutoff 
    base_segment.charging_current                 = bat.charging_current
    base_segment.charging_voltage                 = bat.charging_voltage 
    base_segment.battery_resistance_growth_factor = 1
    base_segment.battery_capacity_fade_factor     = 1     
    base_segment.battery_configuration            = bat.pack_config
    discharge_time                                = 0.9 * (mAh/1000)/current * Units.hrs

    # Thevenin Discharge Model 
    if battery_fidelity == 2:
        segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)    
        segment.tag = "Battery Discharge" 
        segment.analyses.extend(analyses.base)     
        segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_linca
        segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_linca 
        segment.state.unknowns.battery_state_of_charge      = 0.5 * ones_row(1) 
        segment.state.unknowns.battery_thevenin_voltage     = 0.01 * ones_row(1) 
        segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
        segment.state.residuals.network                     = 0.   * ones_row(3)   
        segment.time                                        = discharge_time
        segment.battery_discharge                           = True 
        segment.battery_cell_temperature                    = 20.  
        segment.ambient_temperature                         = 20   
        segment.battery_age_in_days                         = days 
        segment.battery_cumulative_charge_throughput         = 0
        segment.battery_energy                              = bat.max_energy * 1.

        # add to misison
        mission.append_segment(segment)        

        # Thevenin Discharge Model 
        segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)     
        segment.tag = 'Battery Charge'  
        segment.analyses.extend(analyses.base)     
        segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_linca
        segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_linca
        segment.state.unknowns.battery_state_of_charge      = 0.5   * ones_row(1)  
        segment.state.unknowns.battery_thevenin_voltage     = 0.1 * ones_row(1) 
        segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
        segment.state.residuals.network                     = 0. * ones_row(3)    
        segment.battery_discharge                           = False
        segment.battery_age_in_days                         = days 

        # add to misison
        mission.append_segment(segment)


    elif battery_fidelity ==3 :
        segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)
        segment.tag = "Battery Discharge" 
        segment.analyses.extend(analyses.base)     
        segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_linmco
        segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_linmco         
        segment.state.unknowns.battery_state_of_charge      = 0.5 * ones_row(1) 
        segment.state.unknowns.battery_current              = 5 * ones_row(1) 
        segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
        segment.state.residuals.network                     = 0.* ones_row(3)  
        segment.time                                        = discharge_time
        segment.battery_discharge                           = True 
        segment.battery_cell_temperature                    = 27. 
        segment.ambient_temperature                         = 27.
        segment.battery_thevenin_voltage                    = 0
        segment.battery_age_in_days                         = days 
        segment.battery_cumulative_charge_throughput         = 0
        segment.battery_energy                              = bat.max_energy * 1.
        mission.append_segment(segment)        

        ## Charge Model 
        #segment     = Segments.Battery_Cell_Testbench.Charge_Discharge_Test(base_segment)     
        #segment.tag = 'Battery Charge'  
        #segment.analyses.extend(analyses.base)     
        #segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns_linmco
        #segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals_linmco 
        #segment.state.unknowns.battery_state_of_charge      = 0.5 * ones_row(1) 
        #segment.state.unknowns.battery_current              = 150 * ones_row(1) 
        #segment.state.unknowns.battery_cell_temperature     = temp_guess  * ones_row(1) 
        #segment.state.residuals.network                     = 0.* ones_row(3)       
        #segment.battery_discharge                           = False
        #segment.battery_age_in_days                         = days  
        #mission.append_segment(segment) 


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