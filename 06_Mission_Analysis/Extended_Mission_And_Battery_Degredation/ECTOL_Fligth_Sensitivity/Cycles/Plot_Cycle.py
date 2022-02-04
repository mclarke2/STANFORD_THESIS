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

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # load data
    with open('test_mission.pkl', 'rb') as f1:
            results = pickle.load(f1)  
    with open('results_summary.pkl', 'rb') as file:
            res_2 = pickle.load(file)   
            
    # ------------------------------------------------------------
    # PLOT RESULTS
    # ------------------------------------------------------------  
    module_config    = [128,40]    
    label_size       = 12
    legend_font_size = 14 
    line_width       = 3 
    axis_font        = {'size':'12'}     

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    fig.set_size_inches(8, 8)

    for i in range(len(results.segments)):  
        time       = results.segments[i].conditions.frames.inertial.time[:,0]/3600 
        airspeed   = results.segments[i].conditions.freestream.velocity[:,0] 
        range_nm   = results.segments[i].conditions.frames.inertial.position_vector[:,0]  
        altitude   = results.segments[i].conditions.freestream.altitude[:,0]*3.28084  # convert to ft 

        axes = fig.add_subplot(3,1,1)
        axes.plot(time, altitude, 'b-', linewidth=line_width) 
        axes.set_ylabel('Altitude (ft)', axis_font)
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
  
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , range_nm*0.000539957 , 'b-', linewidth= line_width) 
        axes.set_ylabel('Range (nm)', axis_font)
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')    
         
        axes = fig.add_subplot(3,1,3)
        axes.plot( time ,airspeed, 'b-', linewidth= line_width) 
        axes.set_ylabel('Airspeed (m/s)', axis_font)
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   
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
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  
        
        axes = fig.add_subplot(3,2,2) 
        axes.plot(time, energy *0.000277778 , 'b-' , linewidth= line_width)        
        axes.set_ylabel('Battery Energy (W-hr)', axis_font)
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(3,2,3) 
        axes.plot(time, volts   , 'b-',label='ULV' , linewidth = line_width) 
        axes.plot(time, volts_oc , color='darkred', linewidth = line_width , linestyle = '--' ,label='OCV')
        axes.set_ylabel('Battery Voltage (V)', axis_font)  
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        if i == 0:
            axes.legend(loc='upper right')          

        axes = fig.add_subplot(3,2,4)
        axes.plot(time, current , 'b-' ,linewidth= line_width)
        axes.set_ylabel('Current (Amp)', axis_font)  
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')   

        axes = fig.add_subplot(3,2,5)
        axes.plot(time, C_rating , 'b-' , linewidth= line_width) 
        axes.set_ylabel('C-Rating (C)', axis_font)  
        axes.set_xlabel('Time (hr)', axis_font)
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')    

        axes = fig.add_subplot(3,2,6)
        axes.plot(time, eta , 'b-' ,   linewidth= line_width)  
        axes.set_ylabel('Throttle ($\eta$)', axis_font)
        axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        #axes.set_xlabel('Time (hr)', axis_font)                  

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
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(3,2,2)
        axes.plot(time, energy *0.000277778 , 'b-', linewidth= line_width)        
        axes.set_ylabel('Cell Energy (W-hr)', axis_font)
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        
        axes = fig.add_subplot(3,2,3)
        axes.plot(time, volts/128   , 'b-' ,label='ULV', linewidth= line_width) 
        axes.plot(time, volts_oc/128 , color='darkred', linewidth = line_width , linestyle = '--',label='OCV')
        axes.set_ylabel('Cell Voltage (V)', axis_font)  
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')     
        if i == 0:
            axes.legend(loc='upper right')             

        axes = fig.add_subplot(3,2,4)
        axes.plot(time, current/40 , 'b-' , linewidth= line_width) 
        axes.set_ylabel('Current (Amp)', axis_font)  
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        

        axes = fig.add_subplot(3,2,5) 
        axes.plot(time, C_rating , 'b-', linewidth= line_width)
        axes.set_xlabel('Time (hr)', axis_font)
        axes.set_ylabel('C-Rating (C)', axis_font)  
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

        axes = fig.add_subplot(3,2,6)
        axes.plot(time, eta ,  'b-', linewidth= line_width)
        axes.set_ylabel('Throttle ($\eta$)', axis_font) 
        axes.set_xlabel('Time (hr)', axis_font)        
        #axes.minorticks_on()
        #axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        #axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')  

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
    C_Fade     = res_2.C_Fade
    R_Growth   = res_2.R_Growth
    Aging_time = res_2.Aging_time  
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
  
    # ------------------------------------------------------------------
    #   Temperature vs Time in Days
    # ------------------------------------------------------------------
    fig = plt.figure("Temperature")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)): 
        time = results.segments[i].conditions.frames.inertial.time[:,0]/3600
        temp = results.segments[i].conditions.propulsion.battery_cell_temperature[:,0]  
        axes = fig.add_subplot(1,1,1) 
        axes.plot(time, temp, 'b-' , linewidth= line_width )           
        axes.set_ylim([15,40]) 
        axes.set_ylabel(r'Temperature $\degree C$', axis_font)    
        axes.set_xlabel('Cycle', axis_font)  
        axes.minorticks_on()
        axes.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        axes.grid(which='minor', linestyle=':', linewidth='0.5', color='grey') 
        
                
    return
 
 
if __name__ == '__main__': 
    main()    
    plt.show()