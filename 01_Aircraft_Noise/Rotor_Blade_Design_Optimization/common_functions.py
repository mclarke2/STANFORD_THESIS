 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from SUAVE.Core import Data, Units  

# package imports
import numpy as np 
from scipy import interpolate


def evaluate_rotor_aerodynamics(B,c,beta_0,R,chi,Rh,a_geo,a_loc,cl_sur,cd_sur, rho,mu,nu,a,T,V,omega):
    
    
    # Add variavles so code done break 
    tc                    = 0.12 
    Na                    = 24 
    use_2d_analysis       = False 
    rotation              = 1
    pitch_c               = 0.0
 

    # Unpack freestream conditions 
    r_1d              =  r 
    nu                = mu/rho
    rho_0             = rho 
    pi                = np.pi 
    V_thrust          = V
    V                 = V_thrust[:,0,None] 
    total_blade_pitch = beta_0 + pitch_c   
 
    # Number of radial stations and segment control points
    Nr       = len(c)
    ctrl_pts = len(V)

    # Non-dimensional radial distribution and differential radius
    chi           = r_1d/R
    diff_r        = np.diff(r_1d)
    deltar        = np.zeros(len(r_1d))
    deltar[1:-1]  = diff_r[0:-1]/2 + diff_r[1:]/2
    deltar[0]     = diff_r[0]/2
    deltar[-1]    = diff_r[-1]/2

    # Calculating rotational parameters
    omegar   = np.outer(omega,r_1d)
    n        = omega/(2.*pi)   # Rotations per second

    # 2 dimensional radial distribution non dimensionalized
    chi_2d         = np.tile(chi[:, None],(1,Na))
    chi_2d         = np.repeat(chi_2d[None,:,:], ctrl_pts, axis=0)
    r_dim_2d       = np.tile(r_1d[:, None] ,(1,Na))
    r_dim_2d       = np.repeat(r_dim_2d[None,:,:], ctrl_pts, axis=0)
    c_2d           = np.tile(c[:, None] ,(1,Na))
    c_2d           = np.repeat(c_2d[None,:,:], ctrl_pts, axis=0)

    # Azimuthal distribution of stations
    psi            = np.linspace(0,2*pi,Na+1)[:-1]
    psi_2d         = np.tile(np.atleast_2d(psi),(Nr,1))
    psi_2d         = np.repeat(psi_2d[None, :, :], ctrl_pts, axis=0) 

    # Starting with uniform freestream
    ua       = 0
    ut       = 0
    ur       = 0

    # Include velocities introduced by rotor incidence angles
    if (np.any(abs(V_thrust[:,1]) >1e-3) or np.any(abs(V_thrust[:,2]) >1e-3)) and use_2d_analysis:

        # y-component of freestream in the propeller cartesian plane
        Vy  = V_thrust[:,1,None,None]
        Vy  = np.repeat(Vy, Nr,axis=1)
        Vy  = np.repeat(Vy, Na,axis=2)

        # z-component of freestream in the propeller cartesian plane
        Vz  = V_thrust[:,2,None,None]
        Vz  = np.repeat(Vz, Nr,axis=1)
        Vz  = np.repeat(Vz, Na,axis=2)

        # check for invalid rotation angle
        if (rotation == 1) or (rotation == -1):
            pass
        else:
            print("Invalid rotation direction. Setting to 1.")
            rotation = 1

        # compute resulting radial and tangential velocities in polar frame
        utz =  Vz*np.cos(psi_2d* rotation)
        urz =  Vz*np.sin(psi_2d* rotation)
        uty =  Vy*np.sin(psi_2d* rotation)
        ury =  Vy*np.cos(psi_2d* rotation)

        ut +=  (utz + uty)
        ur +=  (urz + ury)
        ua +=  np.zeros_like(ut) 
  
 
    # total velocities
    r      = r_1d
    Ua     = np.outer((V + ua),np.ones_like(r))
    beta   = total_blade_pitch

    # Things that will change with iteration
    size   = (ctrl_pts,Nr)
    PSI    = np.ones(size)
    PSIold = np.zeros(size)

    # Total velocities
    Ut     = omegar - ut
    U      = np.sqrt(Ua*Ua + Ut*Ut + ur*ur)
 
    # Setup a Newton iteration
    diff   = 1.
    tol    = 1e-6  # Convergence tolerance
    ii     = 0

    # BEMT Iteration
    while (diff>tol):
        # compute velocities
        sin_psi      = np.sin(PSI)
        cos_psi      = np.cos(PSI)
        Wa           = 0.5*Ua + 0.5*U*sin_psi
        Wt           = 0.5*Ut + 0.5*U*cos_psi
        va           = Wa - Ua
        vt           = Ut - Wt

        # compute blade airfoil forces and properties
        Cl, Cdval, alpha, Ma, W = compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,a_loc,a_geo,cl_sur,cd_sur,ctrl_pts,Nr,Na,tc,use_2d_analysis)

        # compute inflow velocity and tip loss factor
        lamdaw, F, piece = compute_inflow_and_tip_loss(r,R,Wa,Wt,B)

        # compute Newton residual on circulation
        Gamma       = vt*(4.*pi*r/B)*F*(1.+(4.*lamdaw*R/(pi*B*r))*(4.*lamdaw*R/(pi*B*r)))**0.5
        Rsquiggly   = Gamma - 0.5*W*c*Cl

        # use analytical derivative to get dR_dpsi
        dR_dpsi = compute_dR_dpsi(B,beta,r,R,Wt,Wa,U,Ut,Ua,cos_psi,sin_psi,piece)

        # update inflow angle
        dpsi        = -Rsquiggly/dR_dpsi
        PSI         = PSI + dpsi
        diff        = np.max(abs(PSIold-PSI))
        PSIold      = PSI

        # If omega = 0, do not run BEMT convergence loop
        if all(omega[:,0]) == 0. :
            break

        # If its really not going to converge
        if np.any(PSI>pi/2) and np.any(dpsi>0.0):
            print("Rotor BEMT did not converge to a solution (Stall)")
            break

        ii+=1
        if ii>10000:
            print("Rotor BEMT did not converge to a solution (Iteration Limit)")
            break


    
    # tip loss correction for velocities, since tip loss correction is only applied to loads in prior BEMT iteration
    va     = F*va
    vt     = F*vt
    lamdaw = r*(va+Ua)/(R*(Ut-vt))

    # More Cd scaling from Mach from AA241ab notes for turbulent skin friction
    Tw_Tinf     = 1. + 1.78*(Ma*Ma)
    Tp_Tinf     = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
    Tp          = (Tp_Tinf)*T
    Rp_Rinf     = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
    Cd          = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval

    epsilon                  = Cd/Cl
    epsilon[epsilon==np.inf] = 10.

    # thrust and torque and their derivatives on the blade.
    blade_T_distribution     = rho*(Gamma*(Wt-epsilon*Wa))*deltar
    blade_Q_distribution     = rho*(Gamma*(Wa+epsilon*Wt)*r)*deltar
    blade_dT_dr              = rho*(Gamma*(Wt-epsilon*Wa))
    blade_dQ_dr              = rho*(Gamma*(Wa+epsilon*Wt)*r)
  
    Va_2d   = np.repeat(Wa[ :, :, None], Na, axis=2)
    Vt_2d   = np.repeat(Wt[ :, :, None], Na, axis=2)

    blade_T_distribution_2d  = np.repeat(blade_T_distribution[:, :, None], Na, axis=2)
    blade_Q_distribution_2d  = np.repeat(blade_Q_distribution[:, :, None], Na, axis=2)
    blade_dT_dr_2d           = np.repeat(blade_dT_dr[:, :, None], Na, axis=2)
    blade_dQ_dr_2d           = np.repeat(blade_dQ_dr[:, :, None], Na, axis=2)
    blade_Gamma_2d           = np.repeat(Gamma[ :, :, None], Na, axis=2)
    alpha_2d                 = np.repeat(alpha[ :, :, None], Na, axis=2)

    Vt_avg                  = Wt
    Va_avg                  = Wa
    Vt_ind_avg              = vt
    Va_ind_avg              = va
    Va_ind_2d               = np.repeat(va[ :, :, None], Na, axis=2)
    Vt_ind_2d               = np.repeat(vt[ :, :, None], Na, axis=2)

    # compute the hub force / rotor drag distribution along the blade
    dL    = 0.5*rho*c*Cd*omegar**2*deltar
    dL_2d = np.repeat(dL[:, :, None], Na, axis=2)
    dD    = 0.5*rho*c*Cl*omegar**2*deltar
    dD_2d = np.repeat(dD[:, :, None], Na, axis=2)

    rotor_drag_distribution = np.mean(dL_2d*np.sin(psi_2d) + dD_2d*np.cos(psi_2d),axis=2)

    # forces
    thrust                  = np.atleast_2d((B * np.sum(blade_T_distribution, axis = 1))).T
    torque                  = np.atleast_2d((B * np.sum(blade_Q_distribution, axis = 1))).T
    rotor_drag              = np.atleast_2d((B * np.sum(rotor_drag_distribution, axis=1))).T
    power                   = omega*torque

    # calculate coefficients
    D        = 2*R
    Cq       = torque/(rho_0*(n*n)*(D*D*D*D*D))
    Ct       = thrust/(rho_0*(n*n)*(D*D*D*D))
    Cp       = power/(rho_0*(n*n*n)*(D*D*D*D*D))
    Crd      = rotor_drag/(rho_0*(n*n)*(D*D*D*D))
    etap     = V*thrust/power 

    # Store data 
    results_conditions                            = Data
    outputs                                       = results_conditions(
                number_radial_stations            = Nr,
                number_azimuthal_stations         = Na,
                disc_radial_distribution          = r_dim_2d,
                speed_of_sound                    = a,
                density                           = rho,
                velocity                          = Vv,
                blade_tangential_induced_velocity = Vt_ind_avg,
                blade_axial_induced_velocity      = Va_ind_avg,
                blade_tangential_velocity         = Vt_avg,
                blade_axial_velocity              = Va_avg,
                disc_tangential_induced_velocity  = Vt_ind_2d,
                disc_axial_induced_velocity       = Va_ind_2d,
                disc_tangential_velocity          = Vt_2d,
                disc_axial_velocity               = Va_2d,
                drag_coefficient                  = Cd,
                lift_coefficient                  = Cl,
                omega                             = omega,
                disc_circulation                  = blade_Gamma_2d,
                blade_dT_dr                       = blade_dT_dr,
                disc_dT_dr                        = blade_dT_dr_2d,
                blade_thrust_distribution         = blade_T_distribution,
                disc_thrust_distribution          = blade_T_distribution_2d,
                disc_effective_angle_of_attack    = alpha_2d,
                thrust_per_blade                  = thrust/B,
                thrust_coefficient                = Ct,
                disc_azimuthal_distribution       = psi_2d,
                blade_dQ_dr                       = blade_dQ_dr,
                disc_dQ_dr                        = blade_dQ_dr_2d,
                blade_torque_distribution         = blade_Q_distribution,
                disc_torque_distribution          = blade_Q_distribution_2d,
                torque_per_blade                  = torque/B,
                torque_coefficient                = Cq,
                power                             = power,
                power_coefficient                 = Cp,
                converged_inflow_ratio            = lamdaw,
                propeller_efficiency              = etap,
                blade_H_distribution              = rotor_drag_distribution,
                rotor_drag                        = rotor_drag,
                rotor_drag_coefficient            = Crd,
        )

    return thrust , torque, power, Cp, outputs , etap


def spin_HFW(self,conditions):
    """Analyzes a general rotor given geometry and operating conditions.
    Runs the blade element theory with a helical fixed-wake model for the
    iterative wake analysis.

    Assumptions:
      Helical fixed-wake with wake skew angle

    Source:
      N/A

    Inputs:
    self.inputs.omega                    [radian/s]
    conditions.freestream.
      density                            [kg/m^3]
      dynamic_viscosity                  [kg/(m-s)]
      speed_of_sound                     [m/s]
      temperature                        [K]
    conditions.frames.
      body.transform_to_inertial         (rotation matrix)
      inertial.velocity_vector           [m/s]
    conditions.propulsion.
      throttle                           [-]

    Outputs:
    conditions.propulsion.outputs.
       number_radial_stations            [-]
       number_azimuthal_stations         [-]
       disc_radial_distribution          [m]
       speed_of_sound                    [m/s]
       density                           [kg/m-3]
       velocity                          [m/s]
       disc_tangential_induced_velocity  [m/s]
       disc_axial_induced_velocity       [m/s]
       disc_tangential_velocity          [m/s]
       disc_axial_velocity               [m/s]
       drag_coefficient                  [-]
       lift_coefficient                  [-]
       omega                             [rad/s]
       disc_circulation                  [-]
       blade_dQ_dR                       [N/m]
       blade_dT_dr                       [N]
       blade_thrust_distribution         [N]
       disc_thrust_distribution          [N]
       thrust_per_blade                  [N]
       thrust_coefficient                [-]
       azimuthal_distribution            [rad]
       disc_azimuthal_distribution       [rad]
       blade_dQ_dR                       [N]
       blade_dQ_dr                       [Nm]
       blade_torque_distribution         [Nm]
       disc_torque_distribution          [Nm]
       torque_per_blade                  [Nm]
       torque_coefficient                [-]
       power                             [W]
       power_coefficient                 [-]

    Properties Used:
    self.
      number_of_blades                   [-]
      tip_radius                         [m]
      twist_distribution                 [radians]
      chord_distribution                 [m]
      orientation_euler_angles           [rad, rad, rad]
    """

    #--------------------------------------------------------------------------------
    # Initialize by running BEMT to get initial blade circulation
    #--------------------------------------------------------------------------------
    _, _, _, _, bemt_outputs , _ = self.spin(conditions)
    conditions.noise.sources.propellers[self.tag] = bemt_outputs
    self.outputs = bemt_outputs
    omega = self.inputs.omega

    #--------------------------------------------------------------------------------
    # generate rotor wake vortex distribution
    #--------------------------------------------------------------------------------
    props = Data()
    props.propeller = self

    # generate wake distribution for n rotor rotation
    nrots         = self.number_rotor_rotations
    steps_per_rot = self.number_steps_per_rotation
    rpm           = omega/Units.rpm

    # simulation parameters for n rotor rotations
    init_timestep_offset     = 0.
    time                     = 60*nrots/rpm[0][0]
    number_of_wake_timesteps = steps_per_rot*nrots

    self.wake_settings.init_timestep_offset     = init_timestep_offset
    self.wake_settings.wake_development_time    = time
    self.wake_settings.number_of_wake_timesteps = number_of_wake_timesteps
    self.use_2d_analysis                        = True

    # spin propeller with helical fixed-wake
    self.wake_method = "helical_fixed_wake"
    thrust_vector, torque, power, Cp, outputs , etap = self.spin(conditions)

    return thrust_vector, torque, power, Cp, outputs , etap
 

def compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,a_loc,a_geo,cl_sur,cd_sur,ctrl_pts,Nr,Na,tc,use_2d_analysis):
    """
    Cl, Cdval = compute_airfoil_aerodynamics( beta,c,r,R,B,
                                              Wa,Wt,a,nu,
                                              a_loc,a_geo,cl_sur,cd_sur,
                                              ctrl_pts,Nr,Na,tc,use_2d_analysis )

    Computes the aerodynamic forces at sectional blade locations. If airfoil
    geometry and locations are specified, the forces are computed using the
    airfoil polar lift and drag surrogates, accounting for the local Reynolds
    number and local angle of attack.

    If the airfoils are not specified, an approximation is used.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       beta                       blade twist distribution                        [-]
       c                          chord distribution                              [-]
       r                          radius distribution                             [-]
       R                          tip radius                                      [-]
       B                          number of rotor blades                          [-]

       Wa                         axial velocity                                  [-]
       Wt                         tangential velocity                             [-]
       a                          speed of sound                                  [-]
       nu                         viscosity                                       [-]

       a_loc                      Locations of specified airfoils                 [-]
       a_geo                      Geometry of specified airfoil                   [-]
       cl_sur                     Lift Coefficient Surrogates                     [-]
       cd_sur                     Drag Coefficient Surrogates                     [-]
       ctrl_pts                   Number of control points                        [-]
       Nr                         Number of radial blade sections                 [-]
       Na                         Number of azimuthal blade stations              [-]
       tc                         Thickness to chord                              [-]
       use_2d_analysis            Specifies 2d disc vs. 1d single angle analysis  [Boolean]

    Outputs:
       Cl                       Lift Coefficients                         [-]
       Cdval                    Drag Coefficients  (before scaling)       [-]
       alpha                    section local angle of attack             [rad]

    """

    alpha        = beta - np.arctan2(Wa,Wt)
    W            = (Wa*Wa + Wt*Wt)**0.5
    Ma           = W/a
    Re           = (W*c)/nu

    # If propeller airfoils are defined, use airfoil surrogate
    if a_loc != None:
        # Compute blade Cl and Cd distribution from the airfoil data
        dim_sur = len(cl_sur)
        if use_2d_analysis:
            # return the 2D Cl and CDval of shape (ctrl_pts, Nr, Na)
            Cl      = np.zeros((ctrl_pts,Nr,Na))
            Cdval   = np.zeros((ctrl_pts,Nr,Na))
            for jj in range(dim_sur):
                Cl_af           = cl_sur[a_geo[jj]](Re,alpha,grid=False)
                Cdval_af        = cd_sur[a_geo[jj]](Re,alpha,grid=False)
                locs            = np.where(np.array(a_loc) == jj )
                Cl[:,locs,:]    = Cl_af[:,locs,:]
                Cdval[:,locs,:] = Cdval_af[:,locs,:]
        else:
            # return the 1D Cl and CDval of shape (ctrl_pts, Nr)
            Cl      = np.zeros((ctrl_pts,Nr))
            Cdval   = np.zeros((ctrl_pts,Nr))

            for jj in range(dim_sur):
                Cl_af         = cl_sur[a_geo[jj]](Re,alpha,grid=False)
                Cdval_af      = cd_sur[a_geo[jj]](Re,alpha,grid=False)
                locs          = np.where(np.array(a_loc) == jj )
                Cl[:,locs]    = Cl_af[:,locs]
                Cdval[:,locs] = Cdval_af[:,locs]
    else:
        # Estimate Cl max
        Cl_max_ref = -0.0009*tc**3 + 0.0217*tc**2 - 0.0442*tc + 0.7005
        Re_ref     = 9.*10**6
        Cl1maxp    = Cl_max_ref * ( Re / Re_ref ) **0.1

        # If not airfoil polar provided, use 2*pi as lift curve slope
        Cl = 2.*np.pi*alpha

        # By 90 deg, it's totally stalled.
        Cl[Cl>Cl1maxp]  = Cl1maxp[Cl>Cl1maxp] # This line of code is what changed the regression testing
        Cl[alpha>=np.pi/2] = 0.

        # Scale for Mach, this is Karmen_Tsien
        Cl[Ma[:,:]<1.] = Cl[Ma[:,:]<1.]/((1-Ma[Ma[:,:]<1.]*Ma[Ma[:,:]<1.])**0.5+((Ma[Ma[:,:]<1.]*Ma[Ma[:,:]<1.])/(1+(1-Ma[Ma[:,:]<1.]*Ma[Ma[:,:]<1.])**0.5))*Cl[Ma<1.]/2)

        # If the blade segments are supersonic, don't scale
        Cl[Ma[:,:]>=1.] = Cl[Ma[:,:]>=1.]

        #This is an atrocious fit of DAE51 data at RE=50k for Cd
        Cdval = (0.108*(Cl*Cl*Cl*Cl)-0.2612*(Cl*Cl*Cl)+0.181*(Cl*Cl)-0.0139*Cl+0.0278)*((50000./Re)**0.2)
        Cdval[alpha>=np.pi/2] = 2.


    # prevent zero Cl to keep Cd/Cl from breaking in bemt
    Cl[Cl==0] = 1e-6

    return Cl, Cdval, alpha, Ma, W


def compute_dR_dpsi(B,beta,r,R,Wt,Wa,U,Ut,Ua,cos_psi,sin_psi,piece):
    """
    Computes the analytical derivative for the BEMT iteration.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       B                          number of rotor blades                          [-]
       beta                       blade twist distribution                        [-]
       r                          radius distribution                             [m]
       R                          tip radius                                      [m]
       Wt                         tangential velocity                             [m/s]
       Wa                         axial velocity                                  [m/s]
       U                          total velocity                                  [m/s]
       Ut                         tangential velocity                             [m/s]
       Ua                         axial velocity                                  [m/s]
       cos_psi                    cosine of the inflow angle PSI                  [-]
       sin_psi                    sine of the inflow angle PSI                    [-]
       piece                      output of a step in tip loss calculation        [-]

    Outputs:
       dR_dpsi                    derivative of residual wrt inflow angle         [-]

    """
    # An analytical derivative for dR_dpsi used in the Newton iteration for the BEMT
    # This was solved symbolically in Matlab and exported
    pi          = np.pi
    pi2         = np.pi**2
    BB          = B*B
    BBB         = BB*B
    f_wt_2      = 4*Wt*Wt
    f_wa_2      = 4*Wa*Wa
    arccos_piece = np.arccos(piece)
    Ucospsi     = U*cos_psi
    Usinpsi     = U*sin_psi
    Utcospsi    = Ut*cos_psi
    Uasinpsi    = Ua*sin_psi
    UapUsinpsi  = (Ua + Usinpsi)
    utpUcospsi  = (Ut + Ucospsi)
    utpUcospsi2 = utpUcospsi*utpUcospsi
    UapUsinpsi2 = UapUsinpsi*UapUsinpsi
    dR_dpsi     = ((4.*U*r*arccos_piece*sin_psi*((16.*UapUsinpsi2)/(BB*pi2*f_wt_2) + 1.)**(0.5))/B -
                   (pi*U*(Ua*cos_psi - Ut*sin_psi)*(beta - np.arctan((Wa+Wa)/(Wt+Wt))))/(2.*(f_wt_2 + f_wa_2)**(0.5))
                   + (pi*U*(f_wt_2 +f_wa_2)**(0.5)*(U + Utcospsi  +  Uasinpsi))/(2.*(f_wa_2/(f_wt_2) + 1.)*utpUcospsi2)
                   - (4.*U*piece*((16.*UapUsinpsi2)/(BB*pi2*f_wt_2) + 1.)**(0.5)*(R - r)*(Ut/2. -
                    (Ucospsi)/2.)*(U + Utcospsi + Uasinpsi ))/(f_wa_2*(1. - np.exp(-(B*(Wt+Wt)*(R -
                    r))/(r*(Wa+Wa))))**(0.5)) + (128.*U*r*arccos_piece*(Wa+Wa)*(Ut/2. - (Ucospsi)/2.)*(U +
                    Utcospsi  + Uasinpsi ))/(BBB*pi2*utpUcospsi*utpUcospsi2*((16.*f_wa_2)/(BB*pi2*f_wt_2) + 1.)**(0.5)))

    dR_dpsi[np.isnan(dR_dpsi)] = 0.1
    return dR_dpsi

def compute_inflow_and_tip_loss(r,R,Wa,Wt,B):
    """
    Computes the inflow, lamdaw, and the tip loss factor, F.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       r          radius distribution                                              [m]
       R          tip radius                                                       [m]
       Wa         axial velocity                                                   [m/s]
       Wt         tangential velocity                                              [m/s]
       B          number of rotor blades                                           [-]
                 
    Outputs:               
       lamdaw     inflow ratio                                                     [-]
       F          tip loss factor                                                  [-]
       piece      output of a step in tip loss calculation (needed for residual)   [-]
    """
    lamdaw            = r*Wa/(R*Wt)
    lamdaw[lamdaw<0.] = 0.
    f                 = (B/2.)*(1.-r/R)/lamdaw
    f[f<0.]           = 0.
    piece             = np.exp(-f)
    F                 = 2.*np.arccos(piece)/np.pi

    return lamdaw, F, piece


def linear_discretize(x_pivs,chi,pivot_points):
    
    chi_pivs       = np.zeros(len(x_pivs))
    chi_pivs[0]    = chi[0]
    chi_pivs[-1]   = chi[-1]
    locations      = np.array(pivot_points)*(chi[-1]-chi[0]) + chi[0]
    
    # vectorize 
    chi_2d = np.repeat(np.atleast_2d(chi).T,len(pivot_points),axis = 1)
    pp_2d  = np.repeat(np.atleast_2d(locations),len(chi),axis = 0)
    idxs   = (np.abs(chi_2d - pp_2d)).argmin(axis = 0) 
    
    chi_pivs[1:-1] = chi[idxs]
    
    x_bar  = np.interp(chi,chi_pivs, x_pivs) 
    
    return x_bar 


def spline_discretize(x_pivs,chi,pivot_points):
    chi_pivs       = np.zeros(len(x_pivs))
    chi_pivs[0]    = chi[0]
    chi_pivs[-1]   = chi[-1]
    locations      = np.array(pivot_points)*(chi[-1]-chi[0]) + chi[0]
    
    # vectorize 
    chi_2d = np.repeat(np.atleast_2d(chi).T,len(pivot_points),axis = 1)
    pp_2d  = np.repeat(np.atleast_2d(locations),len(chi),axis = 0)
    idxs   = (np.abs(chi_2d - pp_2d)).argmin(axis = 0) 
    
    chi_pivs[1:-1] = chi[idxs]
    
    fspline = interpolate.CubicSpline(chi_pivs,x_pivs)
    x_bar = fspline(chi)
    
    return x_bar