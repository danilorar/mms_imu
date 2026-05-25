# function to estimate vertical loads using imu data 
import numpy as np
import pandas as pd

def derivative(signal, t):
    return np.gradient(signal, t)

def computeFz(csv_file, setting): 
    
    # from logs get the ax, ay, phiddot, psiddot
    data = pd.read_csv(csv_file)
    
    # accelerometer data
    acc = data[data["sensor"] == "acc"].copy()
    acc = acc.sort_values("t").reset_index(drop=True) # reset index
    
    t = acc["t"].values
    ax = acc["ax"].values
    ay = acc["ay"].values
    
    # gyro
    gyro = data[data["sensor"] == "gyro"].copy()
    gyro = gyro.sort_values("t").reset_index(drop=True) # reset index
    
    t_gyro = gyro["t"].values
    phiddot_gyro = derivative(gyro["wx"].values, t_gyro)
    psiddot_gyro = derivative(gyro["wz"].values, t_gyro)
    
    # interpolate to aligh time
    phiddot = np.interp(t, t_gyro, phiddot_gyro) # roll acceleration
    psiddot = np.interp(t, t_gyro, psiddot_gyro) # yaw acceleration
    
    # Vehicle constants 
    m  = 2600        # total mass [kg]
    m_body = 2289.5  # mass at body [kg]
    g  = 9.81        # gravity [m/s^2]
    h  = 0.646       # CG height = Body.position.z

    lf = 1.317       # CG to front axle [m]
    lr = 1.668       # CG to rear axle [m]
    L  = lf + lr     # Wheelbase

    wF = 1.6713      # front track width [m]
    wR = 1.6535      # rear track width [m]
    w  = 0.5*(wF+wR) # average track [m]

    Jx = 1002.0      # roll inertia [kg m^2]
    Jz = 3921.0      # yaw inertia [kg m^2]

    # Suspension Settings
    if setting == "soft":
        amplification_factor = 2
    elif setting == "medium":
        amplification_factor = 5
    elif setting == "hard":
        amplification_factor = 10
    
    # Spring stiffness
    kF = 25000 * amplification_factor  # front spring stiffness [N/m]
    kR = 25000 * amplification_factor  # rear spring stiffness [N/m]
    
    # Anti-roll bar stiffness [Nm/rad]
    KarbF = 34099.0   
    KarbR = 15947.0   
    
    # Axle Roll center heights [m]
    hrc_f = 0.209   
    hrc_r = 0.090   
    
    # Roll-center height at CG
    hrc = (lr/L)*hrc_f + (lf/L)*hrc_r 

    # Total axle roll stiffness [Nm/rad]
    Cf = kF*wF**2/2 + KarbF
    Cr = kR*wR**2/2 + KarbR
    
    # Yaw moment from longitudinal tire forces neglected
    Mzfx = 0

    # Using vertical load from Mats compendium
    FzFL = lr*m*g/(2*L) - m*h/(2*L)*ax - m*(h*Cf*L - hrc*(Cf*lf - Cr*lr))/(w*L*(Cf + Cr))*ay + Cf*Jx/(w*(Cf + Cr))*phiddot - hrc*Jz/(w*L)*psiddot + hrc/(w*L)*Mzfx
    FzFR = lr*m*g/(2*L)  - m*h/(2*L)*ax + m*(h*Cf*L - hrc*(Cf*lf - Cr*lr))/(w*L*(Cf + Cr))*ay - Cf*Jx/(w*(Cf + Cr))*phiddot + hrc*Jz/(w*L)*psiddot - hrc/(w*L)*Mzfx
    FzRL = lf*m*g/(2*L) + m*h/(2*L)*ax - m*(h*Cr*L + hrc*(Cf*lf - Cr*lr))/(w*L*(Cf + Cr))*ay + Cr*Jx/ (w*(Cf + Cr))*phiddot + hrc*Jz/(w*L)*psiddot - hrc/(w*L)*Mzfx
    FzRR = lf*m*g/(2*L) + m*h/(2*L)*ax + m*(h*Cr*L + hrc*(Cf*lf - Cr*lr))/(w*L*(Cf + Cr))*ay - Cr*Jx/(w*(Cf + Cr))*phiddot - hrc*Jz/(w*L)*psiddot + hrc/(w*L)*Mzfx
    
    
    Fz = pd.DataFrame({
        "t": t,
        "FzFL": FzFL,
        "FzFR": FzFR,
        "FzRL": FzRL,
        "FzRR": FzRR,
    })
    
    return Fz



