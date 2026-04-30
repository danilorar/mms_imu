import numpy as np 

signals = ["ax", "ay", "az", "wx", "wy", "wz"]

def rms(x): 
     return np.sqrt(np.mean(x**2))

def peak_to_peak(x): 
     return np.max(x) - np.min(x)

def compute_jerk(signal, df):
    """Derivative of acceleration"""

    y = df[signal].values
    t = df["t"].values

    dy = np.diff(y)
    dt = np.diff(t)

    valid = dt > 1e-6

    jerk = dy[valid] / dt[valid]

    return jerk


def kpis(df, maneuver, setting, trial): 
     '''Extract one row of kpi's from one filtered imu run '''
    
     # Compress repeated timestamps by averaging all signals at the same time
     df = df.groupby("t", as_index=False).mean()
     
     kpi_row = {"maneuver": maneuver, "setting": setting, "trial": trial}
     
     for signal in signals: 
          kpi_row[f"{signal}_mean"] = np.mean(df[signal].values)
          kpi_row[f"{signal}_rms"] = rms(df[signal].values)
          kpi_row[f"{signal}_std"] = np.std(df[signal].values)
          kpi_row[f"{signal}_peak_to_peak"] = peak_to_peak(df[signal].values)
          
     # jerk metric 
     for signal in ["ax", "ay", "az"]: 
          jerk = compute_jerk(signal, df)
          
          if len(jerk) != 0:
               kpi_row[f"{signal}_jerk_rms"] = rms(jerk)
               kpi_row[f"{signal}_jerk_std"] = np.std(jerk)
               kpi_row[f"{signal}_jerk_max"] = np.max(np.abs(jerk))
          else:     
               kpi_row[f"{signal}_jerk_rms"] = np.nan
               kpi_row[f"{signal}_jerk_std"] = np.nan
               kpi_row[f"{signal}_jerk_max"] = np.nan    
                     
     return kpi_row