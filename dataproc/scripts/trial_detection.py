# functions to detect trials based on GPS speed
import pandas as pd
import numpy as np

# detect cornering trials based on speed thereshold and duration
def detect_cornering(gps, speed_min=5, min_duration=10, padding=5):
    moving = gps["v_kmh"] > speed_min

    runs = []
    start = None

    for i in range(len(gps)):
        # trial starts
        if moving.iloc[i] and start is None:
            start = gps["t"].iloc[i]

        # trial ends
        elif not moving.iloc[i] and start is not None:
            end = gps["t"].iloc[i]

            if end - start > min_duration:
                runs.append((max(0, start - padding), end + padding))

            start = None

    # case where log ends while still moving
    if start is not None:
        end = gps["t"].iloc[-1]

        if end - start > min_duration:
            runs.append((max(0, start - padding), end + padding))

    return runs

# detect acceleration/braking trials based on speed peaks and duration 
def detect_acc_brake(gps, high_speed=60, low_speed=5, padding=5): 
    high = gps["v_kmh"] > high_speed

    runs = []
    
    for i in range(len(gps)): 
        if not high.iloc[i]:
            continue
        
        # avoid detection of same speed peak many times 
        if runs and gps["t"].iloc[i] < runs[-1][1]:
            continue
        
        # go backwards until speed is low
        start_i = i
        while start_i > 0  and gps["v_kmh"].iloc[start_i] > low_speed:
            start_i -= 1
            
        # go forward until speed is low
        end_i = i
        while end_i < len(gps) - 1 and gps["v_kmh"].iloc[end_i] > low_speed:
            end_i += 1
            
        start = max(0, gps["t"].iloc[start_i] - padding)
        end = gps["t"].iloc[end_i] + padding
        
        runs.append((start, end))
        
    return runs
        
        
# wrapper functions based on maneuver: 
def detect_trials(imu, maneuver): 
    gps = imu[imu["sensor"] == "gps"].copy()
    
    if maneuver == "cornering": 
        return detect_cornering(gps)
    
    if maneuver in ["acc_brake", "acceleration", "braking", "acc", "brk"]: 
        return detect_acc_brake(gps=gps)

    raise ValueError(f"error: {maneuver}")        


# split trials into dataframe for post analysis 
def split_trials(data, start, end): 
    return data[(data["t"] >= start) & (data["t"] <= end)].copy()


# track geometry lenght and radius for carmaker
def path_length(gps):
    dx = np.diff(gps["x"].values)
    dy = np.diff(gps["y"].values)

    return np.sqrt(dx**2 + dy**2).sum()

def path_radius(gps): 
    x = gps["x"].values
    y = gps["y"].values
    
    xc = (x.max() + x.min()) / 2
    yc = (y.max() + y.min()) / 2
    radius = np.sqrt((x - xc)**2 + (y - yc)**2)
    
    return radius.mean()

if __name__ == "__main__":
    
    # track geometry
    data = pd.read_csv("data/filtered/acc_brake/soft/acc_brake_soft_trial01.csv")
    gps = data[data["sensor"] == "gps"].copy()
    gps = gps[gps["v_kmh"] > 5]
    
    # radius
    R = path_radius(gps)
    print(f"Track radius: {R:.2f} m")
    
    # length
    l = path_length(gps)
    print(f"Track length: {l:.2f} m")
    
    # plot 
    import matplotlib.pyplot as plt
    
    plt.figure()
    plt.plot(gps["x"], gps["y"])
    plt.xlabel("x (m)");plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid()
    plt.show()  
    

            
    
    