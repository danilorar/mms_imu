from io_utils import kalman_filter
from key_metrics import noise_metrics

# parameters
batchRun = False
q = 0.001
r = 0.1

# clean and filter
if batchRun== True:
     for maneuver in ["acceleration", "braking", "cornering"]:  # for each 3 maneuver and setting loop over trial and apply kalman filter to raw data, save to csv in filtered folder 
          for setting in ["soft", "medium", "hard"]: 
               for trial in range(1, 4): 
                    csv_input = f"data/raw/{maneuver}/{setting}/{maneuver}_{setting}_trial{trial:02d}.csv"
                    kalman_filter(csv_input, q=q, r=r, save=True)
                    
elif batchRun== False:
     # example for one file
     csv_input = "data/raw/braking/soft/braking_soft_trial01.csv"
     kalman_filter(csv_input, q=q, r=r, save=True)


# key metrics 

