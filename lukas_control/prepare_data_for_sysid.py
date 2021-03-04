import glob
import os

import numpy as np


# get the most recent log file
list_of_files = glob.glob('../files/logs/*.npy')
latest_file = max(list_of_files, key=os.path.getctime)

data = np.load(latest_file)
states = data['states'][0, [2, 5], :] # get z_pos and z_vel data
control_inputs = data['control_inputs'][0, :]
timestamps = data['timestamps']

# write .csv files for Matlab
np.savetxt("sysid_data/timestamps.csv", timestamps, delimiter=",")
np.savetxt("sysid_data/states.csv", states, delimiter=",")
np.savetxt("sysid_data/control_inputs.csv", control_inputs, delimiter=",")

# Now: run sysid_lti_dt.m in Matlab
