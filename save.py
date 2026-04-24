import numpy as np
a = np.load('temporal_traces_byte_idx_0_smooth.npy')
np.savetxt('0s2.csv', a[0,:],delimiter=',')