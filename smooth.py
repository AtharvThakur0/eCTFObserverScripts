import argparse
import numpy as np
from scipy.signal import savgol_filter

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="File to read")
    args = parser.parse_args()
    traces=np.load(args.file)
    traces=smooth(traces)
    np.save(args.file.replace('.npy', '_smooth.npy'), traces)

def seconddiffsmooth(traces, threshold=0.3):
    diffs=np.abs(np.concatenate((np.diff(traces, n=2, axis=traces.ndim-1), np.zeros((traces.shape[0], 2))), axis=1)) # 2nd diff
    spikes_starts = diffs > threshold*np.max(diffs) # indicies where large 2nd diff occurs
    spikes = np.concatenate((np.zeros((spikes_starts.shape[0],1), dtype=bool), spikes_starts[:,:-1]), axis=1) # add 0 at the beginning to make up for diff
    traces[spikes] = traces[spikes_starts]
    return traces

def firstdiffsmooth(traces, threshold=1):
    cutoff_diff = np.std(traces,axis=1)*threshold
    fdiff = np.abs(np.diff(traces,axis=1))
    candidates = (fdiff.T > cutoff_diff).T
    leftsfdiff = np.concatenate((np.zeros((candidates.shape[0],1), dtype=np.uint8), fdiff[:,:-1]), axis=1)
    spikes = (leftsfdiff.T > cutoff_diff).T
    spikes = candidates & spikes
    spikes = np.concatenate((spikes, np.zeros((candidates.shape[0],1), dtype=np.bool)), axis=1) # add 0 at the beginning to make up for diff
    traces[spikes] = np.concatenate((np.zeros((candidates.shape[0],1), dtype=traces.dtype), traces[:,:-1]), axis=1)[spikes]
    return traces

def savgol(traces, window_length=6, polyorder=3):
    return savgol_filter(traces, window_length=window_length, polyorder=polyorder, axis=1)

def smooth(traces):
    traces = percentOutlierSmooth(traces)
    traces = savgol(traces)
    return traces
def percentOutlierSmooth(traces, threshold=99.96):
    upper = np.percentile(traces, threshold,axis=1, keepdims=True)
    lower = np.percentile(traces, 100-threshold,axis=1, keepdims=True)
    #print(f"upper: {upper}")
    #print(f"lower: {lower}")
    #outliers = (traces > upper) | (traces < lower)
    #traces[outliers] = shift(traces, 1)[outliers] # replace outliers with left neighbor;
    traces[traces > upper] -= 60;
    traces[traces < lower] += 60;
    return traces
    use = [-3,-2,-1,1,2,3]
    local_vals = np.array([shift(traces, i) for i in use])
    local_discrep = np.array([local_vals[i] - traces for i in range(len(use))])
    #print(f"local_discrep size: {local_discrep.shape}")
    best_vals = np.argmin(np.abs(local_discrep), axis=0)
    #print(f"best_vals: {best_vals}")
    #print(f"best_vals size: {best_vals.shape}")
    for i in range(local_discrep.shape[1]):
        for j in range(local_discrep.shape[2]):
            if outliers[i,j]:
                best_val = best_vals[i,j]
                traces[i,j] = local_vals[best_val][i,j]
    
    #np.concatenate((np.zeros((traces.shape[0],1), dtype=traces.dtype), traces[:,:-1]), axis=1)[outliers]
    return traces

#horizontal shift of 2d array
def shift(traces, shift_amount):
    if shift_amount > 0:
        return np.concatenate((np.zeros((traces.shape[0], shift_amount), dtype=traces.dtype), traces[:,:-shift_amount]), axis=1)
    elif shift_amount < 0:
        return np.concatenate((traces[:,:shift_amount], np.zeros((traces.shape[0], -shift_amount), dtype=traces.dtype)), axis=1)
    else:
        return traces


def getTwoLineFits(traces, min_line_gap=40):
    mabserror = np.zeros((traces.shape[0],256), dtype=np.float32)
    for i in range(256):
        mabserror[:,i] = np.mean(np.abs(traces - i), axis=1)

    minidx = np.argmin(mabserror, axis=1)
    rows = np.arange(traces.shape[0])
    mabserror2 = mabserror.copy()
    candidate_levels = np.arange(mabserror.shape[1])
    too_close = np.abs(candidate_levels[None, :] - minidx[:, None]) < min_line_gap
    mabserror2[too_close] = np.inf
    secondminidx = np.argmin(mabserror2, axis=1)
    return minidx, secondminidx

def twolinefitsmooth(traces, threshold=1, min_line_gap=40):
    
    minidx, secondminidx = getTwoLineFits(traces, min_line_gap)
    print(f"minidx: {minidx}")
    print(f"secondminidx: {secondminidx}")

    # Estimate each line's normal spread from points closest to that line.
    discrep1 = np.abs(traces - minidx[:, None])
    discrep2 = np.abs(traces - secondminidx[:, None])
    print(f"discrep1: {discrep1}")
    print(f"discrep2: {discrep2}")
    
    # min difference from a characteristic line (shape of traces)
    discrep = np.min(np.stack((discrep1, discrep2), axis=2), axis=2)
    print(f"discrep: {discrep}")
    spikes = (discrep.T > threshold*np.mean(discrep, axis=1)).T # indicies where large difference occurs
    
    left_values = np.concatenate((traces[:, :1], traces[:, :-1]), axis=1)
    traces[spikes] = left_values[spikes]
    return traces

if __name__ == "__main__":
    main()