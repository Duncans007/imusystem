from scipy.signal import butter, filtfilt

# returns nothing. modifies input dictionary in-place
# applies butterworth filter with order and fc (freq.cutoff) to signals filter_keys in input_dict with fs (freq.sample)

# fs: sampling frequency of signal
# fc: cutoff frequency of filter

def filter_butter(input_arr, fs, fc, order=4):

    # Compute normalized (nyquist) frequency
    w = fc / (fs / 2)

    # create coefficients
    b, a = butter(order, w)

    filtered = filtfilt(b, a, input_arr)

    return filtered
