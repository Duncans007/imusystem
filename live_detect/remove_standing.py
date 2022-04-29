from statistics import stdev

# Use window size of 1 second (50 frames @ 50 hz)
# Use threshold value of 3 stdev for BOTH shanks simultaneously
# After threshold is passed, include 2 windows prior

def remove_standing_imudict(input_dict, window_samples=50, std_threshold=3, prior_windows_included=1):
    dictionary = input_dict.copy()
    rshank = dictionary["angle rshank"]
    #lshank = dictionary["angle lshank"]

    ##### Analyze shank angles from beginning to determine when they exceed X standard deviations
    for i in range(window_samples, len(rshank)):
        standevr = stdev(rshank[i-window_samples:i])
        #standevl = stdev(lshank[i-window_samples:i])

        #if standevl >= std_threshold and standevr >= std_threshold:
        #    break
        if standevr >= std_threshold:
            break

    ##### Analyze shank angles FROM THE END to cut trailing standing data
    for j in range(len(rshank)-101, window_samples, -1):
        standevr = stdev(rshank[j-window_samples:j])
        #standevl = stdev(lshank[j-window_samples:j])

        #if standevl >= std_threshold and standevr >= std_threshold:
        #    break
        if standevr >= std_threshold:
            break

    ##### Include some data prior to detection
    i = max(0, i-int(window_samples*prior_windows_included))
    j = min(len(rshank), j+int(window_samples*(prior_windows_included-1)))
    #print(len(rshank),i,j)

    ##### Cut section of standing data from ALL entries in the dictionary.
    for key in dictionary:
        dictionary[key] = dictionary[key][i:j]

    dictionary["time"] = [_ - dictionary['time'][0] for _ in dictionary["time"]]

    return dictionary









# same function, but input is list of lists w/ first list used as analysis var
def remove_standing_arrset(list_in, window_samples=50, std_threshold=3, prior_windows_included=1, analysis_pos=0):

    list_analysis = list_in[analysis_pos]

    ##### Analyze shank angles from beginning to determine when they exceed X standard deviations
    for i in range(window_samples, len(list_analysis)):
        standevr = stdev(list_analysis[i-window_samples:i])

        if standevr >= std_threshold:
            break

    ##### Analyze shank angles FROM THE END to cut trailing standing data
    for j in range(len(list_analysis)-101, window_samples, -1):
        standevr = stdev(list_analysis[j-window_samples:j])
        #standevl = stdev(lshank[j-window_samples:j])

        #if standevl >= std_threshold and standevr >= std_threshold:
        #    break
        if standevr >= std_threshold:
            break

    ##### Include some data prior to detection
    i = max(0, i-int(window_samples*prior_windows_included))
    j = min(len(list_analysis), j+int(window_samples*(prior_windows_included-1)))
    #print(len(rshank),i,j)

    ##### Cut section of standing data from ALL entries in the dictionary.
    list_out = []
    for _ in list_in:
        list_out.append(_[i:j])


    # return the new list and the number of frames shifted from teh beginning
    return list_out, i