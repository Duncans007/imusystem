# ====================================================================
# Function: Get list of frames where heel strike occurs
# ====================================================================
def get_hs_frames(gait, perturbation_frame=None):

    # if no perturbation frame is provided, analyze the entire signal
    if not perturbation_frame:
        perturbation_frame = len(gait)

    # get a list of frames where heel strike occurs
    # do not count any heel strikes after the perturbation detection
    hs_frames = []  # list of frame numbers
    for i in range(1, perturbation_frame):
        if gait[i] == 0 and gait[i - 1] != 0:
            hs_frames.append(i)

    return hs_frames