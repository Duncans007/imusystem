# Function to get the signal for analysis.
# Does all necessary computations for either of:
#   1) the lower back angular acceleration
#   2) the hip-heel angular acceleration

import numpy as np
from derivative_of_list import *
from filter_butter import *

def get_analysis_signal(analysis_signal_type, file_dict, l_hh=1):
    # analysis_signal_type = "hipheel" or "lowback"

    if analysis_signal_type == "lowback":

        analysis_signal_unfilt = file_dict["gy z lowback"].copy()
        time_signal = file_dict["time"]

        # Filter some signals w/ butterworth filter
        analysis_signal_filt = filter_butter(analysis_signal_unfilt, 50, 3)

        # Differentiate
        analysis_signal_diff = derivative_of_list(analysis_signal_filt, time_signal)

        return analysis_signal_diff

    elif analysis_signal_type == "hipheelac":

        # get two signals for processing
        thigh_acc_unfilt = file_dict["ac x rthigh"].copy()
        heel_acc_unfilt = file_dict["ac x rheel"].copy()
        ang_thigh = file_dict["angle rthigh"]
        ang_heel = file_dict["angle rheel"]

        # filter signals
        thigh_acc_filt = filter_butter(thigh_acc_unfilt, 50, 3)
        heel_acc_filt = filter_butter(heel_acc_unfilt, 50, 3)

        # remove gravitational component
        # subtract 9.81
        thigh_acc_corr_unfilt = [acc - (-9.81 * np.sin(np.radians(ang))) for acc, ang in zip(thigh_acc_unfilt, ang_thigh)]
        heel_acc_corr_unfilt = [acc - (-9.81 * np.sin(np.radians(ang))) for acc, ang in zip(heel_acc_unfilt, ang_heel)]

        thigh_acc_corr_filt = [acc - (-9.81 * np.sin(np.radians(ang))) for acc, ang in zip(thigh_acc_filt, ang_thigh)]
        heel_acc_corr_filt = [acc - (-9.81 * np.sin(np.radians(ang))) for acc, ang in zip(heel_acc_filt, ang_heel)]

        # compute theta_dd_hh
        theta_dd_hh_unfilt = [(x - y) / l_hh for x, y in zip(thigh_acc_corr_unfilt, heel_acc_corr_unfilt)]
        #theta_dd_hh_filt = [(x - y) / l_hh for x, y in zip(thigh_acc_corr_filt, heel_acc_corr_filt)]
        theta_dd_hh_filt = filter_butter(theta_dd_hh_unfilt, 50, 3)

        out = np.degrees(theta_dd_hh_filt - np.mean(theta_dd_hh_filt[1:10]))

        return list(out)

    elif analysis_signal_type == "hipheelgy":

        # get signals for processing
        thigh_gy_unfilt = file_dict["gy z rthigh"].copy()
        shank_gy_unfilt = file_dict["gy z rheel"].copy()

        # filter signals
        thigh_gy_filt = filter_butter(thigh_gy_unfilt, 50, 3)
        shank_gy_filt = filter_butter(shank_gy_unfilt, 50, 3)

        thigh_acc_filt = derivative_of_list(thigh_gy_filt, file_dict["time"])
        shank_acc_filt = derivative_of_list(shank_gy_filt, file_dict["time"])

        # compute theta_dd_hh
        theta_dd_hh = [np.mean([_1,_2])*l_hh for _1, _2 in zip(shank_acc_filt, thigh_acc_filt)]

        return theta_dd_hh

    elif analysis_signal_type == "tif":

        # get signals for processing
        lowback_theta_dd = get_analysis_signal("lowback", file_dict)
        heel_x_dd = file_dict["ac x rheel"]

        ang_heel = file_dict["angle rheel"]

        # remove gravitational component
        # subtract 9.81
        heel_acc_corr_unfilt = [acc - (-9.81 * np.sin(np.radians(ang))) for acc, ang in zip(heel_x_dd, ang_heel)]

        # compute tif
        tif_unfilt = [(x / y) for x, y in zip(lowback_theta_dd, heel_acc_corr_unfilt)]
        # theta_dd_hh_filt = [(x - y) / l_hh for x, y in zip(thigh_acc_corr_filt, heel_acc_corr_filt)]
        tif_filt = filter_butter(tif_unfilt, 50, 3)

        out = np.degrees(tif_filt - np.mean(tif_filt[1:10]))

        return list(out)

