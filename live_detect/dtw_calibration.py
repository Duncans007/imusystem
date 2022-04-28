# Calibration for Dynamic Time Warping
# Process data from most recent test and create dtw source signal
# save dtw source signal to a file so it can be used repeatedly during tests

import sys
from store_SrcSignal import *
from import_csv import *
from remove_standing import *
from get_hs_frames import *
from get_analysis_signal import *

if __name__ == "__main__":

    save_filename = "dtw_calibration_save"
    load_filename = "../algDump.txt"
    strip_chars = [",", "'", '"']

    try:

        # get_save_filename()
        # sys.argv is run arguments. 0 is filename.
        # look for "save=" and "load="
        if len(sys.argv) > 1:
            for arg in sys.argv[1:]:
                if arg[:5].lower() == "save=":
                    save_filename = arg[5:].split(".")[0]
                    for c in strip_chars:
                        save_filename = save_filename.strip(c)
                elif arg[:5].lower() == "load=":
                    load_filename = arg[5:]
                    for c in strip_chars:
                        load_filename = load_filename.strip(c)



        #import_data()
        file_dict, _ = import_csv(load_filename)

        #cut_data()
        file_dict = remove_standing_imudict(file_dict)

        #detect_heelstrikes()
        hs_frames = get_hs_frames(file_dict["gaitstager"])[1:-2]

        # get analysis signal
        analysis_sig = get_analysis_signal("lowback", file_dict)

        ## assume NO perturbation (so all heelstrikes can be used)
        #create_object()
        dtw_obj = SrcSignal()

        dtw_obj.add(analysis_sig, hs_frames)
        dtw_obj.save_src(save_filename)

        print("Successfully saved calibration file")

    except Exception as e:
        print(f"Failed to complete: {type(e)} {e}")
