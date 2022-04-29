from store_SrcSignal import *
from scipy import signal


class detect_dtw():
    # This class is a live object for DTW trip detection.
    # As detailed by Chadi Ellouzi in his 2021 Master's Thesis.
    #
    # This object contains only live functions: that is, it takes single values at a time.
    # The included "if __name__ == __main__" loop below shows how to process an existing test.
    #
    # Initialization:
    # Create object using detector=detect_dtw(source_signal_obj, detect_thresh, store_frames, f_s)
    #   Arguments:
    #       source_signal_obj (object SrcSignal) is the custom class containing source signal data
    #       detect_thresh (int, default=300) is the loss value to detect trip for boolean output
    #       store_frames (int, default=20) is the frame length of the stored sliding window
    #       f_s (int, default=45) is the sampling rate of the data
    #
    # Live usage:
    # Pass raw lower back angular velocities and the corresponding timestamp with:
    #   out = detector.update(gy_val, timestamp, gait_val, output_type)
    #   Arguments:
    #       gait_val (integer) of 1 or 0 for swing or stance phase respectively
    #       output_type (string, default="boolean") can be "boolean" or "value"
    #           if "boolean", function will return 1/0 for trip/no trip
    #           if "value", function will return raw RMS value
    #           if any other value, function will return both in order bool, val


    def __init__(self, source_signal_obj, detect_thresh=300, store_frames=20, f_s=45, derivative=True):

        # argument inputs
        self.src = source_signal_obj.src_signal
        self.gy_store_frames = store_frames
        self.detect_thresh = detect_thresh
        self.derivative = derivative


        # set up window constraints (max warp length)
        self.q1lim = 1
        self.q2lim = 5


        # Store warped signals over time
        self.warped_signals_all = [self.src]
        self.warped_signals_max_count = 20


        # variable storage arrays
        self.time_store_gy = [0 for _ in range(0, self.gy_store_frames)]
        self.window_store_gy = [0 for _ in range(0, self.gy_store_frames)]
        self.window_gy_d = [0 for _ in range(0, self.gy_store_frames)]


        # butterworth filter setup
        butterworth_order = 3
        butterworth_cutoff = 3  # Hz

        self.b_gy, self.a_gy = signal.butter(butterworth_order, butterworth_cutoff, fs=f_s)
        self.z_gy = signal.lfilter_zi(self.b_gy, self.a_gy) * 0


        # setup all values that get regenerated on heelstrike
        self.reset()



    def update_warped_signals_storage(self, new_sig):
        # Use to update the warped signals array (instead of directly)
        # Done this way to cap the number of steps stored on longer tests and prevent source sig from being removed

        self.warped_signals_all.append(new_sig)

        if len(self.warped_signals_all) > self.warped_signals_max_count:
            self.warped_signals_all.pop(1)



    def reset(self):
        # called on heel strike, reset all frame variables

        # track currently warping signal
        self.current_save = []
        self.last_val = 0
        self.warp_loss = [0]

        # counters
        self.q1 = 0
        self.q2 = 0
        self.counter_src = 0

        # store last gait value
        self.gait_val_last = 0



    def update(self, gy_val, timestamp, gait_val, output_type="value"):
        # main function to insert a new value for detection
        # input values for lower back gyroscope, timestamp, and 1/0 for swing/stance phase
        # output_type can be "value" or "boolean"


        # get time since last run, update time
        self.time_store_gy.append(timestamp)
        self.time_store_gy.pop(0)


        # filter gyroscope values as they come in, save to sliding window
        gy_filt, self.z_gy = signal.lfilter(self.b_gy, self.a_gy, [gy_val], zi=self.z_gy)
        self.window_store_gy.append(gy_filt[0])
        self.window_store_gy.pop(0)


        # compute change in velocity over change in time
        if self.derivative:
            acc_val = (self.window_store_gy[-1] - self.window_store_gy[-2]) / (self.time_store_gy[-1] - self.time_store_gy[-2])
        else:
            acc_val = self.window_store_gy[-1]

        # warp filtered acceleration
        out_val = self.warp(acc_val)

        # get T/F value for detection
        out_bool = int(out_val > self.detect_thresh)


        # test if current frame is a heel strike (reset the source counter if it is)
        if gait_val == 0 and self.gait_val_last != 0:
            self.update_warped_signals_storage(self.current_save)
            self.reset()
        self.gait_val_last = gait_val


        # check output type and return relevant values
        if output_type == "value":
            return out_val
        elif output_type == "boolean":
            return out_bool
        else:
            return out_val, out_bool



    def warp(self, val):
        # DTW function. determines how to warp value based on src signal and most recent input.
        # Called from within object - update() function.
        # Outputs minimum loss value.

        # compute losses
        loss_var_same = abs(val - self.src[ min(len(self.src)-1, self.counter_src + 0) ])
        loss_var_next = abs(val - self.src[ min(len(self.src)-1, self.counter_src + 1) ])
        loss_var_last = abs(val - self.src[ min(len(self.src)-1, self.counter_src - 1) ])


        # run appropriate code based on minimum loss

        if loss_var_last <= loss_var_same and loss_var_last <= loss_var_next and self.q2 < self.q2lim:
            # LAST:: "delete" - no save pt. no increment src. do save loss.

            # handle counters
            self.q2 = self.q2 + 1

            # add to loss array
            self.warp_loss.append(loss_var_last)


        elif loss_var_next <= loss_var_last and loss_var_next <= loss_var_same and self.q1 < self.q1lim:
            # NEXT:: "insert" - save pt, save loss, increment counter, run again

            # handle counters
            self.q1 = self.q1 + 1
            self.counter_src = self.counter_src + 1

            # add value to current save
            self.current_save.append(val)

            # add to loss array
            self.warp_loss.append(loss_var_next)

            # rerun warp function with *same exact* value
            self.warp(val)


        elif (loss_var_same <= loss_var_next and loss_var_same <= loss_var_last) or self.q1 >= self.q1lim or self.q2 >= self.q2lim:
            # CURRENT:: "No Warp" - save pt

            # handle counters
            self.q1 = 0
            self.q2 = 0
            self.counter_src = self.counter_src + 1

            # add value to current save
            self.current_save.append(val)

            # add to loss array
            self.warp_loss.append(loss_var_same)



        # return last item added to the loss array.
        return self.warp_loss[-1]



if __name__ == "__main__":

    ##### IMPORT AND SETUP

    import numpy as np

    from src.data_processing.remove_standing import *
    from src.file_io.import_csv import *
    from src.graph_output.graph_2d import *
    from src.graph_output.html_output import *

    # import source signal from file, initialize detector with src
    source_signal_obj = SrcSignal()
    source_signal_obj.load_src("../dtw_calibration_src")

    # save and filter filenames for analysis
    filenames = ['../data/chadi_trip_full/algDump_chadi_2_12_lateNorm1.txt',
                 '../data/chadi_trip_full/algDump_chadi_2_12_lateSlow1.txt',
                 '../data/chadi_trip_full/algDumpChadi_earlyTrip_1.txt',
                 '../data/chadi_trip_full/algDumpChadi_earlyTrip_3.txt']


    ##### PROCESS

    # Iterate through filenames and store output graphs in g array
    g = []
    for name in filenames:

        # import and process data
        file_dict, unprocessed_lines = import_csv(name)
        file_dict = remove_standing_imudict(file_dict)

        # get trip indicator from DTW over time
        val_arr = []
        detector = detect_dtw(source_signal_obj)
        for _ in range(1, len(file_dict["gy z lowback"])):
            val = detector.update(file_dict["gy z lowback"][_], file_dict["time"][_], file_dict["gaitstager"][_], output_type="value")
            val_arr.append( val )


        ##### OUTPUT


        g.append("<hr><hr>")
        g.append(f"<h2>{name}</h2>")

        # graph all signals starting from heel strike, warped
        g.append(
            graph_2d(
            "All Warped Signals Starting from HS",
            detector.warped_signals_all,
            )
        )

        # graph output value alongside loadcell reading
        g.append(
            graph_2d(
                "Output Value (loss)",
                [val_arr, np.abs(file_dict["loadcell"].copy())*(max(val_arr))/(max(np.abs(file_dict["loadcell"])))],
            )
        )



    ##### SAVE

    html_output(
        f"dtw_live_demo",
        g,
        output_dir="../../data_processed"
    )
