
from scipy.signal import lfilter, lfilter_zi


class detect_rms():
    # This class is a live object for RMS trip detection.
    # As detailed by Chadi Ellouzi in his 2021 Master's Thesis.
    #
    # This object contains only live functions: that is, it takes single values at a time.
    # The included "if __name__ == __main__" loop below shows how to process an existing test.
    #
    # Initialization:
    # Create object using detector=detect_rms(window_frames_rms, window_frames_butter, rms_thresh, f_s)
    #   Arguments:
    #       window_frames_rms (integer, default=10) is the number of frames used to compute most recent RMS value.
    #       window_frames_butter (integer, default=50) is the filtered sliding window stored in the object.
    #       rms_thresh (integer, default=300) is the value at which boolean detection triggers.
    #       f_s (integer, default=45) is the sampling frequency in Hz
    #
    # Live usage:
    # Pass raw lower back angular velocities and the corresponding timestamp with:
    #   out = detector.update(gy_val, timestamp, output_type)
    #   Arguments:
    #       output_type (string, default="boolean") can be "boolean" or "value"
    #       if "boolean", function will return 1/0 for trip/no trip
    #       if "value", function will return raw RMS value
    #       if any other value, function will return both in order bool, val

    def __init__(self, derivative=True, window_frames_rms=10, window_frames_butter=50, rms_thresh=300, f_s=45, filter=True):
        self.window_frames_gy = window_frames_butter
        self.window_frames_rms = window_frames_rms
        self.rms_thresh = rms_thresh
        self.derivative = derivative

        self.window_store_gy = [0 for _ in range(0, self.window_frames_gy)]
        self.time_store_gy = [0 for _ in range(0, self.window_frames_gy)]


        # butterworth filter setup
        self.filter = filter
        if self.filter:
            butterworth_order = 3
            butterworth_cutoff = 3 #Hz

            self.b_gy, self.a_gy = butter(butterworth_order, butterworth_cutoff, fs=f_s)
            self.z_gy = lfilter_zi(self.b_gy, self.a_gy) * 0



    # for live implementation, remove timestamp argument and get time instead.
    def update(self, gy_val, timestamp, output_type="boolean"):
        # main function to insert a new value for detection
        # output => ["value" or "boolean" or "both"]

        # record input timestamps
        self.time_store_gy.append(timestamp)
        self.time_store_gy.pop(0)

        # filter the input value
        if self.filter:
            gy_filt, self.z_gy = lfilter(self.b_gy, self.a_gy, [gy_val], zi=self.z_gy)
            self.window_store_gy.append(gy_filt[0])
        else:
            self.window_store_gy.append(gy_val)
        self.window_store_gy.pop(0)

        # take derivative of gyroscope values
        if self.derivative:
            window_rms = np.diff(self.window_store_gy[-self.window_frames_rms:]) / np.diff(self.time_store_gy[-self.window_frames_rms:])
        else:
            window_rms = self.window_store_gy[-self.window_frames_rms:]

        # perform detection
        rms_val = np.sqrt(np.mean(([_**2 for _ in window_rms])))
        out_val = int(rms_val > self.rms_thresh)

        if output_type == "value":
            return rms_val
        elif output_type == "boolean":
            return out_val
        else:
            return out_val, rms_val






if __name__ == "__main__":


    ##### IMPORTING

    from src.file_io.import_csv import *
    from src.data_processing.get_analysis_signal import *
    from src.graph_output.html_output import *
    from src.graph_output.graph_2d import *



    ##### INITIALIZING

    # initialize detector object
    detector = detect_rms()

    # initialize list to store output graphs
    g = []

    # define files to process
    filenames = ['../data/chadi_trip_full/algDump_chadi_2_12_lateNorm1.txt',
                 '../data/chadi_trip_full/algDump_chadi_2_12_lateSlow1.txt',
                 '../data/chadi_trip_full/algDumpChadi_earlyTrip_1.txt',
                 '../data/chadi_trip_full/algDumpChadi_earlyTrip_3.txt']



    ##### PROCESSING

    # iterate through each file given
    for name in filenames:

        # import file data
        file_dict, unprocessed_lines = import_csv(name)

        # get indicator over time by adding and recording all values from array
        out_arr = [detector.update(_1, _2, output_type="value") for _1, _2 in zip(file_dict["gy z lowback"], file_dict["time"])]

        # Graph complete RMS output against load cell measurements
        g.append(graph_2d(
            f"RMS live detection compared to post-processed load cell. ({name})",
            [out_arr, np.abs(file_dict["loadcell"].copy())],
            highlight_y=[detector.rms_thresh]
        ))
        g.append("<hr><hr>")



    ##### SAVING

    # save output files to single html
    html_output(
        "rms_live_demo",
        g,
        output_dir="../../data_processed",
        top_matter=f"Live-tested RMS detection against load cell.<br>Butter Window={detector.window_frames_gy}<br>RMS Window={detector.window_frames_rms}"
    )