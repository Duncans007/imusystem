# ====================================================================
# Class: Store and process one or multiple subjects' worth of data into source signal for DTW
# ====================================================================
import csv
import statistics

import numpy as np
from scipy.signal import resample


class SrcSignal:
    def __init__(self, std_bound_multiplier=1):
        # initialize constants
        self.std_bound_multiplier = std_bound_multiplier

        # initialize storage and temp storage arrays
        self.stored_signals = []
        self.stored_resampled_signals = []
        self.number_subjects = 0

        # initialize output arrays
        self.src_signal = []
        self.src_std = []



    def add(self, sig, cut_frames):

        # increment number of subjects
        self.number_subjects = self.number_subjects + 1

        # store as many full gait cycles (HS-HS) as possible
        if len(cut_frames) > 1:
            for i in range(1, len(cut_frames)):
                self.stored_signals.append(
                    sig[ cut_frames[i - 1]:cut_frames[i] ]
                )
        else:
            # raise error if not enough heel strikes to form source signal
            raise Exception("Can't determine DTW source: Not enough heel strikes")

        self.regenerate_src()



    def regenerate_src(self):
        # resample all cut HS-HS sections into X frames so they can be averaged
        self.stored_resampled_signals = []
        for i in range(0, len(self.stored_signals)):
            self.stored_resampled_signals.append(
                resample(
                    self.stored_signals[i], # input signals one-by-one
                    round(np.mean([len(x) for x in self.stored_signals])/2) # average length of signals to resample
                )
            )


        # find the average and standard deviation of the cut portions to make source signal
        self.src_signal = []
        self.src_std = []
        for i in range(0, len(self.stored_resampled_signals[0])):
            vars = [signal[i] for signal in self.stored_resampled_signals]
            self.src_signal.append(np.mean(vars))
            try:
                self.src_std.append(statistics.stdev(vars))
            except statistics.StatisticsError:
                self.src_std.append(0)



    def save_src(self, filename):
        # what is the minimal amount of different variables that can be saved?
        # store number of subjects, std_bound_mult, and stored_signals
        # write std_bound on line 1
        # write number of subjects on line 2
        # write 1 stored signal per line after that
        with open(filename+".csv", "w+", newline='') as f:
            csvwriter = csv.writer(f, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            csvwriter.writerow([self.std_bound_multiplier])
            csvwriter.writerow([self.number_subjects])
            for r in self.stored_signals:
                csvwriter.writerow(r)



    def load_src(self, filename):
        with open(filename+".csv", "r", newline='') as f:
            csvreader = csv.reader(f, delimiter=' ', quotechar='|')

            # reset object and read std_bounds from file
            for row in csvreader:
                self.__init__(std_bound_multiplier=row[0])
                break
            # read number of subjects from file
            for row in csvreader:
                self.number_subjects = row[0]
                break
            # iterate over the rest of the file and save all signals to array
            for row in csvreader:
                self.stored_signals.append(row)

        self.regenerate_src()