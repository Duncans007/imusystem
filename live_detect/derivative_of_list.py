
def derivative_of_list(analysis_list, time_list):

    sig = [0]
    for i in range(1, len(analysis_list)):
        sig.append(
            (analysis_list[i] - analysis_list[i - 1]) / (time_list[i] - time_list[i - 1])
        )

    return sig