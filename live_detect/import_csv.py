# Input: CSV filename for single set of data
# Output: Dictionary with keys as file headers and values as lists, number of blank lines
# Unnamed columns get named "unnamed1", "unnamed2", etc. with no spaces

def import_csv(filename):

    ##### Initialize storage and output variables
    data_list = []
    data_dict = {}
    import_file = True
    blank_line_counter = 0

    ##### Define split character as tab (if text file) or comma (if csv)
    if filename[-4:] == ".csv":
        split_char = ","
    elif filename[-4:] == ".txt":
        split_char = "\t"
    else:
        import_file = False


    if import_file:
        ##### Open file
        with open(filename, "r") as f:

            # read just the first line of the file to grab headers
            for row in f:
                # make lowercase, remove line end character, split string into list
                headers = row.lower().strip("\n").split(split_char)
                data_list = [[] for _ in headers]
                break

            # continue reading all data into numerically subscriptable list
            for row in f:
                # remove line end and split into list
                data = row.strip("\n").replace(",)", "").replace("(", "").replace('"', "").split(split_char)

                # handling for blank lines
                #if not "" in data:
                # turn string into float and store in list
                for i in range(0, len(data)):
                    try:
                        data_list[i].append(float(data[i]))
                    except ValueError:
                        if data[i] == "":
                            data_list[i].append(0)
                        else:
                            data_list[i].append(data[i])
                #else:
                #    blank_line_counter = blank_line_counter + 1

        ##### Output

        # turn lists into dictionary for easy subscripting
        blank_header_counter = 1
        for i in range(0, len(headers)):
            if headers[i]:
                data_dict[" ".join(headers[i].split("/"))] = data_list[i]
            else:
                data_dict["unnamed"+str(blank_header_counter)] = data_list[i]
                blank_header_counter = blank_header_counter + 1

    return data_dict, blank_line_counter


##### testing
if __name__ == "__main__":
    outp = import_csv("../data/early_trip_1.csv")
    print(outp)