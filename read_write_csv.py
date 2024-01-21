import csv
import numpy as np


def get_filename(kp, kd, start_com, target_com, singularity_resolve_method, singularity_resolve_parameter):
    if singularity_resolve_parameter is None:
        singularity_resolve_parameter = "no_param"
    return str(kp) + "_" + str(kd) + "_" + str(start_com) + "_" + str(target_com) + "_" + singularity_resolve_method + "_" + str(singularity_resolve_parameter) + ".csv"


def save_parameters(iter, kp, kd, start_com, target_com, progress_com_errs, singularity_resolve_method, singularity_resolve_parameter=None):
    filename = get_filename(kp, kd, start_com, target_com, singularity_resolve_method, singularity_resolve_parameter)
    
    row = [iter] + progress_com_errs.flatten().tolist()

    with open(filename, 'a', newline='') as file:  # Use newline='' for consistent line endings
        writer = csv.writer(file)
        if file.tell() == 0:  # Check if the file is empty
            fields = ["iter", "err_x", "err_y", "err_z"]
            writer.writerow(fields)
        writer.writerow(row)


if __name__ == "__main__":
    arr1 = np.array([[7, 8, 9]])
    arr2 = np.array([[10, 11, 12]])
    save_parameters(4, 50, 4, [1, 2, 3], [4, 5, 6], arr1, "damped_least_squares", 5)
    save_parameters(5, 50, 4, [1, 2, 3], [4, 5, 6], arr2, "damped_least_squares", 5)
