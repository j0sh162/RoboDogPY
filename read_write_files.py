import json

def get_filename(start_com, target_com, err_threshold, damped_least_squares):
    if damped_least_squares:
        damp = "damped"
    else:
        damp = "non-damped"
    return str(start_com) + " " + str(target_com) + " " + str(err_threshold) + " " + damp + ".txt"


def save_parameters(start_com, target_com, err_threshold, damped_least_squares, progress_coms):
    file_name = get_filename(start_com, target_com, err_threshold, damped_least_squares)

    data = {
        "start_com": start_com,
        "target_com": target_com,
        "err_threshold": err_threshold,
        "damped_least_squares": damped_least_squares,
        "progress_coms": progress_coms
    }

    with open(file_name, 'w') as file:
        json.dump(data, file)

def load_parameters(filename):
    with open(filename, 'r') as file:
        data = json.load(file)

    start_com = data["start_com"]
    target_com = data["target_com"]
    err_threshold = data["err_threshold"]
    damped_least_squares = data["damped_least_squares"]
    progress_coms = data["progress_coms"]

    return start_com, target_com, err_threshold, damped_least_squares, progress_coms


save_parameters([1, 2, 3], [4, 5, 6], 0.01, True, [[7, 8, 9], [10, 11, 12]])

# loaded_start_com, loaded_target_com, loaded_err_threshold, loaded_damped_least_squares, loaded_progress_coms = load_parameters(get_filename([1, 2, 3], [4, 5, 6], 0.01, True))
loaded_start_com, loaded_target_com, loaded_err_threshold, loaded_damped_least_squares, loaded_progress_coms = load_parameters("[1, 2, 3] [4, 5, 6] 0.01 damped.txt")

print("start_com:", loaded_start_com)
print("target_com:", loaded_target_com)
print("err_threshold:", loaded_err_threshold)
print("damped_least_squares:", loaded_damped_least_squares)
print("progress_coms:", loaded_progress_coms)
