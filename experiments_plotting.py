import csv
import matplotlib.pyplot as plt
import pandas as pd


def load_csv_data(filename):
    data = {'iter': [], 'err_x': [], 'err_y': [], 'err_z': []}
    with open("./experiments with coordinates/data/" + filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            data['iter'].append(int(row['iter']))
            data['err_x'].append(float(row['err_x']))
            data['err_y'].append(float(row['err_y']))
            data['err_z'].append(float(row['err_z']))
    return data


def plot_data(kp, data, target_com, cutoff=200000):
    plt.figure(figsize=(12,8))
    plt.subplot(311)
    plt.plot(data['iter'][slice(cutoff)], data['err_x'][slice(cutoff)])
    plt.plot(data['iter'][slice(cutoff)], [target_com[0]]*cutoff)
    # plt.xlabel('iter')
    plt.ylabel('com_x')
    plt.title(f'CoM coordinates\nkp={kp} cutoff={cutoff}')

    plt.subplot(312)
    plt.plot(data['iter'][slice(cutoff)], data['err_y'][slice(cutoff)])
    plt.plot(data['iter'][slice(cutoff)], [target_com[1]]*cutoff)
    # plt.xlabel('iter')
    plt.ylabel('com_y')
    # plt.title(f'com_y with kp={kp} and cutoff={cutoff}')

    plt.subplot(313)
    plt.plot(data['iter'][slice(cutoff)], data['err_z'][slice(cutoff)])
    plt.plot(data['iter'][slice(cutoff)], [target_com[2]]*cutoff)
    # plt.xlabel('iter')
    plt.ylabel('com_z')
    # plt.title(f'com_z with kp={kp} and cutoff={cutoff}')


def multiplot_show(kp, data, target_com, cutoff=200000):
    plot_data(kp, data, target_com, cutoff)
    plt.show() 


def multiplot_save(kp, data, target_com, filename, cutoff=200000):
    plot_data(kp, data, target_com, cutoff)
    filename = "./experiments with coordinates/images/" + filename + str(cutoff) + ".png"
    plt.savefig(filename, dpi=600)



if __name__ == "__main__":
    # kp = 3
    # cutoff_iterations = 5000
    # target_com = [0, 0.01, -0.036]
    # filename = str(kp) + "_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv"
    # data = load_csv_data(filename)
    # # multiplot_show(kp, data, target_com, cutoff_iterations)
    # multiplot_save(kp, data, target_com, filename, cutoff_iterations)

    kp_list = [1, 3, 5, 10, 15, 35, 50, 75, 100, 150, 200]
    cutoff_list = [1000, 2500, 10000, 20000]
    target_com = [0, 0.01, -0.036]
    for kp_elem in kp_list:
        filename = str(kp_elem) + "_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv"
        data = load_csv_data(filename)
        for cutoff_elem in cutoff_list:
            multiplot_save(kp_elem, data, target_com, filename, cutoff_elem)

