import csv
import matplotlib.pyplot as plt
import pandas as pd


def load_csv_data_source(filename):
    data = {'iter': [], 'err_x': [], 'err_y': [], 'err_z': []}
    with open("./experiments with coordinates/data/" + filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            i2sec = float(row['iter'])*0.001
            data['iter'].append(i2sec)
            data['err_x'].append(float(row['err_x']))
            data['err_y'].append(float(row['err_y']))
            data['err_z'].append(float(row['err_z']))
    return data


def plot_data(kp, data, target_com, cutoff=200000):

    min_iter = min(data['iter'][slice(cutoff)])
    max_iter = max(data['iter'][slice(cutoff)])


    plt.figure(figsize=(12,8))
    plt.subplot(311)
    plt.plot(data['iter'][slice(cutoff)], data['err_x'][slice(cutoff)])
    plt.plot(data['iter'][slice(cutoff)], [target_com[0]]*cutoff)
    plt.grid(True)
    plt.xlim(min_iter, max_iter)
    # plt.xlabel('iter')
    plt.ylabel('com_x')
    plt.title(f'CoM coordinates\nkp={kp} cutoff={cutoff}')

    plt.subplot(312)
    plt.plot(data['iter'][slice(cutoff)], data['err_y'][slice(cutoff)])
    plt.plot(data['iter'][slice(cutoff)], [target_com[1]]*cutoff)
    plt.grid(True)
    plt.xlim(min_iter, max_iter)
    # plt.xlabel('iter')
    plt.ylabel('com_y')

    plt.subplot(313)
    plt.plot(data['iter'][slice(cutoff)], data['err_z'][slice(cutoff)])
    plt.plot(data['iter'][slice(cutoff)], [target_com[2]]*cutoff)
    plt.grid(True)   
    plt.xlim(min_iter, max_iter)
    # plt.xlabel('iter')
    plt.ylabel('com_z')


def multiplot_show(kp, data, target_com, cutoff=200000):
    plot_data(kp, data, target_com, cutoff)
    plt.show() 


def multiplot_save(kp, data, target_com, filename, cutoff=200000):
    plot_data(kp, data, target_com, cutoff)
    filename = "./experiments with coordinates/images/cut_" + str(cutoff) + "_" + filename + ".png"
    plt.savefig(filename, dpi=600)

def load_csv_data_local(filename):
    data = {'iter': [], 'err_x': [], 'err_y': [], 'err_z': []}
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            i2sec = float(row['iter'])*0.001
            data['iter'].append(i2sec)
            data['err_x'].append(float(row['err_x']))
            data['err_y'].append(float(row['err_y']))
            data['err_z'].append(float(row['err_z']))
    return data


def plot_data_no_cutoff(kp, data, target_com):
    min_iter = min(data['iter'])
    max_iter = max(data['iter'])

    plt.figure(figsize=(12,8))
    plt.subplot(311)
    plt.plot(data['iter'], data['err_x'])
    plt.plot(data['iter'], [target_com[0]]*len(data['iter']))
    plt.grid(True)
    plt.xlim(min_iter, max_iter)
    # plt.xlabel('iter')
    plt.ylabel('com_x')
    plt.title(f'CoM coordinates\nkp={kp}')

    plt.subplot(312)
    plt.plot(data['iter'], data['err_y'])
    plt.plot(data['iter'], [target_com[1]]*len(data['iter']))
    plt.grid(True)
    plt.xlim(min_iter, max_iter)
    # plt.xlabel('iter')
    plt.ylabel('com_y')

    plt.subplot(313)
    plt.plot(data['iter'], data['err_z'])
    plt.plot(data['iter'], [target_com[2]]*len(data['iter']))
    plt.grid(True)   
    plt.xlim(min_iter, max_iter)
    # plt.xlabel('iter')
    plt.ylabel('com_z')

def multiplot_show_no_cutoff(kp, data, target_com):
    plot_data_no_cutoff(kp, data, target_com)
    plt.show() 

def show_experiment_local_no_cutoff(kp, target_com=[0, 0.01, -0.036]):
    filename = str(kp) + "_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv"
    data = load_csv_data_local(filename)
    multiplot_show_no_cutoff(kp, data, target_com)


if __name__ == "__main__":
    # kp = 3
    # cutoff_iterations = 5000
    # target_com = [0, 0.01, -0.036]
    # filename = str(kp) + "_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv"
    # data = load_csv_data(filename)
    # # multiplot_show(kp, data, target_com, cutoff_iterations)
    # multiplot_save(kp, data, target_com, filename, cutoff_iterations)

    kp_list = [1, 3, 5, 10, 15, 35, 50, 75, 100, 150, 200]
    # kp_list = [150, 200]
    cutoff_list = [200, 500, 850, 900, 1000, 2500, 10000, 20000]
    target_com = [0, 0.01, -0.036]

    # kp = kp_list[2]
    # cutoff = cutoff_list[5]
    # filename = str(kp) + "_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv"
    # data = load_csv_data(filename)
    # multiplot_show(kp, data, target_com, cutoff)

    show_experiment_local_no_cutoff(5)
    show_experiment_local_no_cutoff(10)
    show_experiment_local_no_cutoff(15)
    # for i in range(len(kp_list)):
    #     kp = kp_list[i]
    #     filename = str(kp) + "_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv"
    #     data = load_csv_data(filename)
    #     for j in range(len(cutoff_list)):
    #         cutoff = cutoff_list[j]
    #         multiplot_save(kp, data, target_com, filename, cutoff)
    #         # print(str(kp), "__", str(cutoff))

