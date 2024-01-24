import csv
import matplotlib.pyplot as plt
import pandas as pd
from read_write_csv import get_filename

def experiment_plot_show(kp, start_com, target_com, method_type, method_parameter, local, cutoff = None):
    filename = get_filename(kp, 0, start_com, target_com, method_type, method_parameter)
    if local:
        data = load_csv_data_local(filename)
    else:
        data = load_csv_data_source(filename)
    if cutoff is None:
        multiplot_show_no_cutoff(kp, data, target_com)
    else:
        multiplot_show(kp, data, target_com, cutoff)

def experiment_plot_save(kp, start_com, target_com, method_type, method_parameter, local, cutoff = None):
    filename = get_filename(kp, 0, start_com, target_com, method_type, method_parameter)
    if local:
        data = load_csv_data_local(filename)
    else:
        data = load_csv_data_source(filename)
    
    if cutoff is None:
        plot_data_no_cutoff(kp, data, target_com)
    else:
        plot_data(kp, data, target_com, cutoff)
    
    filename = "cut_"+str(cutoff)+"_"+filename+".png"
    if local:
        plt.savefig(filename, dpi=600)
    else:
        plt.savefig("./experiments with coordinates/images/" + filename, dpi = 600)




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

if __name__ == "__main__":
    target_com = [0, 0.01, -0.036]
    start_com = [0.008175819803293415, 0.0008478253536040646, -0.031095085500370673]
    experiment_plot_show(10, start_com, target_com, "no_resolve", None, True, None)

    experiment_plot_save(10, start_com, target_com, "no_resolve", None, True, None)
