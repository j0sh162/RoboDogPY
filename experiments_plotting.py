import csv
import matplotlib.pyplot as plt
import pandas as pd


def load_csv_data(filename):
    data = {'iter': [], 'err_x': [], 'err_y': [], 'err_z': []}
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            data['iter'].append(int(row['iter']))
            data['err_x'].append(float(row['err_x']))
            data['err_y'].append(float(row['err_y']))
            data['err_z'].append(float(row['err_z']))
    return data

def plot_data(x, y, label, kp):
    # plt.plot(x, y, label=label)
    plt.loglog(x, y, label=label)
    plt.xlabel('iter')
    plt.ylabel(label)
    plt.title(f'{label} with kp={kp}')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    filename = '200_0_[ 0.00817582  0.00084783 -0.03109509]_[0, 0.01, -0.036]_no_resolve_no_param.csv'
    data = load_csv_data(filename)

    plot_data(data['iter'], data['err_x'], 'err_x', 5)
    plot_data(data['iter'], data['err_y'], 'err_y', 5)
    plot_data(data['iter'], data['err_z'], 'err_z', 5)
