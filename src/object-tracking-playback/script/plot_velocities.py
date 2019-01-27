import matplotlib.pyplot as plt
import csv
import numpy as np
import math

def read_data(prefix, postfix):

    data = []

    with open(prefix + '_' + postfix + '.txt', newline='') as csv_data:
        for row in csv_data:
            data.append([float(num_string.rstrip()) for num_string in row.split(sep = " ") if num_string != ''])

    return np.array(data)

def plot_y(axes, y_data, label_txt):

    return axes.plot(y_data, label=label_txt)

def main():
    prefix = "object-tracking"

    # pred estimate
    pred_mean_data = read_data(prefix, "pred_mean")
    x_dot_p = pred_mean_data[:, 3];
    y_dot_p = pred_mean_data[:, 4];
    z_dot_p = pred_mean_data[:, 5];
    phi_dot_p = pred_mean_data[:, 6];
    theta_dot_p = pred_mean_data[:, 7];
    psi_dot_p = pred_mean_data[:, 8];

    # corr estimate
    corr_mean_data = read_data(prefix, "cor_mean")
    x_dot_c = corr_mean_data[:, 3];
    y_dot_c = corr_mean_data[:, 4];
    z_dot_c = corr_mean_data[:, 5];
    phi_dot_c = corr_mean_data[:, 6];
    theta_dot_c = corr_mean_data[:, 7];
    psi_dot_c = corr_mean_data[:, 8];

    # target
    target_data = read_data(prefix, "target")
    x_dot_t = target_data[:, 3];
    y_dot_t = target_data[:, 4];
    z_dot_t = target_data[:, 5];
    w_x_t = target_data[:, 10]
    w_y_t = target_data[:, 11]
    w_z_t = target_data[:, 12]

    # make plot
    fig, ax = plt.subplots(2,3)

    corr_x_plot, = plot_y(ax[0, 0], x_dot_c, "x dot est")
    target_x_plot, = plot_y(ax[0, 0], x_dot_t, "x dot")
    ax[0, 0].legend(handles = [corr_x_plot, target_x_plot])

    corr_y_plot, = plot_y(ax[0, 1], y_dot_c, "y dot est")
    target_y_plot, = plot_y(ax[0, 1], y_dot_t, "y dot")
    ax[0, 1].legend(handles = [corr_y_plot, target_y_plot])

    corr_z_plot, = plot_y(ax[0, 2], z_dot_c, "z dot est")
    target_z_plot, = plot_y(ax[0, 2], z_dot_t, "z dot")
    ax[0, 2].legend(handles = [corr_z_plot, target_z_plot])

    plt.show()

if __name__ == "__main__":
    main()
