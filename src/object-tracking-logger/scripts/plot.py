#! /bin/python3

import matplotlib.pyplot as plt
import csv
import numpy as np
import math
import sys

def read_data(prefix, postfix):

    data = []

    with open(prefix + '_' + postfix + '.txt', newline='') as csv_data:
        for row in csv_data:
            data.append([float(num_string.rstrip()) for num_string in row.split(sep = " ") if num_string != ''])

    return np.array(data)

def plot_y(axes, y_data, label_txt):

    return axes.plot(y_data, label=label_txt)

def main():
    prefix = sys.argv[1] + "/data"

    # corr estimate
    corr_mean_data = read_data(prefix, "estimate")
    x_c = corr_mean_data[:, 0];
    y_c = corr_mean_data[:, 1];
    z_c = corr_mean_data[:, 2];
    phi_c = corr_mean_data[:, 3];
    theta_c = corr_mean_data[:, 4];
    psi_c = corr_mean_data[:, 5];

    # theta_c_fixed = [wrap(angle) for angle in theta_c]
    # psi_c_fixed = [wrap(angle) for angle in psi_c]

    # corr estimate
    gt_data = read_data(prefix, "gt_0")
    x_gt = gt_data[:, 0];
    y_gt = gt_data[:, 1];
    z_gt = gt_data[:, 2];
    phi_gt = gt_data[:, 3];
    theta_gt = gt_data[:, 4];
    psi_gt = gt_data[:, 5];

    # make plot
    fig, ax = plt.subplots(2,3)

    corr_x_plot, = plot_y(ax[0, 0], x_c, "x est")
    gt_x_plot, = plot_y(ax[0, 0], x_gt, "x gt")
    ax[0, 0].legend(handles = [corr_x_plot, gt_x_plot])

    corr_y_plot, = plot_y(ax[0, 1], y_c, "y est")
    gt_y_plot, = plot_y(ax[0, 1], y_gt, "y gt")
    ax[0, 1].legend(handles = [corr_y_plot, gt_y_plot])

    corr_z_plot, = plot_y(ax[0, 2], z_c, "z est")
    gt_z_plot, = plot_y(ax[0, 2], z_gt, "z gt")
    ax[0, 2].legend(handles = [corr_z_plot, gt_z_plot])

    corr_phi_plot, = plot_y(ax[1, 0], phi_c, "phi est")
    gt_phi_plot, = plot_y(ax[1, 0], phi_gt, "phi gt")    
    ax[1, 0].legend(handles = [corr_phi_plot, gt_phi_plot])

    corr_theta_plot, = plot_y(ax[1, 1], theta_c, "theta est")
    gt_theta_plot, = plot_y(ax[1, 1], theta_gt, "theta gt")
    ax[1, 1].legend(handles = [corr_theta_plot, gt_theta_plot])

    corr_psi_plot, = plot_y(ax[1, 2], psi_c, "psi est")
    gt_psi_plot, = plot_y(ax[1, 2], psi_gt, "phi gt")
    ax[1, 2].legend(handles = [corr_psi_plot, gt_psi_plot])

    plt.show()

if __name__ == "__main__":
    main()
