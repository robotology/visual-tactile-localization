#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

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
    x_c = corr_mean_data[:, 6];
    y_c = corr_mean_data[:, 7];
    z_c = corr_mean_data[:, 8];
    phi_c = corr_mean_data[:, 9];
    theta_c = corr_mean_data[:, 10];
    psi_c = corr_mean_data[:, 11];

    # theta_c_fixed = [wrap(angle) for angle in theta_c]
    # psi_c_fixed = [wrap(angle) for angle in psi_c]

    # corr estimate
    gt_data = read_data(prefix, "gt_0")
    x_gt = gt_data[:, 6];
    y_gt = gt_data[:, 7];
    z_gt = gt_data[:, 8];
    phi_gt = gt_data[:, 9];
    theta_gt = gt_data[:, 10];
    psi_gt = gt_data[:, 11];

    # make plot
    fig, ax = plt.subplots(2,3)

    corr_x_plot, = plot_y(ax[0, 0], x_gt - x_c, "xd err")
    ax[0, 0].legend(handles = [corr_x_plot])

    corr_y_plot, = plot_y(ax[0, 1], y_gt - y_c, "yd err")
    ax[0, 1].legend(handles = [corr_y_plot])

    corr_z_plot, = plot_y(ax[0, 2], z_gt - z_c, "zd err")
    ax[0, 2].legend(handles = [corr_z_plot])

    corr_phi_plot, = plot_y(ax[1, 0], (phi_gt - phi_c) * 180 / math.pi, "phid err")
    ax[1, 0].legend(handles = [corr_phi_plot])

    corr_theta_plot, = plot_y(ax[1, 1], (theta_gt - theta_c) * 180 / math.pi, "thetad err")
    ax[1, 1].legend(handles = [corr_theta_plot])

    corr_psi_plot, = plot_y(ax[1, 2], (psi_gt - psi_c) * 180 / math.pi, "psid err")
    ax[1, 2].legend(handles = [corr_psi_plot])

    plt.show()

if __name__ == "__main__":
    main()
