#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

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

def quat_to_zyx(q):
    r11 = 2 * (q[1] * q[2] + q[0] * q[3])
    r12 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]
    r21 = - 2 * (q[1] * q[3] - q[0] * q[2])
    r31 = 2 * (q[2] * q[3] + q[0] * q[1])
    r32 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    psi = math.atan2(r31, r32);
    theta = math.asin(r21);
    phi = math.atan2(r11, r12);

    return [phi, theta, psi]

def quats_to_zyx(quaternions):
    return np.array([quat_to_zyx(q) for q in quaternions])

def main():
    prefix = "object-tracking"

    # pred estimate
    pred_mean_data = read_data(prefix, "pred_mean")
    x_p = pred_mean_data[:, 0];
    y_p = pred_mean_data[:, 1];
    z_p = pred_mean_data[:, 2];
    phi_p = pred_mean_data[:, 9];
    theta_p = pred_mean_data[:, 10];
    psi_p = pred_mean_data[:, 11];

    # corr estimate
    corr_mean_data = read_data(prefix, "cor_mean")
    x_c = corr_mean_data[:, 0];
    y_c = corr_mean_data[:, 1];
    z_c = corr_mean_data[:, 2];
    phi_c = corr_mean_data[:, 9];
    theta_c = corr_mean_data[:, 10];
    psi_c = corr_mean_data[:, 11];

    # target
    target_data = read_data(prefix, "target")
    x_t = target_data[:, 0];
    y_t = target_data[:, 1];
    z_t = target_data[:, 2];
    q_t = target_data[:, 6:10];
    angles = quats_to_zyx(q_t)
    phi_t = angles[:, 0]
    theta_t = angles[:, 1]
    psi_t = angles[:, 2]

    # make plot
    fig, ax = plt.subplots(2,3)

    corr_x_plot, = plot_y(ax[0, 0], x_c, "x est")
    target_x_plot, = plot_y(ax[0, 0], x_t, "x")
    ax[0, 0].legend(handles = [corr_x_plot, target_x_plot])

    corr_y_plot, = plot_y(ax[0, 1], y_c, "y est")
    target_y_plot, = plot_y(ax[0, 1], y_t, "y")
    ax[0, 1].legend(handles = [corr_y_plot, target_y_plot])

    corr_z_plot, = plot_y(ax[0, 2], z_c, "z est")
    target_z_plot, = plot_y(ax[0, 2], z_t, "z")
    ax[0, 2].legend(handles = [corr_z_plot, target_z_plot])

    corr_phi_plot, = plot_y(ax[1, 0], phi_c, "phi est")
    target_phi_plot, = plot_y(ax[1, 0], phi_t, "phi")
    ax[1, 0].legend(handles = [corr_phi_plot, target_phi_plot])

    corr_theta_plot, = plot_y(ax[1, 1], theta_c, "theta est")
    target_theta_plot, = plot_y(ax[1, 1], theta_t, "theta")
    ax[1, 1].legend(handles = [corr_theta_plot, target_theta_plot])

    corr_psi_plot, = plot_y(ax[1, 2], psi_c, "psi est")
    target_psi_plot, = plot_y(ax[1, 2], psi_t, "psi")
    ax[1, 2].legend(handles = [corr_psi_plot, target_psi_plot])

    plt.show()

if __name__ == "__main__":
    main()
