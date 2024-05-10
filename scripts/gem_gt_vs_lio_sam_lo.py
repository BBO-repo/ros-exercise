#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt

def plot_ground_truth_vs_lio_sam_lo(odo_sync_file):
    data = np.genfromtxt(odo_sync_file, delimiter=",", skip_header=1)

    # retrieve ground truth
    x_gt = data[0:,1]
    y_gt = data[0:,2]

    # retrieve lio sam odometries
    x_lo = data[0:,4]
    y_lo = data[0:,5]

    # remove ground truth initial position
    x_gt = x_gt - x_gt[0]
    y_gt = y_gt - y_gt[0]

    # compute MSE
    mse_x = (np.square(x_gt - x_lo)).mean()
    mse_y = (np.square(y_gt - y_lo)).mean()

    # plot display param
    plt.rcParams.update({'font.size': 22})

    # plot ground truth and LIO SAM localize trajectory
    plt.plot(x_gt, y_gt, label = "ground truth")
    plt.plot(x_lo, y_lo, label = "LIO-SAM-LO") 
    plt.legend()
    plt.grid()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")

    # display min square error directly in plot
    plt.text(0.5, -4,   "mse_x: {:.2f}".format(mse_x))
    plt.text(0.5, -4.5, "mse_y: {:.2f}".format(mse_y))

    # add title
    plt.title("ground truth odometry vs LIO-SAM-LO odometry")
    
    plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("odo_sync_file", help="the path to synchronized odometry log file")
    args = parser.parse_args()

    plot_ground_truth_vs_lio_sam_lo(args.odo_sync_file)