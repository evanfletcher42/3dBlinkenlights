import json
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import sys

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def plot(points):
    """

    :param points: (N x 3) Numpy array containing points to plot.
    """

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    ax.scatter(points[:, 0], points[:, 1], points[:, 2], marker='+')
    ax.plot(points[:, 0], points[:, 1], points[:, 2], linewidth=0.5, color='black')

    n_not_reconstructed = 0

    for i in range(len(points)):
        if np.isnan(points[i, 0]):
            n_not_reconstructed += 1
            continue
        # ax.text(points[i, 0], points[i, 1], points[i, 2], str(i))

    ax.set_xlim3d([-1, 1])
    ax.set_ylim3d([-1, 1])
    ax.set_zlim3d([-1, 1])
    
    set_axes_equal(ax)

    print("%d of %d points not reconstructed" % (n_not_reconstructed, len(points)))

    plt.show()


def main():
    json_path = sys.argv[1]

    with open(json_path) as json_file:
        json_blob = json.load(json_file)

        points = np.array(json_blob["leds"], dtype=np.float)

        plot(points)


if __name__ == "__main__":
    main()
