import json
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import sys


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
        ax.text(points[i, 0], points[i, 1], points[i, 2], str(i))

    ax.set_xlim3d([-1, 1])
    ax.set_ylim3d([-1, 1])
    ax.set_zlim3d([-1, 1])

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
