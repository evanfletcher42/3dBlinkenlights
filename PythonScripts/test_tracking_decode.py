import cv2
import numpy as np
import scipy.signal
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import sys


def find_nearest_point(p, points, max_dist=15):
    best_i = None
    min_dist = None

    for i in range(len(points)):
        dist = np.linalg.norm(p - points[i])

        if dist > max_dist:
            continue

        if best_i is None or dist < min_dist:
            best_i = i
            min_dist = dist

    return best_i


def find_points_in_img(img):

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, img_thresh = cv2.threshold(img_gray, 10, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(img_thresh, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE,
                                           contours=None, hierarchy=None, offset=None)

    img_points = []
    img_rgb = []

    for contour in contours:

        area = cv2.contourArea(contour)

        if area < 10:
            continue

        # extract centroid coords
        M = cv2.moments(contour)
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]

        img_points.append(np.array([cX, cY]))

        # extract apparent color of this blob by masking out almost all of the image
        # TODO: This is hilariously inefficient; crop ROI first
        mask_img = np.zeros_like(img_thresh)
        cv2.drawContours(mask_img, [contour], -1, 255, -1)
        mean_clr = cv2.mean(img, mask_img)
        img_rgb.append(mean_clr)

    return img_points, img_rgb


def extract_sync(rgb_per_frame):
    conv_w = 10

    intensity_per_frame = np.mean(rgb_per_frame, axis=-1)
    intensity_mavg = np.convolve(intensity_per_frame, np.ones(conv_w), mode='same')

    peaks, meta = scipy.signal.find_peaks(intensity_mavg, prominence=30, distance=40)

    # trim peaks that are too close to an edge
    peaks = peaks[ peaks > conv_w ]
    peaks = peaks[ peaks < len(intensity_mavg) - conv_w]

    return peaks


def extract_slot_rgb(observed_rgb, sync_start, sync_width, offset):

    slot_width = float(sync_width) / 10
    slot_rgb = np.zeros((10, 3))

    for i in range(10):
        slot_start = sync_start + i * slot_width - offset
        slot_end = sync_start + (i + 1) * slot_width - offset
        slot_mid = int(round((slot_start + slot_end) / 2))

        slot_rgb[i, :] = observed_rgb[slot_mid, 0:3]

    return slot_rgb


def decode_word(a):
    """
    Decode a four bit word from eight edge-encoded bits.
    :param a: Analog value of eight bits
    :return: np.array, shape (4), dtype bool, containing the decoded word as 0s and 1s
    """

    out = np.zeros(4)

    for i in range(4):
        out[i] = a[2*i] > a[2*i + 1]

    return out


def main():
    vidcap = cv2.VideoCapture("DSC_0368.MOV")

    success = True

    # plt.ion()  # interactive mode
    img_plt = None

    observed_frame_ids = []
    observed_points = []
    observed_rgb = []

    full_frame_rgb = []

    n_observed_leds = 0

    led_ids_seen_last_frame = []
    led_points_last_frame = []

    success, img = vidcap.read()
    n_frame_id = 0
    while success:
        led_ids_seen_this_frame = []

        led_points_seen_this_frame, img_rgb = find_points_in_img(img)
        led_ids_seen_this_frame = [None for x in led_points_seen_this_frame]

        print(len(led_ids_seen_this_frame))

        if len(led_ids_seen_this_frame) == 0:
            success, img = vidcap.read()
            n_frame_id += 1
            continue

        for p_prev, id_prev in zip(led_points_last_frame, led_ids_seen_last_frame):
            list_idx = find_nearest_point(p_prev, led_points_seen_this_frame)

            if list_idx is None:
                # Disappeared
                continue

            # Propagate ID forward
            led_ids_seen_this_frame[list_idx] = id_prev

        for i in range(len(led_ids_seen_this_frame)):

            led_idx = led_ids_seen_this_frame[i]
            point = led_points_seen_this_frame[i]
            rgb = img_rgb[i]

            # print("ID", led_idx, "Point", point, "rgb", rgb)

            if led_idx is None:
                # New LED
                led_idx = n_observed_leds
                n_observed_leds += 1
                led_ids_seen_this_frame[i] = led_idx

                observed_frame_ids.append([n_frame_id])
                observed_points.append([point])
                observed_rgb.append([rgb])

                continue

            observed_frame_ids[led_idx].append(n_frame_id)
            observed_points[led_idx].append(point)
            observed_rgb[led_idx].append(rgb)

        led_ids_seen_last_frame = led_ids_seen_this_frame
        led_points_last_frame = led_points_seen_this_frame

        # do the whole frame
        all_rgb = np.mean(img_rgb, axis=0)
        full_frame_rgb.append(all_rgb)

        success, img = vidcap.read()
        n_frame_id += 1

    for led_idx in range(len(observed_frame_ids)):

        if len(observed_frame_ids[led_idx]) < 50:
            continue

        observed_rgb[led_idx] = np.array(observed_rgb[led_idx])

        full_frame_rgb = np.array(full_frame_rgb)

        sync_centers = extract_sync(full_frame_rgb)

        fig, ax = plt.subplots(4, 1, sharex=True)
        ax[0].plot(observed_frame_ids[led_idx], observed_rgb[led_idx][:, 0], color='blue')
        ax[1].plot(observed_frame_ids[led_idx], observed_rgb[led_idx][:, 1], color='green')
        ax[2].plot(observed_frame_ids[led_idx], observed_rgb[led_idx][:, 2], color='red')

        ax[3].plot(full_frame_rgb[:, 0], color='blue')
        ax[3].plot(full_frame_rgb[:, 1], color='green')
        ax[3].plot(full_frame_rgb[:, 2], color='red')

        for i in range(4):
            for ctr in sync_centers:
                ax[i].axvline(ctr, color='black', linewidth=1)

        for i in range(len(sync_centers) - 1):
            sync_start = sync_centers[i]
            sync_end = sync_centers[i+1]
            sync_w = sync_end - sync_start

            # Confirm we have data for this range
            if sync_start < observed_frame_ids[led_idx][0] or sync_end > observed_frame_ids[led_idx][-1]:
                continue

            offset = observed_frame_ids[led_idx][0]

            slot_rgb = extract_slot_rgb(observed_rgb[led_idx], sync_start, sync_w, offset)

            for j in range(3):
                word = decode_word(slot_rgb[1:9, j])
                ax[j].text(sync_start + 1, np.max(observed_rgb[led_idx][:, j]), str(word))

            slot_w = float(sync_w) / 10

            for j in range(1,9,1):
                slot_start = sync_start + j * slot_w
                slot_end = sync_start + (j+1) * slot_w
                slot_mid = int(round((slot_start + slot_end) / 2))

                for k in range(4):
                    ax[k].axvline(slot_mid, color='gray', linewidth='1')

        plt.suptitle("LED %d RGB vs Time" % led_idx)

        # plt.figure()
        # observed_points[led_idx] = np.array(observed_points[led_idx])
        # plt.plot(observed_points[led_idx][:, 0], observed_points[led_idx][:, 1])
        # plt.xlim([0, 1920])
        # plt.ylim([0, 1080])
        # plt.gca().set_aspect("equal")
        # plt.grid()
        # plt.title("LED %d Positions" % led_idx)

        plt.show()


if __name__ == "__main__":
    main()
