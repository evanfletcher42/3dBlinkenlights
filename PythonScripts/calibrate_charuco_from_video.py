"""
Calibrate a camera from a video of a ChAruCO target.
"""

import cv2
import json
import sys


class CameraCalib:

    def __init__(self):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.board = cv2.aruco.CharucoBoard_create(squaresX=16,
                                                   squaresY=9,
                                                   squareLength=1.0,
                                                   markerLength=0.5,
                                                   dictionary=self.aruco_dict)

        self.all_corners = []
        self.all_ids = []
        self.img_shape = None

        self.skip_frames = 20

    def find_corners(self, img):
        detected_marker_corners, detected_marker_ids, _ = cv2.aruco.detectMarkers(
            image=img,
            dictionary=self.aruco_dict)

        if detected_marker_ids is None or len(detected_marker_ids) == 0:
            return

        n_corners, charuco_corners, charuco_ids = \
            cv2.aruco.interpolateCornersCharuco(markerCorners=detected_marker_corners,
                                                markerIds=detected_marker_ids,
                                                image=img,
                                                board=self.board)

        # Ensure minimum PnP constraint
        if charuco_corners is None or charuco_ids is None or n_corners <= 4:
            return 0

        self.all_corners.append(charuco_corners)
        self.all_ids.append(charuco_ids)
        return len(charuco_ids)

    def process_video(self, video_path):
        print("Reading video: " + video_path)
        vidcap = cv2.VideoCapture(video_path)

        n_frames = 0
        success, img = vidcap.read()

        while success:
            self.img_shape = img.shape

            if n_frames % self.skip_frames == 0:
                n_corners = self.find_corners(img)
                print("Frame %d: %d corners" % (n_frames, n_corners))

            n_frames += 1
            success, img = vidcap.read()

    def solve_calibration(self):
        print("Solving calibration")
        reproj_error, cam_matrix, dist_coefs, cam_from_chessboard_r, cam_from_chessboard_t = \
            cv2.aruco.calibrateCameraCharuco(charucoCorners=self.all_corners,
                                             charucoIds=self.all_ids,
                                             board=self.board,
                                             imageSize=(self.img_shape[0], self.img_shape[1]),
                                             cameraMatrix=None,
                                             distCoeffs=None)

        return reproj_error, cam_matrix, dist_coefs, cam_from_chessboard_r, cam_from_chessboard_t

    def write_intrinsics_to_json(self, cam_matrix, dist_coefs, json_path):
        out_json = dict()
        out_json["fx"] = cam_matrix[0, 0]
        out_json["fy"] = cam_matrix[1, 1]
        out_json["cx"] = cam_matrix[0, 2]
        out_json["cy"] = cam_matrix[1, 2]
        out_json['dist'] = dist_coefs.tolist()

        with open(json_path, 'w') as out_file:
            json.dump(out_json, out_file, indent=2)


def main():
    calib = CameraCalib()

    calib.process_video(sys.argv[1])
    reproj_err, cam_matrix, dist_coefs, _, _ = calib.solve_calibration()

    print("Solved with reprojection error: " + str(reproj_err))

    calib.write_intrinsics_to_json(cam_matrix, dist_coefs, "calib.json")


if __name__ == "__main__":
    main()
