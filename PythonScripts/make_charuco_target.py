import cv2
from matplotlib import pyplot as plt

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard_create(16, 9, 1, .5, aruco_dict)
imboard = board.draw((1920, 1080))

cv2.imwrite("chessboard.png", imboard)
