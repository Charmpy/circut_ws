import cv2 as cv
import numpy as np


class Finder:
    def __init__(self):
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_250)
        parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(dictionary, parameters)

    def get_target_marker(self, frame):
        marker_corners, marker_IDs, reject = self.detector.detectMarkers(frame)
        target_id = -1
        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                target_id = ids
                print(ids)
                
        return target_id

    def get_marker_pos(self, frame, color_frame, id):
        marker_corners, marker_IDs, reject = self.detector.detectMarkers(frame)
        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                if ids[0] == id:
                    cv.polylines(color_frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_left = corners[0].ravel()
                    top_right = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()

                    width, heigth = int(top_left[0] + bottom_right[0]), int(top_left[1] + bottom_right[1])  
                    center = [width // 2, heigth // 2]  
  
                    return center, int(width * heigth)
