import itertools
import os

import cv2 as cv
from ctypes import *

from display_manager import Monitor
from threaded_video_capture import ThreadedVideoCapture


class Calibration:
    def __init__(self, data_folder):
        self.data_folder = data_folder

    def capture(self, projection_size: tuple[int, int], camera: ThreadedVideoCapture, projector: Monitor):
        """
        Capture gray code pattern frames using provided camera and projector
        :param projection_size: Size of the projection
        :param camera: Video Capture to use when capturing the images
        :param projector: Monitor representing the projector
        """
        gcp = cv.structured_light.GrayCodePattern.create(projection_size[0],
                                                         projection_size[1])
        captured_frames = []
        projector.open_fullscreen()
        black_projection, white_projection = gcp.getImagesForShadowMasks(projection_size, projection_size)
        projector.display_image(black_projection)
        for projection in itertools.chain(gcp.generate()[1], [black_projection, white_projection]):
            projector.display_image(projection)
            cv.waitKey(300)
            captured_frames.append(camera.read())

        projector.close()
        camera.close()

        if not os.path.exists(self.data_folder):
            os.makedirs(self.data_folder)

        fs = cv.FileStorage(os.path.join(self.data_folder, "projection_size.ext"), cv.FILE_STORAGE_WRITE)
        fs.write("h", projection_size[1])
        fs.write("w", projection_size[0])
        fs.release()
        for i, pattern_image in enumerate(captured_frames[:-2]):
            path = os.path.join(self.data_folder, "pattern" + str(i) + ".png")
            cv.imwrite(path, pattern_image)
        cv.imwrite(os.path.join(self.data_folder, "blackFrame.png"), captured_frames[-2])
        cv.imwrite(os.path.join(self.data_folder, "whiteFrame.png"), captured_frames[-1])

    def calibrate(self):
        """
        Run the calibration script on this classes data folder
        """
        calib_dll = cdll.LoadLibrary("dlls/ProjectionCalibration.dll")
        r = cv.selectROI("Select the display contour", cv.imread(os.path.join(self.data_folder, "blackFrame.png")))
        cv.destroyWindow("Select the display contour")
        contour = (c_int * 8)(r[0], r[1], r[0] + r[2], r[1], r[0] + r[2], r[1] + r[3], r[0], r[1] + r[3])
        calib_dll.calibrate(self.data_folder.encode(), contour)

    def read_maps(self):
        """
        Get remap maps from the data folder
        :return:
        """
        fs = cv.FileStorage(os.path.join(self.data_folder, "map.ext"), cv.FILE_STORAGE_READ)
        map1 = fs.getNode("map1").mat()
        map2 = fs.getNode("map2").mat()
        fs.release()
        return map1, map2
