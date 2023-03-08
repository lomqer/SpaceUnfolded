from calibration import Calibration
from display_manager import DisplayManager
from threaded_video_capture import ThreadedVideoCapture
import cv2 as cv

DATA_FOLDER = "data_test"

if __name__ == "__main__":
    # Getting the projector. Choose appropriate monitor, usually under index 1
    dm = DisplayManager()
    projector = dm.available_monitors[1]

    # Getting the camera, remember to set correct camera ID
    tvc = ThreadedVideoCapture(0)

    calib = Calibration(DATA_FOLDER)

    # Capturing data, we need to provide projection equal to at most the projector resolution
    calib.capture((1920, 1080), tvc, projector)

    # Calibrating and getting remap maps
    calib.calibrate()
    map1, map2 = calib.read_maps()

    # Warping example image and saving it to result.png
    test_img = cv.imread("test_rainbow.png")
    result = cv.remap(test_img, map1, map2, cv.INTER_LINEAR)
    cv.imwrite("result.png", result)
    cv.imshow("img", result)
    cv.waitKey(0)
