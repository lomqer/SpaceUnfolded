import cv2 as cv
import queue
import threading
from numpy.typing import NDArray


class ThreadedVideoCapture:

    def __init__(self, camera_id: int):
        self.camera = cv.VideoCapture(camera_id)
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self._frame_reader, daemon=True)
        self.thread.start()

    def _frame_reader(self):
        """
        Method used internally to read frames in a separate thread, to keep buffer empty
        """
        while self.camera.isOpened():
            result, frame = self.camera.read()
            if not result:
                break
            if not self.queue.empty():
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass
            self.queue.put(frame)

    def read(self) -> NDArray:
        """
        Read last frame from camera
        :return: Last camera frame
        """
        return self.queue.get()

    def close(self):
        """
        Close the video stream
        """
        self.camera.release()

