from dataclasses import dataclass

import screeninfo
import cv2 as cv
from numpy.typing import NDArray


@dataclass
class Monitor:
    name: str
    position: tuple[int, int]
    resolution: tuple[int, int]

    def open_fullscreen(self):
        """
        Open a fullscreen window on the monitor
        """
        cv.namedWindow(self.name, cv.WINDOW_NORMAL)
        cv.moveWindow(self.name, self.position[0], self.position[1])
        cv.setWindowProperty(self.name, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)

    def display_image(self, img: NDArray):
        """
        Display given image on the monitor
        :param img: Image to display
        """
        cv.imshow(self.name, img)

    def close(self):
        """
        Destroy the opencv window
        """
        cv.destroyWindow(self.name)


class DisplayManager:
    def __init__(self):
        self.available_monitors = None
        self.update_monitors()

    def update_monitors(self):
        """
        Update the list of available monitors
        """
        self.available_monitors = [Monitor('screen' + str(i), (monitor.x, monitor.y), (monitor.width, monitor.height))
                                   for i, monitor in enumerate(screeninfo.get_monitors())]

    def __len__(self):
        return len(self.available_monitors)

    def __getitem__(self, item):
        return self.available_monitors[item]
