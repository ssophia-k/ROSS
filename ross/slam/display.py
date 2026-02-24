"""
2-D image display using OpenCV highgui.

Replaces the SDL2-based display from the original tutorial with
OpenCV's built-in ``imshow`` – no extra dependencies required.
"""

import cv2


class Display:
    """Simple 2-D image viewer backed by an OpenCV window.

    Parameters
    ----------
    W : int
        Display width in pixels.
    H : int
        Display height in pixels.
    window_name : str
        Name of the OpenCV window.
    """

    def __init__(self, W: int, H: int, window_name: str = "ROSS – Monocular SLAM"):
        self.W = W
        self.H = H
        self.window_name = window_name
        # Explicitly create the window before the first imshow call.
        # Without this, the Qt backend on Linux can fail to initialise
        # its raster painter and render a black window.
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, W, H)

    def paint(self, img) -> None:
        """Render *img* in the display window.

        Parameters
        ----------
        img : np.ndarray
            BGR image to display. Will be resized to (W, H).
        """
        img = cv2.resize(img, (self.W, self.H))
        cv2.imshow(self.window_name, img)

        # Process window events; press 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            raise SystemExit("User pressed 'q' – exiting.")
