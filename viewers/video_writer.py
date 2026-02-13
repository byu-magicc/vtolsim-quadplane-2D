import numpy as np
import imageio
from OpenGL import GL
import pyqtgraph.opengl as gl


class videoWriter:
    def __init__(
        self, widget: gl.GLViewWidget, video_name="video.mp4", fps=20, output_rate=0.1
    ):
        self.widget = widget
        self.output_rate = output_rate
        self.last_t = 0
        self.fps = fps

        # Widget dimensions
        self.width = widget.width()
        self.height = widget.height()

        # imageio MP4 writer (no fourcc required!)

    def grab_frame(self):
        """
        Grab OpenGL framebuffer from the GLViewWidget.
        Returns RGB frame for imageio.
        """

        w = self.width
        h = self.height

        # Bind correct OpenGL context
        self.widget.makeCurrent()

        # Read pixels from GL framebuffer
        data = GL.glReadPixels(0, 0, w, h, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)

        # Convert to NumPy
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)

        # OpenGL framebuffer is bottom-up → flip vertically
        arr = np.flip(arr, axis=0)

        # Convert RGBA → RGB
        rgb = arr[..., :3]

        return rgb

    def update(self, t: float):
        if (t - self.last_t) >= self.output_rate:
            frame = self.grab_frame()

            self.last_t = t

    def close(self):
        pass
