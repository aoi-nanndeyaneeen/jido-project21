import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import cv2

class ViewGraph:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(6, 4), dpi=100) # 600x400 px
        self.history_z = []
        self.history_target = []
        self.max_len = 100

    def get_image(self, current_z, target_alt):
        self.history_z.append(current_z)
        self.history_target.append(target_alt)
        if len(self.history_z) > self.max_len:
            self.history_z.pop(0)
            self.history_target.pop(0)

        self.ax.cla()
        self.ax.set_ylim(0, 15)
        self.ax.plot(self.history_z, color='blue', label='Current Alt(m)')
        self.ax.plot(self.history_target, color='red', linestyle='--', label='Target Alt(m)')
        self.ax.set_title("Altitude History")
        self.ax.legend(loc='upper right')
        self.ax.grid(True)

        self.fig.canvas.draw()
        img = np.asarray(self.fig.canvas.buffer_rgba())
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    def close(self):
        plt.close(self.fig)