import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.image as mpimg
import matplotlib.transforms as transforms

class GridMap:
    # def __init__(self, origin_x, origin_y, width, length, resolution):
    #     self.origin = np.zeros(2, dtype=float)
    #     self.origin[0] = origin_x
    #     self.origin[1] = origin_y

    #     self.resolution = resolution
    #     self.resolution_inv = 1.0 / resolution
    #     self.size = np.zeros(2, dtype=int)
    #     self.size[0] = width * self.resolution_inv
    #     self.size[1] = length * self.resolution_inv
        
    #     self.min_boundary = self.origin
    #     self.max_boundary = np.zeros(2, dtype=float)
    #     self.max_boundary[0] = self.min_boundary[0] + width
    #     self.max_boundary[1] = self.min_boundary[1] + length
    #     self.width = width
    #     self.length = length

    #     self.data_size = self.size[0] * self.size[1]
    #     self.data = np.zeros(self.data_size, dtype=int)

    def __init__(self, path, resolution):
        raw_img = mpimg.imread(path)
        self.resolution = resolution
        self.resolution_inv = 1.0 / resolution
        rows = raw_img.shape[0]
        cols = raw_img.shape[1]
        self.length = rows * self.resolution
        self.width = cols * self.resolution
        self.size = np.zeros(2, dtype=int)
        self.size[0] = cols
        self.size[1] = rows

        self.origin = np.zeros(2, dtype=float)
        self.min_boundary = self.origin
        self.max_boundary = np.zeros(2, dtype=float)
        self.max_boundary[0] = self.min_boundary[0] + self.width
        self.max_boundary[1] = self.min_boundary[1] + self.length

        self.data_size = cols * rows
        self.data = []
        for line in raw_img[::-1]:
            for row in line:
                if row[0] < 1:
                    self.data.append(255)
                else:
                    self.data.append(0)

    def is_pos_in_map(self, pos):
        if (pos(0) < self.min_boundary[0] + 1e-4 or
            pos(1) < self.min_boundary[1] + 1e-4):
            return False
        if (pos(0) > self.max_boundary[0] - 1e-4 or
            pos(1) > self.max_boundary[1] - 1e-4):
            return False
        return True

    def is_idx_in_map(self, idx):
        if (idx[0] < 0 or idx[0] > self.size[0] - 1) \
            or (idx[1] < 0 or idx[1] > self.size[1] - 1):
            return False
        return True

    def is_adr_in_map(self, adr):
        if adr < 0 or adr > self.data_size - 1:
            return False
        return True

    def pos_to_idx(self, pos):
        idx = np.zeros(2, dtype=int)
        idx[0] = math.floor((pos[0] - self.origin[0]) * self.resolution_inv)
        idx[1] = math.floor((pos[1] - self.origin[1]) * self.resolution_inv)
        return idx
    
    def idx_to_pos(self, idx):
        pos = np.zeros(2, dtype=float)
        pos[0] = (idx[0] + 0.5) * self.resolution + self.origin[0]
        pos[1] = (idx[1] + 0.5) * self.resolution + self.origin[1]
        return pos

    def idx_to_adr(self, idx):
        return idx[1] * self.size[0] + idx[0]

    def adr_to_idx(self, adr):
        idx = np.zeros(2, dtype=int)
        idx[1] = adr / self.size[0]
        idx[0] = adr % self.size[0]
        return idx
    
    def is_adr_occ(self, adr):
        if self.data[adr] > 10:
            return True
        return False

    def is_idx_occ(self, idx):
        adr = self.idx_to_adr(idx)
        return self.is_adr_occ(adr)

    def is_pos_occ(self, pos):
        idx = self.pos_to_idx(pos)
        return self.is_idx_occ(idx)
    
    def render_map(self, ax):
        ax.set_xlim([self.min_boundary[0], self.max_boundary[0]])
        ax.set_ylim([self.min_boundary[1], self.max_boundary[1]])
        for adr in range(len(self.data)):
            if self.is_adr_occ(adr):
                idx = self.adr_to_idx(adr)
                pos = self.idx_to_pos(idx)
                occ_box = patches.Rectangle((-self.resolution * 0.5 + pos[0],\
                                             -self.resolution * 0.5 + pos[1]),\
                                            self.resolution, self.resolution, \
                                            edgecolor='black', facecolor='black')
                ax.add_patch(occ_box)