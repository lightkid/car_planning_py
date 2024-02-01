import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.image as mpimg
import matplotlib.transforms as transforms
import numpy as np
import time
# x = [1,2,3,4]
# y = [1,2,3,4]
# plt.plot(x,y)
# plt.show()
from utils import planning
from utils import gridmap

map = gridmap.GridMap('./map/map_basic.png', 1.0)
fig, ax = plt.subplots()
map.render_map(ax)
plt.gca().set_aspect(1)
# plt.show()

# I = mpimg.imread('./map_basic.png')
# plt.imshow(I)
# print(I.shape)
# print(len(I))
# data = []
# for line in I:
#     line_str=""
#     for row in line:
#         if row[0] > 0:
#             line_str += '1'
#             data.append(1)
#         else:
#             line_str += '0'
#             data.append(0)
#     print(line_str)
            


# print(I[0])
# plt.pause(10)

# fig, ax = plt.subplots()
# # ax.set_xlim([-10, 10])
# # ax.set_ylim([-10, 10])

dt = 0.1
end_time = 10
car = planning.VehicleModel(v=1)
car1 = planning.VehicleModel(v=1)
cmd = planning.VehicleCmd(0,0.2)

car_states = list()
timestamp = 0
car_x = []
car_y = []
car_heading = []
car_v = []
while timestamp < end_time:
    start = time.time()
    state = car.get_state()
    car_states.append(state)
    
    geometric_center_x = car.x + car.length * 0.5 * np.cos(car.heading)
    geometric_center_y = car.y + car.length * 0.5 * np.sin(car.heading)

    car_box = patches.Rectangle((-car.length * 0.5 + geometric_center_x,\
                                 -car.width * 0.5 + geometric_center_y),\
                                car.length, car.width, edgecolor='blue', fill=False)
    tr = transforms.Affine2D().rotate_deg_around(geometric_center_x,\
                                                 geometric_center_y,
                                                 car.heading / np.pi * 180.0)
    car_box.set_transform(tr + ax.transData)
    car_patches = ax.add_patch(car_box)
    plt.show(block = False)
    # canvas = fig.canvas
    # canvas.draw()
    # buf = canvas.tostring_rgb()
    # cols, rows = canvas.get_width_height()
    # img = np.fromstring(buf, dtype=np.uint8).reshape(rows, cols, 3)
    # plt.imshow(img)
    end = time.time()
    plt.pause(max(0.0001, dt - (end - start)))
    car_x.append(geometric_center_x)
    car_y.append(geometric_center_y)
    # car_heading.append(state[2])
    # car_v.append(state[3])

    print(car.__str__())
    car.updateRK4(cmd, dt)
    timestamp += dt
    car_patches.remove()

ax.plot(car_x, car_y)

plt.show()