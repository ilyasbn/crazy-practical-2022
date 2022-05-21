import cflib.crtp
import time
import sys
import numpy as np
from threading import Event
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
import random
from matplotlib import pyplot as plt
from matplotlib import cm
from matplotlib.colors import ListedColormap, LinearSegmentedColormap
import matplotlib.colors
import copy

# map colors
xdata=[0,1,2]
ydata=[x*x for x in xdata]
norm=plt.Normalize(1,150)
cmap = ListedColormap(["white", "grey", "red"])

# defining map
# can take different cell values
# 0 -> uncknown
# 1 -> free
# 2 -> obstacle
real_w, real_h = 300, 500 # real map size in cm
distretization_ratio = 10
w, h = int(real_w/distretization_ratio), int(real_h/distretization_ratio)
map = [[0 for y in range(h)] for x in range(w)]


def real_to_map(real_x, real_y):
    return int(real_x/distretization_ratio), int(real_y/distretization_ratio)

def update_map(map, real_x, real_y, cell_value):
    map_x, map_y = real_to_map(real_x, real_y)
    map[map_x][map_y] = cell_value

def simulation(map, nb_points):
    x = np.random.choice(range(0, 300), nb_points)
    y = np.random.choice(range(0, 500), nb_points)
    cell_value = np.random.choice(range(0, 3), nb_points)
    for i in range(nb_points):
        update_map(map, x[i], y[i], cell_value[i])

def map_to_dikjstra_map(map):
    dikjstra_map = copy.deepcopy(map)
    for i in range(len(dikjstra_map)):
        for j in range(len(dikjstra_map[0])):
            if dikjstra_map[i][j] == 1:
                dikjstra_map[i][j] = 0
    return dikjstra_map

# for i in range multiranger:
#     if multiranger(i) < threshold:
#         send position of obstalcle to map

def real_time_display(map, nb_points):
    x = np.random.choice(range(0, 300), nb_points)
    y = np.random.choice(range(0, 500), nb_points)
    cell_value = np.random.choice(range(0, 3), nb_points)

    for i in range(nb_points):
        update_map(map, x[i], y[i], cell_value[i])
        plt.imshow(map, origin='lower', interpolation='None', cmap=cmap)
        plt.pause(0.05)

# Display map
def display(map, nb_points):
    simulation(map,nb_points)
    dikjstra_map = map_to_dikjstra_map(map)
    fig = plt.figure(figsize=(8, 6))
    plt.imshow(map, origin='lower', interpolation='None', cmap=cmap)
    fig = plt.figure(figsize=(8, 6))
    plt.imshow(dikjstra_map, origin='lower', interpolation='None', cmap=cmap)
    plt.show()

real_time_display(map, 50)
display(map, 50)
