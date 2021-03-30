import matplotlib.pyplot as plt
import math
import shapely.affinity as aff
import shapely.ops as ops
from shapely.geometry import LineString
from shapely.geometry import Point
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import box
import shapely.geometry as geometry
import copy as cp
from PSO.sko.rectangle_PSO import PSO_rotate


def figure(points, color='y'):
    # for data in points:
    #     plt.scatter(data[0],data[1],marker='*',c='b')
    for i, data in enumerate(points):
        # plt.scatter(data[0],data[1],c=color)
        plt.plot([points[i][0], points[(i + 1) % len(points)][0]], [points[i][1], points[(i + 1) % len(points)][1]],
                 c=color)


def cos_multi(v1, v2):  # when the rotate is unclock the value > 0

    return v1[1] * v2[0] - v1[0] * v2[1]


def vector_norm(v):
    return np.sqrt(np.sum(np.square(v)))


plt.figure()


def get_rectangle(central_pos, width=20, height=17):
    minx, maxx = central_pos[0] - width / 2, central_pos[0] + width / 2
    miny, maxy = central_pos[1] - height / 2, central_pos[1] + height / 2
    rectangle = box(minx, miny, maxx, maxy)
    theta = central_pos[2]
    if theta is not None:
        return aff.rotate(rectangle, angle=theta, origin=rectangle.centroid)
    else:
        return rectangle


frame = Polygon([[0, 0], [45, 0], [30, 34], [40, 50], [0, 34]]).convex_hull

frame = Polygon([[0, 0], [34, 0], [34, 40], [0, 40]])


def object_fun(x):
    pos = x.reshape(-1, 3)
    # frame = Polygon([[0, 0], [45, 0], [30, 34], [40, 50], [0, 34]]).convex_hull
    squares = [get_rectangle(pos[i], 20, 17) for i in range(pos.shape[0])]
    return -ops.unary_union(squares).intersection(frame).area / frame.area





def mini_mum_number(frame: Polygon, width=None, height=None, max_iter=30)-> int:
    square = get_rectangle([0, 0, 0], width, height)
    ini_n = int(frame.area / square.area)+1
    x_lower, y_lower, x_upper, y_upper = frame.bounds
    threshould = 0
    n = ini_n
    ini_state = None
    while threshould > -0.99:
        pso = PSO_rotate(func=object_fun, n_dim=3 * n, pop=80, max_iter=max_iter, x_lb=x_lower, x_ub=x_upper,
                         y_lb=y_lower,ini_state=ini_state,
                         y_ub=y_upper)
        pso.run()
        threshould = pso.gbest_y
        #print('The {} time iteration best score is {}'.format(n - ini_n + 1,-threshould))
        n += 1
        ini_state = pso.gbest_x
        mean = ini_state.reshape(-1,3)
        mean = np.mean(mean,0)
        ini_state = np.concatenate([ini_state,mean])

    print('the minimum num of the uav is :{}'.format(n-1))
    return n, pso.gbest_x

frame = Polygon([[0, 0], [45, 0], [30, 34], [40, 50], [0, 34]]).convex_hull
frame = Polygon([[0,0],[25,0],[25,30],[17,30],[0,20]]).convex_hull
#frame = Polygon([[0,0],[37,0],[40,40],[0,50]]).convex_hull
import time
t0 = time.process_time()
num, x = mini_mum_number(frame, width=20, height=17)
print('the cost of the time is:{}'.format(time.process_time()-t0))
squares = []
for i in range(int(len(x) / 3)):
    print('the location of the {} uav is :{} and the tha is :{} '.format(i+1,(x[3 * i] ,x[3 * i + 1]), x[3 * i + 2]))
    squares.append(get_rectangle([x[3 * i], x[3 * i + 1], x[3 * i + 2]]))
figure(frame.exterior.coords[:],color='r')
for square in squares:
    figure(square.exterior.coords[:])
plt.show()
