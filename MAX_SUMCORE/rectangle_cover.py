import matplotlib.pyplot as plt
import math
import shapely.ops as ops
from shapely.geometry import LineString
from shapely.geometry import Point
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import box
import shapely.geometry as geometry
import copy as cp
from PSO.sko.rectangle_PSO import PSO
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
def get_rectangle(central_pos,width = 20,height = 17):
    minx,maxx = central_pos[0] - width/2, central_pos[0] + width/2
    miny,maxy = central_pos[1] - height/2, central_pos[1] + height/2
    return box(minx,miny,maxx,maxy)

frame = Polygon([[0,0],[45,0],[30,34],[40,50],[0,34]]).convex_hull
frame = Polygon([[0,0],[40,0],[40,34],[0,34]]).convex_hull
def object_fun(x):
    pos = x.reshape(-1,2)
    squares = [get_rectangle(pos[i], 20, 17) for i in range(pos.shape[0])]
    return -ops.unary_union(squares).intersection(frame).area / frame.area

max_iter = 200
x_lower,y_lower,x_upper,y_upper = frame.bounds

n = 4
pso = PSO(func=object_fun, n_dim=2 * n, pop=40, max_iter=max_iter, x_lb=x_lower, x_ub=x_upper, y_lb=y_lower,
          y_ub=y_upper)


def mini_mum_number(frame:Polygon,width:float,height:float)->int:
    square = get_rectangle([0,0,0],width,height)
    n = int(frame.area/square.area)

    pso  = PSO(func=object_fun, n_dim=2 * n, pop=40, max_iter=max_iter, x_lb=x_lower, x_ub=x_upper, y_lb=y_lower,
          y_ub=y_upper)


pso.run()
squares = []
for i in range(int(len(pso.gbest_x)/2)):
    squares.append(get_rectangle([pso.gbest_x[2 * i],pso.gbest_x[2*i+1]]))

figure(frame.exterior.coords[:])
for square in squares:
    figure(square.exterior.coords[:])
plt.show()
