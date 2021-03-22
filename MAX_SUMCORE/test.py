import numpy as np
import time
import shapely.geometry as geometry
import shapely.affinity as aff
import shapely.ops as ops
from shapely.geometry import Polygon  # 多边形
from shapely.geometry import LineString
from shapely.geometry import Point
import scipy.io as io
import math
from shapely.geometry import box
import matplotlib.pyplot as plt

# box(minx, miny, maxx, maxy, ccw=True), 默认情况下右下角作为第一个点,为逆时针顺序
# b = box(0.0, 0.0, 1.0, 1.0)
# list(b.exterior.coords)
# contain cross and within is the function that judge the location of two object
def graham_scan(points):# the algorithm of convex hull given the point
    def get_bottom_point( points):  # return a point which has the minimium y axis
        min_index = 0
        n = len(points)
        for i in range(0, n):
            if points[i][1] < points[min_index][1] or (
                    points[i][1] == points[min_index][1] and points[i][0] < points[min_index][0]):
                min_index = i
        return min_index

    def sort_polar_angle_cos(points, center_point):

        n = len(points)
        cos_value = []
        rank = []
        norm_list = []
        for i in range(0, n):
            point_ = points[i]
            point = [point_[0] - center_point[0], point_[1] - center_point[1]]
            rank.append(i)
            norm_value = math.sqrt(point[0] * point[0] + point[1] * point[1])
            norm_list.append(norm_value)
            if norm_value == 0:
                cos_value.append(1)
            else:
                cos_value.append(point[0] / norm_value)  # get the cos of all the point

        for i in range(0, n - 1):
            index = i + 1
            while index > 0:
                if cos_value[index] > cos_value[index - 1] or (
                        cos_value[index] == cos_value[index - 1] and norm_list[index] > norm_list[
                    index - 1]):  # if the points have the same cos then the maxn length will ahead
                    temp = cos_value[index]
                    temp_rank = rank[index]
                    temp_norm = norm_list[index]
                    cos_value[index] = cos_value[index - 1]
                    rank[index] = rank[index - 1]
                    norm_list[index] = norm_list[index - 1]
                    cos_value[index - 1] = temp
                    rank[index - 1] = temp_rank
                    norm_list[index - 1] = temp_norm
                    index = index - 1
                else:
                    break  # sorted algorithm that the max is the first

        sorted_points = []
        for i in rank:
            sorted_points.append(points[i])

        return sorted_points, cos_value

    def vector_angle( vector):
        norm_ = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])
        if norm_ == 0:
            return 0
        angle = math.acos(vector[0] / norm_)
        if vector[1] >= 0:
            return angle
        else:
            return 2 * math.pi - angle

    def cos_multi(v1, v2):
        return v1[1] * v2[0]-v1[0] * v2[1]
    bottom_index = get_bottom_point(points)

    bottom_point = points.pop(bottom_index)

    sorted_points, cos_value = sort_polar_angle_cos(points, bottom_point)

    cos = []
    # the sorted_points list have te order that the maxim cos is in the head

    m = len(sorted_points)
    if m < 2:
        print("点的数量过少，无法构成凸包")
        return
    stack = []
    stack.append(bottom_point)
    stack.append(sorted_points[0])
    stack.append(sorted_points[1])

    for i in range(2, m):  # the core of the convex algorithm
        length = len(stack)
        top = stack[length - 1]
        next_top = stack[length - 2]
        v1 = [sorted_points[i][0] - next_top[0], sorted_points[i][1] - next_top[1]]
        v2 = [top[0] - next_top[0], top[1] - next_top[1]]

        while cos_multi(v1, v2) >= 0:
            stack.pop()
            length = len(stack)
            top = stack[length - 1]
            next_top = stack[length - 2]
            v1 = [sorted_points[i][0] - next_top[0], sorted_points[i][1] - next_top[1]]
            v2 = [top[0] - next_top[0], top[1] - next_top[1]]

        stack.append(sorted_points[i])

    for data in stack:
        if data == bottom_point:
            cos.append(1)
        else:
            cos.append(cos_value[sorted_points.index(data)])

    return stack, cos###
def figure(points,color = 'r'):
    for i,data in enumerate(points):
        plt.plot([points[i][0],points[(i+1) % len(points)][0]],[points[i][1],points[(i+1) % len(points)][1]],c=color)

def unitity(agent,other):
    assert isinstance(agent, Polygon) and isinstance(other, Polygon)
    interset = agent.intersection(other)
    union = ops.unary_union([agent, other])
    interset_coords = union.boundary.intersection(interset)
    line = LineString(interset_coords)
    unitity = ops.split(union, line)
    for i in range(len(unitity)):
        if agent.contains(unitity[i]):
          index = i
    return unitity[index], unitity[1-index]
def cos_multi(v1, v2):
    return v1[1] * v2[0]-v1[0] * v2[1]
def get_cycle(x, y, radius, num=32):
    theta = np.arange(0, 2 * np.pi, 2 * np.pi / num)
    cycle_points = []
    for i in range(num):
        cycle_points.append([x + radius * np.cos(theta[i]), y + radius * np.sin(theta[i])])
    return Polygon(cycle_points)



# a = get_cycle(0,0,radius=5)
# b = get_cycle(2,2,radius=5)
#
# plt.figure()
# figure(a.exterior.coords[:])
# figure(b.exterior.coords[:],color='y')
# u1,u2= unitity(a,b)
# plt.figure()
# figure(u1.exterior.coords[:])
# figure(u2.exterior.coords[:])
#
#
# plt.figure()
# lines = u1.boundary.intersection(a.boundary)
# print(lines)
# for line in lines:
#     figure(line.coords[:])
#
# a = LineString([[0,0],[1,1]])
# print(a.length)
# plt.show()
a = Polygon([[0,0],[10,0],[10,20],[0,20]])

plt.figure()
figure(a.exterior.coords[:])
b = aff.rotate(a,angle=719,origin=a.centroid)
figure(b.exterior.coords[:])
print(a,b)
print(a.area,b.area)
plt.show()
# # b = LineString ([[5 ,4], [7, 2]])
# # plt.figure()
# # figure(a.exterior.coords[:])
# # figure(b.coords[:])
# # a = LineString([[0,0],[1,1]])
# # b = Point([1/3,1/3])
# # points = a.intersection(b)
# # print(points)
# # print(a)
# # print(b)
# plt.show()

# a = np.array([0,1])
# b = np.array([-1,-1])
# c = np.array([0,-2])
# d= np.array([-3/2, -1/2])
# e = np.array([-2,0])
# v = b-a
# v1 = c-b
# v2 = d-b
# v3 = e-b
# print(cos_multi(v1,v),cos_multi(v2,v),cos_multi(v3,v))
# # square = box(0,0,10,10)
# # square1 = box(0,0,10,5)
# # line = LineString([[0,1],[11,1]])
# # line1 = LineString([[0,-1],[12,-1]])
# # print(isinstance(square1.boundary.intersection(square.boundary), tuple))
# # for line in square1.boundary.intersection(square.boundary):
# #     print(line.coords[:])
# # figure(square.exterior.coords[:])
#
# #square1_contain = unitity(square1,square2)
# # figure(square1.exterior.coords[:])
# # figure(square1_contain.exterior.coords[:])
#
# # # print(square1.exterior.coords[:])
# # plt.figure()
# # coords1 = list(square1.exterior.coords[:])
# # coords2 = list(square2.exterior.coords[:])
# # coodrs3 = list(square3.exterior.coords[:])
# # figure(coords1)
# # figure(coords2)
# # figure(coodrs3)
#
# # square_union = square1.union(square2.union(square3))
# # square_union = ops.unary_union([square1,square2,square3])
# # plt.figure()
# # figure(list(square_union.exterior.coords[:]))
#
# # line1 = LineString([[0,0],[2,2]])
# # line2 = LineString([[0,2],[2,0]])
# # insert = line1.intersection(line2)
# # print(insert.coords[:])
# # figure(line1.coords[:])
# # figure(insert.coords[:])
# # print (line1.coords[:])
# from shapely.ops import nearest_points
# # triangle = Polygon([(0, 0), (1, 0), (0.5, 1), (0, 0)])
# # square = Polygon([(0, 2), (1, 2), (1, 3), (0, 3), (0, 2)])
# # square1 = Polygon([[0.5,2],[1,2],[1,4],[0.5,4]]).convex_hull
# # big_triangle = Polygon([[0,0],[1,0],[0.5,4]]).convex_hull
# #
# #
# # union = ops.unary_union([triangle,square,square1])
# #
# # insection = big_triangle.intersection(union)
# # print(insection.area)
# # plt.figure()
# # for shape in insection:
# #     figure(shape.exterior.coords[:])
# # plt.figure()
# # for shape in union:
# #     figure(shape.exterior.coords[:])
# #     print(shape.boundary)
# #
# #
# #
# # shared = ops.shared_paths(square.boundary,square1.boundary)
# # line = big_triangle.boundary
# # print(line.convex_hull.area)
# # plt.figure()
# # figure(square1.exterior.coords[:])
# # figure(triangle.exterior.coords[:])
# # figure(square.exterior.coords[:])
# # figure(big_triangle.exterior.coords[:])
# # union = ops.unary_union([triangle,square])
# # figure(triangle.exterior.coords[:])
# # figure(square.exterior.coords[:])
# # print(union.area)
# # print(ops.nearest_points(triangle,square)[0].coords[:])
# # print(ops.nearest_points(triangle,square)[1].coords[:])
#
# # plt.figure()
# # plt.figure()
# # figure(list(square1.difference(square2).exterior.coords[:]))
# # plt.figure()
# # figure(list(square1.intersection(square2).intersection(square3).exterior.coords[:]))
# plt.show()

#the test program
# points = np.random.randint(0,100,(5,2))
# points = points.tolist()
# convex_points ,cos = graham_scan(points)
# import matplotlib.pyplot as plt
#
# plt.figure()
# for data in points:
#     plt.scatter(data[0],data[1],marker='*',c='b')
# for i,data in enumerate(convex_points):
#     plt.scatter(data[0],data[1],c='r')
#     plt.plot([convex_points[i][0],convex_points[(i+1) % len(convex_points)][0]],[convex_points[i][1],convex_points[(i+1) % len(convex_points)][1]],c='y')
# plt.show()