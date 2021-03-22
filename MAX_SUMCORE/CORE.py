import matplotlib.pyplot as plt
import math
import shapely.ops as ops
from shapely.geometry import LineString
from shapely.geometry import Point
import numpy as np
from shapely.geometry import Polygon
from shapely.geometry import box
import shapely.geometry as geometry
from PSO.sko.PSO import PSO
import copy as cp
plt.figure()
def unitity(agent, other):
    assert isinstance(agent, Polygon) and isinstance(other, Polygon)
    interset = agent.intersection(other)
    union = ops.unary_union([agent, other])
    interset_coords = union.boundary.intersection(interset)
    line = LineString(interset_coords)
    unitity = ops.split(union, line)
    index = 0
    for i in range(len(unitity)):
        if agent.contains(unitity[i]):
            index = i
    return unitity[index], unitity[1 - index]

def figure(points, color='r'):
    # for data in points:
    #     plt.scatter(data[0],data[1],marker='*',c='b')
    for i, data in enumerate(points):
        # plt.scatter(data[0],data[1],c=color)
        plt.plot([points[i][0], points[(i + 1) % len(points)][0]], [points[i][1], points[(i + 1) % len(points)][1]],
                 c='y')

def get_cycpe(x, y, radius, num=32):
    theta = np.arange(0, 2 * np.pi, 2 * np.pi / num)
    cycle_points = []
    for i in range(num):
        cycle_points.append([x + radius * np.cos(theta[i]), y + radius * np.sin(theta[i])])
    return Polygon(cycle_points)

class world():
    def __init__(self, points, radius):
        self.points = points  # the point is to construct the world
        self.graph = Polygon(self.points).convex_hull
        self.L = 0
        self.radius = radius
        minx, miny, max_x, max_y = self.graph.bounds
        self.out_frame = box(minx, miny, max_x, max_y)
        self.cycles = []
        self.inner_points = None
        self.central_point = self.graph.centroid.coords[0]

        self.reset()
        self.inner_points_unchan = cp.deepcopy(self.inner_points)

    def reset(self):
        point = self.central_point
        x, y = point[0], point[1]
        height = self.radius * np.sqrt(3)
        width = self.radius * 1.5
        minx, miny, max_x, max_y = self.graph.bounds
        points = []
        num_rightline = 2 * int((max_x - x) / width)
        num_leftline = 2 * int((x - minx) / width)
        for i in range(num_rightline + 1):
            if i % 2 == 0:
                x, y = point[0] + i * width, point[1]
            else:
                x, y = point[0] + i * width, point[1] + height / 2
            for j in range(3 * int((max_y - y) / height)):
                points.append([x, y + j * height])
            for j in range(3 * int((y - miny) / height)):
                points.append([x, y - (j + 1) * height])

        for i in range(1, num_leftline + 1):
            if i % 2 == 0:
                x, y = point[0] - i * width, point[1]
            else:
                x, y = point[0] - i * width, point[1] + height / 2

            for j in range(3 * int((max_y - y) / height)):
                points.append([x, y + j * height])
            for j in range(3 * int((y - miny) / height)):
                points.append([x, y - (j + 1) * height])
        self.inner_points = points

    def coverage_cycle(self):
        points = self.inner_points
        cycles = []
        for point in points:
            cycle = get_cycpe(point[0], point[1], radius=self.radius)
            a = cycle.overlaps(self.graph)
            b = self.graph.contains(cycle)
            if a or b:
                if a and self.graph.intersection(cycle).area / cycle.area > 0.1:
                    cycles.append(cycle)
                elif b:
                    cycles.append(cycle)
        return cycles  # , points

    def reset_points(self, point):
        delta = [point[0] - self.central_point[0], point[1] - self.central_point[1]]
        for i in range(len(self.inner_points)):
            self.inner_points[i][0] = self.inner_points_unchan[i][0] + delta[0]
            self.inner_points[i][1] = self.inner_points_unchan[i][1] + delta[1]


    def cover_rate(self, cycles):
        assert isinstance(cycles, list) and isinstance(cycles[0], Polygon)
        c_s = ops.unary_union(cycles).intersection(self.graph).area
        s = self.graph.area
        rate = c_s / s
        return c_s, s, rate, len(cycles)
    def PSO_fun(self, x):
        point = x
        self.reset_points(x)
        cycles = self.coverage_cycle()
        _, _, rate, num = self.cover_rate(cycles)
        return -rate / num

# data1 = [[2,3],[5,3],[5,6],[2,6]]
# data2 = [[3.2,4],[5.5,4],[3.2,8],[5.5,8]]
# data3 = [[4,1],[7,1],[4,5],[7,5]]
# square1 = Polygon(data1).convex_hull
# square2 = Polygon(data2).convex_hull
# figure(square1.exterior.coords[:])
# figure(square2.exterior.coords[:])
# print(square1.overlaps(square2))

env = world(points=[[0, 0], [100, 0],[0,100],[80,150],[100,50],[100,0]], radius=15)
central_point =cp.deepcopy(env.graph.centroid.coords[:][0])
radius = cp.copy(env.radius)

objec_fun = lambda x: env.PSO_fun(x)
# for point in points:
#     plt.scatter(point[0],point[1],c='r')
# figure(env.graph.exterior.coords[:])
#
#
#
#
# reset_point1 = central_point[0] + radius,central_point[1]+radius
# reset_point2 =central_point[0] -radius,central_point[1] + radius
#
# env.reset_points(reset_point1)
# plt.figure()
# for point in env.inner_points:
#     plt.scatter(point[0],point[1],c='r')
# figure(env.graph.exterior.coords[:])
#
# env.reset_points(reset_point2)
# plt.figure()
# for point in env.inner_points:
#     plt.scatter(point[0],point[1],c='r')
# figure(env.graph.exterior.coords[:])
#
# env.reset_points(central_point)
# plt.figure()
# for point in env.inner_points:
#     plt.scatter(point[0],point[1],c='r')
# figure(env.graph.exterior.coords[:])
# env.reset_points(reset_point2)
# plt.show()
constraint_ueq = (
    lambda x: (x[0] - central_point[0]) ** 2 + (x[1] - central_point[1]) ** 2 - radius ** 2
    ,
)
radius = radius
x_upper,x_lower = central_point[0] + radius,central_point[0] - radius
y_upper,y_lower = central_point[1] + radius,central_point[0] - radius
max_iter = 20
pso = PSO(func=objec_fun, n_dim=2, pop=40, max_iter=max_iter,x_lb=x_lower,x_ub=x_upper,y_lb=y_lower,y_ub=y_upper)
pso.record_mode = True
pso.run()
print('best_x is ', pso.gbest_x, 'best_y is', pso.gbest_y)

env.reset_points(pso.gbest_x)
cylces = env.coverage_cycle()

c_s,s,rate,num = env.cover_rate(cylces)

print('coverage area :{},the graph area :{}, coverage rate:{}, num of the cycle:{}'.format(c_s, s, rate, num))
for cylce in cylces:
    figure(cylce.exterior.coords[:])
figure(env.graph.exterior.coords[:])

plt.figure()
for point in env.inner_points:
    plt.scatter(point[0],point[1],c='r')
figure(env.graph.exterior.coords[:])
plt.show()


# cycles = env.coverage_cycle()
# print(env.cover_rate(cycles))
# plt.figure()
# figure(env.graph.exterior.coords[:])
# for cycle in cycles:
#     figure(cycle.exterior.coords[:])
#
# reset_point = [env.graph.centroid.coords[:][0][0], env.graph.centroid.coords[:][0][1] + env.radius]
#
# env.reset_points(reset_point)
#
# cycles = env.coverage_cycle()
#
# print(env.cover_rate(cycles))
#
# plt.figure()
# figure(env.graph.exterior.coords[:])
# for cycle in cycles:
#     figure(cycle.exterior.coords[:])
# plt.show()

# plt.figure()
# figure(env.out_frame.exterior.coords[:])
# for point in points:
#     cycle = get_cycpe(point[0],point[1],radius=5)
#     figure(cycle.exterior.coords[:])
# plt.show()


# point = geometry.MultiPoint([[1,1],[2,2]])
# figure(point.coords[:])
# print(point.coords[:])
#
# # data1 = [[2,3],[5,3],[5,6],[2,6]]
# # data1 = [[1,1],[4,1],[2,2]]
# # data2 = [[3.2,4],[5.5,4],[3.2,8],[5.5,8]]
# # data3 = [[4,1],[7,1],[4,5],[7,5]]
# # square1 = Polygon(data1).convex_hull
# #
# # square2 = square1.buffer(distance=1)
# # square1.bounds
# # print()
# # figure(square1.exterior.coords[:])
# # figure(square2.exterior.coords[:])
# plt.show()


# circle1 = get_cycpe(0, 0, 2, num=32)
# print(circle1.exterior.xy)
# circle2 = get_cycpe(1,1,2,num=32)
# c_1,c_2 = unitity(circle1,circle2)
# print(c_1.area+c_2.area)
# print(ops.unary_union([circle1,circle2]).area)
# figure(c_1.exterior.coords[:])
# figure(c_2.exterior.coords[:])
# plt.figure()
# figure(ops.unary_union([circle1,circle2]).exterior.coords[:])
# plt.show()

# class Agent():
#     def __init__(self, pos, radius=15, color='b', alpha=0):
#         self.pos = pos
#         self.radius = radius
#         self.r, self.q = [0], [0]
#         self.negibors = []
#
#     def show(self):
#         theta = theta = np.linspace(0, 2 * np.pi, 100)
#         x = self.x + self.radius * np.sin(theta)
#         y = self.y + self.radius * np.cos(theta)
#         plt.plot(x, y, self.color)
#
#     def computeMTV(self):
#         mtv = [] * len(self.negibors)
#         pass
#
#     def computeMTF(self):
#         mtf = [] * len(self.negibors)
#         for i in range(len(self.negibors)):
#             nb = self.r.pop(i)
#             mtf.append(sum(self.r))
#             self.r.insert(i, nb)
#         return mtf
#
#
#
#
#
# class world():
#     def __init__(self, points):
#         self.world_points = points
#         self.reset()
#
#     def reset_world(self):
#         pass
# class world():
#     def __init__(self,points):
#         self.agents = []
#         self.points = points
#         self.side_points, self.cos_value = self.graham_scan(self.points)
#     def get_bottom_point(self,points):  # return a point which has the minimium y axis
#         min_index = 0
#         n = len(points)
#         for i in range(0, n):
#             if points[i][1] < points[min_index][1] or (
#                     points[i][1] == points[min_index][1] and points[i][0] < points[min_index][0]):
#                 min_index = i
#         return min_index
#
#
#     def sort_polar_angle_cos(self,points, center_point):
#
#         n = len(points)
#         cos_value = []
#         rank = []
#         norm_list = []
#         for i in range(0, n):
#             point_ = points[i]
#             point = [point_[0] - center_point[0], point_[1] - center_point[1]]
#             rank.append(i)
#             norm_value = math.sqrt(point[0] * point[0] + point[1] * point[1])
#             norm_list.append(norm_value)
#             if norm_value == 0:
#                 cos_value.append(1)
#             else:
#                 cos_value.append(point[0] / norm_value)  # get the cos of all the point
#
#         for i in range(0, n - 1):
#             index = i + 1
#             while index > 0:
#                 if cos_value[index] > cos_value[index - 1] or (
#                         cos_value[index] == cos_value[index - 1] and norm_list[index] > norm_list[
#                     index - 1]):  # if the points have the same cos then the maxn length will ahead
#                     temp = cos_value[index]
#                     temp_rank = rank[index]
#                     temp_norm = norm_list[index]
#                     cos_value[index] = cos_value[index - 1]
#                     rank[index] = rank[index - 1]
#                     norm_list[index] = norm_list[index - 1]
#                     cos_value[index - 1] = temp
#                     rank[index - 1] = temp_rank
#                     norm_list[index - 1] = temp_norm
#                     index = index - 1
#                 else:
#                     break  # sorted algorithm that the max is the first
#
#         sorted_points = []
#         for i in rank:
#             sorted_points.append(points[i])
#
#         return sorted_points, cos_value
#
#
#     def vector_angle(self,vector):
#         norm_ = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])
#         if norm_ == 0:
#             return 0
#         angle = math.acos(vector[0] / norm_)
#         if vector[1] >= 0:
#             return angle
#         else:
#             return 2 * math.pi - angle
#
#
#     def cos_multi(self,v1, v2):
#         return v1[0] * v2[1] - v1[1] * v2[0]
#
#
#     def graham_scan(self,points):
#         points = self.points
#         bottom_index = self.get_bottom_point(points)
#
#
#         bottom_point = points.pop(bottom_index)
#
#         sorted_points, cos_value = self.sort_polar_angle_cos(points, bottom_point)
#
#         points.append(bottom_point)
#         cos = []
#         # the sorted_points list have te order that the maxim cos is in the head
#
#         m = len(sorted_points)
#         if m < 2:
#             print("点的数量过少，无法构成凸包")
#             return
#         stack = []
#         stack.append(bottom_point)
#         stack.append(sorted_points[0])
#         stack.append(sorted_points[1])
#
#         for i in range(2, m):  # the core of the convex algorithm
#             length = len(stack)
#             top = stack[length - 1]
#             next_top = stack[length - 2]
#             v1 = [sorted_points[i][0] - next_top[0], sorted_points[i][1] - next_top[1]]
#             v2 = [top[0] - next_top[0], top[1] - next_top[1]]
#
#             while self.cos_multi(v1, v2) >= 0:
#                 stack.pop()
#                 length = len(stack)
#                 top = stack[length - 1]
#                 next_top = stack[length - 2]
#                 v1 = [sorted_points[i][0] - next_top[0], sorted_points[i][1] - next_top[1]]
#                 v2 = [top[0] - next_top[0], top[1] - next_top[1]]
#
#             stack.append(sorted_points[i])
#
#         for data in stack:
#             if data == bottom_point:
#                 cos.append(1)
#             else:
#                 cos.append(cos_value[sorted_points.index(data)])
#
#         return stack, cos
#     def in_convex(self, point):  # point is the form of the list
#         side_points = self.side_points
#         cos_value = self.cos_value
#         bottom_point = side_points[0]
#         if point[1] < bottom_point[1]:
#             return False
#         delta = np.array(point) - np.array(bottom_point)
#         point_norm = np.sqrt(np.sum(np.square(delta)))
#         cos = delta[0] / point_norm
#
#         for index, data in enumerate(side_points):
#             if cos < cos_value[index]:
#                 continue
#             # v1 = [sorted_points[i][0] - next_top[0], sorted_points[i][1] - next_top[1]]
#             # v2 = [top[0] - next_top[0], top[1] - next_top[1]]
#             v1 = [side_points[index][0] - side_points[index - 1][0], data[1] - side_points[index - 1][1]]
#             v2 = [point[0] - side_points[index - 1][0], point[1] - side_points[index - 1][1]]
#             return self.cos_multi(v1, v2) > 0
#         return False
#
#
#     def test(self):
#         result, cos_value = self.side_points, self.cos_value
#         points =self.points
#         plt.figure()
#         length = len(result)
#         for i in range(0, length - 1):
#             plt.plot([result[i][0], result[i + 1][0]], [result[i][1], result[i + 1][1]], c='r')
#         plt.plot([result[0][0], result[length - 1][0]], [result[0][1], result[length - 1][1]], c='r')
#
#         initial_pos = np.array(self.points).mean(axis=0)
#         agents = []
#         agents.append(Agent(initial_pos))
#         agent = agents[0]
#         for agent in agents:
#             while not agent.full():
#                 pos = agent.vtg(self)
#                 if pos is not None:
#                     agents.append(Agent(pos))
#
#         for agent in agents:
#             agent.show()
#
#
#         # for data in agents:
#         #     data.show()
#
#
#
#
#         plt.show()
