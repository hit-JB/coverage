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
from MAX_SUMCORE.full_coverage import Agent
from MAX_SUMCORE.full_coverage import World


# box(minx, miny, maxx, maxy, ccw=True), 默认情况下右下角作为第一个点,为逆时针顺序
# b = box(0.0, 0.0, 1.0, 1.0)
# list(b.exterior.coords)
# contain cross and within is the function that judge the location of two object
def graham_scan(points):  # the algorithm of convex hull given the point
    def get_bottom_point(points):  # return a point which has the minimium y axis
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

    def vector_angle(vector):
        norm_ = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1])
        if norm_ == 0:
            return 0
        angle = math.acos(vector[0] / norm_)
        if vector[1] >= 0:
            return angle
        else:
            return 2 * math.pi - angle

    def cos_multi(v1, v2):
        return v1[1] * v2[0] - v1[0] * v2[1]

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

    return stack, cos  ###


def figure(points, color='r'):
    for i, data in enumerate(points):
        plt.plot([points[i][0], points[(i + 1) % len(points)][0]], [points[i][1], points[(i + 1) % len(points)][1]],
                 c=color)


def unitity(agent, other):
    assert isinstance(agent, Polygon) and isinstance(other, Polygon)
    interset = agent.intersection(other)
    union = ops.unary_union([agent, other])
    interset_coords = union.boundary.intersection(interset.boundary)
    if isinstance(interset_coords, geometry.MultiLineString):
        coord1 = interset_coords[0].coords[0]
        coord2 = interset_coords[1].coords[0] if interset_coords[1].coords[0][0] != coord1[0] and \
                                                 interset_coords[1].coords[0][1] != coord1[1] else \
            interset_coords[1].coords[1]
        line = LineString([coord1, coord2])
    else:
        line = LineString(interset_coords)
    unitity = ops.split(union, line)

    index = 0
    for i in range(len(unitity)):
        if np.abs(agent.intersection(unitity[i]).area - unitity[i].area) < 1e-4:
            index = i

    return unitity[index], unitity[1 - index], line


def cos_multi(v1, v2):
    return v1[1] * v2[0] - v1[0] * v2[1]



def get_circle(central_pos, radius, num=64):
    x, y = central_pos[0], central_pos[1]
    theta = np.arange(0, 2 * np.pi, 2 * np.pi / num)
    cycle_points = []
    for i in range(num):
        cycle_points.append([x + radius * np.cos(theta[i]), y + radius * np.sin(theta[i])])
    return Polygon(cycle_points)


def get_insect(line: LineString, boundary: LineString):
    coord1 = line.coords[0]
    coord2 = line.coords[-1]
    if coord1[0] - coord2[0] == 0:
        k = 1000
    else:
        k = np.abs((coord1[1] - coord2[1]) / (coord1[0] - coord2[0]))
    coord1 = boundary.coords[0]
    for coord2 in boundary.coords[1:]:
        if coord2[0] - coord1[0] == 0:
            k1 = 1000
        else:
            k1 = np.abs((coord2[1] - coord1[1]) / (coord2[0] - coord1[0]))
        if np.abs(k1 - k) < 0.01:
            return LineString([coord1, coord2])
        coord1 = coord2


def get_function(agent: Agent, others: list):
    pass


def vector_norm(v):
    return np.sqrt(np.sum(np.square(v)))


class U_fun():
    def __init__(self, central_agent: Polygon, other_agents: list) -> 'None':
        self.agent = central_agent
        self.central_pos = np.array(self.agent.centroid.coords[0])
        self.others = other_agents
        self.u = None
        self.A = []
        self.B = []

    def computer_edge(self, world: World):
        lines = []
        intersect_pos = []
        u = self.agent
        for other in self.others:
            if not self.agent.intersects(other):
                lines.append(None)
                intersect_pos.append(None)
                continue
            U, _, line = unitity(self.agent, other)
            u = u.intersection(U)
            lines.append(line)
            intersect_pos.append(0.5 * self.central_pos + 0.5 * np.array(other.centroid.coords[0]))
        self.u = u.intersection(world.edge)
        lines_A = self.agent.boundary.intersection(self.u)
        if not lines_A.is_empty and not isinstance(lines_A, Point):
            if isinstance(lines_A, LineString):
                lines_A = [lines_A]
            for line in lines_A:
                if isinstance(line, Point): continue
                coord0 = np.array(line.coords[0])
                coord1 = np.array(line.coords[1])
                coord = 0.5 * coord0 + 0.5 * coord1
                n = coord - self.central_pos
                n = n / vector_norm(n)
                length = line.length
                self.A.append(dict(v_n=n, length=length))
        lines_B = []
        for i, line in enumerate(lines):
            if line is None or not line.intersects(self.u):
                lines_B.append(None)
                self.B.append(None)
                continue
            if not self.u.boundary.is_ring:
                print('not closed')
                self.u = self.u.convex_hull
            l = get_insect(line, self.u.boundary)
            lines_B.append(l)
            coord0 = np.array(l.coords[0])
            coord1 = np.array(l.coords[1])
            v0 = coord0 - intersect_pos[i]
            v1 = coord1 - intersect_pos[i]
            v = intersect_pos[i] - self.central_pos
            index = 0 if cos_multi(v0, v) >= cos_multi(v1, v) else 1
            edge_upper = np.sign(cos_multi(v0, v)) * vector_norm(v0) if index == 0 else np.sign(
                cos_multi(v1, v)) * vector_norm(v1)
            edge_low = np.sign(cos_multi(v1, v)) * vector_norm(v1) if index == 0 else np.sign(
                cos_multi(v0, v)) * vector_norm(v0)
            self.B.append((edge_low, edge_upper))

    def computer_g(self):
        g_self = np.zeros(2)
        g_other = []
        for line in self.A:
            g_self += line['v_n'] * line['length']

        for i, line in enumerate(self.B):
            if line is None:
                g_other.append(None)
                continue
            v_n = np.array(self.others[i].centroid.coords[0]) - self.central_pos
            v_n = v_n / vector_norm(v_n)

            integral_low, integral_upper = line[0], line[1]
            c1, c2, c2_other = self.integral_c(self.others[i])
            c1 = c1 * (integral_upper - integral_low)
            c2 = c2 * (integral_upper ** 2 - integral_low * 2)
            c2_other = c2_other * (integral_upper ** 2 - integral_low * 2)

            g_other.append(v_n @ (c1 + c2_other))
            g_self += v_n @ (c1 + c2)
        return g_self, g_other

    def integral_c(self, other: Polygon):
        p = np.zeros(2)
        c1 = np.zeros((2, 2))
        c2 = np.zeros((2, 2))
        x_i, x_j = self.central_pos, other.centroid.coords[0]
        p[0], p[1] = 0.5 * (x_i[1] - x_j[1]), 0.5 * (x_j[0] - x_i[0])
        p = vector_norm(p)
        c1[0][0] = 0.5
        c1[1][1] = 0.5
        c2[0][0] = -1 / 16 * p ** -3 * (x_i[1] - x_j[1]) * (x_i[0] - x_j[0])
        c2[0][1] = 1 / 4 * p ** -3 * (p ** 2 - 0.25 * (x_i[1] - x_j[1]) ** 2)
        c2[1][0] = -1 / 4 * p ** -3 * (p ** 2 - 0.25 * (x_i[0] - x_j[0]) ** 2)
        c2[1][1] = 1 / 16 * p ** -3 * (x_j[0] - x_i[0]) * (x_j[1] - x_i[1])
        return c1, c2, -c2

if __name__ == '__main__':
    frame = Polygon()
    pos = np.array([[2, 2], [5, 2], [4, 4]])

    world = World([[0, 0], [10, 0], [5, 10]])
    agents = [get_circle(pos[i], radius=2) for i in range(pos.shape[0])]

    u1 = U_fun(agents[0], agents[1:3])
    u2 = U_fun(agents[1], [agents[0], agents[2]])
    u3 = U_fun(agents[2], agents[0:2])
    u1.computer_edge(world)
    u2.computer_edge(world)
    u3.computer_edge(world)


    print(u1.computer_g())
    plt.figure()
    figure(world.edge.exterior.coords[:], color='y')
    figure(u1.u.exterior.coords[:], color='r')
    figure(u2.u.exterior.coords[:], color='r')
    figure(u3.u.exterior.coords[:], color='r')

    plt.show()
