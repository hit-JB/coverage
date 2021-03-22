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

def figure(points, color='y'):
    # for data in points:
    #     plt.scatter(data[0],data[1],marker='*',c='b')
    for i, data in enumerate(points):
        # plt.scatter(data[0],data[1],c=color)
        plt.plot([points[i][0], points[(i + 1) % len(points)][0]], [points[i][1], points[(i + 1) % len(points)][1]],
                 c=color)

def get_side(agent, other):
    pass

def get_circle(central_pos, radius, num=64):
    x, y = central_pos[0], central_pos[1]
    theta = np.arange(0, 2 * np.pi, 2 * np.pi / num)
    cycle_points = []
    for i in range(num):
        cycle_points.append([x + radius * np.cos(theta[i]), y + radius * np.sin(theta[i])])
    return Polygon(cycle_points)

def cos_multi(v1, v2):  # when the rotate is unclock the value > 0

    return v1[1] * v2[0] - v1[0] * v2[1]

def vector_norm(v):

    return np.sqrt(np.sum(np.square(v)))

def make_agent(central_pos,radius):
    return Agent(central_pos,radius)

class World():
    def __init__(self, points, agents_pos,radius):
        self.agents = [make_agent(agents_pos[i],radius) for i in range(agents_pos.shape[0])]
        self.edge = Polygon(points).convex_hull

class Agent():
    def __init__(self, central_pos,radius):  # note all the coords are operared based np.array
        self.central_pos = np.array(central_pos)
        self.radius = radius
        self.sense_area = get_circle(self.central_pos,self.radius)
        self.unitity_area = None
        self.Q = []
        self.R = []
        self.neighbors = None
        self.calculate_A = []
        self.calculate_B = []
        self.p_v = []

    def reset_central(self,central_pos):
        self.central_pos = np.array(central_pos)
        self.sense_area = get_circle(self.central_pos, self.radius)
        self.unitity_area = None
    def show_sense_area(self):
        figure(self.sense_area.exterior.coords[:])

    def get_neighbors(self, world):
        self.neighbors = []
        for agent in world.agents:
            if self is agent:continue
            if self.sense_area.intersects(agent.sense_area):
                self.neighbors.append(agent)

    def reset_calculate(self):
        self.p_v = []
        self.calculate_A = []
        self.calculate_B = []

    def compute_edge(self, world):
        assert isinstance(world, World)
        lines = []
        intersect_pos = []
        vs_area = self.sense_area
        for other in self.neighbors:
            intersect_pos.append(0.5 * self.central_pos + 0.5 * other.central_pos)
            u, _, line = unitity(self.sense_area,
                                 other.sense_area)  # the line is the intersection between the agent and other agent
            vs_area = vs_area.intersection(u)
            self.p_v.append(vector_norm(np.array(line.coords[0]) - np.array(line.coords[1])))
            lines.append(line)
        u = vs_area.intersection(world.edge)

        boundary = u.boundary
        lines_A = self.sense_area.boundary.intersection(u.boundary)
        if not lines_A.is_empty and not isinstance(lines_A,Point):
            if isinstance(lines_A,LineString):
                print(lines_A)
                lines_A = [lines_A]
            for line in lines_A:
                if isinstance(line,Point): continue
                coord0 = np.array(line.coords[0])
                coord1 = np.array(line.coords[1])
                coord = 0.5 *coord0 + 0.5 *coord1
                n = coord - self.central_pos
                n = n / vector_norm(n)
                length = line.length
                self.calculate_A.append(dict(v_n = n,length = length))
            # if coord[0] == 0:
            #     if coord[1] >= 0:
            #         self.calculate_A['up'].append(vector_norm(coord))
            #     else:
            #         self.calculate_A['down'].append(vector_norm(coord))
            # else:
            #     if coord[0] >= 0:
            #         self.calculate_A['right'].append(vector_norm(coord))
            #     else:
            #         self.calculate_A['left'].append(vector_norm(coord))
        # plt.figure()
        # figure(u.exterior.coords[:],color = 'y')


        for i in range(len(lines)):
            l = lines[i].intersection(boundary)

            if not isinstance(l, LineString):
                if isinstance(l,geometry.MultiPoint):
                    #print('warning this is a Multipoint')
                    l = LineString(l)
                    # plt.figure()
                    # figure(u.boundary.coords[:])
                    # figure(l.coords[:],color='r')
                    # plt.show()
                elif isinstance(l,Point):
                    #print('warning this is a point:{}'.format(l))
                    l = ops.snap(lines[i], boundary,tolerance=0.1)
                    l = l.intersection(boundary)
                    if isinstance(l,geometry.MultiLineString):
                        max_line = l[0]
                        for line in l:
                           if line.length > max_line.length:
                               max_line  = line
                        l = max_line
                    # plt.figure()
                    # figure(u.boundary.coords[:])
                    # figure(l.coords[:], color='r')
                    # plt.show()

                elif isinstance(l,geometry.MultiLineString):
                    points =[]
                    points.append(l[0].coords[0])
                    points.append(l[-1].coords[1])
                    l = LineString(points)
                    # print('MultilineString')
                    # plt.figure()
                    # figure(u.boundary.coords[:])
                    # figure(l.coords[:], color='r')
                    # plt.show()
                else:
                    print(l)
                    plt.figure()
                    figure(u.boundary.coords[:])
                    for line in l:
                        figure(line.coords[:],color='b')
                    plt.show()
                    raise ValueError
            #figure(l.coords[:],color='r')
            if l.is_empty:
                self.calculate_B.append(None)
                continue
            else:
                coord0 = np.array(l.coords[0])
                coord1 = np.array(l.coords[1])
                v0 = coord0 - intersect_pos[i]
                v1 = coord1 - intersect_pos[i]
                v = intersect_pos[i] - self.central_pos
                index = 0 if cos_multi(v0, v) >= cos_multi(v1, v) else 1
                edge_upper = np.sign(cos_multi(v0, v)) * vector_norm(v0) if index == 0 else np.sign(cos_multi(v1, v)) * vector_norm(v1)
                edge_low = np.sign(cos_multi(v1, v)) * vector_norm(v1) if index ==0 else  np.sign(cos_multi(v0, v)) * vector_norm(v0)
                self.calculate_B.append((edge_low, edge_upper))
        #plt.show()

        self.unitity_area = u
        return u

    def integral_c(self,other:'Agent'):
        p = np.zeros(2)
        c1 = np.zeros((2,2))
        c2 = np.zeros((2,2))
        x_i,x_j = self.central_pos,other.central_pos
        p[0], p[1]  = 0.5 * (x_i[1] - x_j[1]),0.5 * (x_j[0] - x_i[0])
        p = vector_norm(p)
        c1[0][0] = 0.5
        c1[1][1] = 0.5
        c2[0][0] = -1/16 * p**-3 * (x_i[1] - x_j[1]) * (x_i[0] - x_j[0])
        c2[0][1] = 1/4 * p**-3 * (p**2 - 0.25 * (x_i[1] - x_j[1])**2)
        c2[1][0] = -1/4 * p**-3 * (p**2 - 0.25 * (x_i[0] -x_j[0])**2)
        c2[1][1] = 1/16 * p**-3 * (x_j[0] - x_i[0]) * (x_j[1] - x_i[1])
        return c1 , c2
    def computer_graindient_U(self,world = None):
        graindient1 = np.zeros(2)
        for line in self.calculate_A:
            graindient1 += line['v_n'] * line['length']
        graindient2 = np.zeros(2)
        other_graidient = []
        for i,line in enumerate(self.calculate_B):
            if line is None: continue
            v_n = self.neighbors[i].central_pos - self.central_pos
            v_n = v_n / vector_norm(v_n)
            integral_low,integral_upper = line[0],line[1]
            c1,c2 = self.integral_c(self.neighbors[i])
            c1 = c1 * (integral_upper - integral_low)
            c2 = c2 * (integral_upper**2 - integral_low*2)
            graindient2 += v_n @ (c1 + c2)
        return graindient1,graindient2

frame = Polygon([[0, 0], [10, 0], [5, 10]])

points = frame.boundary.coords[:]
pos = np.array([[2,2],[5,2],[4,4],[3,3]])

world = World(points=points, agents_pos=pos,radius=2)
# figure(world.edge.exterior.coords[:],color='b')

for iter_count in range(200):
    u, r_central = [], []
    for agent in world.agents:
        agent.get_neighbors(world)
        agent.reset_calculate()
        u.append(agent.compute_edge(world))
        g1,g2 = agent.computer_graindient_U()
        g = g1 + g2
        g = g/ (vector_norm(g) + 1e-6)
        r_central .append(agent.central_pos + 0.02*g)
    s = [agent.sense_area for agent in world.agents]
    if iter_count % 30 ==0:
        print('the full coverage area:{} and :{}'.format(ops.unary_union(u).area, ops.unary_union(s).intersection(world.edge).area))
    i = 0
    for agent in world.agents:
        agent.reset_central(r_central[i])
        i +=1
plt.figure()
figure(world.edge.exterior.coords[:],color='b')
for agent in world.agents:
    figure(agent.sense_area.exterior.coords[:],color='r')









# for agent in world.agents:
#     print(agent.calculate_B)
# for agent in world.agents:
#     print(agent.calculate_A)

plt.show()

# plt.figure()
# figure(frame.exterior.coords[:])
# figure(square1.exterior.coords[:])
# figure(square2.exterior.coords[:])
# figure(square3.exterior.coords[:])
# plt.show()

# plt.figure()
# figure(square1.exterior.coords[:])
# figure(square2.exterior.coords[:])
# figure(square3.exterior.coords[:])
#
# u = [None] * 3
# lines = [[],[],[]]
# plt.figure()
# for i, bx in enumerate(boxs):
#     u[i] = bx
#     for j in range(len(boxs)):
#         if j == i: continue
#         intersect, _ , line = unitity(bx, boxs[j])
#         u[i] = intersect.intersection(u[i])
#         lines[i].append(line)
#     # plt.figure()
#     figure(u[i].exterior.coords[:])
# for U in u:
#     print(U.exterior.coords[:][::-1])
#
# centrals = []
# for b in boxs:
#     centrals.append(np.array(b.centroid.coords[:][0]))
# coord1 = np.array(lines[2][1].coords[0])
# coord2 = np.array(lines[2][1].coords[1])
# print(coord1, coord2)
# coord3 = 0.5 *centrals[0] + 0.5 *centrals[2]
# print(coord3)
#
# v1 = coord3 - centrals[0]
# v2 = coord1 - coord3
# v3 = coord2 - coord3
# print('v1:{} v2:{}'.format(v1,v2))
# print('v1_:{},v2_:{}'.format(cos_multi(v1,v2),cos_multi(v1,v3)))

