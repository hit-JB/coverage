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
import cv2

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


def make_agent(central_pos, radius, num, world):
    return Agent(central_pos, radius, num, world)


def get_agent(agents, num):
    if isinstance(agents, Agent):
        return agents
    for i, agent in enumerate(agents):
        if agent.num == num:
            return i, agent
    return None

def offset_points(offset,points:np.ndarray):
    if points.ndim == 1:
        points = points + offset
        return np.array(points,dtype=np.int)
    else:
        for i in range(points.shape[0]):
            points[i] = np.array((points[i] + offset),dtype=np.int)
        return points

class World():
    def __init__(self, points, agents_pos=None, radius=None):
        self.agents = None if agents_pos is None and radius is None else [make_agent(agents_pos[i], radius, i, self) for
                                                                          i in range(agents_pos.shape[0])]
        self.k = 50
        self.edge = Polygon(points)
        _,_,maxx,maxy = self.edge.bounds
        self.W = int(max(maxy, maxx) * self.k * 2)
    def render(self,iteration):
        points = np.array(self.edge.exterior.coords[:]) * self.k
        offset = np.array([self.W /4,self.W/4])
        points = offset_points(offset,points)
        img = np.zeros((self.W, self.W, 3), np.uint8)
        img[:] = (200, 200, 200)

        for i,point in enumerate(points):
            j = (i+1) %(len(points))
            cv2.line(img, (int(points[i][0]),int(points[i][1])), (int(points[j][0]),int(points[j][1])), (200, 0, 0), 3,lineType=cv2.LINE_AA)
        for agent in self.agents:
            central_pos = agent.central_pos * self.k
            central = offset_points(offset,central_pos)
            radius = agent.radius * self.k
            cv2.circle(img, (central[0], central[1]), int(radius), (0, 0, 200), -1, lineType=cv2.LINE_AA)
            cv2.circle(img, (central[0], central[1]), int(radius/self.k * 5), (40, 40, 40), -1, lineType=cv2.LINE_AA)



        cv2.putText(img, "Iteration: " +str(iteration), (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40),
                    lineType=cv2.LINE_AA)
        cv2.namedWindow("Environment")
        cv2.imshow("Environment", img)
        cv2.waitKey(1)







class base_agent():
    def __init__(self, central_pos, radius, num):
        self.central_pos = np.array(central_pos)
        self.radius = radius
        self.sense_area = get_circle(self.central_pos, self.radius)
        self.num = num
        self.unitity_area = None

    def reset_central(self, central_pos):
        self.central_pos = np.array(central_pos)
        self.sense_area = get_circle(self.central_pos, self.radius)
        self.unitity_area = None


class q_fun():
    def __init__(self, q_ij: dict) -> 'Any':
        self.q_ij = q_ij

    def computer_g(self, agent: base_agent, world: World):
        assert isinstance(agent, base_agent)
        gradient = np.zeros(2)
        for message in self.q_ij:
            message['value'].reset_u(agent)
            message['value'].computer_edge(world)
            gradient += message['value'].computer_g(agent)
        return gradient


class Agent(base_agent):
    def __init__(self, central_pos, radius, num, world):  # note all the coords are operared based np.array
        super(Agent, self).__init__(central_pos, radius, num)
        self.receive_Q = None
        self.receive_R = None
        self.send_R = None
        self.send_Q = None
        self.neighbors = None
        self.U = []
        self.world = world

    def message_change(self):
        self.send_Q = cp.deepcopy(self.receive_R)

    @property
    def Agent_copy(self):
        return base_agent(self.central_pos, self.radius, self.num)

    def receive_message(self, agents):
        self.receive_R = []
        self.receive_Q = []
        for agent in agents:
            if agent.send_Q is None:
                self.receive_Q is None
            else:
                q_ij = []
                for message in agent.send_Q:  # send_Q is the copy of receive_R
                    if message['num'] == self.num: continue
                    q_ij.append(dict(num=message['num'], value=message['value']))
                q_ij = q_fun(q_ij)
                self.receive_Q.append(dict(num=agent.num, value=q_ij))
            for message in agent.send_R:
                if message['num'] == self.num:
                    self.receive_R.append(dict(num=agent.num, value=message['value']))

    def send_r(self):
        self.send_R = []
        self.get_neighbors(self.world)
        agents = [self] + self.neighbors
        r_g = self.message_initialize(agents)
        x_star = [agents[i].central_pos + 0.1 * r_g[i]/(1e-4+vector_norm(r_g[i])) for i in range(len(r_g))]
        agents_cp = [agents[i].Agent_copy for i in range(len(agents))]
        for i,agent in enumerate(agents_cp): agent.reset_central(x_star[i])
        for agent in agents:
            self.send_R.append(dict(num=agent.num, value=U_fun(agents_cp[0], agents_cp[1:])))

    def message_initialize(self, agents):  # the agents is the form of list [self, self.neighbors]
        agents_cp = [agent.Agent_copy for agent in agents]
        U_self = U_fun(agents_cp[0], agents_cp[1:])
        U_self.computer_edge(self.world)

        u_g = [U_self.computer_g(agent) for agent in agents_cp]

        if self.receive_Q is None:
            return u_g
        for message in self.receive_Q:
            p = get_agent(agents, message['num'])
            if p is None:
                continue
            i, agent = p[0], p[1].Agent_copy
            u_g[i] += message['value'].computer_g(agent, self.world)
        return u_g

    def show_sense_area(self):
        figure(self.sense_area.exterior.coords[:], color='r')

    def update_state(self):
        g = np.zeros(2)
        for message in self.receive_R:
            u = message['value']
            u.reset_u(self.Agent_copy)
            u.computer_edge(self.world)
            g += u.computer_g(self.Agent_copy)
        step = 0.1 * g/vector_norm(g)
        central_points = self.central_pos + step
        self.reset_central(central_points)

    def get_neighbors(self, world):
        self.neighbors = []
        for agent in world.agents:
            if self is agent:  continue
            if self.sense_area.intersects(agent.sense_area):
                self.neighbors.append(agent)


class U_fun():
    def __init__(self, central_agent=None, other_agents=None):
        self.a = central_agent
        self.b = other_agents

        self.agent = central_agent.sense_area
        self.central_pos = np.array(self.agent.centroid.coords[0])
        self.others = [other.sense_area for other in other_agents]

        self.num = [agent.num for agent in [self.a] + self.b]
        self.g = None
        self.u = None
        self.A = []
        self.B = []

    def reset_u(self, agent):
        assert isinstance(agent, Agent) or isinstance(agent, base_agent)
        if agent.num == self.a.num:
            self.a = agent
            self.agent = self.a.sense_area
            self.central_pos = np.array(self.agent.centroid.coords[0])
        else:
            for i, ag in enumerate(self.b):
                if ag.num == agent.num:
                    self.b[i] = agent
            self.others = [other.sense_area for other in self.b]
        self.num = [agent.num for agent in [self.a] + self.b]
        self.g = None
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
            if line is None or self.u.distance(line) > 1e-5:
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


        g_self = np.zeros(2)
        g_other = []
        for line in self.A:
            g_self += line['v_n'] * line['length']

        for i, line in enumerate(self.B):
            if line is None:
                g_other.append(np.zeros(2))
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
        self.g = [g_self] + g_other

    def computer_g(self, agent):
        for i, num in enumerate(self.num):
            if agent.num == num:
                return self.g[i]  # return the gradient of the given agent
        return np.zeros(2)

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
    pos = np.array([[2, 2], [3.5, 2], [3, 3],[4,4],[10,10],[0,10]])
    #world = World([[0, 0], [8, 0], [8, 8],[0,8]], agents_pos=pos, radius=2)
    world = World([[0,0],[10,0],[10,10],[0,10]],agents_pos=pos, radius=2)
    agents = world.agents
    for i in range(200):
        for agent in agents:
            agent.send_r()
        for agent in agents:
            agent.receive_message([agent]+agent.neighbors)
        for agent in agents:
            agent.message_change()
            agent.update_state()
        area =ops.unary_union([agent.sense_area for agent in agents])
        print('the coverage area is :{}'.format(area.intersection(world.edge).area))
        world.render(i)
    plt.figure()
    figure(world.edge.exterior.coords[:])
    for agent in agents:
        figure(agent.sense_area.exterior.coords[:],color='r')
    plt.show()
#     figure(world.edge.exterior.coords[:])
#     for agent in world.agents:
#         agent.show_sense_area()
# if __name__ == '__main__':
#     pos = np.array([[2, 2], [5, 2], [4, 4], [3, 3]])
#     world = World([[0, 0], [10, 0], [5, 10]], agents_pos=pos, radius=2)
#     agents = world.agents
#     plt.figure()
#     figure(world.edge.exterior.coords[:])
#     for agent in world.agents:
#         agent.show_sense_area()
#     for t in range(400):
#         for agent in agents:
#             agent.update_state(world)
#         if t % 10 == 0:
#             cover_area = ops.unary_union([agent.u for agent in agents])
#             print('in {} the full coverage area is :{}'.format(t, cover_area.area))
#     plt.figure()
#     figure(world.edge.exterior.coords[:])
#     for agent in world.agents:
#         agent.show_sense_area()
#     plt.show()

# class Agent():
#     def __init__(self, central_pos,radius):  # note all the coords are operared based np.array
#         self.central_pos = np.array(central_pos)
#         self.radius = radius
#         self.sense_area = get_circle(self.central_pos,self.radius)
#         self.unitity_area = None
#         self.Q = []
#         self.R = []
#         self.neighbors = None
#         self.calculate_A = []
#         self.calculate_B = []
#         self.p_v = []
#
#     def reset_central(self,central_pos):
#         self.central_pos = np.array(central_pos)
#         self.sense_area = get_circle(self.central_pos, self.radius)
#         self.unitity_area = None
#     def show_sense_area(self):
#         figure(self.sense_area.exterior.coords[:])
#
#     def get_neighbors(self, world):
#         self.neighbors = []
#         for agent in world.agents:
#             if self is agent:continue
#             if self.sense_area.intersects(agent.sense_area):
#                 self.neighbors.append(agent)
#
#     def reset_calculate(self):
#         self.p_v = []
#         self.calculate_A = []
#         self.calculate_B = []
#
#     def compute_edge(self, world):
#         assert isinstance(world, World)
#         lines = []
#         intersect_pos = []
#         vs_area = self.sense_area
#         for other in self.neighbors:
#             intersect_pos.append(0.5 * self.central_pos + 0.5 * other.central_pos)
#             u, _, line = unitity(self.sense_area,
#                                  other.sense_area)  # the line is the intersection between the agent and other agent
#             vs_area = vs_area.intersection(u)
#             self.p_v.append(vector_norm(np.array(line.coords[0]) - np.array(line.coords[1])))
#             lines.append(line)
#         u = vs_area.intersection(world.edge)
#
#         boundary = u.boundary
#         lines_A = self.sense_area.boundary.intersection(u.boundary)
#         if not lines_A.is_empty and not isinstance(lines_A,Point):
#             if isinstance(lines_A,LineString):
#                 print(lines_A)
#                 lines_A = [lines_A]
#             for line in lines_A:
#                 if isinstance(line,Point): continue
#                 coord0 = np.array(line.coords[0])
#                 coord1 = np.array(line.coords[1])
#                 coord = 0.5 *coord0 + 0.5 *coord1
#                 n = coord - self.central_pos
#                 n = n / vector_norm(n)
#                 length = line.length
#                 self.calculate_A.append(dict(v_n = n,length = length))
#             # if coord[0] == 0:
#             #     if coord[1] >= 0:
#             #         self.calculate_A['up'].append(vector_norm(coord))
#             #     else:
#             #         self.calculate_A['down'].append(vector_norm(coord))
#             # else:
#             #     if coord[0] >= 0:
#             #         self.calculate_A['right'].append(vector_norm(coord))
#             #     else:
#             #         self.calculate_A['left'].append(vector_norm(coord))
#         # plt.figure()
#         # figure(u.exterior.coords[:],color = 'y')
#
#
#         for i in range(len(lines)):
#             l = lines[i].intersection(boundary)
#
#             if not isinstance(l, LineString):
#                 if isinstance(l,geometry.MultiPoint):
#                     #print('warning this is a Multipoint')
#                     l = LineString(l)
#                     # plt.figure()
#                     # figure(u.boundary.coords[:])
#                     # figure(l.coords[:],color='r')
#                     # plt.show()
#                 elif isinstance(l,Point):
#                     #print('warning this is a point:{}'.format(l))
#                     l = ops.snap(lines[i], boundary,tolerance=0.1)
#                     l = l.intersection(boundary)
#                     if isinstance(l,geometry.MultiLineString):
#                         max_line = l[0]
#                         for line in l:
#                            if line.length > max_line.length:
#                                max_line  = line
#                         l = max_line
#                     # plt.figure()
#                     # figure(u.boundary.coords[:])
#                     # figure(l.coords[:], color='r')
#                     # plt.show()
#
#                 elif isinstance(l,geometry.MultiLineString):
#                     points =[]
#                     points.append(l[0].coords[0])
#                     points.append(l[-1].coords[1])
#                     l = LineString(points)
#                     # print('MultilineString')
#                     # plt.figure()
#                     # figure(u.boundary.coords[:])
#                     # figure(l.coords[:], color='r')
#                     # plt.show()
#                 else:
#                     print(l)
#                     plt.figure()
#                     figure(u.boundary.coords[:])
#                     for line in l:
#                         figure(line.coords[:],color='b')
#                     plt.show()
#                     raise ValueError
#             #figure(l.coords[:],color='r')
#             if l.is_empty:
#                 self.calculate_B.append(None)
#                 continue
#             else:
#                 coord0 = np.array(l.coords[0])
#                 coord1 = np.array(l.coords[1])
#                 v0 = coord0 - intersect_pos[i]
#                 v1 = coord1 - intersect_pos[i]
#                 v = intersect_pos[i] - self.central_pos
#                 index = 0 if cos_multi(v0, v) >= cos_multi(v1, v) else 1
#                 edge_upper = np.sign(cos_multi(v0, v)) * vector_norm(v0) if index == 0 else np.sign(cos_multi(v1, v)) * vector_norm(v1)
#                 edge_low = np.sign(cos_multi(v1, v)) * vector_norm(v1) if index ==0 else  np.sign(cos_multi(v0, v)) * vector_norm(v0)
#                 self.calculate_B.append((edge_low, edge_upper))
#         #plt.show()
#
#         self.unitity_area = u
#         return u
#
#     def integral_c(self,other:'Agent'):
#         p = np.zeros(2)
#         c1 = np.zeros((2,2))
#         c2 = np.zeros((2,2))
#         x_i,x_j = self.central_pos,other.central_pos
#         p[0], p[1]  = 0.5 * (x_i[1] - x_j[1]),0.5 * (x_j[0] - x_i[0])
#         p = vector_norm(p)
#         c1[0][0] = 0.5
#         c1[1][1] = 0.5
#         c2[0][0] = -1/16 * p**-3 * (x_i[1] - x_j[1]) * (x_i[0] - x_j[0])
#         c2[0][1] = 1/4 * p**-3 * (p**2 - 0.25 * (x_i[1] - x_j[1])**2)
#         c2[1][0] = -1/4 * p**-3 * (p**2 - 0.25 * (x_i[0] -x_j[0])**2)
#         c2[1][1] = 1/16 * p**-3 * (x_j[0] - x_i[0]) * (x_j[1] - x_i[1])
#         return c1 ,c2,-c2
#     def computer_graindient_U(self,world = None):
#         graindient1 = np.zeros(2)
#         others_graindient = []
#         for line in self.calculate_A:
#             graindient1 += line['v_n'] * line['length']
#         graindient2 = np.zeros(2)
#         other_graidient = []
#         for i,line in enumerate(self.calculate_B):
#             if line is None:
#                 other_graidient.append(None)
#                 continue
#             v_n = self.neighbors[i].central_pos - self.central_pos
#             v_n = v_n / vector_norm(v_n)
#             integral_low,integral_upper = line[0],line[1]
#             c1,c2,c2_other = self.integral_c(self.neighbors[i])
#             c1 = c1 * (integral_upper - integral_low)
#             c2 = c2 * (integral_upper**2 - integral_low*2)
#             c2_other * (integral_upper**2 - integral_low*2)
#             other_graidient.append(c1+c2_other)
#             graindient2 += v_n @ (c1 + c2)
#         return graindient1, graindient2,other_graidient


# #if __name__ == '__main__':
#     frame = Polygon([[0, 0], [10, 0], [5, 10]])
#
#     points = frame.boundary.coords[:]
#     pos = np.array([[2,2],[5,2],[4,4],[3,3]])
#
#     world = World(points=points, agents_pos=pos,radius=2)
#     # figure(world.edge.exterior.coords[:],color='b')
#
#     for iter_count in range(200):
#         u, r_central = [], []
#         for agent in world.agents:
#             agent.get_neighbors(world)
#             agent.reset_calculate()
#             u.append(agent.compute_edge(world))
#             g1,g2 = agent.computer_graindient_U()
#             g = g1 + g2
#             g = g/ (vector_norm(g) + 1e-6)
#             r_central .append(agent.central_pos + 0.02*g)
#         s = [agent.sense_area for agent in world.agents]
#         if iter_count % 30 ==0:
#             print('the full coverage area:{} and :{}'.format(ops.unary_union(u).area, ops.unary_union(s).intersection(world.edge).area))
#         i = 0
#         for agent in world.agents:
#             agent.reset_central(r_central[i])
#             i +=1
#     plt.figure()
#     figure(world.edge.exterior.coords[:],color='b')
#     for agent in world.agents:
#         figure(agent.sense_area.exterior.coords[:],color='r')
#
#
#
#
#
#
#
#
#
#     # for agent in world.agents:
#     #     print(agent.calculate_B)
#     # for agent in world.agents:
#     #     print(agent.calculate_A)
#
#     plt.show()

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
# print('v1_:{},v2_:{}