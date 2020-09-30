from enum import Enum
import heapq
import copy
import numpy as np
import math
import random

class DStar:

    class Node:

        def __init__(self, coos):
            self.coos = coos
            self.state = DStar.State.NEW
            self.h = 0
            self.k = 0
            self.b = None  # back-pointer
            self.neighbors = dict()

        def get_coos(self):
            return self.coos

        def get_x(self):
            return self.coos[0]

        def get_y(self):
            return self.coos[1]

        def set_state(self, state):
            self.state = state

        def get_state(self):
            return self.state

        def set_h(self, h):
            self.h = h

        def get_h(self):
            return self.h

        def set_k(self, k):
            self.k = k

        def get_k(self):
            return self.k

        def add_neighbor(self, neighbor, cost):
            self.neighbors[neighbor] = cost

        def get_neighbors(self):
            return self.neighbors.keys()

        def set_back_pointer(self, neighbor):
            self.b = neighbor

        def get_back_pointer(self):
            return self.b

        # cost to neighbor
        def get_c(self, neighbor):
            return self.neighbors[neighbor]

        # sensor-computer cost to neighbor
        def get_r(self, neighbor):
            pass

        # sets cost to other node given a cost value
        def set_cost(self, other, cost_value):
            self.neighbors[other] = cost_value

        def print_node(self):
            print("state %s and coos %s and back-pointer %s" % (self.state, self.coos, self.b))

    class State(Enum):
        NEW = 0
        OPEN = 1
        CLOSED = 2
        RAISE = 3
        LOWER = 4

    def __init__(self, parent, elevation_map, start_coos, goal_coos):
        self.queue = []
        self.map = elevation_map
        self.path = []
        self.height, self.width = self.map.shape
        self.graph = copy.deepcopy(self.map)
        self.graph = np.empty([self.height, self.width], dtype=DStar.Node)
        self.heuristic = int(parent.get_heuristic())
        self.init_nodes()
        self.start = self.graph[start_coos[1]][start_coos[0]]
        self.goal = self.graph[goal_coos[1]][goal_coos[0]]
        self.current_state = self.start

    # how much we penalize taking high cost environment paths
    #heuristic = 20

    def init_nodes(self):
        for y in range(self.height):
            for x in range(self.width):
                new_node: DStar.Node = DStar.Node((x, y))
                self.graph[y][x] = new_node
        for y in range(self.height):
            for x in range(self.width):
                for (x_n, y_n) in self.get_neighboring_coos(x, y):
                    current_node = self.graph[y][x]
                    cost = 1 if x_n == x or y_n == y else 1.41
                    cost += random.uniform(-0.01, 0.01) # for some reason, heapq doesn't handle elements with exact same k_value...
                    cost *= (1 + self.heuristic * abs(self.map[y_n][x_n] - self.map[y][x]))
                    current_node.add_neighbor(self.graph[y_n][x_n], cost)

    def get_neighboring_coos(self, x_0, y_0):
        neighboring_coos = []
        for y in range(-1, 2):
            for x in range(-1, 2):
                if self.is_in_bounds(y_0 + y, x_0 + x) and not (y == 0 and x == 0):
                    neighboring_coos.append((x_0 + x, y_0 + y))
        return neighboring_coos

    def is_in_bounds(self, y, x):
        return 0 <= x < self.width and 0 <= y < self.height

    def run(self):
        # Planning
        self.goal.set_h(0)
        self.insert(self.goal, self.goal.get_h())
        self.path = self.init_plan()
        if not self.path:
            print("No path found\n")
            return None
        self.print_path()
        return self.path
        # Re-planning
        # while self.current_state != self.goal:
        #     self.prepare_repair([])
        #     new_path = self.repair_replan()
        #     if not self.path:
        #         print("Failed to re-plan path, returning old path\n")
        #         return self.path
        #     else:
        #         self.path = new_path
        #     # current_state = second element of P (move to next state in P)
        #     # probably do this when getting a notification that the rover
        #     # has moved and is now in a new cell
        # return self.current_state

    def print_path(self):
        for node in self.path:
            (x, y) = node.get_coos()
            print("--> (%s, %s)"%(x,y), end='')
        print("\n")

    # modify costs based on sensors
    def prepare_repair(self, list_of_changes):
        for node in list_of_changes:
            for neighbor in node.neighbors:
                new_cost = 1  # replace by some value
                # if cost changed, modify cost (in both directions) then
                self.modify_cost(neighbor, node, new_cost)
                self.modify_cost(node, neighbor, new_cost)

    def insert(self, node, h_new):
        if node.state == self.State.NEW:
            node.set_k(h_new)
        elif node.state == self.State.OPEN:
            node.set_k(min(node.get_k(), h_new))
        elif node.state == self.State.CLOSED:
            node.set_k(min(node.get_h(), h_new))
        node.set_h(h_new)
        node.set_state(self.State.OPEN)
        heapq.heappush(self.queue, (node.get_k(), node))
        # sort open list : no need if it s a priority queue

    def modify_cost(self, node1: Node, node2: Node, cost_value):
        node1.set_cost(node2, cost_value)
        if node1.state == self.State.CLOSED:
            self.insert(node1, node1.get_h())
        return self.get_k_min()

    def init_plan(self):
        while True:
            k_min = self.process_state()
            #print("k_min : %s"%(k_min))
            if k_min == -1 or self.current_state == self.State.CLOSED:
                break
        return self.get_back_pointer_list()

    def repair_replan(self):
        while True:
            k_min = self.process_state()
            if k_min >= self.current_state.get_h() or k_min == -1:
                break
        return self.get_back_pointer_list()

    def get_min_state(self):
        return self.queue[0][1]

    # TODO added the -1 clause to not crash...
    def get_k_min(self):
        return self.queue[0][0] if len(self.queue) > 0 else -1

    def process_state(self):
        node: DStar.Node = self.get_min_state()
        if node is None:
            return -1
        k_old = self.get_k_min()
        heapq.heappop(self.queue)
        if k_old < node.get_h():
            for neighbor in node.get_neighbors():
                if neighbor.state != self.State.NEW and neighbor.get_h() <= k_old \
                        and node.get_h() > neighbor.get_h() + neighbor.get_c(node):
                    node.set_back_pointer(neighbor)
                    node.set_h(neighbor.get_h + neighbor.get_c(node))
        elif math.isclose(k_old, node.get_h()):
            for neighbor in node.neighbors:
                if neighbor.state == self.State.NEW or \
                        (neighbor.get_back_pointer() == node and not math.isclose(neighbor.get_h(), node.get_h() + node.get_c(neighbor))) or \
                        (neighbor.get_back_pointer() != node and neighbor.get_h() > node.get_h() + node.get_c(neighbor)):
                    neighbor.set_back_pointer(node)
                    self.insert(neighbor, node.get_h() + node.get_c(neighbor))
        else:
            for neighbor in node.neighbors:
                if neighbor.state == self.State.NEW or \
                        (neighbor.get_back_pointer() == node and not math.isclose(neighbor.get_h(), node.get_h() + node.get_c(neighbor))):
                    neighbor.set_back_pointer(node)
                    self.insert(neighbor, node.get_h() + node.get_c(neighbor))
                else:
                    if neighbor.get_back_pointer() != node and neighbor.get_h() > node.get_h() + node.get_c(neighbor):
                        self.insert(node, node.get_h())
                    else:
                        if neighbor.get_back_pointer() != node and node.get_h() > neighbor.get_h() + node.get_c(neighbor) and \
                            neighbor.get_state() == self.State.CLOSED and neighbor.get_h() > k_old:
                            self.insert(neighbor, neighbor.get_h())
        return self.get_k_min()

    # return list of back-pointers from goal to rover position
    def get_back_pointer_list(self):
        path = []
        node_iterator = self.start
        node_iterator.print_node()
        while node_iterator != self.goal:
            node_iterator = node_iterator.get_back_pointer()
            path.append(node_iterator)
        path.append(node_iterator)
        return path
