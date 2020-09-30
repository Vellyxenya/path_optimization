import random
import math
import numpy as np

class RTT:

    class Tree:

        def __init__(self, start):
            # self.root = self.Node(None, start)
            self.xs = [float(start[0])]
            self.ys = [float(start[1])]
            self.costs = [0.0]
            self.parents = [0]
            self.nb_nodes = 1

        def nearest(self, coos):
            min_dist = 1000000
            min_node = None
            # for (x, y) in zip(self.xs, self.ys):
            for i in range(self.nb_nodes):
                x, y = self.xs[i], self.ys[i]
                dist = self.dist(coos, (x, y))  # TODO change metric
                if dist < min_dist:
                    min_dist = dist
                    min_node = i
            return min_node

        def dist(self, n1, n2):
            return np.sqrt((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2)

        def near(self, coos):
            gamma = 10
            search_radius = gamma * math.sqrt(math.log(self.nb_nodes)/self.nb_nodes)
            node_indices = []
            node_distances = []
            for i in range(self.nb_nodes):
                x, y = self.xs[i], self.ys[i]
                dist = self.dist(coos, (x, y))  # TODO change metric
                if dist < search_radius:
                    node_indices.append(i)
                    node_distances.append(dist)
            return node_indices, node_distances

        def get_node(self, index):
            node = (self.xs[index], self.ys[index])
            return node

        def get_cost(self, index):
            return self.costs[index]

        def rewire(self, a, b, c):
            pass

        def insert_node(self, parent_index, coos):
            self.xs.append(coos[0])
            self.ys.append(coos[1])
            self.parents.append(parent_index)
            cost = self.get_cost(parent_index) + self.dist(coos, self.get_node(parent_index))
            self.costs.append(cost)
            self.nb_nodes += 1
            return self.nb_nodes - 1

        def reconstruct_path(self, final_index):
            path = []
            iterator = final_index
            path.append(self.get_node(iterator))
            while iterator != 0:
                iterator = self.parents[iterator]
                path.insert(0, self.get_node(iterator))
            print(path)
            return path

        # class Node:
        #
        #     def __init__(self, parent, coos):
        #         self.coos = coos
        #         self.parent = parent
        #         self.children = []
        #         self.data = []
        #
        #     def get_coos(self):
        #         return self.coos

    def __init__(self, parent, elevation_map, start_coos, goal_cos):
        self.map = elevation_map
        self.height, self.width = self.map.shape
        self.start = start_coos
        self.goal = goal_cos
        self.T = self.initialize_tree()
        self.goal_found = False
        self.goal_threshold = 0.3
        self.temp_final = None

    def run(self, max_iter):
        i = 0
        final_index = None
        while i < max_iter or not self.goal_found:
            z_rand = self.sample()
            z_nearest_index = self.T.nearest(z_rand)
            z_new = self.steer(z_nearest_index, z_rand)
            if self.obstacle_free(z_new, z_new):  # TODO this method is bs for now
                z_near_indices, distances = self.T.near(z_new)
                z_min_index = self.choose_parent(z_near_indices, distances, z_nearest_index, z_new)
                z_new_index = self.T.insert_node(z_min_index, z_new)
                # self.T.rewire(z_near, z_min, z_new)  # TODO implement this
                if self.is_goal_reached(z_new):
                    final_index = z_new_index
                    print("Goal found in %d iterations" % i)
            i += 1
        if not self.goal_found:
            print("Terminated without finding goal")
            return None
        else:
            path = self.T.reconstruct_path(final_index)
            return path

    def is_goal_reached(self, z_new):
        if not self.goal_found and self.distance(z_new, self.goal) < self.goal_threshold:
            self.goal_found = True
            print(">>>>>>>>>>>>>>>>>GOAL FOUND<<<<<<<<<<<<<<<<<<<<")
        return self.goal_found

    def choose_parent(self, z_near_indices, distances, z_nearest, z_new):
        if not z_near_indices:
            return z_nearest
        else:
            min_dist = 1000000
            z_min_index = None
            for i, node_index in enumerate(z_near_indices):
                dist = distances[i] + self.T.get_cost(node_index)
                if dist < min_dist:
                    min_dist = dist
                    z_min_index = node_index
            return z_min_index

    @staticmethod
    def obstacle_free(self, z):
        return True

    def steer(self, z_nearest_index, z_rand):
        max_dist = 1
        z_nearest = self.T.get_node(z_nearest_index)
        distance = self.distance(z_nearest, z_rand)
        if distance < max_dist:
            return z_rand
        else:
            ratio = max_dist / distance
            dx = z_rand[0] - z_nearest[0]
            dy = z_rand[1] - z_nearest[1]
            new_x = z_nearest[0] + ratio * dx
            new_y = z_nearest[1] + ratio * dy
            new_coos = (new_x, new_y)
            return new_coos

    @staticmethod
    def distance(z1, z2):
        return math.sqrt((z2[0] - z1[0]) ** 2 + (z2[1] - z1[1]) ** 2)

    def sample(self):
        x = random.uniform(0, 1) * self.width
        y = random.uniform(0, 1) * self.height
        coos = (x, y)
        return coos

    def initialize_tree(self):
        return self.Tree(self.start)
