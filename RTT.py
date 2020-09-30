import random
import math


class RTT:
    class Tree:

        def __init__(self, start):
            self.root = self.Node(None, start)

        def nearest(self, coos):
            return self.root.get_coos()

        def near(self, node):
            pass

        def rewire(self, a, b, c):
            pass

        def insert_node(self, parent, coos):
            pass

        class Node:

            def __init__(self, parent, coos):
                self.coos = coos
                self.parent = parent
                self.children = []
                self.data = []

            def get_coos(self):
                return self.coos

    def __init__(self, parent, elevation_map, start_coos, goal_cos):
        self.map = elevation_map
        self.height, self.width = self.map.shape
        self.start = start_coos
        self.goal = goal_cos
        self.T = self.initialize_tree()

    def run(self, nb_iter):
        for i in range(nb_iter):
            z_rand = self.sample()
            z_nearest = self.T.nearest(z_rand)
            z_new = self.steer(z_nearest, z_rand)
            if self.obstacle_free(z_new):
                z_near = self.T.near(z_new)
                z_min = self.choose_parent(z_near, z_nearest, z_new)
                self.T.insert_node(z_min, z_new)
                self.T.rewire(z_near, z_min, z_new)

    def choose_parent(self, z_near, z_nearest, z_new):
        if not z_near:
            return z_nearest
        else:
            min_dist = 1000000
            z_min = None
            for node in z_near:
                dist = self.distance(z_new, node)
                if dist < min_dist:
                    min_dist = dist
                    z_min = node
            return z_min

    @staticmethod
    def obstacle_free(self, z):
        return True

    def steer(self, z_nearest, z_rand):
        max_dist = 1
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
