import random
import math
import numpy as np

from rtree import index


class RRT:
    class Tree:

        def __init__(self, parent, start):
            # self.root = self.Node(None, start)
            self.rrt = parent
            self.xs = [float(start[0])]
            self.ys = [float(start[1])]
            self.costs = [0.0]
            self.parents = [0]
            self.nb_nodes = 1
            self.idx = index.Index()
            self.idx.insert(0, (float(start[0]), float(start[1]), float(start[0]), float(start[1])))

        def nearest(self, coos):
            min_dist = 1000000
            min_node = None
            # for (x, y) in zip(self.xs, self.ys):
            for i in range(self.nb_nodes):
                x, y = self.xs[i], self.ys[i]
                dist = self.rrt.metric(coos, (x, y))  # self.dist(coos, (x, y))  # TODO change metric
                if dist < min_dist:
                    min_dist = dist
                    min_node = i
            return min_node

        # def metric(self, n1, n2):
        # return np.sqrt((n1[0] - n2[0]) ** 2 + (n1[1] - n2[1]) ** 2)

        def near(self, coos):
            gamma = 8
            # search_radius = gamma * math.sqrt(math.log(self.nb_nodes) ** 2 / self.nb_nodes)
            search_radius = gamma * math.sqrt(math.log(self.nb_nodes) ** 2 / self.nb_nodes)
            node_indices = list(self.idx.intersection((coos[0] - search_radius, coos[1] - search_radius,
                                                       coos[0] + search_radius, coos[1] + search_radius)))
            return node_indices

        def get_node(self, index):
            node = (self.xs[index], self.ys[index])
            return node

        def get_cost(self, index):
            return self.costs[index]

        def rewire(self, z_near_indices, z_min_index, z_new_index):
            # TODO check if collision free
            for i in z_near_indices:
                coos_i = (self.xs[i], self.ys[i])
                coos_new = (self.xs[z_new_index], self.ys[z_new_index])
                new_potential_cost = self.get_cost(z_new_index) + self.rrt.metric(coos_i, coos_new)
                if self.get_cost(i) > new_potential_cost:
                    # print("REWIRED")
                    self.parents[i] = z_new_index
                    self.costs[i] = new_potential_cost

        def insert_node(self, parent_index, coos):
            self.xs.append(coos[0])
            self.ys.append(coos[1])
            self.parents.append(parent_index)
            cost = self.get_cost(parent_index) + self.rrt.metric(coos, self.get_node(parent_index))
            self.costs.append(cost)
            self.idx.insert(self.nb_nodes, (coos[0], coos[1], coos[0], coos[1]))  # insert in RTree
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
        self.parent = parent
        self.map = elevation_map
        self.height, self.width = self.map.shape
        self.start = start_coos
        self.goal = goal_cos
        self.T = self.initialize_tree()
        self.goal_found = False
        self.goal_threshold = 5  # 5 # 0.3
        self.temp_final = None
        self.heuristic = 50

    def run(self, max_iter):
        i = 0
        final_index = None
        min_dist_to_goal = 1000000
        while i < max_iter or not self.goal_found:
            z_rand = self.sample()
            # print("z_rand :", z_rand)
            z_nearest_index = self.T.nearest(z_rand)
            # print("z_nearest_index :", z_nearest_index)
            z_new = self.steer(z_nearest_index, z_rand)
            if self.obstacle_free(z_new, z_new):  # TODO this method is bs for now
                z_near_indices = self.T.near(z_new)
                # print(z_near_indices)
                z_min_index = self.choose_parent(z_near_indices, z_nearest_index, z_new)
                # print(z_min_index)
                z_new_index = self.T.insert_node(z_min_index, z_new)
                self.T.rewire(z_near_indices, z_min_index, z_new_index)
                dist, is_reached = self.is_goal_reached(z_new)
                if dist < min_dist_to_goal and is_reached:
                    final_index = z_new_index
                    min_dist_to_goal = dist
                    print("Goal found in %d iterations" % i)
            i += 1
            if i % 100 == 0:
                print("iteration :", i)
            # self.parent.draw_tree(self.T)
        if not self.goal_found:
            print("Terminated without finding goal")
            return None, self.T
        else:
            path = self.T.reconstruct_path(final_index)
            return path, self.T

    def is_goal_reached(self, z_new):
        dist = self.metric(z_new, self.goal)
        if not self.goal_found and dist < self.goal_threshold:
            self.goal_found = True
            print(">>>>>>>>>>>>>>>>>GOAL FOUND<<<<<<<<<<<<<<<<<<<<")
        return dist, self.goal_found

    def choose_parent(self, z_near_indices, z_nearest, z_new):
        if not z_near_indices:
            return z_nearest
        else:
            min_dist = 1000000
            z_min_index = None
            for node_index in z_near_indices:
                dist_to_node = self.metric(self.T.get_node(node_index), z_new)
                dist = dist_to_node + self.T.get_cost(node_index)
                if dist < min_dist:
                    min_dist = dist
                    z_min_index = node_index
            return z_min_index

    @staticmethod
    def obstacle_free(self, z):
        return True

    def steer(self, z_nearest_index, z_rand):
        max_dist = 1  # TODO 0.5
        z_nearest = self.T.get_node(z_nearest_index)
        distance = self.metric(z_nearest, z_rand)
        if distance < max_dist:
            return z_rand
        else:
            dx = z_rand[0] - z_nearest[0]
            dy = z_rand[1] - z_nearest[1]
            angle = math.atan2(dy, dx)
            new_x = z_nearest[0] + max_dist * math.cos(angle)
            new_y = z_nearest[1] + max_dist * math.sin(angle)
            new_coos = (new_x, new_y)
            return new_coos

    def metric(self, z1, z2):
        euclidean_distance = math.sqrt((z2[0] - z1[0]) ** 2 + (z2[1] - z1[1]) ** 2)
        traversed_cells = self.get_traversed_cells(z1, z2)
        cost = 0
        first_cell = traversed_cells[0]
        #print("z1 :", z1, "z2 :", z2)
        #print("traversed cells : ", traversed_cells)
        for next_cell in traversed_cells[1:]:
            cost += abs(self.get_cell_height(first_cell) - self.get_cell_height(next_cell))
            first_cell = next_cell
        return euclidean_distance * (1 + self.heuristic * cost)

    @staticmethod
    def get_traversed_cells(a, b):
        a = (int(a[0]), int(a[1]))
        b = (int(b[0]), int(b[1]))
        #print("a :", a, "b :", b)
        dx = b[0] - a[0]
        dy = b[1] - a[1]

        directionX = np.sign(dx)
        directionY = np.sign(dy)

        directionModifierX = 0 if directionX < 0 else directionX
        directionModifierY = 0 if directionY < 0 else directionY

        currentCell = (int(math.floor(a[0])), int(math.floor((a[1]))))
        targetCell = (math.floor(b[0]), math.floor((b[1])))

        traversed = [currentCell]

        def calcIntersectionDistanceX(): return abs(dy * (currentCell[0] + directionModifierX - a[0]))
        def calcIntersectionDistanceY(): return abs(dx * (currentCell[1] + directionModifierY - a[1]))

        intersectionDistanceX = float("+inf") if dx == 0 else calcIntersectionDistanceX()
        intersectionDistanceY = float("+inf") if dy == 0 else calcIntersectionDistanceY()

        it = 0
        while (targetCell[0] != currentCell[0] or targetCell[1] != currentCell[1]) and it < 5:
            xMove = intersectionDistanceX < intersectionDistanceY
            yMove = intersectionDistanceY < intersectionDistanceX

            if xMove:
                currentCell = (currentCell[0] + directionX, currentCell[1])
                intersectionDistanceX = calcIntersectionDistanceX()

            if yMove:
                currentCell = (currentCell[0], currentCell[1] + directionY)
                intersectionDistanceY = calcIntersectionDistanceY()

            traversed.append((int(currentCell[0]), int(currentCell[1])))
            it += 1

        return traversed

    # def get_traversed_cells(self, A, B):
    #     """ Return all cells of the unit grid crossed by the line segment between
    #         A and B.
    #     """
    #
    #     (xA, yA) = A
    #     (xB, yB) = B
    #     (dx, dy) = (xB - xA, yB - yA)
    #     (sx, sy) = (self.sign(dx), self.sign(dy))
    #
    #     grid_A = (math.floor(A[0]), math.floor(A[1]))
    #     grid_B = (math.floor(B[0]), math.floor(B[1]))
    #     (x, y) = grid_A
    #     traversed = [grid_A]
    #
    #     tIx = dy * (x + sx - xA) if dx != 0 else float("+inf")
    #     tIy = dx * (y + sy - yA) if dy != 0 else float("+inf")
    #
    #     it = 0
    #     while (x, y) != grid_B and it < 20:
    #         # NB if tIx == tIy we increment both x and y
    #         (movx, movy) = (tIx <= tIy, tIy <= tIx)
    #
    #         if movx:
    #             # intersection is at (x + sx, yA + tIx / dx^2)
    #             x += sx
    #             tIx = dy * (x + sx - xA)
    #
    #         if movy:
    #             # intersection is at (xA + tIy / dy^2, y + sy)
    #             y += sy
    #             tIy = dx * (y + sy - yA)
    #
    #         traversed.append((x, y))
    #         it += 1
    #
    #     return traversed

    def get_cell_height(self, coos):
        return self.map[coos[1], coos[0]]

    def sample(self):
        x = random.uniform(0, 1)
        y = random.uniform(0, 1)
        coos = (x * self.width, y * self.height)
        return coos

    def initialize_tree(self):
        return self.Tree(self, self.start)
