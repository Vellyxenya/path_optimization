import time
from D_star import DStar
from RRT import RRT
import numpy as np
from PIL import Image
import math
import os
import matplotlib.pyplot as plt


class Benchmarker:

    def __init__(self, output_file_name="output.txt"):

        # Setup
        self.output_file_name = output_file_name
        self.maps_folder_path = "maps/"

        # Maps
        maze_maps = ["maze1.csv"]

        small_maps = ["map1.csv"]  # , "map2.csv", "map3.csv"]
        small_maps_labels = ["(22x22)"]  # , "(22x22)_2", "(22x22)_3"]

        medium_maps = ["the_valley.png", "small_mars.png", "hills.png", "medium_deimos.png", "medium_phobos.png", "medium_mars.png"]
        medium_maps_labels = ["(200x100)", "(256x125)", "(257x257)", "(400x200)", "(500x250)", "(600x300)"]
        # 20000, 32000, 66049, 80000, 125000, 180000

        big_maps = ["big_mars.png", "big_deimos.png"]  # , "big_phobos.png"]
        big_maps_labels = ["(1000x500)", "(1200x600)"]  # , "(1400x700)"]

        self.dataset = medium_maps + big_maps # + mars_different_size_maps + medium_maps + big_maps + other_maps
        self.dataset_labels = medium_maps_labels + big_maps_labels

        # Inputs
        self.map = None
        self.dimensions = None
        self.algorithm = None
        self.start = None
        self.goal = None

        # Outputs
        self.execution_time = None
        self.path_length = None
        self.path_cost = None
        self.path = None
        self.exec_times_d_star = []
        self.exec_times_rrt_star = []
        self.path_lengths_d_star = []
        self.path_lengths_rrt_star = []
        self.path_costs_d_star = []
        self.path_costs_rrt_star = []
        self.map_sizes = []

    def run(self):
        print("\n=== Running benchmark ===\n")

        write_mode = 'w'
        for input_file_name in self.dataset:
            if not self.setup(self.maps_folder_path + input_file_name):
                print("Setup failed")
                return

            print("=== Running :", input_file_name)
            self.write_setup(input_file_name, write_mode)
            write_mode = 'a'

            self.benchmark("D*")
            self.save_results()
            self.benchmark("RRT*")
            self.save_results()

            self.write_conclusion()

        print("=== Benchmark finished ===")
        print("=== See results in : output/" + self.output_file_name)
        cmd = "gedit output/" + self.output_file_name
        self.plot_results()
        os.system(cmd)

    def plot_results(self):
        # Plot execution times
        plt.plot(self.map_sizes, self.exec_times_d_star, label="A*")
        plt.plot(self.map_sizes, self.exec_times_rrt_star, label="RRT*")

        plt.xticks(self.map_sizes, rotation=30)
        plt.axes().set_xticklabels(self.dataset_labels)

        plt.xlabel('map sizes (number of cells)')
        plt.ylabel('execution time (sec)')
        plt.title('A* and RRT* execution times')
        plt.legend()
        plt.savefig('execution_time_plot.png')
        plt.show()

        # Plot path lengths
        plt.plot(self.map_sizes, self.path_lengths_d_star, label="A*")
        plt.plot(self.map_sizes, self.path_lengths_rrt_star, label="RRT*")

        plt.xticks(self.map_sizes, rotation=35)
        plt.axes().set_xticklabels(self.dataset_labels)

        plt.xlabel('map sizes (number of cells)')
        plt.ylabel('path length')
        plt.title('A* and RRT* path lengths')
        plt.legend()
        plt.savefig('path_length_plot.png')
        plt.show()

        # Plot path costs
        plt.plot(self.map_sizes, self.path_costs_d_star, label="A*")
        plt.plot(self.map_sizes, self.path_costs_rrt_star, label="RRT*")

        plt.xticks(self.map_sizes, rotation=35)
        plt.axes().set_xticklabels(self.dataset_labels)

        plt.xlabel('map sizes (number of cells)')
        plt.ylabel('path cost')
        plt.title('A* and RRT* path costs')
        plt.legend()
        plt.savefig('path_cost_plot.png')
        plt.show()

    def benchmark(self, algorithm_name):
        width, height = self.map.shape
        if algorithm_name == "D*":
            time_start = time.time()
            self.algorithm = DStar(self, self.map, self.start, self.goal)
            self.path = self.algorithm.run()
            time_end = time.time()
        elif algorithm_name == "RRT*":
            time_start = time.time()
            self.algorithm = RRT(self, self.map, self.start, self.goal)
            self.path, _ = self.algorithm.run(500, width*height*2)
            time_end = time.time()
        else:
            print("Unknown algorithm")
            return

        self.execution_time = time_end - time_start
        self.analyze(self.path)
        if algorithm_name == "D*":
            self.exec_times_d_star.append(self.execution_time)
            self.path_lengths_d_star.append(self.path_length)
            self.path_costs_d_star.append(self.path_cost)
        elif algorithm_name == "RRT*":
            self.exec_times_rrt_star.append(self.execution_time)
            self.path_lengths_rrt_star.append(self.path_length)
            self.path_costs_rrt_star.append(self.path_cost)

    def write_setup(self, input_file_name, write_mode):
        f = open("output/" + self.output_file_name, write_mode)
        f.write("File : " + input_file_name + "\n")
        f.write("Map size : (" + str(self.dimensions[1]) + " x " + str(self.dimensions[0]) + ")" + "\n")
        f.write("Start point : " + str(self.start) + "\n")
        f.write("Goal point : " + str(self.goal) + "\n\n")
        f.close()

    def write_conclusion(self):
        f = open("output/" + self.output_file_name, 'a')
        # f.write("\nAlgo ... is better than algo ...\n")
        f.write("=======================================\n\n")
        f.close()

    def save_results(self):
        f = open("output/" + self.output_file_name, 'a')
        f.write("Algorithm : " + self.algorithm.get_name() + "\n")
        if not self.path:
            f.write("\tNo path found\n")
        else:
            f.write("\tExecution time : " + str(self.execution_time) + "\n")
            f.write("\tPath length : " + str(self.path_length) + "\n")
            f.write("\tPath cost : " + str(self.path_cost) + "\n")
            # f.write("\tPath : " + str(self.path) + "\n")
        f.close()

    def analyze(self, path):
        if not path:
            self.path_length = -1
            return
        heights, distances, self.path_length = self.get_path_metrics(path)
        print(len(heights), len(distances))
        self.path_cost = 0
        for i, distance in enumerate(distances):
            height_diff = abs(heights[i] - heights[i+1])
            self.path_cost += distance * (1 + height_diff)**3  # TODO make path cost more realistic

    def get_path_metrics(self, path):
        heights = []
        distances = []
        total_distance = 0.0
        prev = None
        for node in path:
            if prev is not None:
                distance = self.euclidean_distance(prev, node)
                distances.append(distance)
                total_distance += distance
            heights.append(self.map[math.floor(node[1]), math.floor(node[0])])
            prev = node
        return heights, distances, total_distance

    @staticmethod
    def euclidean_distance(z1, z2):
        return math.sqrt((z2[0] - z1[0]) ** 2 + (z2[1] - z1[1]) ** 2)

    def setup(self, map_name=""):
        File = map_name
        extension = File[-4:]
        if extension == ".csv":
            self.map = np.loadtxt(File, delimiter=',')
        elif extension == ".png" or extension == ".jpg":
            im = Image.open(File, 'r')
            im_height = im.size[1]
            pix_val = list(im.getdata())
            if type(pix_val[0]) == tuple:
                pixel_val_flat = [aTuple[0] for aTuple in pix_val]
            else:
                pixel_val_flat = pix_val
            self.map = np.asarray(pixel_val_flat).reshape((im_height, -1))
            print("map size : ", im.size)
        else:
            print("Unknown extension. Supported formats : .csv, .png, .jpg")
            return False

        self.dimensions = self.map.shape

        minVal, maxVal = np.min(self.map), np.max(self.map)
        print(minVal, maxVal)
        self.map = (self.map - minVal) / (maxVal - minVal)
        self.map = (self.map - np.mean(self.map)) * 2

        self.start = (1, 1)
        self.goal = (self.dimensions[1] - 2, self.dimensions[0] - 2)
        self.map_sizes.append(self.dimensions[1] * self.dimensions[0])
        return True
