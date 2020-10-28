import time
from D_star import DStar
import numpy as np
from PIL import Image
import math
import os


class Benchmarker:

    def __init__(self, output_file_name="output.txt"):
        self.output_file_name = output_file_name
        self.maps_folder_path = "maps/"

        maze_maps = ["maze1.csv"]
        easy_maps = ["map1.csv", "map2.csv", "map3.csv"]
        mars_different_size_maps = ["mars/small.png", "mars/medium.png", "mars/big.png"]
        medium_maps = ["medium_deimos.png", "medium_mars.png", "medium_phobos.png"]
        big_maps = ["big_deimos.png", "big_mars.png", "big_phobos.png"]
        other_maps = ["the_valley.png", "hills.png"]
        self.dataset = maze_maps + easy_maps + mars_different_size_maps + medium_maps + big_maps + other_maps

        # Inputs
        self.map = None
        self.dimensions = None
        self.algorithm = None
        self.start = None
        self.goal = None

        # Outputs
        self.path = None
        self.path_length = None
        self.execution_time = None

    def run(self):
        print("\n=== Running benchmark ===\n")

        write_mode = 'w'
        for input_file_name in self.dataset:
            if not self.setup(self.maps_folder_path + input_file_name):
                print("Setup failed")
                return

            print("=== Running :", input_file_name)
            time_start = time.time()
            self.algorithm = DStar(self, self.map, self.start, self.goal)
            self.path = self.algorithm.run()
            time_end = time.time()
            self.execution_time = time_end - time_start

            self.analyze(self.path)
            self.save_results_of(input_file_name, write_mode)
            write_mode = 'a'

        print("=== Benchmark finished ===")
        print("=== See results in :", self.output_file_name)
        cmd = "gedit output/" + self.output_file_name
        os.system(cmd)

    def save_results_of(self, input_file_name, write_mode):
        f = open("output/"+self.output_file_name, write_mode)
        f.write("File : " + input_file_name + "\n")
        f.write("Map size : (" + str(self.dimensions[1]) + " x " + str(self.dimensions[0]) + ")" + "\n")
        f.write("Start point : " + str(self.start) + "\n")
        f.write("Goal point : " + str(self.goal) + "\n")
        f.write("\tExecution time : " + str(self.execution_time) + "\n")
        f.write("\tPath length : " + str(self.path_length) + "\n")
        f.write("\tPath : " + str(self.path) + "\n")
        f.write("=================================================================\n\n")
        f.close()

    def analyze(self, path):
        _, _, self.path_length = self.path_to_height_profile(path)

    def path_to_height_profile(self, path):
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
        return True
