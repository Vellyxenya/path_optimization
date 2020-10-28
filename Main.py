from PIL import Image, ImageTk
import tkinter as Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter import simpledialog, messagebox
from tkinter import LEFT, TOP, X, FLAT, RAISED
import numpy as np
from enum import Enum
import random
from D_star import DStar
from RRT import RRT
import time
import math
from Benchmarker import Benchmarker


class State(Enum):
    S1 = 1
    S2 = 2
    S3 = 3


class Algos(Enum):
    DSTAR = 1
    RRT = 2


class PathType(Enum):
    PATH_TO_FOLLOW = 1
    PATH_HISTORY = 2


# ----------------------------------------------------------------------
class MainWindow:

    gui = True

    # ----------------
    def __init__(self, main):
        if not self.gui:
            benchmark = Benchmarker()
            benchmark.run()
            main.destroy()
            return
        self.image = Image.open("Resources/lena.png")
        self.photo = ImageTk.PhotoImage(self.image)
        self.canvas_width = 1000
        self.canvas_height = 650

        self.canvas = Tk.Canvas(main, width=self.canvas_width, height=self.canvas_height)
        self.image_on_canvas = self.canvas.create_image(0, 0, anchor=Tk.NW, image=self.photo)

        # mouse click on canvas event
        self.canvas.bind("<Button 1>", self.getorigin)

        # self.map = np.random.randn(10, 10)
        self.visibility_map = None
        self.path = None
        self.path_history = []
        self.algorithm = None
        self.sensor_range = 10
        self.steps_per_cycle = self.sensor_range // 2

        self.menu = Tk.Menu(main)
        self.menu.add_command(label="Read file", command=self.ask_for_file)
        self.menu.add_command(label="Write to file", command=self.save_map)
        self.menu.add_command(label="Custom Map", command=self.empty_map)

        self.toolbar = Tk.Frame(main, relief=RAISED)  # bd=1,
        self.slider_bar = Tk.Frame(main, relief=RAISED)
        self.metrics_bar = Tk.Frame(main, relief=RAISED)

        self.run_menu = Tk.Menu(self.menu, tearoff=0)

        self.run_menu.add_command(label="D*", command=self.run_algorithm_dstar)
        self.run_menu.add_command(label="RRT", command=self.run_algorithm_rrt)
        self.menu.add_cascade(label="Run Menu", menu=self.run_menu)
        # self.menu.add_command(label="Run current", command=self.run_algorithm)

        self.menu.add_command(label="Go to destination", command=self.go_to_destination)

        self.step_button = Tk.Button(self.toolbar, text="Step", relief=FLAT, command=self.step)
        self.step_button.pack(side=LEFT, padx=2, pady=2)
        self.clear_path_button = Tk.Button(self.toolbar, text="Clear path", relief=FLAT, command=self.clear_path)
        self.clear_path_button.pack(side=LEFT, padx=2, pady=2)

        self.change_start_position_button = Tk.Button(self.toolbar, text="Change start", relief=FLAT,
                                                      command=self.change_start_position)
        self.change_start_position_button.pack(side=LEFT, padx=2, pady=2)
        self.change_goal_position_button = Tk.Button(self.toolbar, text="Change goal", relief=FLAT,
                                                     command=self.change_goal_position)
        self.change_goal_position_button.pack(side=LEFT, padx=2, pady=2)

        self.use_full_map = Tk.IntVar(value=1)
        self.is_full_map_box = Tk.Checkbutton(self.toolbar, text="Full map", variable=self.use_full_map,
                                              command=self.update_color_map)
        self.is_full_map_box.pack(side=LEFT, padx=2, pady=2)
        self.use_dynamic_mode = Tk.IntVar(value=0)
        self.use_dynamic_mode_box = Tk.Checkbutton(self.toolbar, text="Dynamic mode", variable=self.use_dynamic_mode)
        self.use_dynamic_mode_box.pack(side=LEFT, padx=2, pady=2)

        self.sensor_range_button = Tk.Button(self.toolbar, text="Sensor range", relief=FLAT,
                                             command=self.change_sensor_range)
        self.sensor_range_label_var = Tk.StringVar()
        self.sensor_range_label_var.set(self.sensor_range)
        self.sensor_range_button.pack(side=LEFT, padx=2, pady=2)
        self.sensor_range_label = Tk.Label(self.toolbar, textvariable=self.sensor_range_label_var)
        self.sensor_range_label.pack(side=LEFT, padx=2, pady=2)

        self.heuristic = 20
        self.heuristic_label = Tk.Label(self.slider_bar, text="Heuristic value :")
        self.heuristic_label.pack(side=LEFT, padx=2, pady=2)
        self.heuristic_slider = Tk.Scale(self.slider_bar, from_=0, to=120, orient=Tk.HORIZONTAL,
                                         command=self.broadcast_heuristic_value)
        self.heuristic_slider.set(self.heuristic)
        self.heuristic_slider.pack(side=LEFT, padx=2, pady=2)

        self.simulation_speed = 10
        self.simulation_speed_label = Tk.Label(self.slider_bar, text="Simulation speed :")
        self.simulation_speed_label.pack(side=LEFT, padx=2, pady=2)
        self.simulation_speed_slider = Tk.Scale(self.slider_bar, from_=1, to=300, orient=Tk.HORIZONTAL,
                                                command=self.broadcast_simulation_speed)
        self.simulation_speed_slider.set(self.simulation_speed)
        self.simulation_speed_slider.pack(side=LEFT, padx=2, pady=2)

        self.path_length_label = Tk.Label(self.metrics_bar, text="Path length :")
        self.path_length_var = Tk.StringVar()
        self.path_length_var.set("{:.2f}".format(0))
        self.path_length_label_value = Tk.Label(self.metrics_bar, textvariable=self.path_length_var)
        self.path_length_label.pack(side=LEFT, padx=2, pady=2)
        self.path_length_label_value.pack(side=LEFT, padx=2, pady=2)

        self.execution_time_label = Tk.Label(self.metrics_bar, text="Execution time :")
        self.execution_time_var = Tk.StringVar()
        self.execution_time_var.set(0.0)
        self.execution_time_label_value = Tk.Label(self.metrics_bar, textvariable=self.execution_time_var)
        self.execution_time_label.pack(side=LEFT, padx=2, pady=2)
        self.execution_time_label_value.pack(side=LEFT, padx=2, pady=2)

        self.path_profile_width = self.canvas_width
        self.path_profile_height = 120
        self.path_profile = Tk.Canvas(main, bg="white", width=self.path_profile_width, height=self.path_profile_height)

        self.toolbar.pack(side=TOP, fill=X)
        self.slider_bar.pack(side=TOP, fill=X)
        self.metrics_bar.pack(side=TOP, fill=X)
        self.canvas.pack(side=TOP)
        self.path_profile.pack(side=TOP)

        self.map = np.loadtxt("my_file2.csv", delimiter=',')
        self.init_map()
        self.reinit_canvas()

        self.start = (1, 1)
        self.end = (self.map_width - 2, self.map_height - 3)
        self.current_position = self.start
        self.is_simulation_running = False

        self.algorithm = DStar(self, self.map, self.start, self.end)

        main.config(menu=self.menu)
        print(self.canvas.winfo_width(), self.canvas.winfo_screenwidth(), self.canvas.winfo_vrootwidth())

    # ----------------

    def go_to_destination(self):
        # self.is_simulation_running = True
        # self.algorithm = DStar(self, self.map, self.start, self.end)
        # self.run_algorithm()
        # self.menu.update()
        # while self.current_position != self.end and self.is_simulation_running:
        #     self.step()
        #     self.menu.update()  # necessary b/c canvas does not update since the call comes from a menu 'command='
        #     sleep_time = (2.0 / int(self.simulation_speed))
        #     time.sleep(sleep_time)
        # self.is_simulation_running = False

        self.is_simulation_running = True

        while self.current_position != self.end and self.is_simulation_running:
            self.algorithm = DStar(self, self.height_visibility_map, self.current_position, self.end)
            self.run_algorithm()
            for i in range(self.steps_per_cycle):
                if self.current_position == self.end:
                    break
                self.step()
                self.menu.update()  # necessary otherwise canvas does not update since the call comes from a menu 'command='
                sleep_time = (2.0 / int(self.simulation_speed))
                # time.sleep(sleep_time)
        self.draw_path_profile(self.path)
        self.is_simulation_running = False

    def broadcast_heuristic_value(self, value):
        self.heuristic = value

    def broadcast_simulation_speed(self, value):
        self.simulation_speed = value

    def get_heuristic(self):
        return self.heuristic

    def change_sensor_range(self):
        new_range = simpledialog.askinteger("New value : ", "Enter new sensor range",
                                            parent=root, minvalue=2, maxvalue=1000)
        self.sensor_range = new_range
        self.sensor_range_label_var.set(new_range)
        self.update_steps_per_cycle()

    def update_steps_per_cycle(self):
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        self.steps_per_cycle = self.sensor_range // 2
        print(self.steps_per_cycle)

    def change_start_position(self):
        start_x = simpledialog.askinteger("X : ", "Enter new X coordinate",
                                          parent=root, minvalue=1, maxvalue=self.map_width - 1)

        start_y = simpledialog.askinteger("Y : ", "Enter new Y coordinate",
                                          parent=root, minvalue=1, maxvalue=self.map_height - 1)
        self.set_start_position((start_x, start_y))
        self.draw_positions()
        self.show_current_position()

    def change_goal_position(self):
        goal_x = simpledialog.askinteger("X : ", "Enter new X coordinate",
                                         parent=root, minvalue=0, maxvalue=self.map_width - 1)
        if not goal_x:
            return
        goal_y = simpledialog.askinteger("Y : ", "Enter new Y coordinate",
                                         parent=root, minvalue=0, maxvalue=self.map_height - 1)
        if not goal_y:
            return
        self.set_goal_position((goal_x, goal_y))
        self.draw_positions()
        self.show_current_position()

    def set_start_position(self, new_start):
        self.start = new_start

    def set_goal_position(self, new_goal):
        self.end = new_goal

    def clear_path(self):
        self.path = None
        # self.is_simulation_running = False
        self.canvas.delete("path_line")
        self.canvas.delete("path_points")
        self.canvas.delete("current_position")
        self.canvas.delete("history_branch_points")
        self.canvas.delete("history_branch_line")
        self.canvas.delete("tree_nodes")
        self.canvas.delete("tree_branches")

    def step(self):
        if self.path:
            if self.current_position != self.end:
                self.path_index += 1
            self.current_position = self.path[self.path_index]
            self.show_current_position()
            self.update_visibility_map()
            self.path_history.append(self.current_position)

    def show_current_position(self):
        self.canvas.delete("current_position")
        shape_size = max(self.square_size // 6, 3)
        color = "#cccccc"
        offset = self.square_size // 2
        centerX = offset + self.current_position[0] * self.square_size
        centerY = offset + self.current_position[1] * self.square_size
        self.canvas.create_oval(centerX - shape_size, centerY - shape_size,
                                centerX + shape_size, centerY + shape_size, fill=color, tag="current_position")

    def run_algorithm_rrt(self):
        # TODO refactor this
        self.clear_path()
        start = time.time()
        self.algorithm = RRT(self, self.map, self.start, self.end)
        self.path, self.tree = self.algorithm.run(500)  # iterations
        end = time.time()
        self.execution_time_var.set("{:.3f}".format(end - start))
        self.draw_tree(self.tree)
        if self.path:
            self.draw_branch(self.path, PathType.PATH_HISTORY)
            self.draw_path_profile(self.path)
        else:
            print("No path found, so can't draw path branch")

    def run_algorithm_dstar(self):
        self.clear_path()
        start = time.time()
        self.algorithm = DStar(self, self.height_visibility_map, self.start, self.end)
        self.path = self.algorithm.run()
        end = time.time()
        self.execution_time_var.set("{:.3f}".format(end - start))
        self.path_index = -1
        self.draw_path(self.path, PathType.PATH_TO_FOLLOW)
        self.draw_path(self.path_history, PathType.PATH_HISTORY)
        # self.current_position = self.start
        self.show_current_position()
        self.draw_path_profile(self.path)

    def draw_path_profile(self, path):
        self.path_profile.delete("profile_line")
        heights, distances, total_distance = self.path_to_height_profile(path)
        self.path_length_var.set("{:.2f}".format(total_distance))
        ratio = self.path_profile_width/total_distance  # * 0.9
        prev_height = heights[0]
        prev_x = 0
        for (height, distance) in zip(heights[:-1], distances):
            dx = distance * ratio
            self.path_profile.create_line(prev_x, self.path_profile_height/2-prev_height*30, prev_x+dx,
                                          self.path_profile_height/2-height*30,
                                          fill="black", tag="profile_line")
            prev_height = height
            prev_x += dx

    def path_to_height_profile(self, path):
        heights = []
        distances = []
        total_distance = 0.0
        prev = None
        for node in path:
            if prev is not None:
                distance = RRT.euclidean_distance(prev, node)
                distances.append(distance)
                total_distance += distance
            heights.append(self.map[math.floor(node[1]), math.floor(node[0])])
            prev = node
        return heights, distances, total_distance

    def run_algorithm(self):
        if self.algorithm is None:
            return
        self.clear_path()
        self.path = self.algorithm.run()
        self.path_index = -1
        self.draw_path(self.path, PathType.PATH_TO_FOLLOW)
        self.draw_path(self.path_history, PathType.PATH_HISTORY)
        # self.current_position = self.start
        self.show_current_position()

    def draw_branch(self, path, path_type):
        if path_type == PathType.PATH_TO_FOLLOW:
            color = "black"
            (x_0, y_0) = self.current_position
            points_tag = "path_points"
            line_tag = "path_line"
        elif path_type == PathType.PATH_HISTORY:
            color = "cyan"
            (x_0, y_0) = self.start
            points_tag = "history_branch_points"
            line_tag = "history_branch_line"
        else:
            print("Cannot draw unknown path type")
            return
        offset = 0  # self.square_size // 2
        centerX0 = offset + x_0 * self.square_size
        centerY0 = offset + y_0 * self.square_size
        for node in path:
            if path_type == PathType.PATH_TO_FOLLOW:
                (x, y) = (node[0], node[1])
            elif path_type == PathType.PATH_HISTORY:
                (x, y) = (node[0], node[1])
            centerX = offset + x * self.square_size
            centerY = offset + y * self.square_size
            shape_size = max(self.square_size // 8, 1)
            self.canvas.create_oval(centerX - shape_size, centerY - shape_size,
                                    centerX + shape_size, centerY + shape_size, fill=color, tag=points_tag)
            self.canvas.create_line(centerX0, centerY0, centerX, centerY, fill=color, tag=line_tag)
            centerX0 = centerX
            centerY0 = centerY

    def draw_tree(self, tree: RRT.Tree):
        self.canvas.delete("tree_nodes")
        self.canvas.delete("tree_branches")
        offset = 0  # self.square_size // 2
        for i in range(tree.nb_nodes):
            centerX = offset + tree.xs[i] * self.square_size
            centerY = offset + tree.ys[i] * self.square_size
            shape_size = max(self.square_size // 10, 1)
            coos_parent = tree.get_node(tree.parents[i])
            centerX0 = offset + self.square_size * coos_parent[0]
            centerY0 = offset + self.square_size * coos_parent[1]
            self.canvas.create_oval(centerX - shape_size, centerY - shape_size,
                                    centerX + shape_size, centerY + shape_size, fill="#f68a41", tag="tree_nodes")
            self.canvas.create_line(centerX0, centerY0, centerX, centerY, fill="black", tag="tree_branches")
        self.menu.update()
        #time.sleep(2)

    def draw_path(self, path, path_type):
        if path_type == PathType.PATH_TO_FOLLOW:
            color = "black"
            (x_0, y_0) = self.current_position
            points_tag = "path_points"
            line_tag = "path_line"
        elif path_type == PathType.PATH_HISTORY:
            color = "green"
            (x_0, y_0) = self.start
            points_tag = "history_path_points"
            line_tag = "history_path_line"
        else:
            print("Cannot draw unknown path type")
            return
        offset = self.square_size // 2
        centerX0 = offset + x_0 * self.square_size
        centerY0 = offset + y_0 * self.square_size
        for node in path:
            (x, y) = (node[0], node[1])
            centerX = offset + x * self.square_size
            centerY = offset + y * self.square_size
            shape_size = max(self.square_size // 8, 1)
            self.canvas.create_oval(centerX - shape_size, centerY - shape_size,
                                    centerX + shape_size, centerY + shape_size, fill=color, tag=points_tag)
            self.canvas.create_line(centerX0, centerY0, centerX, centerY, fill=color, tag=line_tag)
            centerX0 = centerX
            centerY0 = centerY

    def draw_algo_state(self):
        offset = self.square_size // 2
        half_size = self.square_size * 3 // 14
        for i in range(self.map_height):
            for j in range(self.map_width):
                centerX = offset + j * self.square_size
                centerY = offset + i * self.square_size
                color = "#bfff00"
                if self.algo_state_map[i, j] == State.S2:
                    color = "#00ffbf"
                elif self.algo_state_map[i, j] == State.S3:
                    color = "#00bfff"
                self.canvas.create_oval(centerX - half_size, centerY - half_size,
                                        centerX + half_size, centerY + half_size, fill=color, tag="cell_state")

    def draw_positions(self):
        self.canvas.delete("start")
        self.canvas.delete("end")
        d = self.square_size * 3 // 4
        r = max(d // 2, 4)
        offset = self.square_size // 2
        centerX = offset + self.start[0] * self.square_size
        centerY = offset + self.start[1] * self.square_size
        self.canvas.create_oval(centerX - r, centerY - r, centerX + r, centerY + r, fill="#8000ff", tag="start")
        centerX = offset + self.end[0] * self.square_size
        centerY = offset + self.end[1] * self.square_size
        self.canvas.create_oval(centerX - r, centerY - r, centerX + r, centerY + r, fill="magenta", tag="end")

    def getorigin(self, eventorigin):
        global x0, y0
        x0 = eventorigin.x
        y0 = eventorigin.y
        square_x = x0 // self.square_size
        square_y = y0 // self.square_size
        print(square_x, square_y)
        self.update_map(square_x, square_y)

    # By clicking on the map, the elevation of the terrain increases
    def update_map(self, x, y):
        # self.map[y, x] = 4  # TODO change this value
        # self.update_color_map()
        pass

    def ask_for_file(self):
        File = askopenfilename(parent=root, initialdir="./", title='Select a .csv or .png file')
        print(File)
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
            print("image size : ", im.size)

        print(type(self.map), self.map.shape)

        minVal, maxVal = np.min(self.map), np.max(self.map)
        print(minVal, maxVal)
        self.map = (self.map - minVal) / (maxVal - minVal)
        self.map = (self.map - np.mean(self.map)) * 2
        self.init_map()
        self.reinit_canvas()

    def empty_map(self):
        width = simpledialog.askinteger("Width", "Enter Map width",
                                        parent=root, minvalue=1, maxvalue=300)
        if not width:
            return
        height = simpledialog.askinteger("Height", "Enter Map height",
                                         parent=root, minvalue=1, maxvalue=300)
        if not height:
            return
        randomize = messagebox.askyesno("Question", "Random map?")

        self.map = np.zeros((height, width)) if not randomize else np.random.randn(height, width)
        self.init_map()
        self.reinit_canvas()

    def reinit_canvas(self):
        self.canvas.delete("start")
        self.canvas.delete("end")
        self.canvas.delete("cell_state")
        self.canvas.delete("history_path_points")
        self.canvas.delete("history_path_line")
        self.canvas.delete("history_branch_points")
        self.canvas.delete("history_branch_line")
        self.canvas.delete("tree_nodes")
        self.canvas.delete("tree_branches")
        self.clear_path()
        # self.draw_algo_state()
        self.draw_positions()
        self.show_current_position()

    def init_map(self):
        self.map_height, self.map_width = self.map.shape
        self.square_size = max(1, int(min(self.canvas_width / self.map_width,
                                   self.canvas_height / self.map_height)))
        self.color_map = np.zeros((self.map_height, self.map_width, 3), np.uint8)
        self.start = (1, 1)
        self.end = (self.map_width - 2, self.map_height - 3)
        self.current_position = self.start
        self.path = None
        self.path_history = []
        self.visibility_map = np.zeros((self.map_height, self.map_width), bool)
        self.update_visibility_map()
        self.algo_state_map = np.zeros((self.map_height, self.map_width), State)
        for i in range(self.map_height):
            for j in range(self.map_width):
                self.algo_state_map[i, j] = random.choice(list(State))

    def update_visibility_map(self):
        # TODO add condition to update visibility_map and color map only if we are using the 'non-full-map' mode
        range_squared = self.sensor_range ** 2
        for y in range(self.map_height):
            for x in range(self.map_width):
                if (x - self.current_position[0]) ** 2 + (y - self.current_position[1]) ** 2 <= range_squared:
                    self.visibility_map[y, x] = True
        self.update_color_map()

    def save_map(self):
        file_name = asksaveasfilename(parent=root, initialdir="./", title='Save as .csv file')
        print(file_name)
        np.savetxt(file_name, np.asarray(self.map), delimiter=',', fmt='%.1e')

    def update_height_visibility_map(self):
        if self.use_full_map.get() == 0:
            self.height_visibility_map = self.map * self.visibility_map  # TODO  comment the last part out to display the whole map
        else:
            self.height_visibility_map = self.map
        self.menu.update()

    def update_color_map(self):
        max_height = np.max(self.map)
        min_height = np.min(self.map)
        extreme_value = max(abs(max_height), abs(min_height))
        self.update_height_visibility_map()
        for i in range(self.map_height):
            for j in range(self.map_width):
                self.color_map[i, j] = self.map_height_to_color(self.height_visibility_map[i, j], extreme_value)
        image = Image.fromarray(self.color_map)
        drawn_width = self.map_width * max(1, self.square_size)
        drawn_height = self.map_height * max(1, self.square_size)
        resized_image = image.resize((drawn_width, drawn_height), Image.NEAREST)
        self.photo = ImageTk.PhotoImage(resized_image)
        self.canvas.itemconfig(self.image_on_canvas, image=self.photo)
        self.canvas.config(width=drawn_width)
        self.canvas.config(height=drawn_height)
        self.path_profile.config(width=drawn_width)
        root.geometry(str(drawn_width + 200) + "x" + str(drawn_height + self.path_profile_height + 100))

    @staticmethod
    def map_height_to_color(height, extreme_value):
        val = 255.0 / extreme_value if extreme_value != 0 else 0
        ratio = height * val
        return (255 - abs(int(ratio)), 255 - abs(int(ratio)), 255) if height < 0 else (
            255, 255 - abs(int(ratio)), 255 - abs(int(ratio)))


root = Tk.Tk()
MainWindow(root)
root.mainloop()
