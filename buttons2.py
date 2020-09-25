from PIL import Image, ImageTk
import tkinter as Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter import simpledialog, messagebox
from tkinter import LEFT, TOP, X, FLAT, RAISED
import numpy as np
from enum import Enum
import random
from D_star import DStar


class State(Enum):
    S1 = 1
    S2 = 2
    S3 = 3


# ----------------------------------------------------------------------
class MainWindow:

    # ----------------
    def __init__(self, main):
        self.image = Image.open("Resources/lena.png")
        self.photo = ImageTk.PhotoImage(self.image)
        self.canvas_width = 800
        self.canvas_height = 800

        self.canvas = Tk.Canvas(main, width=self.canvas_width, height=self.canvas_height)
        self.image_on_canvas = self.canvas.create_image(0, 0, anchor=Tk.NW, image=self.photo)

        # self.read_file_button = Tk.Button(main, width=12, height=2, text='Read file', command=self.askforfile)
        # self.read_file_button.grid(row=1, column=0)
        # self.write_to_file_button = Tk.Button(main, width=12, height=2, text='Write to file', command=self.save_map)
        # self.write_to_file_button.grid(row=2, column=0)
        # self.write_to_file_button = Tk.Button(main, width=12, height=2, text='Custom Map', command=self.empty_map)
        # self.write_to_file_button.grid(row=3, column=0)

        self.menu = Tk.Menu(main)
        self.menu.add_command(label="Read file", command=self.askforfile)
        self.menu.add_command(label="Write to file", command=self.save_map)
        self.menu.add_command(label="Custom Map", command=self.empty_map)

        self.toolbar = Tk.Frame(main, relief=RAISED)  # bd=1,

        self.run_button = Tk.Button(self.toolbar, text="Run algorithm", relief=FLAT, command=self.run_algorithm)
        self.run_button.pack(side=LEFT, padx=2, pady=2)
        self.step_button = Tk.Button(self.toolbar, text="Step", relief=FLAT, command=self.step)
        self.step_button.pack(side=LEFT, padx=2, pady=2)

        self.toolbar.pack(side=TOP, fill=X)
        self.canvas.pack(side=TOP)
        # self.toolbar.grid(row=0, column=0)
        # self.canvas.grid(row=1, column=0, columnspan=1)

        main.config(menu=self.menu)

        # mouse click on canvas event
        self.canvas.bind("<Button 1>", self.getorigin)

        # self.map = np.random.randn(10, 10)
        self.map = np.loadtxt("my_empty_map.csv", delimiter=',')
        self.init_map()
        self.reinit_canvas()
        self.path = None

    # ----------------

    def step(self):
        if self.path:
            if self.current_position != self.end:
                self.path_index += 1
            self.current_position = self.path[self.path_index].get_coos()
            self.show_current_position()

    def show_current_position(self):
        self.canvas.delete("current_position")
        shape_size = 8
        color = "#cccccc"
        offset = self.square_size // 2
        centerX = offset + self.current_position[0] * self.square_size
        centerY = offset + self.current_position[1] * self.square_size
        self.canvas.create_oval(centerX - shape_size, centerY - shape_size,
                                centerX + shape_size, centerY + shape_size, fill=color, tag="current_position")

    def run_algorithm(self):
        self.algorithm = DStar(self.map, self.start, self.end)
        self.path = self.algorithm.run()
        self.path_index = -1
        self.draw_path(self.path)

    def draw_path(self, path):
        (x_0, y_0) = self.start
        offset = self.square_size // 2
        centerX0 = offset + x_0 * self.square_size
        centerY0 = offset + y_0 * self.square_size
        for node in path:
            (x, y) = node.get_coos()
            centerX = offset + x * self.square_size
            centerY = offset + y * self.square_size
            shape_size = 5
            color = "black"
            self.canvas.create_oval(centerX - shape_size, centerY - shape_size,
                                    centerX + shape_size, centerY + shape_size, fill=color, tag="path_points")
            self.canvas.create_line(centerX0, centerY0, centerX, centerY, fill=color, tag="path_line")
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
        d = self.square_size * 3 // 4
        r = d // 2
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

    def update_map(self, x, y):
        self.map[y, x] = 4
        self.update_color_map()

    def askforfile(self):
        im = Image.open('test_map.png', 'r')
        pix_val = list(im.getdata())
        pixel_val_flat = [aTuple[0] for aTuple in pix_val]
        self.map = np.asarray(pixel_val_flat).reshape((100, 200))

        # File = askopenfilename(parent=root, initialdir="./", title='Select an image')
        # print(File)
        # original = Image.open(File).resize((512, 512))
        # self.photo = ImageTk.PhotoImage(original)
        # self.canvas.itemconfig(self.image_on_canvas, image=self.photo)
        # self.map = np.loadtxt('my_file1.csv', delimiter=',')
        # print(self.data)

        File = askopenfilename(parent=root, initialdir="./", title='Select a .csv file')
        print(File)
        self.map = np.loadtxt(File, delimiter=',')

        print(type(self.map), self.map.shape)

        minVal, maxVal = np.min(self.map), np.max(self.map)
        print(minVal, maxVal)
        self.map = (self.map - minVal) / (maxVal - minVal)
        self.map = (self.map - np.mean(self.map))*2
        self.init_map()
        self.reinit_canvas()

    def empty_map(self):
        width = simpledialog.askinteger("Width", "Enter Map width",
                                        parent=root, minvalue=1, maxvalue=300)

        height = simpledialog.askinteger("Height", "Enter Map height",
                                         parent=root, minvalue=1, maxvalue=300)

        randomize = messagebox.askyesno("Question", "Random map?")

        self.map = np.zeros((height, width)) if not randomize else np.random.randn(height, width)
        self.init_map()
        self.reinit_canvas()

    def reinit_canvas(self):
        self.canvas.delete("start")
        self.canvas.delete("end")
        self.canvas.delete("cell_state")
        self.canvas.delete("path_line")
        self.canvas.delete("path_points")
        self.draw_algo_state()
        self.draw_positions()
        self.show_current_position()

    def init_map(self):
        self.map_height, self.map_width = self.map.shape
        self.square_size = int(min(self.canvas_width / self.map_width,
                                   self.canvas_height / self.map_height))
        self.color_map = np.zeros((self.map_height, self.map_width, 3), np.uint8)
        self.update_color_map()
        self.algo_state_map = np.zeros((self.map_height, self.map_width), State)
        for i in range(self.map_height):
            for j in range(self.map_width):
                self.algo_state_map[i, j] = random.choice(list(State))
        self.start = (1, 1)
        self.end = (self.map_width - 2, self.map_height - 3)
        self.current_position = self.start
        self.path = None

    def save_map(self):
        file_name = asksaveasfilename(parent=root, initialdir="./", title='Save as .csv file')
        print(file_name)
        np.savetxt(file_name, np.asarray(self.map), delimiter=',', fmt='%.1e')

    def update_color_map(self):
        max_height = np.max(self.map)
        min_height = np.min(self.map)
        extreme_value = max(abs(max_height), abs(min_height))
        for i in range(self.map_height):
            for j in range(self.map_width):
                self.color_map[i, j] = self.map_height_to_color(self.map[i, j], extreme_value)
        image = Image.fromarray(self.color_map)
        resized_image = image.resize((self.map_width * self.square_size, self.map_height * self.square_size),
                                     Image.NEAREST)
        self.photo = ImageTk.PhotoImage(resized_image)
        self.canvas.itemconfig(self.image_on_canvas, image=self.photo)

    def map_height_to_color(self, height, extreme_value):
        val = 255.0 / extreme_value if extreme_value != 0 else 0
        ratio = height * val
        return (255 - abs(int(ratio)), 255 - abs(int(ratio)), 255) if height < 0 else (
            255, 255 - abs(int(ratio)), 255 - abs(int(ratio)))


root = Tk.Tk()
MainWindow(root)
root.mainloop()
