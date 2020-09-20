from PIL import Image, ImageTk
import tkinter as Tk
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter import simpledialog, messagebox
import numpy as np
from enum import Enum
import random
from D_star import DStar

class State(Enum):
    S1 = 1
    S2 = 2
    S3 = 3


# ----------------------------------------------------------------------
class MainWindow():

    # ----------------
    def __init__(self, main):
        self.image = Image.open("Resources/lena.png")
        self.photo = ImageTk.PhotoImage(self.image)
        self.canvas_width = 800
        self.canvas_height = 800

        self.canvas = Tk.Canvas(main, width=self.canvas_width, height=self.canvas_height)
        self.image_on_canvas = self.canvas.create_image(0, 0, anchor=Tk.NW, image=self.photo)
        self.canvas.grid(row=0, column=1, columnspan=2)

        self.read_file_button = Tk.Button(main, width=12, height=2, text='Read file', command=self.askforfile)
        self.read_file_button.grid(row=1, column=0)
        self.write_to_file_button = Tk.Button(main, width=12, height=2, text='Write to file', command=self.save_map)
        self.write_to_file_button.grid(row=2, column=0)
        self.write_to_file_button = Tk.Button(main, width=12, height=2, text='Custom Map', command=self.empty_map)
        self.write_to_file_button.grid(row=3, column=0)

        # mouse click on canvas event
        self.canvas.bind("<Button 1>", self.getorigin)

        # self.map = np.array([[1, 2, 0, 0], [3, 4, 0, -3], [4, 10, -2, 0]])
        # self.map = np.random.randn(8, 8)
        self.map = np.random.randn(10, 10)
        self.init_map()

        self.start = (1, 8)
        self.end = (8, 2)
        self.draw_algo_state()
        self.draw_positions()

        algorithm = DStar(self.map, self.start, self.end)
        path = algorithm.run()
        self.draw_path(path)

    # ----------------

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
            color="black"
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
        # File = askopenfilename(parent=root, initialdir="./", title='Select an image')
        # print(File)
        # original = Image.open(File).resize((512, 512))
        # self.photo = ImageTk.PhotoImage(original)
        # self.canvas.itemconfig(self.image_on_canvas, image=self.photo)
        # self.map = np.loadtxt('my_file.csv', delimiter=',')
        # print(self.data)
        File = askopenfilename(parent=root, initialdir="./", title='Select a .csv file')
        print(File)
        self.map = np.loadtxt(File, delimiter=',')
        self.init_map()
        self.canvas.delete("start")
        self.canvas.delete("end")
        self.canvas.delete("cell_state")
        self.draw_algo_state()
        self.draw_positions()

    def empty_map(self):
        width = simpledialog.askinteger("Width", "Enter Map width",
                                        parent=root, minvalue=1, maxvalue=100)

        height = simpledialog.askinteger("Height", "Enter Map height",
                                         parent=root, minvalue=1, maxvalue=100)

        randomize = messagebox.askyesno("Question", "Random map?")

        self.map = np.zeros((height, width)) if not randomize else np.random.randn(height, width)
        self.init_map()
        self.start = (1, 1)
        self.end = (width - 2, height - 3)
        self.canvas.delete("start")
        self.canvas.delete("end")
        self.canvas.delete("cell_state")
        self.draw_algo_state()
        self.draw_positions()

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
