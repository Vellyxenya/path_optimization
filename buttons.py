from PIL import Image, ImageTk
from tkinter import *
import cv2 as cv

root = Tk()

w = Canvas(root, width=500, height=300, bd = 10, bg = 'white')
w.grid(row = 0, column = 0, columnspan = 2)

b = Button(width = 10, height = 2, text = 'Button1')
b.grid(row = 1, column = 0)
b2 = Button(width = 10, height = 2, text = 'Button2')
b2.grid(row = 1,column = 1)

cv.namedWindow("camera", 1)
capture = cv.VideoCapture(0)
capture.set(3, 640)
capture.set(4, 480)
capture.set(10, 10)  # brightness

while True:
    success, img = capture.read()
    photo = ImageTk.PhotoImage(img)
    w.create_image(0,0, image=photo)
    if cv.waitKey(10) == 27:
        break

root.mainloop()