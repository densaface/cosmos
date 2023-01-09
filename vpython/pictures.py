from vpython import *
#Web VPython 3.2
import time

scene.center = vector(1,1,1)
axes = [vector(1,0,0), vector(0,1,0), vector(0,0,1)]

scene.caption= """Рисуем поясняющие картинки.

To rotate "camera", drag with right button or Ctrl-drag.
To zoom, drag with middle button or Alt/Option depressed, or use scroll wheel.
  On a two-button mouse, middle is left + right.
To pan left/right and up/down, Shift-drag.
Touch screen: pinch/extend to zoom, swipe or two-finger rotate."""

Radius = 10000

# строим 3 оси X,Y,Z
len_axel = Radius
axel1 = arrow(pos=vector(0, 0, 0), axis=vector(len_axel, 0, 0), shaftwidth=100, color=color.blue)
axel2 = arrow(pos=vector(0, 0, 0), axis=vector(0, len_axel, 0), shaftwidth=100, color=color.cyan)
axel3 = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, len_axel), shaftwidth=100, color=color.purple)
scene.width = 1600
scene.height = 750

squar_model = True
if squar_model:
    distance = 1000  # расстояние между рамками
    squar_side = 1.57 * Radius
    box1 = box(pos=vector(-0/2, squar_side/2, 0/2), axis=vector(1, 0, 0), length=squar_side, height=100, width=100, color=color.red)
    box2 = box(pos=vector(-0/2,-squar_side/2, 0/2), axis=vector(1, 0, 0), length=squar_side, height=100, width=100, color=color.red)
    box3 = box(pos=vector(-squar_side/2, 0/2, 0/2), axis=vector(0, 1, 0), length=squar_side, height=100, width=100, color=color.red)
    box4 = box(pos=vector( squar_side/2, 0/2, 0/2), axis=vector(0, 1, 0), length=squar_side, height=100, width=100, color=color.red)

    squar_side2 = 1.57 * (Radius - 1000)
    box1 = box(pos=vector(-0/2, squar_side2/2, 0/2 + distance), axis=vector(1, 0, 0), length=squar_side2, height=100, width=100, color=color.red)
    box2 = box(pos=vector(-0/2,-squar_side2/2, 0/2 + distance), axis=vector(1, 0, 0), length=squar_side2, height=100, width=100, color=color.red)
    box3 = box(pos=vector(-squar_side2/2, 0/2, 0/2 + distance), axis=vector(0, 1, 0), length=squar_side2, height=100, width=100, color=color.red)
    box4 = box(pos=vector( squar_side2/2, 0/2, 0/2 + distance), axis=vector(0, 1, 0), length=squar_side2, height=100, width=100, color=color.red)

    # для показа угла между осью и вектором соединяющим рамки
    vect4 = arrow(pos=vector(0, squar_side/2, 0/2), axis=vector(0, 0, len_axel), shaftwidth=100, color=color.green)
    vect5 = arrow(pos=vector(0, squar_side/2, 0/2), axis=vector(0, -distance, distance), shaftwidth=100, color=color.green)


while True:
    time.sleep(1)