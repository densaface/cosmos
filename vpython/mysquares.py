import time
import json

from vpython import *
#Web VPython 3.2

import model_parameters as param

scene.center = vector(1,1,1)
axes = [vector(1,0,0), vector(0,1,0), vector(0,0,1)]

scene.caption= """Модель квадратных рамок.

To rotate "camera", drag with right button or Ctrl-drag.
To zoom, drag with middle button or Alt/Option depressed, or use scroll wheel.
  On a two-button mouse, middle is left + right.
To pan left/right and up/down, Shift-drag.
Touch screen: pinch/extend to zoom, swipe or two-finger rotate."""


def length_coor(pos1, pos2):
    dx = abs(pos1.x - pos2.x)
    dy = abs(pos1.y - pos2.y)
    dz = abs(pos1.z - pos2.z)
    return sqrt(dx * dx + dy * dy + dz * dz)


squar_side1 = 1.57 * param.Radius
squar_side2 = 1.57 * (param.Radius - param.Dif_radius)


class squares:
    def __init__(self, ):
        self.squar = [[], []]

        # первая рамка
        box1 = box(pos=vector(-0 / 2, squar_side1 / 2, 0 / 2), axis=vector(1, 0, 0), length=squar_side1, height=100,
                   width=100, color=color.red)
        box2 = box(pos=vector(-0 / 2, -squar_side1 / 2, 0 / 2), axis=vector(1, 0, 0), length=squar_side1, height=100,
                   width=100, color=color.red)
        box3 = box(pos=vector(-squar_side1 / 2, 0 / 2, 0 / 2), axis=vector(0, 1, 0), length=squar_side1, height=100,
                   width=100, color=color.red)
        box4 = box(pos=vector(squar_side1 / 2, 0 / 2, 0 / 2), axis=vector(0, 1, 0), length=squar_side1, height=100,
                   width=100, color=color.red)
        self.squar[0].append(box1)
        self.squar[0].append(box2)
        self.squar[0].append(box3)
        self.squar[0].append(box4)

        # вторая рамка
        distance = param.Distance
        box5 = box(pos=vector(-0 / 2, squar_side2 / 2, 0 / 2 + distance), axis=vector(1, 0, 0), length=squar_side2,
                   height=100, width=100, color=color.red)
        box6 = box(pos=vector(-0 / 2, -squar_side2 / 2, 0 / 2 + distance), axis=vector(1, 0, 0), length=squar_side2,
                   height=100, width=100, color=color.red)
        box7 = box(pos=vector(-squar_side2 / 2, 0 / 2, 0 / 2 + distance), axis=vector(0, 1, 0), length=squar_side2,
                   height=100, width=100, color=color.red)
        box8 = box(pos=vector(squar_side2 / 2, 0 / 2, 0 / 2 + distance), axis=vector(0, 1, 0), length=squar_side2,
                   height=100, width=100, color=color.red)
        self.squar[1].append(box5)
        self.squar[1].append(box6)
        self.squar[1].append(box7)
        self.squar[1].append(box8)

        # строим 3 оси X,Y,Z
        len_axel = param.Radius
        axel1 = arrow(pos=vector(0, 0, 0), axis=vector(len_axel, 0, 0), shaftwidth=10, color=color.blue)
        axel2 = arrow(pos=vector(0, 0, 0), axis=vector(0, len_axel, 0), shaftwidth=10, color=color.cyan)
        axel3 = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, len_axel), shaftwidth=10, color=color.purple)

        # воспроизводим положение камеры для показа кольца в покое
        # scene.camera.pos = vector(-595.562, 119.407, 560.849)
        # scene.camera.axis = vector(596.562, -118.407, -259.849)
        scene.width = 1600
        scene.height = 750

class vpython_encoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, vector):
            tmp_obj = {'x': obj.x, 'y': obj.y, 'z': obj.z}
            try:
                return tmp_obj
            except:
                print(type(obj))
        try:
            return json.JSONEncoder.default(self, obj)
        except:
            print(type(obj))
def save_json_to_file(json_var, filename, indent=0):
    f = open(filename, 'w+')
    if indent:
        f.write(json.dumps(json_var, indent=indent, cls=vpython_encoder))
    else:
        f.write(json.dumps(json_var, cls=vpython_encoder))
    f.close()

def save_whole_model(fn):
    tmp_model = {
        'm0': TORS.tors[0].massa,
        'm1': TORS.tors[1].massa,
        'I0': TORS.tors[0].II,
        'I1': TORS.tors[1].II,
        'F0':TORS.tors[0].part_f,
        'F1':TORS.tors[1].part_f,
        'parts_tors': parts_tors,
    }
    save_json_to_file(tmp_model, fn)

SQUA = squares()

mu0 = 0.0000012566370614
parts_tors = []
visual = False  # нужна ли визуализация вектора магнитного поля и вектора силы

# используемые для визуализации объекты
probe_sphere = sphere(pos=vector(0, 0, 0), radius=20)
# probe_sphere2 = sphere(pos=vector(0, 0, 0), radius=20)
pointer = arrow(pos=vector(0, 0, 0), axis=vector(0, 100, 0), shaftwidth=10, color=color.green)
# start_radius_vector = arrow(pos=vector(0, 0, 0), axis=vector(0, 1, 0), shaftwidth=10, color=color.green)

dt = 0.03
t = 0.0

for iter in range(100000):
    rate(1)

    # меняем временной интервал с учетом близости торов, чтобы уменьшить накапливаемую ошибку
    distance = SQUA.squar[0][0].pos.z - SQUA.squar[0][1].pos.z
    # dt = distance * distance * 0.000003
    time0 = time.time()

    # считаем СРЕДНЕЕ магнитное поле по аналитической формуле магнитного поля для проводника конечной длины
    # https://docs.google.com/document/d/1poKxAfoTo00Uzc1kCST7BjsAnRmW-JNwpUJRZuhU6NM/edit#bookmark=id.bq3e38m0h6l5
    time1 = time.time()
    # расстояние между стороной одной рамки и стороной другой, которые располагаютя максимально близко друг к другу
    real_dist = sqrt(param.Dif_radius * param.Dif_radius + distance * distance)
    BB_max = mu0 * param.I_current / 4 / pi / real_dist
    # максимальная разность косинусов в центре стороны рамки, где максимальное магнитное поле
    cos_alph_max = 2 * squar_side1 / 2 / sqrt(squar_side1 * squar_side1 / 4 + real_dist * real_dist)
    # минимальная разность косинусов в углу квадратной рамки, где магнитное поле будет минимальным
    cos_alph_min = squar_side1 / sqrt(squar_side1 * squar_side1 + real_dist * real_dist)
    # среднее магнитное поле от тока одной стороны рамки, рассчитанное в точках стороны другой рамки
    BB_average = BB_max * (cos_alph_max + cos_alph_min) / 2

    # считаем силы от каждой части одного тора ко всем частям другого тора
    FF = BB_average * param.I_current * squar_side1

    # print(f'Суммарная проекция силы на ось Z part_f = {TORS.part_f[0].z}')
    # e_k = TORS.tors[0].massa * TORS.tors[0].velocity.mag * TORS.tors[0].velocity.mag / 2
    t += dt
    if iter % 10 == 0:
        print('V1 = %0.0f м/с, V2 = %0.0f м/с, dt=%f, distance=%0.0f, t=%f, частей=%d, масса=%d, радиус=%d, ток=%d' % (
            TORS.tors[0].velocity.z, TORS.tors[1].velocity.z, dt, distance, t, TORS.parts, TORS.tors[1].massa, TORS.diametr, TORS.tors[1].II))
    # print(f'Суммарная проекция силы на ось Z для второго тора sum_f_z2 = {sum_f_z2}')
    # time.sleep(1)
    
    # суммируем силы от разных частей одного тора
    for jj in range(len(TORS.tors)):
        TORS.tors[jj].SUM_F = vector(0.0, 0.0, 0.0)
        for kk in range(len(TORS.tors[jj].part_f)):
            TORS.tors[jj].SUM_F += TORS.tors[jj].part_f[kk]

    # считаем итоговые скорость и положение тела
    for jj in range(len(TORS.tors)):
        TORS.tors[jj].pos += TORS.tors[jj].velocity * dt + dt * dt * (TORS.tors[jj].SUM_F)/2/TORS.tors[jj].massa
        TORS.tors[jj].velocity += dt * TORS.tors[jj].SUM_F / TORS.tors[jj].massa
    if iter % 10 == 0:
        save_whole_model('tors_%d_%d_dt=%f.txt' % (TORS.parts, iter, dt))
