import time
import json

from vpython import *
import model_parameters as param

#Web VPython 3.2

scene.center = vector(1,1,1)
axes = [vector(1,0,0), vector(0,1,0), vector(0,0,1)]

scene.caption= """A model of a solid represented as atoms connected by interatomic bonds.

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

class tors:
    def __init__(self, ):
        self.tors = []
        self.parts = param.parts

        tor1 = ring(pos=vector(0,0,0), axis=vector(0,0,1), radius=param.Radius, thickness=200, color=color.red)
        tor2 = ring(pos=vector(0,0,-param.Distance), axis=vector(0,0,1), radius=param.Radius - param.Dif_radius,
                    thickness=200, color=color.red)

        # строим 3 оси X,Y,Z
        len_axel = param.Radius
        axel1 = arrow(pos=vector(0, 0, 0), axis=vector(len_axel, 0, 0), shaftwidth=10, color=color.blue)
        axel2 = arrow(pos=vector(0, 0, 0), axis=vector(0, len_axel, 0), shaftwidth=10, color=color.cyan)
        axel3 = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, len_axel), shaftwidth=10, color=color.purple)
        # center_sphere = sphere(pos=tor1.pos, radius=10, color=color.orange)

        tor1.massa = param.Massa
        tor1.II = param.I_current  # ток в торе

        tor2.massa = param.Massa
        tor2.II = param.I_current  # ток в торе
        tor1.velocity = param.Velocity[0]
        tor2.velocity = param.Velocity[1]
        self.tors.append(tor1)
        self.tors.append(tor2)

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

TORS = tors()

mu0 = 0.0000012566370614
parts_tors = []
visual = False  # нужна ли визуализация вектора магнитного поля и вектора силы

for tor in TORS.tors:
    parts_tors.append([])

# используемые для визуализации объекты
probe_sphere = sphere(pos=vector(0, 0, 0), radius=20)
# probe_sphere2 = sphere(pos=vector(0, 0, 0), radius=20)
pointer = arrow(pos=vector(0, 0, 0), axis=vector(0, 100, 0), shaftwidth=10, color=color.green)
# start_radius_vector = arrow(pos=vector(0, 0, 0), axis=vector(0, 1, 0), shaftwidth=10, color=color.green)

dt = 0.00001
t = 0.0
old_V = param.Velocity[0].z

for iter in range(100000):
    rate(1)

    # меняем временной интервал с учетом близости торов, чтобы уменьшить накапливаемую ошибку
    distance = TORS.tors[0].pos.z - TORS.tors[1].pos.z
    # dt = distance * distance * 0.000003
    time0 = time.time()

    # разбиваем Торы на части, чтобы потом к каждой из этих частей считать силы магнитного взаимодействия
    for tor in TORS.tors:
        ind_tor = TORS.tors.index(tor)
        # построение пробной сферы в одной из точек тора
        probe_sphere.pos = tor.pos
        probe_sphere.pos.x += tor.radius * sin(tor.axis.diff_angle(vector(1,0,0)))
        # probe_sphere.pos.y += tor.radius * sin(tor.axis.diff_angle(vector(0,1,0)))
        # probe_sphere.pos.z += tor.radius * sin(tor.axis.diff_angle(vector(0,0,1)))

        # start_radius_vector.delete()
        start_radius_vector = vector(probe_sphere.pos) - vector(tor.pos)
        pointer.pos = tor.pos
        pointer.stop = probe_sphere.pos
        pointer.axis = probe_sphere.pos - tor.pos

        # probe_sphere2.delete()
        # probe_sphere2 = sphere(pos=pointer.axis + tor.pos, radius=20)

        # разбиваем тор на заданное число частей
        part_len = 2 * pi * tor.radius / TORS.parts
        part_massa = tor.massa/TORS.parts
        for ii in range(TORS.parts):
            if ii >= len(parts_tors[ind_tor]):
                parts_tors[ind_tor].append(
                    {
                        'pos': tor.pos + pointer.axis,
                        'massa': part_massa,
                        'tangent': cross(tor.axis, pointer.axis),
                        'len': part_len
                    })
                # parts_tors[ind_tor][-1]['tangent'] = vector(parts_tors[ind_tor][-1]['pos'])
                # parts_tors[ind_tor][-1]['tangent'].hat = cross(tor.axis, pointer.axis)
            else:
                parts_tors[ind_tor][ii]['pos'] = pointer.axis + tor.pos
                parts_tors[ind_tor][ii]['tangent'] = cross(tor.axis, pointer.axis)
            pointer.rotate(2 * pi / TORS.parts, tor.axis)

            # sphere(pos=parts_tors[-1][-1]['pos'], radius=20)
    # if iter % 10 == 0:
    #     print("Время построения касательных к разным частям торов: " + str(int(time.time() - time0)))


    # суммируем магнитное поле от каждой части одного тора ко всем частям другого тора
    # по закону Био-Савара-Лапласа https://magn.ru/prakt/online/coil.html
    time1 = time.time()
    for jj in range(len(TORS.tors)):
        for ii in range(TORS.parts):
            parts_tors[jj][ii]['B'] = vector(0, 0, 0)
            for kk in range(len(TORS.tors)):
                if kk == jj:
                    continue  # магнитное поле от самого себя не считаем
                for zz in range(TORS.parts):
                    # if zz != ii:  # для теста направления сил
                    #     continue
                    r = vector(parts_tors[kk][zz]['pos']) - vector(parts_tors[jj][ii]['pos'])
                    r_len = mag(r)
                    vect_cross = cross(parts_tors[kk][zz]['tangent'], r)
                    if r_len != 0.0:
                        vect_cross.mag = mu0 * TORS.tors[kk].II * parts_tors[kk][zz]['len'] / 4 / pi / r_len / r_len
                        parts_tors[jj][ii]['B'] += vect_cross
                        # if jj == 0 and ii == 0 and iter == 0 and zz % 4 == 0:
                        #     print(f'Bx+ {vect_cross.x}')

    # if iter % 10 == 0:
    #     print("Время расчета Магнитного поля: " + str(int(time.time() - time1)))
    # считаем силы от каждой части одного тора ко всем частям другого тора
    for jj in range(len(TORS.tors)):
        TORS.tors[jj].part_f = []
        for ii in range(TORS.parts):
            if TORS.tors[1].pos.z > TORS.tors[0].pos.z:
                temp_vector = cross(parts_tors[jj][ii]['tangent'], parts_tors[jj][ii]['B'])
            else:
                temp_vector =-cross(parts_tors[jj][ii]['tangent'], parts_tors[jj][ii]['B'])
            temp_vector.mag = TORS.tors[jj].II * mag(parts_tors[jj][ii]['B']) * parts_tors[jj][ii]['len']
            # temp_vector = vector(0, 0, 10)
            if len(TORS.tors[jj].part_f) <= jj:
                TORS.tors[jj].part_f.append(-temp_vector)
            else:
                TORS.tors[jj].part_f[jj] += -temp_vector

            # визуализируем расположение векторов магнитного поля
            if visual and ii % 10 == 0:
                temp_vector2 = parts_tors[jj][ii]['B']
                if jj == 1:
                    temp_vector2 = -temp_vector2
                temp_vector2.mag *= 4_000_00
                if 'BB_vis' not in parts_tors[jj][ii]:
                    parts_tors[jj][ii]['BB_vis'] = arrow(pos=parts_tors[jj][ii]['pos'],
                      axis=temp_vector2, shaftwidth=10, color=color.magenta)
                    # parts_tors[jj][ii]['BB_vis'].pos = parts_tors[jj][ii]['pos']
                    # parts_tors[jj][ii]['BB_vis'].start = parts_tors[jj][ii]['pos']
                else:
                    parts_tors[jj][ii]['BB_vis'].pos = parts_tors[jj][ii]['pos']
                    # parts_tors[jj][ii]['BB_vis'].start = parts_tors[jj][ii]['pos']
                    parts_tors[jj][ii]['BB_vis'].axis = temp_vector2

            # визуализируем вектора сил
            if visual and ii % 10 == 0:
                # if jj == 0:
                #     temp_vector = -temp_vector
                parts_tors[jj][ii]['FF'] = temp_vector
                temp_vector.mag *= 0.1
                if 'FF_vis' not in parts_tors[jj][ii]:
                    parts_tors[jj][ii]['FF_vis'] = arrow(pos=vector(parts_tors[jj][ii]['pos']),
                      axis=temp_vector, shaftwidth=10, color=color.orange)
                else:
                    parts_tors[jj][ii]['FF_vis'].pos = parts_tors[jj][ii]['pos']
                    parts_tors[jj][ii]['FF_vis'].axis = temp_vector
                    # parts_tors[jj][ii]['FF_vis'] = arrow(pos=vector(parts_tors[jj][ii]['pos']),
                    #                                      axis=temp_vector, shaftwidth=10, color=color.orange)

    # print(f'Суммарная проекция силы на ось Z part_f = {TORS.part_f[0].z}')
    # e_k = TORS.tors[0].massa * TORS.tors[0].velocity.mag * TORS.tors[0].velocity.mag / 2
    t += dt
    if iter % 10 == 0:
        print('V1 = %0.0f м/с, V2 = %0.0f м/с, dt=%f, distance=%0.0f, t=%f, частей=%d, масса=%d, радиус=%d, ток=%d' % (
            TORS.tors[0].velocity.z, TORS.tors[1].velocity.z, dt, distance, t, TORS.parts, TORS.tors[1].massa, param.Radius, TORS.tors[1].II))

    # суммируем силы от разных частей одного тора
    for jj in range(len(TORS.tors)):
        TORS.tors[jj].SUM_F = vector(0.0, 0.0, 0.0)
        for kk in range(len(TORS.tors[jj].part_f)):
            TORS.tors[jj].SUM_F += TORS.tors[jj].part_f[kk]

    # print("Модель с торами: F={:.2e}, I={:.2e}, R={:.2e}, distance={:.2e}".format(
    #     TORS.tors[0].SUM_F.z, param.I_current, param.Radius, distance))

    # считаем итоговые скорость и положение тела
    for jj in range(len(TORS.tors)):
        TORS.tors[jj].pos += TORS.tors[jj].velocity * dt + dt * dt * (TORS.tors[jj].SUM_F)/2/TORS.tors[jj].massa
        TORS.tors[jj].velocity += dt * TORS.tors[jj].SUM_F / TORS.tors[jj].massa

    # динамически увеличиваем dt, если скорость меняется меньше чем на 0.1 %
    if abs(TORS.tors[0].velocity.z / old_V) < 1.01:
        dt *= 1.5
    old_V = TORS.tors[0].velocity.z

    if iter % 10 == 0:
        save_whole_model('data_logs/tors_%d_%d_dt=%f.txt' % (TORS.parts, iter, dt))
