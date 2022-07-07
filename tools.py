import numpy as np

# import cupy as cp


def unit_vector(v):
    L = np.sqrt(v[0]**2+v[1]**2+v[2]**2)
    return v / L


def degree2radian(alpha):
    alpha = (alpha % 360)
    if alpha > 180:
        alpha -= 360
    alpha = np.radians(alpha)
    return alpha



def time2name(t, l=10):
    name = str(int(float(t) * 1000))
    if len(name) > 10:
        return name
    zeros = ['0'] * (10 - len(name))
    zeros = ''.join(zeros)
    return zeros + name


CYCLIST = ['vehicle.bh.crossbike',
           'vehicle.diamondback.century',
           'vehicle.harley-davidson.low_rider',
           'vehicle.gazelle.omafiets',
           'vehicle.kawasaki.ninja',
           'vehicle.yamaha.yzf']


# @functools.lru_cache()
def classify_actor(actor):
    if actor['type'].startswith('walker'):
        return 'Pedestrian'
    elif actor['l'] >= 4:
        return 'Truck'
    elif actor['type'] in CYCLIST:
        return 'Cyclist'
    else:
        return 'Car'