from math import atan2
import numpy as np 

PI = np.pi

def is_close_enough(x: float, y: float, dist: float = 0.5) -> bool:
    return abs(x - y) < dist

def is_close_enough_2d(x_1: float, y_1: float, x_2: float, y_2: float, dist: float) -> bool:
    real_dist = np.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)
    return real_dist <= dist

def distance_2d(x_1: float, y_1: float, x_2: float, y_2: float) -> float:
    return np.sqrt((x_1 - x_2) ** 2 + (y_1 - y_2) ** 2)

def modulo_2d(x: float, y: float) -> float:
    return np.sqrt(x ** 2 + y ** 2)

def get_angle(x_1: float, y_1: float, x_2: float, y_2: float) -> float:
    return atan2(y_2 - y_1, x_2 - x_1)*180/PI

def bounding_box_vertices(img_mask: np.ndarray) -> tuple:
    '''
        Bounding box of detected region (ex: red area)
    '''
    xys_pts = np.nonzero(img_mask)
    x_of_interest = xys_pts[1]
    y_of_interest = xys_pts[0]
    
    xmin, xmax = min(x_of_interest), max(x_of_interest)
    ymin, ymax = min(y_of_interest), max(y_of_interest)
    
    return xmin, ymin, xmax, ymax

# Transformation functions

# Transform class that works with homogeneous coordinates
def translation(tx: float,ty: float,tz: float) -> np.ndarray:
    model = [[1, 0, 0 , tx],
            [0, 1, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0, 1]]
    return np.array(model)

def rotationZ(angle: float) -> np.ndarray:
    model = [[np.cos(angle), -np.sin(angle), 0 , 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
    return np.array(model)

def rotationY(angle: float) -> np.ndarray:
    model = [[np.cos(angle), 0, np.sin(angle) , 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]]
    return np.array(model)

def rotationX(angle: float) -> np.ndarray:
    model = [[1, 0, 0 , 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]]
    return np.array(model)

def scale(sx: float, sy: float, sz: float) -> np.ndarray:
    model = [[sx, 0, 0 , 0],
            [0, sy, 0, 0],
            [0, 0, sz, 0],
            [0, 0, 0, 1]]
    return np.array(model)

def circle(N: int, L: float, angle0: float = 0) -> np.ndarray:  # L é o raio e N é o numero de drones
    xc = yc = 0
    if 2*PI*L < N:
        L = round(N/(2*PI),2)
        print("Distance between drones is too short\nSetting new length as {}".format(L))
    coord = np.empty((0,4))
    z0 = 0
    angle = 2*PI/N
    for idx in range(N):    
        xi = L*np.cos(angle0+idx*angle)
        yi = L*np.sin(angle0+idx*angle)
        point = [round(xc+xi,2), round(yc+yi,2), z0, 1]
        coord = np.concatenate((coord, [point]))
    return coord

def translateFormation(form_pts: np.ndarray, tx: float, ty: float, tz: float) -> np.ndarray:

    # assert(form_pts.shape == (4,-1), "Formation points with wrong format!")
    model = translation(tx, ty, tz)
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T

def rotateFormation(form_pts: np.ndarray, anglex: float, angley: float, anglez: float) -> np.ndarray:

    # assert(form_pts.shape == (1,3), "Formation points with wrong format!")
    
    model = np.eye(4)

    # Eq 1: model equals to rotz*roty*rotx
    if anglex != 0:
        model = np.matmul(rotationX(anglex), model)
    if angley != 0:
        model = np.matmul(rotationY(angley), model)
    if anglez != 0:
        model = np.matmul(rotationZ(anglez), model)
    
    # Eq 2: form_pts equals to model*form_pts
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T

def scaleFormation(form_pts: np.ndarray, sx: float, sy: float, sz: float) -> np.ndarray:

    # assert(form_pts.shape == (1,3), "Formation points with wrong format!")

    model = scale(sx, sy, sz)
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T

def transformFormation(form_pts: np.ndarray, sx: float, sy: float, sz: float, anglex: float, angley: float, anglez: float, tx: float, ty: float, tz: float) -> np.ndarray:

    # assert(form_pts.shape == (1,3), "Formation points with wrong format!")

    modelS = scale(sx, sy, sz)

    modelR = np.eye(4)
    if anglex != 0:
        modelR = np.matmul(rotationX(anglex), modelR)
    if angley != 0:
        modelR = np.matmul(rotationY(angley), modelR)
    if anglez != 0:
        modelR = np.matmul(rotationZ(anglez), modelR)
    
    modelT = translation(tx, ty, tz)

    # Crate model matrix
    # Rotate first!!! Next scale and finally translate
    model = np.eye(4)
    model = np.matmul(modelR, model)
    model = np.matmul(modelS, model)
    model = np.matmul(modelT, model)

    # Apply model to form_points
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T
