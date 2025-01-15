from math import atan2, atan, asin, sin, cos, pi
import numpy as np

def Eular2Quater(roll, pitch, yaw):
    """frome simulation, found that yaw control doesn't work"""
    """
    x=sin(pitch/2)*sin(yaw/2)*cos(roll/2)+cos(pitch/2)*cos(yaw/2)*sin(roll/2)
    y=sin(pitch/2)*cos(yaw/2)*cos(roll/2)+cos(pitch/2)*sin(yaw/2)*sin(roll/2)
    z=cos(pitch/2)*sin(yaw/2)*cos(roll/2)-sin(pitch/2)*cos(yaw/2)*sin(roll/2)
    w=cos(pitch/2)*cos(yaw/2)*cos(roll/2)-sin(pitch/2)*sin(yaw/2)*sin(roll/2)
    """

    w=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2)
    x=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2)
    y=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2)
    z=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2)
    return x, y, z, w

def Orien2Eular(x,y,z,w):
    roll = atan2(2*(w*x+y*z),1.-2*(x*x+y*y))
    pitch = asin(2*(w*y-z*x))
    yaw = atan2(2*(w*z+x*y),1.0-2*(z*z+y*y))

    return [roll, pitch, yaw]

'''
def GPS2Relative(uav1_gps, uav2_gps):
    R_EARTH = 6371393   #m
    PI = pi
    relative = [0,0,0]
    relative[0] = (uav2_gps[0]-uav1_gps[0])*R_EARTH/180*PI
    relative[1] = (uav2_gps[1]-uav1_gps[1])*cos(uav1_gps[0]/180*PI)*R_EARTH/180*PI
    relative[2] = (uav2_gps[2]-uav1_gps[2])
    return relative
'''

def GPS2Relative(ori_lon, ori_lat, longitude, latitude):
    earth_r = 6374*1000
    ori_r = earth_r*cos(ori_lat/180*pi)

    relative_x = ori_r*(longitude-ori_lon)/180*pi
    Relative_y = earth_r*(latitude-ori_lat)/180*pi
    return(relative_x, Relative_y)

def Relative2GPS(ori_lon, ori_lat, x, y):
    # origin longitude: ori_lon
    # origin latitude: ori_lat
    earth_r = 6374*1000
    relative_lat = (y/earth_r)/pi*180
    ori_r = earth_r*cos(ori_lat/180*pi)
    relative_lon = (x/ori_r)/pi*180

    longitude = ori_lon+relative_lon
    latitude = ori_lat+relative_lat
    return [longitude, latitude]

def Velocity2Force(vel):
    '''input velocity range from 11 to 23(m/s), out put force range from 0.23 to 1'''
    if vel < 11.:
        return 0.25
        #raise ValueError("the velocity cannot be lower than 11")
    elif vel > 25:
        return 0.9
    p_1 = -7.619
    p_2 = 226.7
    p_3 = 243.2
    q_1 = -87.11
    q_2 = 1545
    q_3 = 126.4
    force = (p_1*vel**2 + p_2*vel + p_3)/(vel**3 + q_1*vel**2 + q_2*vel + q_3)
    if force > 1:
        force = 1
    return force 
"""
def Eular2Matirx(roll, pitch, yaw):
    '''
    first: around z rotate yaw; then around y ratate pitch; then x
    '''
    cr = cos(roll)
    cp = cos(pitch)
    cy = cos(yaw)
    sr = sin(roll)
    sp = sin(pitch)
    sy = sin(yaw)

    z_y = np.array([[ cy, sy, 0], \
                    [-sy, cy, 0],\
                    [0,   0,  1]])
    
    y_p = np.array([[cp, 0, -sp], \
                    [0,  1, 0], \
                    [sp, 0, cp]])    
    
    x_r = np.array([[1, 0,  0],\
                    [0, cr, sr],\
                    [0, -sr, cr]])

    temp = np.matmul(y_p, z_y)
    matrix = np.matmul(x_r, temp)
    return matrix.tolist()


def VectorRotate(vector, matrix):
    vec = np.array(vec)
    mat = np.array(matrix)
    if vec.shape == (1,3):
        vec.transpose()
    elif not vec.shape == (3,1):
        raise ValueError("'vector' size is wrong")
    
    if not mat.shape == (3,3):
        raise ValueError("'matrix' size is wrong")
    ans = np.matmul(matrix, vec)
    ans.transpose()
    return ans.tolist()
"""
