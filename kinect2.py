import ctypes 

from ctypes import CDLL

dll = CDLL('kinect2')

dll.init.argtypes = (ctypes.c_ulong, )
dll.init.restype = ctypes.c_void_p

dll.release.argtypes = (ctypes.c_void_p, )
dll.release.restype = None

dll.update.argtypes = (ctypes.c_ulong, )
dll.update.restype = ctypes.c_void_p

from contextlib import contextmanager

class Vec3(ctypes.Structure):

    _fields_ = [ ("x", ctypes.c_float),
                 ("y", ctypes.c_float),
                 ("z", ctypes.c_float) ]


class Body(ctypes.Structure):

    _fields_ = [ ("index", ctypes.c_uint),
                 ("joint", Vec3 * 25 ) ]


class Image(ctypes.Structure):

    Pixel = ctypes.c_ubyte * 4
    
    _fields_ = [ ('width', ctypes.c_int),
                 ('height', ctypes.c_int),
                 ('data', ctypes.c_void_p) ]
    

BodyCallback = ctypes.CFUNCTYPE(None, ctypes.POINTER(Body), ctypes.c_uint)
ColorCallback = ctypes.CFUNCTYPE(None, Image)

dll.body_callback.argstype = (ctypes.c_void_p, ctypes.POINTER(BodyCallback))
dll.body_callback.restype = None

dll.color_callback.argstype = (ctypes.c_void_p, ctypes.POINTER(ColorCallback))
dll.color_callback.restype = None

class SourceType: pass
source_type = SourceType()

source_type.color = 1
source_type.body = 32

@contextmanager
def sensor(flags):
    
    instance = dll.init(flags)
    
    try:
        yield instance
    finally:
        dll.release( instance )


def make_body_callback(payload):

    def callback(data, size):

        res = payload.get('body', {})
        res.clear()
        
        for i in range(size):
            body = data[i]
            pose = [[0, 0, 0]] * 25
            
            for j in range(25):
                jth = body.joint[j]
                pose[j] = [jth.x, jth.y, jth.z]
            
            res[body.index] = pose

        payload['body'] = res
        
    return callback
        

def make_color_callback(payload):
    import numpy as np
    
    def callback(img):
        ptr = ctypes.cast(img.data, ctypes.POINTER(ctypes.c_ubyte))
        shape = (img.height, img.width, 4)

        payload['color'] = np.ctypeslib.as_array(ptr, shape)

    return callback



def frames(flags = source_type.body + source_type.color):
    '''generates frames for sources described in flags'''
    with sensor(flags) as s:

        latest = {}
        
        if flags & source_type.color:
            
            color_callback = make_color_callback(latest)
            color_cb = ColorCallback(color_callback)              
            dll.color_callback(s, color_cb)

            
        if flags & source_type.body:            
            
            body_callback = make_body_callback(latest)
            body_cb = BodyCallback(body_callback)              
            dll.body_callback(s, body_cb)

        while True:
            dll.update(s)
            yield latest



class JointType: pass

joint_type = JointType()


joint_type.spine_base = 0
joint_type.spine_mid = 1
joint_type.neck = 2
joint_type.head = 3
joint_type.shoulder_left = 4
joint_type.elbow_left = 5
joint_type.wrist_left = 6
joint_type.hand_left = 7
joint_type.shoulder_right = 8
joint_type.elbow_right = 9
joint_type.wrist_right = 10
joint_type.hand_right = 11
joint_type.hip_left = 12
joint_type.knee_left = 13
joint_type.ankle_left = 14
joint_type.foot_left = 15
joint_type.hip_right = 16
joint_type.knee_right = 17
joint_type.ankle_right = 18
joint_type.foot_right = 19
joint_type.spine_shoulder = 20
joint_type.hand_tip_left = 21
joint_type.thumb_left = 22
joint_type.hand_tip_right = 23
joint_type.thumb_right = 24
joint_type.count = 25

            
if __name__ == '__main__':

    import cv2
    import numpy as np
    import sys

    flags = source_type.color + source_type.body
    for f in frames(flags):

        img = f.get('color', None)
        if img is not None:
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                 sys.exit(0)

        body = f.get('body', None)
        if body is not None:
            print(body)
            sys.stdout.flush()

        
