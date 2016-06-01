
import kinect2

import cv2
import sys

flags = kinect2.source_type.color + kinect2.source_type.body
for f in kinect2.frames(flags):

    img = f.get('color', None)
    if img is not None:
        cv2.imshow('image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            sys.exit(0)

    body = f.get('body', None)
    if body is not None:
        print(body)
        sys.stdout.flush()
