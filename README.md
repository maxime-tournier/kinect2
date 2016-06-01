# Requirements

- kinect2 sdk

- visual studio (unless you manage to feed `Kinect.h` to a
  standard-compliant c++ compiler)

# Building

- open [kinect2.vcxproj](kinect2.vcxproj), select your preferred
  arch/type then `Build/Build Solution` (F7)

# Python Bindings

Make sure the resulting dll is in the same directory as
[kinect2.py](kinect2.py), then the following [example](example.py)
should work (you need opencv2):

```python
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
```