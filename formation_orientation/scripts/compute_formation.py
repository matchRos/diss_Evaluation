#!/usr/bin/env python3

import math

r = (0.75**2 + 1.5**2)
r = math.sqrt(r) * 2

print(r)

robot1 = [35.0,36.50]

for theta in range(0, 91, 15):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    print(theta,x/2, y/2, x+robot1[0], - y + robot1[1])
