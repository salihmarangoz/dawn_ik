import numpy as np

r = 0.01
x = [0.35]
y = np.arange(-0.5, 0.5, r*2)
z = np.arange(0, 0.5, r*2)
frame = "link_base"


for x_ in x:
    for y_ in y:
        for z_ in z:
            print("    - link: \"{}\"".format(frame))
            print("      shape: \"Sphere({:.3f})\"".format(r))
            print("      translation: [{:.3f}, {:.3f}, {:.3f}]".format(x_, y_, z_))

