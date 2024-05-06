# Python script for graphing a found path onto a map image. 
# Running From The Night: Calculating The Lunar Magellan Route in Parallel
# Authors: Kevin Fang (kevinfan) and Nikolai Stefanov (nstefano)

from PIL import Image

import matplotlib.pyplot as plt
import numpy as np

f = open("16x16path.txt") # Change this to the path file you want to graph
s = f.read()
print(s)
A = s.split("-> ")
print(A)
x = []
y = []
for i in range(len(A) - 1):
    print(A[i])
    j = A[i].split(",")
    print(j)
    x.append(int(j[0][1:]))
    y.append(int(j[1][:-1]))


img = np.asarray(Image.open('images/LPSR_85S_060M_201608.jpg')) # Image you want to graph onto
altImg = np.copy(img[0:2529, 0:5058]) # New image size for width/height chopped from image above

for i in range(len(x)):
    xi = x[i]
    yi = y[i]
    # print(altImg)
    altImg[xi][yi] = 16711680
imgplot = plt.imshow(altImg)
plt.show()
