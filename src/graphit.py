from PIL import Image

import matplotlib.pyplot as plt
import numpy as np

f = open("16x16path.txt")
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

# print(x,y)

img = np.asarray(Image.open('images/LPSR_85S_060M_201608.jpg'))
altImg = np.copy(img[0:2529, 0:5058]) # New altimage size for width/height

for i in range(len(x)):
    xi = x[i]
    yi = y[i]
    # print(altImg)
    altImg[xi][yi] = 16711680
imgplot = plt.imshow(altImg)
plt.show()
