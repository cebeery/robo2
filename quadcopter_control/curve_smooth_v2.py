import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline


x = np.array([1,2,3,4])

y= np.array([1,2,8,12])

x_new= np.linspace(x.min(),x.max(),300)
y_new= spline(x,y,x_new)

plt.plot(x_new,y_new)
plt.plot(x,y)

plt.show()