import numpy as np 
import matplotlib.pyplot as plt

data1 = np.genfromtxt("error_data_p0.5_d2.csv", delimiter=",", names=["x", "y"])
data2 = np.genfromtxt("error_data_p0.9_d2.csv", delimiter=",", names=["x", "y"])
data3 = np.genfromtxt("error_data_p0.9_d3.csv", delimiter=",", names=["x", "y"])
data4 = np.genfromtxt("error_data_p1_d1.8.csv", delimiter=",", names=["x", "y"])
#plt.plot(data['x'], data['y'])
#plt.show()




fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(data1['x'], data1['y'])
axs[0, 0].set_title('p = 0.5, d=2')
axs[0, 1].plot(data2['x'], data2['y'], 'tab:orange')
axs[0, 1].set_title('p = 0.9, d = 2')
axs[1, 0].plot(data3['x'], data3['y'], 'tab:green')
axs[1, 0].set_title('p = 0.9, d=3')
axs[1, 1].plot(data4['x'], data4['y'], 'tab:red')
axs[1, 1].set_title('p = 1, d = 1.8')

for ax in axs.flat:
    ax.set(xlabel='x-label', ylabel='y-label')

# Hide x labels and tick labels for top plots and y ticks for right plots.
#for ax in axs.flat:
    #ax.label_outer()

plt.show()
