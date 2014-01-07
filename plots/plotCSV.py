# requires pyplot to be installed!!!!

#"""
#Simple demo with multiple subplots.
#"""
import numpy as np
import matplotlib.pyplot as plt
import getpass

user = getpass.getuser()
fname="/home/"+user+"/Development/projects/gimbal/plots/data.csv"

data = np.genfromtxt(fname, delimiter=',', skip_header=3, skip_footer=10, names=['sample','xgyro','xaccel','xfilter','ygyro','yaccel','yfilter'])



fig1 = plt.figure()
fig2 = plt.figure()
ax1=fig1.add_subplot(111)
ax1.set_title("xaxis")
ax1.set_xlabel('sample')
ax1.set_ylabel('angle')
ax1.plot(data['sample'],data['xgyro'], data['sample'],data['xaccel'], data['sample'], data['xfilter'])
ax2=fig2.add_subplot(111)
ax2.set_title("yaxis")
ax2.set_xlabel('sample')
ax2.set_ylabel('angle')
ax2.plot(data['sample'],data['ygyro'], data['sample'],data['yaccel'], data['sample'], data['yfilter'])


plt.show()