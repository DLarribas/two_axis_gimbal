# requires pyplot to be installed!!!!


import numpy as np
import matplotlib.pyplot as plt
import getpass

# data is stored on desktop
user = getpass.getuser()

#if linux:
fname="/home/"+user+"/Desktop/tmp/gimbalData/data.csv"
#if mac:
#fname = "/Users/"+user+"/Desktop/tmp/gimbalData/data.csv"

#load data
data = np.genfromtxt(fname, delimiter=',', skip_header=1, skip_footer=1, names=['xgyro','xaccel','xfilter','xoutput','ygyro','yaccel','yfilter','youtput'])


#initialize (zeroes) a sample array, then fill it with sample data
sample = [0] * len(data)
for x in xrange(0,len(data)):
 	sample[x]=x

#plot all x axis measurements
fig1 = plt.figure()
ax1=fig1.add_subplot(111)
ax1.set_title("xAxis")
ax1.set_xlabel('sample')
ax1.set_ylabel('angle')
ax1.plot(sample,data['xgyro'])
ax1.plot(sample,data['xaccel'])
ax1.plot(sample,data['xfilter'])
ax1.legend(('xGyro', 'xAccel', 'xFilter'))

#plot all y axis measurements
fig2 = plt.figure()
ax2=fig2.add_subplot(111)
ax2.set_title("yAxis")
ax2.set_xlabel('sample')
ax2.set_ylabel('angle')
ax2.plot(sample,data['ygyro'])
ax2.plot(sample,data['yaccel'])
ax2.plot(sample,data['yfilter'])
ax2.legend(('yGyro', 'yAccel', 'yFilter'))

#plot x axis control
fig3 = plt.figure()
ax3 = fig3.add_subplot(111)
ax3.set_title("xAxis control")
ax3.set_xlabel('sample')
ax3.set_ylabel('angle')
ax3.plot(sample,data['xfilter'])
ax3.plot(sample,data['xoutput'])
ax3.legend(('xFilter','xOutput'))

#plot y axis control
fig4 = plt.figure()
ax4 = fig4.add_subplot(111)
ax4.set_title("yAxis control")
ax4.set_xlabel('sample')
ax4.set_ylabel('angle')
ax4.plot(sample,data['yfilter'])
ax4.plot(sample,data['youtput'])
ax4.legend(('yFilter', 'yOutput'))
plt.show()