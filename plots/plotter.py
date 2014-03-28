################################################################################
# plotter.py
#
# PLOTS GYRO, ACCEL, AND  FILTERED ANGLE ONLY!!!!
# gimbal_prod.ino MUST BE CONFIGURED WITH graphME('axis','v') option!!!!!!!!!
#
# Modified from example found on:
# electronut.in
# https://gist.github.com/electronut/5730160
#
################################################################################

import sys
import serial
import numpy as np
#from time import sleep
from collections import deque
from matplotlib import pyplot as plt

# class that holds analog data for N samples
# aka sets up buffers
class AnalogData:
  # constr
  def __init__(self, maxLen):
    self.gyro = deque([0.0]*maxLen)
    self.accel = deque([0.0]*maxLen)
    self.filtered = deque([0.0]*maxLen)
    self.maxLen = maxLen

  # ring buffer
  def addToBuf(self, buf, val):
    if len(buf) < self.maxLen:
      buf.append(val)
    else:
      buf.pop()
      buf.appendleft(val)

  # add data
  def add(self, data):
    #if you're plotting more or less data, you need to adjust this assert length
    assert(len(data) == 3)
    #depending on the location of the values on the data stream, modify what is what
    self.addToBuf(self.gyro, data[0])
    self.addToBuf(self.accel, data[1])
    self.addToBuf(self.filtered, data[2])
    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, analogData):
    # set plot to animated
    plt.ion() 
    plt.title('Real Time Data')
    plt.xlabel('sample')
    plt.ylabel('angle (deg)')
    self.gyro_line, = plt.plot(analogData.gyro, label = 'gyro')
    self.accel_line, = plt.plot(analogData.accel, label = 'accel')
    self.filtered_line, = plt.plot(analogData.filtered, label = 'filtered')
    plt.legend(('gyro','accel','filtered'))
    #if you're dealing with data outside a range of -90 to 90 (probably not), these values set the limits of the plot
    plt.ylim([-90, 90])

  # update plot
  def update(self, analogData):
    self.gyro_line.set_ydata(analogData.gyro)
    self.accel_line.set_ydata(analogData.accel)
    self.filtered_line.set_ydata(analogData.filtered)
    plt.draw()



# main() function
def main():
     # expects 1 arg - serial port string
    if(len(sys.argv) != 2):
        print "not called correctly."
        print "python showdata.py 'device' (expects a serial port string)"
        print 'Example usage: python plotter.py /dev/ttyUSB0'
        exit(1)


    strPort = sys.argv[1]

    # plot parameters
    analogData = AnalogData(100)
    analogPlot = AnalogPlot(analogData)

    print 'plotting data...'

    ser = serial.Serial(strPort, 9600)
    while True:
        try:
            line = ser.readline()
            #data stores values from incoming serial datastream of float vals
            #DON'T TRY TO SEND THIS ANY STRINGS OR WHATNOT
            # example of clean datastream:
            #  0.00, 10, 14, 1.8, 111
            # note: commas aren't needed to seperate values
            try:
              data = [float(val) for val in line.split()]
              # this only attempts to plot if 3 values are stored in data, helps if data isn't returned
              if(len(data) == 3):
                  analogData.add(data)
                  analogPlot.update(analogData)
            except ValueError:
              print 'bad data point'
        except KeyboardInterrupt:
            print 'exiting'
            break

# call main
if __name__ == '__main__':
  main()
