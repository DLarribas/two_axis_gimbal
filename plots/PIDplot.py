################################################################################
# outputPlotter.py
#
# PLOTS ONLY FILTERED ANGLE AND OUTPUTS!!!!
# but allows for tuning of 
# gimbal_prod.ino MUST BE CONFIGURED WITH graphME('axis','t') option!!!!!!!!!
#
# Modified from example found on:
# electronut.in
# https://gist.github.com/electronut/5730160
#
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
    self.angle = deque([0.0]*maxLen)
    self.output = deque([0.0]*maxLen)
    self.Kp = deque([0.0]*maxLen)
    self.Ki = deque([0.0]*maxLen)
    self.Kd = deque([0.0]*maxLen)
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
    assert(len(data) == 5)
    #depending on the location of the values on the data stream, modify what is what
    self.addToBuf(self.angle, data[0])
    self.addToBuf(self.output, data[1])
    self.addToBuf(self.Kp, data[2])
    self.addToBuf(self.Ki, data[3])
    self.addToBuf(self.Kd, data[4])
    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, analogData):
    # set plot to animated
    plt.ion() 
    plt.title('Real Time Data')
    plt.xlabel('sample')
    plt.ylabel('angle (deg)')
    self.angle_line, = plt.plot(analogData.angle)
    self.output_line, = plt.plot(analogData.output)
    #if you're dealing with data outside a range of -90 to 90 (probably not), these values set the limits of the plot
    plt.legend(('angle','output'))
    plt.ylim([-90, 90])

  # update plot
  def update(self, analogData):
    self.angle_line.set_ydata(analogData.angle)
    self.output_line.set_ydata(analogData.output)
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
    analogData = AnalogData(1000)
    analogPlot = AnalogPlot(analogData)

    print '\n\n\n Twist pots to tune PID constants\n\n\n'
    print 'plotting data...'

    ser = serial.Serial(strPort, 9600)
    while True:
        try:
            line = ser.readline()
            #data stores values from incoming serial datastream of float vals
            #DON'T TRY TO SEND THIS ANY STRINGS OR WHATNOT
            #we need a clean data stream
            #debugging on arduino can sometimes return shit like "..0" when you start the script up
            # example of clean datastream:
            #  0.00, 10, 14, 1.8, 111
            # note: commas aren't needed to seperate values
            try:
              data = [float(val) for val in line.split()]
              # this only attempts to plot if 2 values are stored in data, helps if data isn't returned
              if(len(data) == 5):
                analogData.add(data)
                analogPlot.update(analogData)
                # PERHAPS INTRODUCE A CSV WRITER HERE
                Kp = data[2]
                Ki = data[3]
                Kd = data[4]
            except ValueError:
              print'bad data point'
        except KeyboardInterrupt:
            print 'exiting'
            print ''
            print 'The final values of the PID constants were:'
            print 'Kp = ' + str(Kp)
            print 'Ki = ' + str(Ki)
            print 'Kd = ' + str(Kd)
            break

# call main
if __name__ == '__main__':
  main()
