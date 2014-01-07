%matlab code to read dat data
% tutorial: http://home.iitb.ac.in/~rahul./ITSP/serial_comm_matlab.pdf
clear all
clc

comm_port = 'COM1'
baud = 9600;



s=serial('comm_port')
set(s, 'BaudRate', baud); %or s.BaudRate=baud;
 %verified with 
get(s, 'BaudRate') %or s.BaudRate

