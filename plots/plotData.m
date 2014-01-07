clear all
clc
clf

%import the data, this makes the code traversable between linux hosts
user=char(system('whoami'))
datapath=sprintf('/home/%s/Development/projects/gimbal/data.csv', user);

%read in the data
data=csvread(datapath,1,0);


%time is first column, this had to be added using libreoffice calc. i'll look up ways to add a time via matlab in future?
time=data(:1);
gyro=data(:2);
accel=data(:3);
complementaryfilter=data(:4);


figure(1); clf;
subplot(2,1,1)
plot(gyro)
plot(accel)
plot(complementaryfilter)
xlabel('time(s)','fontsize', 10);
ylabel('angle(deg)','fontsize', 10);
legend('gyro','accel','filtered');
title('x axis','fontsize',10);
axis([0 200 -90 90])


gyroy=data(:5);
accely=data(:6);
complementaryfiltery=data(:7);

subplot(2,1,2)
plot(gyroy, accely, complementaryfiltery)
title('y axis', 'fontsize',10);
xlabel('time(s)','fontsize',10);
ylabel('angle(deg)','fontsize',10);
legend('gyro','accel','filtered');
axis([0 200 -90 90])