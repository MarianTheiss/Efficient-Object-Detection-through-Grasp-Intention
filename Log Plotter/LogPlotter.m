%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%The Script reads in logfiles from the Myo logger, also created while the Thesis and visualizes the Plots
%How-To: Fill in name and timestamp; copy log files in the directory

clear all; close all; clc;

%Enter Name of the logfile
name=''
timestamp=''


filenameEMG = strcat(name, '-emg-', timestamp ,'.csv')
filenameOrientation =  strcat(name, '-orient-', timestamp ,'.csv')
filenameGyroscope =  strcat(name, '-gyro-', timestamp ,'.csv')
filenameAcceleration =  strcat(name, '-accel-', timestamp ,'.csv')
filenameActions =  strcat(name, '-actions-', timestamp ,'.csv')

%loads the logdata
logEmg=importdata(filenameEMG,';');
nEmg=size(logEmg.data,1);
logAccel=importdata(filenameAcceleration,';');
nAccel=size(logAccel.data,1);
logGyro=importdata(filenameGyroscope,';');
nGyro=size(logGyro.data,1);
logActions=importdata(filenameActions,';');
nlogActions=size(logActions.data,1);

 
%plots EMG over whole time
figure('Name','EMG')
subplot(2,4,1);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,2));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg1');
subplot(2,4,2);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,3));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg2');
subplot(2,4,3);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,4));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg3');
subplot(2,4,4);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,5));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg4');
subplot(2,4,5);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,6));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg5');
subplot(2,4,6);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,7));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg6');
subplot(2,4,7);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,8));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg7');
subplot(2,4,8);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,9));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg8');
 
%plots Gyrometer and Acceleration over whole time
figure('Name','Gyroscrope')
hold on; grid on;
plot((logGyro.data(:,1)-logGyro.data(1,1)),logGyro.data(:,2),'black');
plot((logGyro.data(:,1)-logGyro.data(1,1)),logGyro.data(:,3),'g');
plot((logGyro.data(:,1)-logGyro.data(1,1)),logGyro.data(:,4),'b');
plotEventMarker(logActions.data(:,1), 300, size(logActions.data,1));
title('Gyroscrope');
legend('x','y','z');
axis([0 logGyro.data(nGyro,1)-logGyro.data(1,1) -300 300]);
 
figure('Name','Accelerometer')
hold on; grid on;
plot((logAccel.data(:,1)-logAccel.data(1,1)),logAccel.data(:,2),'black');
plot((logAccel.data(:,1)-logAccel.data(1,1)),logAccel.data(:,3),'g');
plot((logAccel.data(:,1)-logAccel.data(1,1)),logAccel.data(:,4),'b');
title('Acceleration');
legend('x','y','z');
plotEventMarker(logActions.data(:,1), 3, size(logActions.data,1));
axis([0 logAccel.data(nAccel,1)-logAccel.data(1,1) -3 3]);

%plots EMG over whole time
figure('Name','EMG')
subplot(3,1,1);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,2));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg1');
subplot(3,1,2);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,5));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg4');
subplot(3,1,3);
plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,7));
axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -128 128]);
hold on;grid on;
plotEventMarker(logActions.data(:,1), 128, size(logActions.data,1));
title('emg6');
