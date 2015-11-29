%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%it reads in logfiles from the Myo logger also created while the Thesis and visualizes the feature spaces
%%How-To: Choose output; insert name and timestamp; Copy log files in this directory

clear all; close all; clc;

visualizeDataPoints=true
visualizePDF=true
loadFFTFeatures=true %disables visualizeFFTFeatures
plotFFTFeaturesOverTimeFeaturesPDFs=false %only when load FFTFeatures active
plotFFTFeaturesOverFFTFeaturesPDFs=false %only when load FFTFeatures active
visualizeFFTFeatures=true
plotRawDataOverTime=true
visualizePCA=true
visualizeEMGOverAcc=true
visualizeEMGOverEulerAngles=true
visualizeAngleoverAcc=true
visualizeEMGOverGyro=true
visualizeGyroOverAcc=true



%Enter Name of the logfile
name=''
timestamp=''
mkdir(strcat(name,'-',timestamp))

filenameEMG = strcat(name, '-emg-', timestamp ,'.csv')
filenameOrientation =  strcat(name, '-orient-', timestamp ,'.csv')
filenameGyroscope =  strcat(name, '-gyro-', timestamp ,'.csv')
filenameAcceleration =  strcat(name, '-accel-', timestamp ,'.csv')
filenameActions =  strcat(name, '-actions-', timestamp ,'.csv')
filenameTestRange =  strcat(name, '-featurespacetest-', timestamp ,'.csv')

%loads the logdata
logEmg=importdata(filenameEMG,';');
logAccel=importdata(filenameAcceleration,';');
logOrient=importdata(filenameOrientation,';');
logGyro=importdata(filenameGyroscope,';');
logActions=importdata(filenameActions,';');
logTestRange=importdata(filenameTestRange,';');
logTestRange.data(:,:)=logTestRange.data(:,:)*1000+logAccel.data(1,1);

rows=size(logTestRange.data, 1);
columns=size(logTestRange.data, 2);

%trims the logfile to Test Range
logEmg.data(find(logEmg.data(:,1)>logTestRange.data(rows,columns), 1, 'first'):size(logEmg.data,1),:)=[];
logEmg.data(1:find(logEmg.data(:,1)>logTestRange.data(1,1), 1, 'first'),:)=[];
nEmg=size(logEmg.data,1);
logAccel.data(find(logAccel.data(:,1)>logTestRange.data(rows,columns), 1, 'first'):size(logAccel.data,1),:)=[];
logAccel.data(1:find(logAccel.data(:,1)>logTestRange.data(1,1), 1, 'first'),:)=[];
nAccel=size(logAccel.data,1);
logOrient.data(find(logOrient.data(:,1)>logTestRange.data(rows,columns), 1, 'first'):size(logOrient.data,1),:)=[];
logOrient.data(1:find(logOrient.data(:,1)>logTestRange.data(1,1), 1, 'first'),:)=[];
nOrient=size(logOrient.data,1);
logGyro.data(find(logGyro.data(:,1)>logTestRange.data(rows,columns), 1, 'first'):size(logGyro.data,1),:)=[];
logGyro.data(1:find(logGyro.data(:,1)>logTestRange.data(1,1), 1, 'first'),:)=[];
nGyro=size(logGyro.data,1);
logGyro.data(:,5)=sqrt(logGyro.data(:,2).^2 + logGyro.data(:,3).^2 + logGyro.data(:,4).^2);
[eulerAnglesypr(:,1) eulerAnglesypr(:,2) eulerAnglesypr(:,3)]= quat2angle([logOrient.data(:,5), logOrient.data(:,2), logOrient.data(:,3), logOrient.data(:,4)]);
%estimates the acceleration without gravity due a highpass
linAccel=[logAccel.data(:,1) zeros(size(logAccel.data,1), size(logAccel.data,2)-1)]
lp=[0 0 0]
for i=2:size(logAccel.data,1)
    [lp linAccel(i, 2:4)]=lowAndHighpass(18000, logAccel.data(i,1)-logAccel.data(i-1,1), logAccel.data(i, 2:4), lp);
end


%sets a colorscheme, , MovingArm, ArmToGrasp, GrasptoDeposit,
%DepositToStop, Nullclass
colorScheme=['c'; 'g'; 'r'; 'm'; 'b']';
xyzm=['X','Y','Z','M'];
yawpitchroll={'Yaw','Pitch','Roll'};


if plotRawDataOverTime==true
    %Accerleration over time with bars at start(r), stop (r),
    %armmoving(magenta), grasp(magenta), losing grasp(magenta)
    figure('Name','Accelerometer/time')
    hold on; grid off;
    plot(logAccel.data(:,1)-logAccel.data(1,1),logAccel.data(:,2),'black');
    plot(logAccel.data(:,1)-logAccel.data(1,1),logAccel.data(:,3),'g');
    plot(logAccel.data(:,1)-logAccel.data(1,1),logAccel.data(:,4),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logAccel.data(nAccel,1)-logAccel.data(1,1) -2 2]);
    legend('x-Acceleration', 'y-Acceleration', 'z-Acceleration')
    xlabel('Time (ns)'), ylabel('Acceleration (g)')
    hgsave(strcat(name, '-', timestamp, '\AccelTime'))

    %Gyrometer over time with bars at start(r), stop (r),
    %armmoving(magenta), grasp(magenta), losing grasp(magenta)
    figure('Name','Gyrometer/time')
    hold on; grid off;
    plot(logGyro.data(:,1)-logGyro.data(1,1),logGyro.data(:,2),'black');
    plot(logGyro.data(:,1)-logGyro.data(1,1),logGyro.data(:,3),'g');
    plot(logGyro.data(:,1)-logGyro.data(1,1),logGyro.data(:,4),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logGyro.data(nGyro,1)-logGyro.data(1,1) -200 200]);
    legend('wx', 'wy', 'wz')
    xlabel('Time (ns)'), ylabel('Angular velocities (deg/s)')
    hgsave(strcat(name, '-', timestamp, '\GyroTime'))

    %Orientation over time with bars at start(r), stop (r),
    %armmoving(magenta), grasp(magenta), losing grasp(magenta)
    figure('Name','Orientation/time')
    hold on; grid off;
    plot(logOrient.data(:,1)-logOrient.data(1,1),logOrient.data(:,2),'black');
    plot(logOrient.data(:,1)-logOrient.data(1,1),logOrient.data(:,3),'g');
    plot(logOrient.data(:,1)-logOrient.data(1,1),logOrient.data(:,4),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logOrient.data(nOrient,1)-logOrient.data(1,1) -1 1]);
    hgsave(strcat(name, '-', timestamp, '\OrientTime'))


    %Euler Angles over time with bars at start(r), stop (r),
    %armmoving(magenta), grasp(magenta), losing grasp(magenta)
    figure('Name','EulerAngles/Time')
    hold on; grid on;
    plot((logOrient.data(:,1)-logOrient.data(1,1)),eulerAnglesypr(:,2),'m');
    plot((logOrient.data(:,1)-logOrient.data(1,1)),eulerAnglesypr(:,3),'g');
    plot((logOrient.data(:,1)-logOrient.data(1,1)),eulerAnglesypr(:,1),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    title('Euler Angles');
    legend('pitch','roll','yaw');
    xlabel('Time (s)'), ylabel('Angle (rad)')
    axis([0 logOrient.data(nOrient,1)-logOrient.data(1,1) -3 3]);
    hgsave(strcat(name, '-', timestamp, '\AnglesTime'))
    
    figure('Name','IMU/time')
    subplot(3,1,1)
    hold on; grid off;
    plot(logAccel.data(:,1)-logAccel.data(1,1),logAccel.data(:,2),'black');
    plot(logAccel.data(:,1)-logAccel.data(1,1),logAccel.data(:,3),'g');
    plot(logAccel.data(:,1)-logAccel.data(1,1),logAccel.data(:,4),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logAccel.data(nAccel,1)-logAccel.data(1,1) -2 2]);
    legend('x-Acceleration', 'y-Acceleration', 'z-Acceleration')
    ylabel('Acceleration (g)')
    subplot(3,1,2)
    hold on; grid off;
    plot(logGyro.data(:,1)-logGyro.data(1,1),logGyro.data(:,2),'black');
    plot(logGyro.data(:,1)-logGyro.data(1,1),logGyro.data(:,3),'g');
    plot(logGyro.data(:,1)-logGyro.data(1,1),logGyro.data(:,4),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logGyro.data(nGyro,1)-logGyro.data(1,1) -200 200]);
    legend('wx', 'wy', 'wz') 
    ylabel('Angular velocities (deg/s)')
    subplot(3,1,3)
    hold on; grid on;
    plot((logOrient.data(:,1)-logOrient.data(1,1)),eulerAnglesypr(:,2),'black');
    plot((logOrient.data(:,1)-logOrient.data(1,1)),eulerAnglesypr(:,3),'g');
    plot((logOrient.data(:,1)-logOrient.data(1,1)),eulerAnglesypr(:,1),'b');
    plotEventMarker(logTestRange.data, 10000, rows);
    legend('pitch','roll','yaw');
    xlabel('Time (ns)'), ylabel('Euler Angle (rad)')
    axis([0 logOrient.data(nOrient,1)-logOrient.data(1,1) -3 3]);

    %EMG over time with bars at start(r), stop (r),
    %armmoving(magenta), grasp(magenta), losing grasp(magenta)
    figure('Name','EMG1/time')
    grid on; hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,2));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG2/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,3));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG3/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,4));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG4/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,5));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG5/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,6));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG6/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,7));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG7/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,8));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    figure('Name','EMG8/time')
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,9));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    hgsave(strcat(name, '-', timestamp, '\EmgTime'))
    
    %EMG over time with bars at start(r), stop (r),
    %armmoving(magenta), grasp(magenta), losing grasp(magenta)
    figure('Name','EMG/time')
    subplot(8,1,1)
    grid on; hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,2));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG1')
    subplot(8,1,2)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,3));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG2')
    subplot(8,1,3)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,4));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG3')
    subplot(8,1,4)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,5));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG4')
    subplot(8,1,5)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,6));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG5')
    subplot(8,1,6)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,7));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG6')
    subplot(8,1,7)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,8));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG7')
    subplot(8,1,8)
    grid on;  hold on;
    plot((logEmg.data(:,1)-logEmg.data(1,1)),logEmg.data(:,9));
    plotEventMarker(logTestRange.data, 10000, rows);
    axis([0 logEmg.data(nEmg,1)-logEmg.data(1,1) -50 50]);
    ylabel('EMG8'), xlabel('Time/s')
end







%%Acceleration over EMG
if visualizeEMGOverAcc==true
    nearestTimestamps=knnsearch(logAccel.data(:,1), logEmg.data(1:end,1));
    if visualizeDataPoints==true
        emgXHandler=figure('Name','EMG/AccelerationX'); 
        emgYHandler=figure('Name','EMG/AccelerationY');
        emgZHandler=figure('Name','EMG/AccelerationZ');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:8
                subplot(2,4,i);
                hold on;
                axis([-2 2 -50 50]);
                xlabel(strcat('Acceleration ', xyzm(j))); ylabel(strcat('emg', num2str(i)))
                grid on;
             end
        end
        showDataPoints(rows, columns, logEmg.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, logAccel.data(:,2:4), logEmg.data(:,2:9), colorScheme, [2,4], 'EmgAccel',timestamp, name)
    end


    %plots the pdf of Emg over Acceleration
    if visualizePDF==true
        emgXHandler=figure('Name','PDF_EMG/AccelerationX'); 
        emgYHandler=figure('Name','PDF_EMG/AccelerationY');
        emgZHandler=figure('Name','PDF_EMG/AccelerationZ');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:8
                subplot(2,4,i);
                hold on;
                axis([-2 2 -50 50]);
                xlabel(strcat('Acceleration ', xyzm(j))); ylabel(strcat('emg', num2str(i)))
                grid on;
             end
        end
        showPDF(rows, columns, logEmg.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, logAccel.data(:,2:4), logEmg.data(:,2:9), colorScheme, [2,4], 'EmgAccel', timestamp, name)
    end
end

%%Linear Acceleration over Gyro
if visualizeGyroOverAcc==true
    nearestTimestamps=knnsearch(linAccel(:,1), logGyro.data(:,1));
    if visualizeDataPoints==true
        emgXHandler=figure('Name','Gyro/LinAccelerationX'); 
        emgYHandler=figure('Name','Gyro/LinAccelerationY');
        emgZHandler=figure('Name','Gyro/LinAccelerationZ');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:4
                subplot(2,2,i);
                hold on;
                axis([-2 2 -150 150]);
                xlabel(strcat('LinAcceleration ', xyzm(j))); ylabel(strcat('Gyro', xyzm(i)))
                grid on;
             end
        end
        showDataPoints(rows, columns, logGyro.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, linAccel(:,2:4), logGyro.data(:,2:5), colorScheme, [2,2], 'GyroLinAccel',timestamp, name)
    end


    %plots the pdf of Emg over Acceleration
    if visualizePDF==true
        emgXHandler=figure('Name','PDF_Gyrp/LinAccelerationX'); 
        emgYHandler=figure('Name','PDF_Gyro/LinAccelerationY');
        emgZHandler=figure('Name','PDF_Gyro/LinAccelerationZ');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:4
                subplot(2,2,i);
                hold on;
                axis([-2 2 -150 150]);
                xlabel(strcat('LinAcceleration ', xyzm(j))); ylabel(strcat('Gyro', xyzm(i)))
                grid on;
             end
        end
        showPDF(rows, columns, logGyro.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, linAccel(:,2:4), logGyro.data(:,2:5), colorScheme, [2,2], 'GyroLinAccel', timestamp, name)
    end
end



%Euler Angles over EMG
if visualizeEMGOverEulerAngles==true
    nearestTimestamps=knnsearch(logOrient.data(:,1), logEmg.data(1:end,1));
    if visualizeDataPoints==true
        emgXHandler=figure('Name','EMG/Pitch');
        emgYHandler=figure('Name','EMG/Roll'); 
        emgZHandler=figure('Name','EMG/Yaw');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:8
                subplot(2,4,i);
                hold on;
                axis([-3 3 -50 50]);
                xlabel(yawpitchroll(j)); ylabel(strcat('emg', num2str(i)))
                grid on;
             end
        end
        showDataPoints(rows, columns, logEmg.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, eulerAnglesypr(:,1:3), logEmg.data(:,2:9), colorScheme, [2,4], 'EmgAngles', timestamp, name)
    end
    if visualizePDF==true
        emgXHandler=figure('Name','PDF_EMG/Pitch'); 
        emgYHandler=figure('Name','PDF_EMG/Roll');
        emgZHandler=figure('Name','PDF_EMG/Yaw');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:8
                subplot(2,4,i);
                hold on;
                axis([-3 3 -50 50]);
                xlabel(yawpitchroll(j)); ylabel(strcat('emg', num2str(i)))
                grid on;
             end
        end
        showPDF(rows, columns, logEmg.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, eulerAnglesypr(:,1:3), logEmg.data(:,2:9), colorScheme, [2,4], 'EmgAngles', timestamp, name)
    end
end




%Euler Angles over Acceleration
if visualizeAngleoverAcc==true
    nearestTimestamps=knnsearch(logAccel.data(1:end,1), logOrient.data(:,1));
    if visualizeDataPoints==true
        emgXHandler=figure('Name','EulerAngles/AccelerationX');
        emgYHandler=figure('Name','EulerAngles/AccelerationY'); 
        emgZHandler=figure('Name','EulerAngles/AccelerationZ');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:3
                subplot(1,3,i);
                hold on;
                axis([-2 2 -3 3]);
                xlabel(strcat('Acceleration', xyzm(j))); ylabel(yawpitchroll(i));
                grid on;
             end
        end
        showDataPoints(rows, columns, logOrient.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, logAccel.data(:,2:4), eulerAnglesypr(:,1:3), colorScheme, [1,3], 'AnglesAccel', timestamp, name)
    end
    
    
    if visualizePDF==true
        
        emgXHandler=figure('Name','PDF_EulerAngles/AccelerationX');
        emgYHandler=figure('Name','PDF_EulerAngles/AccelerationY'); 
        emgZHandler=figure('Name','PDF_EulerAngles/AccelerationZ');
        for j=1:3
            figure(emgXHandler+j-1)
             for i=1:3
                subplot(1,3,i);
                hold on;
                axis([-2 2 -3 3]);
                xlabel(strcat('Acceleration', xyzm(j))); ylabel(yawpitchroll(i));
                grid on;
             end
        end
        showPDF(rows, columns, logOrient.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, logAccel.data(:,2:4), eulerAnglesypr(:,1:3), colorScheme, [1,3], 'AnglesAccel', timestamp, name)
    end
end




%EMG over Gyro
if visualizeEMGOverGyro==true
    nearestTimestamps=knnsearch(logGyro.data(:,1), logEmg.data(1:end,1));
    %Plots the Datapoints
    if visualizeDataPoints==true
        emgXHandler=figure('Name','EMG/GyroX'); 
        emgYHandler=figure('Name','EMG/GyroY');
        emgZHandler=figure('Name','EMG/GyroZ');
        emgMHandler=figure('Name','EMG/GyroM');
        for j=1:4
            figure(emgXHandler+j-1)
             for i=1:8
                subplot(2,4,i);
                hold on;
                axis([-70 70 -300 300]);
                xlabel(strcat('Gyro ', xyzm(j))); ylabel(strcat('emg', num2str(i)))
                grid on;
             end
        end
        showDataPoints(rows, columns, logEmg.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, logGyro.data(:,5:5), logEmg.data(:,2:9), colorScheme, [2,4], 'EmgGyro', timestamp, name)
    end
    
    %plots the pdf
    if visualizePDF==true
        emgXHandler=figure('Name','PDF_EMG/GyroX'); 
        emgYHandler=figure('Name','PDF_EMG/GyroY');
        emgZHandler=figure('Name','PDF_EMG/GyroZ');
        emgMHandler=figure('Name','PDF_EMG/GyroM');
        for j=1:4
            figure(emgXHandler+j-1)
             for i=1:8
                subplot(2,4,i);
                hold on;
                axis([-300 300 -50 50]);
                xlabel(strcat('Gyro ', xyzm(j))); ylabel(strcat('emg', num2str(i)))
                grid on;
             end
        end
        showPDF(rows, columns, logEmg.data(:,1), nearestTimestamps, logTestRange.data, emgXHandler, logGyro.data(:,2:5), logEmg.data(:,2:9), colorScheme, [2,4], 'EmgGyro', timestamp, name)
    end
end

if loadFFTFeatures==true
    if visualizeFFTFeatures==true
        visible='visible';
    else
        visible='invisible';
    end
   figurehandlerstart=openfig(strcat(name, '-', timestamp, '\accelx cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accely cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accelz cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyrox cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyroy cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyroz cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyrom cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\pitch cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\roll cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\yaw cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg1 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg2 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg3 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg4 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg5 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg6 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg7 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg8 cepstral.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accelx expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accely expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accelz expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyrox expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyroy expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyroz expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyrom expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\pitch expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\roll expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\yaw expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg1 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg2 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg3 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg4 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg5 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg6 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg7 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg8 expBand.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accelx entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accely entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\accelz entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyrox entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyroy entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyroz entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\gyrom entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\pitch entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\roll entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\yaw entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg1 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg2 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg3 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg4 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg5 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg6 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg7 entropy.fig'), 'reuse', visible);
   openfig(strcat(name, '-', timestamp, '\emg8 entropy.fig'), 'reuse', visible);
   figurehandlerend=gcf
   %plot the FFT features over time
   if plotFFTFeaturesOverTimeFeaturesPDFs == true
        for i=figurehandlerstart:figurehandlerend
            [X,Y,yVarName,nrows,ncols]=getDataFromFig(i, name, timestamp);
            plotFFTPDFs(X, Y, rows, columns, logAccel.data(:,1), logTestRange.data, logAccel.data(:,2), colorScheme, [ncols,nrows], 'AccelX', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logAccel.data(:,1), logTestRange.data, logAccel.data(:,3), colorScheme, [ncols,nrows], 'AccelY', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logAccel.data(:,1), logTestRange.data, logAccel.data(:,4), colorScheme, [ncols,nrows], 'AccelZ', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logGyro.data(:,1), logTestRange.data, logGyro.data(:,2), colorScheme, [ncols,nrows], 'GyroX', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logGyro.data(:,1), logTestRange.data, logGyro.data(:,3), colorScheme, [ncols,nrows], 'GyroY', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logGyro.data(:,1), logTestRange.data, logGyro.data(:,4), colorScheme, [ncols,nrows], 'GyroZ', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logGyro.data(:,1), logTestRange.data, logGyro.data(:,5), colorScheme, [ncols,nrows], 'GyroM', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logOrient.data(:,1), logTestRange.data, logOrient.data(:,2), colorScheme, [ncols,nrows], 'OrientX', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logOrient.data(:,1), logTestRange.data, logOrient.data(:,3), colorScheme, [ncols,nrows], 'OrientY', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logOrient.data(:,1), logTestRange.data, logOrient.data(:,4), colorScheme, [ncols,nrows], 'OrientZ', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logOrient.data(:,1), logTestRange.data, eulerAnglesypr(:,1), colorScheme, [ncols,nrows], 'Yaw', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logOrient.data(:,1), logTestRange.data, eulerAnglesypr(:,2), colorScheme, [ncols,nrows], 'Pitch', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logOrient.data(:,1), logTestRange.data, eulerAnglesypr(:,3), colorScheme, [ncols,nrows], 'Roll', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,2), colorScheme, [ncols,nrows], 'Emg1', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,3), colorScheme, [ncols,nrows], 'Emg2', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,4), colorScheme, [ncols,nrows], 'Emg3', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,5), colorScheme, [ncols,nrows], 'Emg4', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,6), colorScheme, [ncols,nrows], 'Emg5', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,7), colorScheme, [ncols,nrows], 'Emg6', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,8), colorScheme, [ncols,nrows], 'Emg7', yVarName, timestamp, name);
            plotFFTPDFs(X, Y, rows, columns, logEmg.data(:,1), logTestRange.data, logEmg.data(:,9), colorScheme, [ncols,nrows], 'Emg8', yVarName, timestamp, name);
        end
    end
    %plot the FFT Features over the FFT Features
    if plotFFTFeaturesOverFFTFeaturesPDFs == true
        for i=figurehandlerstart:figurehandlerend-1
            for j=i+1:figurehandlerend
                [X,Y,yVarName,nrows,ncols]=getDataFromFig(i, name, timestamp);
                [X2,Y2,xVarName,nrows2,ncols2]=getDataFromFig(j, name, timestamp);
                %only if same subplot size -> only cepstral with cepstral,
                %energy with energy.....
                if (ncols==ncols2 && nrows==nrows2)
                    plotFFToverFFTPDFs(X, Y, rows, columns, X2(:,1), logTestRange.data, Y2, colorScheme, [ncols,nrows], xVarName, yVarName, timestamp, name);
                end
            end
        end
    end
    %Calculates the FFT Featzres and save them as figure file
elseif visualizeFFTFeatures==true
    showFFTFeatures(rows, 'AccelX', logAccel.data(:,1), logTestRange.data, logAccel.data(:,2), 50, name, timestamp)
    showFFTFeatures(rows, 'AccelY', logAccel.data(:,1), logTestRange.data, logAccel.data(:,3), 50, name, timestamp);
    showFFTFeatures(rows, 'AccelZ', logAccel.data(:,1), logTestRange.data, logAccel.data(:,4), 50, name, timestamp)
    showFFTFeatures(rows, 'GyroX', logGyro.data(:,1), logTestRange.data, logGyro.data(:,2), 50, name, timestamp)
    showFFTFeatures(rows, 'GyroY', logGyro.data(:,1), logTestRange.data, logGyro.data(:,3), 50, name, timestamp)
    showFFTFeatures(rows, 'GyroZ', logGyro.data(:,1), logTestRange.data, logGyro.data(:,4), 50, name, timestamp)
    showFFTFeatures(rows, 'GyroM', logGyro.data(:,1), logTestRange.data, logGyro.data(:,5), 50, name, timestamp)
    showFFTFeatures(rows, 'Pitch', logOrient.data(:,1), logTestRange.data, eulerAnglesypr(:,2), 50, name, timestamp)
    showFFTFeatures(rows, 'Roll', logOrient.data(:,1), logTestRange.data, eulerAnglesypr(:,3), 50, name, timestamp)
    showFFTFeatures(rows, 'Yaw', logOrient.data(:,1), logTestRange.data, eulerAnglesypr(:,1), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG1', logEmg.data(:,1), logTestRange.data, logEmg.data(:,2), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG2', logEmg.data(:,1), logTestRange.data, logEmg.data(:,3), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG3', logEmg.data(:,1), logTestRange.data, logEmg.data(:,4), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG4', logEmg.data(:,1), logTestRange.data, logEmg.data(:,5), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG5', logEmg.data(:,1), logTestRange.data, logEmg.data(:,6), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG6', logEmg.data(:,1), logTestRange.data, logEmg.data(:,7), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG7', logEmg.data(:,1), logTestRange.data, logEmg.data(:,8), 50, name, timestamp)
    showFFTFeatures(rows, 'EMG8', logEmg.data(:,1), logTestRange.data, logEmg.data(:,9), 50, name, timestamp)
end


%Principal Component Analysis
if visualizePCA==true
    nearestTimestampsAccel=knnsearch(logAccel.data(:,1), logEmg.data(:,1));
    nearestTimestampsEuler=knnsearch(logOrient.data(:,1), logEmg.data(:,1));
    nearestTimestampsGyro=knnsearch(logGyro.data(:,1), logEmg.data(:,1));
    figure('Name','PCA')
    axis([-150 150 -150 150]);
    hold on;
    idx=1;
    for i=1:rows
        for j=1:columns-1
            
            endidx=find(logEmg.data(:,1)>=logEmg.data(1,1)+logTestRange.data(i,j+1)-logTestRange.data(1,1),1,'first');
            if ~isempty(endidx)
                [pc, zscores, pcvars] = princomp([logEmg.data(idx:endidx,2:8), logAccel.data(nearestTimestampsAccel(idx:endidx,1),2:4), eulerAnglesypr(nearestTimestampsEuler(idx:endidx,1),1:3),logGyro.data(nearestTimestampsGyro(idx:endidx,1),2:4)]);        
                plotPDF(1:size(zscores,1), zscores(:,1), zscores(:,2), colorScheme(j));
                cumsum(pcvars./sum(pcvars) * 100)
            end
            idx=endidx;
        end
        if i<rows
            endidx=find(logEmg.data(:,1)>=logEmg.data(1,1)+logTestRange.data(i+1,1)-logTestRange.data(1,1),1,'first');
            [pc, zscores, pcvars] = princomp([logEmg.data(idx:endidx,2:8), logAccel.data(nearestTimestampsAccel(idx:endidx,1),2:4), eulerAnglesypr(nearestTimestampsEuler(idx:endidx,1),1:3),logGyro.data(nearestTimestampsGyro(idx:endidx,1),2:4)]);
            plotPDF(1:size(zscores,1), zscores(:,1), zscores(:,2), colorScheme(end));
            cumsum(pcvars./sum(pcvars) * 100)
            idx=endidx;
        end
    end
    hgsave(strcat(name, '-', timestamp, '\PCA'))
end
