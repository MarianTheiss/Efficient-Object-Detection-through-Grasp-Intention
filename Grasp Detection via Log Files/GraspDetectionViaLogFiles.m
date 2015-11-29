%%this script is made for the Master Thesis on RFID Grasps
%%it reads in logfiles from the mYo logger also created while the Thesis
%%and tries to detect the grasps
%%Author: Marian Theiss

clear all; close all; clc;

%% setting filenames and flags

%Enter Name of the logfile
%name='einfache_bewegungen'
%timestamp='5_28-12_19_54'
name=''
timestamp=''

filenameEMG = strcat(name, '-emg-', timestamp ,'.csv')
filenameOrientation =  strcat(name, '-orient-', timestamp ,'.csv')
filenameGyroscope =  strcat(name, '-gyro-', timestamp ,'.csv')
filenameAcceleration =  strcat(name, '-accel-', timestamp ,'.csv')
filenameActions =  strcat(name, '-actions-', timestamp ,'.csv')
%filenameTestRange =  strcat(name, '-featurespacetest-', timestamp ,'.csv')

%% building the Matrix

%loads the logdata
logEmg=importdata(filenameEMG,';');
logAccel=importdata(filenameAcceleration,';');
logOrient=importdata(filenameOrientation,';');
logGyro=importdata(filenameGyroscope,';');
logActions=importdata(filenameActions,';');
%logTestRange=importdata(filenameTestRange,';');
%logTestRange.data(:,:)=logTestRange.data(:,:)*1000+logAccel.data(1,1);

%set the logfiledata to a 0 time
mutualStartTime = max([logEmg.data(1,1), logOrient.data(1,1), logGyro.data(1,1), logAccel.data(1,1), logActions.data(1,1)]);
%mutualStartTime= max([logEmg.data(1,1), logOrient.data(1,1), logGyro.data(1,1), logAccel.data(1,1)]);
logEmg.data(:,1)=logEmg.data(:,1)-mutualStartTime;
logAccel.data(:,1)=logAccel.data(:,1)-mutualStartTime;
logOrient.data(:,1)=logOrient.data(:,1)-mutualStartTime;
logGyro.data(:,1)=logGyro.data(:,1)-mutualStartTime;
logActions.data(:,1)=logActions.data(:,1)-mutualStartTime;
absEmgData(:,2:9)=abs(logEmg.data(:,2:9));
absEmgData(:,1)=logEmg.data(:,1);

%estimates the acceleration without gravity due a highpass
linAccel=[logAccel.data(:,1) zeros(size(logAccel.data,1), size(logAccel.data,2)-1)];
lp=[0 0 0];
for i=2:size(logAccel.data,1)
    [lp linAccel(i, 2:4)]=lowAndHighpass(18000, logAccel.data(i,1)-logAccel.data(i-1,1), logAccel.data(i, 2:4), lp);
end

mutualEndTime= min([logEmg.data(end,1), logOrient.data(end,1), logGyro.data(end,1), logAccel.data(end,1)]);
startIndexActions=knnsearch(logActions.data(:,1), 0);
startIndexEmg=knnsearch(logEmg.data(:,1), 0);
endIndexActions=knnsearch(logActions.data(:,1), mutualEndTime);
endIndexEmg=knnsearch(logEmg.data(:,1), mutualEndTime);

%sets a colorscheme, , MovingArm, ArmToGrasp, GrasptoDeposit,
%DepositToStop, Nullclass
colorScheme=['c'; 'g'; 'r'; 'm'; 'b']';
xyzm=['X','Y','Z','M'];
yawpitchroll={'Yaw','Pitch','Roll'};

%% time constants

timeBetweenEmgSamples=5000; %200Hz
timeBetweenIMUSamples=20000; %50 Hz
windowSize=50;

%% calculate FFT Features

%check for EMG6 (should be before grasp when worn right with usb to elbow)
% ->Enerrgy
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg6.mat'), 'file')~=0
    emg6Features=load(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg6.mat')) ;
else
    disp('Calculate EMG6 Features...')
    emg6Features.featuresOverTime=showFFTFeatures(1, 'Emg6', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,7), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end
%check for EMG5 (before grasp and when losing grasp)
% ->spectral Energy
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg5.mat'), 'file')~=0
    emg5Features=load(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg5.mat')) ;
else
    disp('Calculate EMG5 Features...')
    emg5Features.featuresOverTime=showFFTFeatures(1, 'Emg5', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,6), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end
%check for EMG7 (grasp strongest and holding strong)
% ->spectral Energy, cepstral1, expband doesnt matter, maybe energy best at
% grasptime, also time possible
%showFFTFeatures(rows, 'Emg7', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,8), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'))
%check for EMG8 (grasp strongest and holding strong)
% ->like emg7
%showFFTFeatures(rows, 'Emg8', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,9), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'))
%check for EMG1 (grasp strongest and holding strong)
% ->like emg7
%showFFTFeatures(rows, 'Emg1', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,2), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'))
  
%check for EMG4 (losing grasp)
% -> Energy
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg4.mat'), 'file')~=0
    emg4Features=load(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg4.mat')) ;
else
    disp('Calculate EMG4 Features...')
    emg4Features.featuresOverTime=showFFTFeatures(1, 'Emg4', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,5), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end


%check for EMG2 (holding)
%energy
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg2.mat'), 'file')~=0
    emg2Features=load(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg2.mat')) ;
else
    disp('Calculate EMG2 Features...')
    emg2Features.featuresOverTime=showFFTFeatures(1, 'Emg2', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,3), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end

%check for EMG3 (holding)
%energy
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg3.mat'), 'file')~=0
    emg3Features=load(strcat(name,'-',timestamp,'slidingWindowTresholds\Emg3.mat')) ;
else
    disp('Calculate EMG3 Features...')
    emg3Features.featuresOverTime=showFFTFeatures(1, 'Emg3', logEmg.data(:,1), [0 mutualEndTime], logEmg.data(:,4), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end

%check for AccelX (walking)
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\AccelX.mat'), 'file')~=0
    accelXFeatures=load(strcat(name,'-',timestamp,'slidingWindowTresholds\AccelX.mat')) ;
else
    disp('Calculate AccelX Features...')
    accelXFeatures.featuresOverTime=showFFTFeatures(1, 'AccelX', logAccel.data(:,1), [0 mutualEndTime], logAccel.data(:,2), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end

%check for AccelY (walking)
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\AccelY.mat'), 'file')~=0
    accelYFeatures=load(strcat(name,'-',timestamp,'slidingWindowTresholds\AccelY.mat')) ;
else
    disp('Calculate AccelY Features...')
    accelYFeatures.featuresOverTime=showFFTFeatures(1, 'AccelY', logAccel.data(:,1), [0 mutualEndTime], logAccel.data(:,3), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end

%check for AccelZ (walking)
if exist(strcat(name,'-',timestamp,'slidingWindowTresholds\AccelZ.mat'), 'file')~=0
    accelZFeatures=load(strcat(name,'-',timestamp,'slidingWindowTresholds\AccelZ.mat')) ;
else
    disp('Calculate AccelZ Features...')
    accelZFeatures.featuresOverTime=showFFTFeatures(1, 'AccelZ', logAccel.data(:,1), [0 mutualEndTime], logAccel.data(:,4), windowSize, name, strcat(timestamp, 'slidingWindowTresholds'));
end



%% pre-allocations    
%column1: time, column2 grasp detected, column3: before time of grasp, column4 grasp, column5 lose grasp, 
%1 probability all .22 seconds
probabilities=zeros(ceil(mutualEndTime/220000),5);
probabilities(:,1)=[0:220000:mutualEndTime+1]';
graspDetectionMatrix=zeros(size(probabilities,1),12);
graspDetectionMatrix(:,1)=probabilities(:,1);
restDetectionMatrix=zeros(size(probabilities,1),4);
restDetectionMatrix(:,1)=probabilities(:,1);
walkDetectionMatrix=zeros(size(probabilities,1),5);
walkDetectionMatrix(:,1)=probabilities(:,1);
detectionTimestamps=zeros(size(probabilities,1),1);
normalizationVector=zeros(1,size(graspDetectionMatrix,2));

% figure('Name','Sliding Window Treshold Test Single Detections Spectral Energy')
% subplot(5,1,1)
% ylabel('Emg4 lose')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,5))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,5))), size(logActions.data,1))
% subplot(5,1,2)
% ylabel('Emg5 before and lose')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,6))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,6))), size(logActions.data,1))
% subplot(5,1,3)
% ylabel('Emg6 before')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,7))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,7))), size(logActions.data,1))
% subplot(5,1,4)
% ylabel('Emg2 holding')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,3))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,3))), size(logActions.data,1))
% subplot(5,1,5)
% ylabel('Emg3 holding')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,4))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,4))), size(logActions.data,1))


%% get correlation of timestamps
%get the time of the middle Position between 2 probabilities to get the
%mean of the logEmg data around the probability position => sample from 1
%middlepoint between the probabilities to the next (Emgindex i, Emgindex i+1)
pToEmgIndex=knnsearch(logEmg.data(:,1),(probabilities(:,1)-(probabilities(2,1)/2)));
pToAccelIndex=knnsearch(logAccel.data(:,1),(probabilities(:,1)-(probabilities(2,1)/2)));
pToFFTAccelIndex=knnsearch(accelXFeatures.featuresOverTime(:,1),(probabilities(:,1)-(probabilities(2,1)/2)));
pToGyroIndex=knnsearch(logGyro.data(:,1),(probabilities(:,1)-(probabilities(2,1)/2)));
accelToProbIndex=knnsearch((probabilities(:,1)-(probabilities(2,1)/2)), accelXFeatures.featuresOverTime(:,1));
emgToProbIndex=knnsearch((probabilities(:,1)-(probabilities(2,1)/2)), emg6Features.featuresOverTime(:,1));

%% detection walking


%Detection for walking
for i=2:size(probabilities,1)-3;
    %check for walking
    %check for Accel X (holding)
    %avg energy in freqdomain not >35
    if mean(accelXFeatures.featuresOverTime(pToFFTAccelIndex(i):pToFFTAccelIndex(i+1),17)) <35
        walkDetectionMatrix(i,2)=1;
    end

    %check for Accel X (holding)
    %avg cepstral 1 in freqdomain not >-1.7
    if mean(accelXFeatures.featuresOverTime(pToFFTAccelIndex(i):pToFFTAccelIndex(i+1),6)) <-1.7
        walkDetectionMatrix(i,3)=1;
    end

    %check for Accel Y (holding)
    %avg entropy in freqdomain not >2.1
    if mean(accelYFeatures.featuresOverTime(pToFFTAccelIndex(i):pToFFTAccelIndex(i+1),16)) <2.1
        walkDetectionMatrix(i,4)=1;
    end
    
    %check for Accel Z (holding)
    %avg cepstral 1 in freqdomain not >-1.7
    if mean(accelZFeatures.featuresOverTime(pToFFTAccelIndex(i):pToFFTAccelIndex(i+1),6)) <-2
        walkDetectionMatrix(i,5)=1;
    end
end


%% detection for resting
for i=2:size(probabilities,1)-3;
    %check for linAccelX not resting (before and lose)
    %avg magnitude in timedomain <0.01
    if mean(abs(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),2))) >0.01
        restDetectionMatrix(i,2)=1;
    end

    %check for linAccelY not resting (before and lose)
    %avg magnitude in timedomain <0.01
    if mean(abs(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),3))) >0.01
        restDetectionMatrix(i,3)=1;
    end

    %check for linAccelZ not resting (before and lose)
    %avg magnitude in timedomain <0.01
    if mean(abs(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),4))) >0.01
        restDetectionMatrix(i,4)=1;
    end
end


%% detection for grasping with gyroscope
for i=2:size(probabilities,1)-3;
    %check for gyroX (before) 
    %max > 20 (and max < 100?) in timedomain NOT ABS because of right moving direction      
    if max(logGyro.data(pToGyroIndex(i):pToGyroIndex(i+1),2)) >14
        for j=i:i+2
            probabilities(j,2)=probabilities(j,2)+1;
            graspDetectionMatrix(j,13)=1;
        end
        probabilities(i,3)=probabilities(i,3)+1;
    end
end
    
%% detection for grasping with Acceleration
for i=2:size(probabilities,1)-3;
    %check for accelX+accelY (before and after) 
    %max > 0.25 in timedomain ... its absolute even without abs command   
    if ( max(logAccel.data(pToAccelIndex(i):pToAccelIndex(i+1),2))-min(logAccel.data(pToAccelIndex(i):pToAccelIndex(i+1),2)) + max(logAccel.data(pToAccelIndex(i):pToAccelIndex(i+1),3))-min(logAccel.data(pToAccelIndex(i):pToAccelIndex(i+1),3)) ) >0.2
        for j=i:i+2
            probabilities(j,2)=probabilities(j,2)+1;
            graspDetectionMatrix(j,14)=1;
        end
        probabilities(i,3)=probabilities(i,3)+1;
        probabilities(i,5)=probabilities(i,5)+1;
    end

    %check for linAccelX (before)
    %max > 0.04 in timedomain ... directed, no abs!  
    if max(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),2)) >0.04
        for j=i:i+2
            probabilities(j,2)=probabilities(j,2)+1;
            graspDetectionMatrix(j,15)=1;
        end
        probabilities(i,3)=probabilities(i,3)+1;
    end
end

%% detection grasping with EMG in fourier domain

%Detection for Grasp Spectral Energy based
for i=2:size(emg6Features.featuresOverTime,1)-2
    %all 0.25 seconds is a probability decision -> the probability before
    %and 0.5 seconds (5 after) should be +1 for grasp detected!
%     if emg6Features.featuresOverTime(i,17) > 800%1200
    if emg6Features.featuresOverTime(i,17) > normalizationVector(2)*1.5
        for j=i:i+2
            probabilities(emgToProbIndex(j),2)=probabilities(emgToProbIndex(j),2)+1;
            graspDetectionMatrix(emgToProbIndex(j),2)=1;
        end
        probabilities(emgToProbIndex(i),3)=probabilities(emgToProbIndex(i),3)+1;
%         subplot(5,1,3)      
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,7))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,7))), 'g')
    end
    normalizationVector(2)=(normalizationVector(2)*9+emg6Features.featuresOverTime(i,17))/10;
    
%     if emg5Features.featuresOverTime(i,17) > 800%1700
     if emg5Features.featuresOverTime(i,17) > normalizationVector(3)*1.5
        for j=i:i+2
            probabilities(emgToProbIndex(j),2)=probabilities(emgToProbIndex(j),2)+1;
            graspDetectionMatrix(emgToProbIndex(j),3)=1;
        end
        probabilities(emgToProbIndex(i),3)=probabilities(emgToProbIndex(i),3)+1;
        probabilities(emgToProbIndex(i),5)=probabilities(emgToProbIndex(i),5)+1;
%         subplot(5,1,2)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,6))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,6))), 'g')
    end
    normalizationVector(3)=(normalizationVector(3)*9+emg5Features.featuresOverTime(i,17))/10;
    
%     if emg4Features.featuresOverTime(i,17) >800% 1700
     if emg4Features.featuresOverTime(i,17) >normalizationVector(4)*1.5
        for j=i:i+2
            probabilities(emgToProbIndex(j),2)=probabilities(emgToProbIndex(j),2)+1;
            graspDetectionMatrix(emgToProbIndex(j),4)=1;
        end
        probabilities(emgToProbIndex(i),5)=probabilities(emgToProbIndex(i),5)+1;
%         subplot(5,1,1)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,5))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,5))), 'g')
     end
     normalizationVector(4)=(normalizationVector(4)*9+emg4Features.featuresOverTime(i,17))/10;
     
%     if emg2Features.featuresOverTime(i,17) >800% 1700
    if emg2Features.featuresOverTime(i,17) >normalizationVector(5)*1.5
        probabilities(emgToProbIndex(i),2)=probabilities(emgToProbIndex(i),2)+1;
        graspDetectionMatrix(emgToProbIndex(i),5)=1;
        probabilities(emgToProbIndex(i),4)=probabilities(emgToProbIndex(i),4)+1;
%         subplot(5,1,4)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,3))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,3))), 'g')
    end
    normalizationVector(5)=(normalizationVector(5)*9+emg2Features.featuresOverTime(i,17))/10;
    
%     if emg3Features.featuresOverTime(i,17) >350% 400
    if emg3Features.featuresOverTime(i,17) >normalizationVector(6)*1.5
        probabilities(emgToProbIndex(i),2)=probabilities(emgToProbIndex(i),2)+1;
        graspDetectionMatrix(emgToProbIndex(i),6)=1;
        probabilities(emgToProbIndex(i),4)=probabilities(emgToProbIndex(i),4)+1;
%         subplot(5,1,5)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,4))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,4))), 'g')
    end
    normalizationVector(6)=(normalizationVector(6)*9+emg3Features.featuresOverTime(i,17))/10;
end

% hgsave(strcat(name, '-', timestamp, 'slidingWindowTresholds', '\slidingWindowTestSingleDetections'))

% %Check Time domain Features
% figure('Name','Sliding Window Treshold Test Single Detections Time Domain based')
% subplot(6,1,1)
% ylabel('Emg1 holding')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,2))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,2))), size(logActions.data,1))
% subplot(6,1,2)
% ylabel('Emg4 holding')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,5))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,5))), size(logActions.data,1))
% subplot(6,1,3)
% ylabel('Emg5 before and after')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,6))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,6))), size(logActions.data,1))
% subplot(6,1,4)
% ylabel('Emg6 before and after')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,7))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,7))), size(logActions.data,1))
% subplot(6,1,5)
% ylabel('Emg7 holding')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,8))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,8))), size(logActions.data,1))
% subplot(6,1,6)
% ylabel('Emg8 holding')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,9))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,9))), size(logActions.data,1))


%% detect grasping with EMG in time domain

for i=2:size(probabilities,1)-3
    %check for resting
    %check for linAccelX not resting (before and lose)
    %avg magnitude in timedomain >0.01
    if mean(abs(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),2))) >0.01
        for j=i:i+2
            restDetectionMatrix(j,2)=1;
        end
    end

    %check for linAccelY not resting (before and lose)
    %avg magnitude in timedomain <0.01
    if mean(abs(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),3))) >0.01
        for j=i:i+2
            restDetectionMatrix(j,3)=1;
        end
    end

    %check for linAccelZ not resting (before and lose)
    %avg magnitude in timedomain <0.01
    if mean(abs(linAccel(pToAccelIndex(i):pToAccelIndex(i+1),4))) >0.01
        for j=i:i+2
            restDetectionMatrix(j,4)=1;
        end
    end
    
    
    %check for grasping
    %check for EMG1(holding)
    %avg magnitude in timedomain >10
%     if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),2)) >5
    if max(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),2)) > normalizationVector(7)*1.5  
        probabilities(i,2)=probabilities(i,2)+1;
        graspDetectionMatrix(i,7)=1;
        probabilities(i,4)=probabilities(i,4)+1;
%         subplot(6,1,1)      
%         stem(probabilities(i,1), max(abs(logEmg.data(:,2))), 'g')
%         stem(probabilities(i,1), -max(abs(logEmg.data(:,2))), 'g')
    end
    normalizationVector(7)=max(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),2));

    %check for EMG4 (holding)
    %avg magnitude in timedomain >10
%     if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),5)) >10
    if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),5)) >normalizationVector(8)*1.5
        probabilities(i,2)=probabilities(i,2)+1;
        graspDetectionMatrix(i,8)=1;
        probabilities(i,4)=probabilities(i,4)+1;
%         subplot(6,1,2)      
%         stem(probabilities(i,1), max(abs(logEmg.data(:,5))), 'g')
%         stem(probabilities(i,1), -max(abs(logEmg.data(:,5))), 'g')
    end
    normalizationVector(8)=(normalizationVector(8)*9+mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),5)))/10;  
    
    %check for EMG5 (before and lose)
    %avg magnitude in timedomain >10
%     if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),6)) >10
    if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),6)) >normalizationVector(9)*1.5
        for j=i:i+2
            probabilities(j,2)=probabilities(j,2)+1;
            graspDetectionMatrix(j,9)=1;
        end
        probabilities(i,3)=probabilities(i,3)+1;
        probabilities(i,5)=probabilities(i,5)+1;
%         subplot(6,1,3)      
%         stem(probabilities(i,1), max(abs(logEmg.data(:,6))), 'g')
%         stem(probabilities(i,1), -max(abs(logEmg.data(:,6))), 'g')
    end
    normalizationVector(9)=(normalizationVector(9)*9+mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),6)))/10;

    %check for EMG 6 (before and lose)
    %avg magnitude in timedomain >15
%     if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),7)) >15
    if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),7)) >normalizationVector(10)*1.5
        for j=i:i+2
            probabilities(j,2)=probabilities(j,2)+1;
            graspDetectionMatrix(j,10)=1;
        end
        probabilities(i,3)=probabilities(i,3)+1;
        probabilities(i,5)=probabilities(i,5)+1;
%         subplot(6,1,4)      
%         stem(probabilities(i,1), max(abs(logEmg.data(:,7))), 'g')
%         stem(probabilities(i,1), -max(abs(logEmg.data(:,7))), 'g')
    end
    normalizationVector(10)=(normalizationVector(10)*9+mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),7)))/10;

    %check for EMG7 (holding)
    %avg magnitude in timedomain >10
%     if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),8)) >10
    if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),8)) > normalizationVector(11)*1.5
        probabilities(i,2)=probabilities(i,2)+1;
        graspDetectionMatrix(i,11)=1;
        probabilities(i,4)=probabilities(i,4)+1;
%         subplot(6,1,5)      
%         stem(probabilities(i,1), max(abs(logEmg.data(:,8))), 'g')
%         stem(probabilities(i,1), -max(abs(logEmg.data(:,8))), 'g')
    end
    normalizationVector(11)=(normalizationVector(11)*9+mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),8)))/10;

    %check for EMG8 (holding)
    %avg magnitude in timedomain > 5
%     if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),9)) >5
    if mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),9)) > normalizationVector(12)*1.5
        probabilities(i,2)=probabilities(i,2)+1;
        graspDetectionMatrix(i,12)=1;
        probabilities(i,4)=probabilities(i,4)+1;
%         subplot(6,1,6)      
%         stem(probabilities(i,1), max(abs(logEmg.data(:,9))), 'g')
%         stem(probabilities(i,1), -max(abs(logEmg.data(:,9))), 'g')
    end
    normalizationVector(12)=(normalizationVector(12)*9+mean(absEmgData(pToEmgIndex(i):pToEmgIndex(i+1),9)))/10;
end




%% Evaluation
% figure('Name','Sliding Window Treshold Test Before Grasp')
% subplot(3,1,1)
% ylabel('Emg5')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,6))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,6))), size(logActions.data,1))
% subplot(3,1,2)
% ylabel('Emg6')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,7))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,7))), size(logActions.data,1))
% 
% for i=1:size(probabilities,1)
%     if probabilities(i,3)>=2
%         subplot(3,1,1)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,6))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,6))), 'g')
%         subplot(3,1,2)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,7))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,7))), 'g')
%     end
% end
% hgsave(strcat(name, '-', timestamp, 'slidingWindowTresholds', '\slidingWindowTestBeforeGrasp'))
% 
% figure('Name','Sliding Window Treshold Test lose Grasp')
% subplot(3,1,1)
% ylabel('Emg4')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,5))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,5))), size(logActions.data,1))
% subplot(3,1,2)
% ylabel('Emg5')
% hold on; grid on;
% plot(logEmg.data(:,1), logEmg.data(:,6))
% plotEventMarker(logActions.data(:,1), max(abs(logEmg.data(:,6))), size(logActions.data,1))
% 
% for i=1:size(probabilities,1)
%     if probabilities(i,3)>=2
%         subplot(3,1,1)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,5))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,5))), 'g')
%         subplot(3,1,2)
%         stem(probabilities(emgToProbIndex(i),1), max(abs(logEmg.data(:,6))), 'g')
%         stem(probabilities(emgToProbIndex(i),1), -max(abs(logEmg.data(:,6))), 'g')
%     end
% end
% hgsave(strcat(name, '-', timestamp, 'slidingWindowTresholds', '\slidingWindowTestLoseGrasp'))

Detections(:,1)=graspDetectionMatrix(:,1);
Detections(:,2)=sum(graspDetectionMatrix(:,2:end),2);
Detections=Detections(Detections(:,2)>=9,:);
DetectedFeatures=graspDetectionMatrix(ismember(graspDetectionMatrix(:,1), Detections(:,1)),:)

%Plot the detected grasps, EMG streams and action bars
figure('Name','Sliding Window Treshold Test Grasp Detected')
for i=1:8
    subplot(8,1,i)
    ylabel(strcat('Emg', num2str(i)))
    hold on; grid on;
    plot(logEmg.data(:,1), logEmg.data(:,i+1))
    plotEventMarker(logActions.data(:,:), max(abs(logEmg.data(:,i+1))), size(logActions.data,1))
end

for i=1:size(graspDetectionMatrix,1)
    if sum(graspDetectionMatrix(i,2:end))>=9 %Not all Features have to be detected!!!
        for j=1:8
            subplot(8,1,j)
            stem(graspDetectionMatrix(i,1), 200, 'g')%max(abs(logEmg.data(:,j+1)))
            stem(graspDetectionMatrix(i,1), -200, 'g')%-max(abs(logEmg.data(:,j+1)))
        end
    end
end
hgsave(strcat(name, '-', timestamp, 'slidingWindowTresholds', '\slidingWindowTestGraspDetected'))

% %Plot the detected walks, accel data and action bars
% figure('Name','Sliding Window Treshold Test Walk Detected')
% for i=1:3
%     subplot(3,1,i)
%     ylabel(strcat('Accel', num2str(i)))
%     hold on; grid on;
%     plot(logAccel.data(:,1), logAccel.data(:,i+1))
%     plotEventMarker(logActions.data(:,1), max(abs(logAccel.data(:,i+1))), size(logActions.data,1))
% end
% 
% for i=1:size(walkDetectionMatrix,1)
%     if sum(walkDetectionMatrix(i,2:end))==0
%         for j=1:3
%             subplot(3,1,j)
%             stem(walkDetectionMatrix(i,1), max(abs(logAccel.data(:,j+1))), 'g')
%             stem(walkDetectionMatrix(i,1), -max(abs(logAccel.data(:,j+1))), 'g')
%         end
%     end
% end
% hgsave(strcat(name, '-', timestamp, 'slidingWindowTresholds', '\slidingWindowTestWalkDetected'))
% 
% %Plot the detected rests, accel data and action bars
% figure('Name','Sliding Window Treshold Test Rest Detected')
% for i=1:3
%     subplot(3,1,i)
%     ylabel(strcat('Accel', num2str(i)))
%     hold on; grid on;
%     plot(logAccel.data(:,1), logAccel.data(:,i+1))
%     plotEventMarker(logActions.data(:,1), max(abs(logAccel.data(:,i+1))), size(logActions.data,1))
% end
% 
% for i=1:size(restDetectionMatrix,1)
%     if sum(restDetectionMatrix(i,2:end))==0
%         for j=1:3
%             subplot(3,1,j)
%             stem(restDetectionMatrix(i,1), max(abs(logAccel.data(:,j+1))), 'g')
%             stem(restDetectionMatrix(i,1), -max(abs(logAccel.data(:,j+1))), 'g')
%         end
%     end
% end
% hgsave(strcat(name, '-', timestamp, 'slidingWindowTresholds', '\slidingWindowTestRestDetected'))
%%
