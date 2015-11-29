%script to display myo in realtime
%author Marian Theiss
%Project Masterthesis
clear all; close all; clc;



myo_device_name='marian thesis';
rfduino_device_name='RFIDReader';
%rfduino_device_name='wrifd_m1'; Dont React on the wrifd answers








%%
%%Connect to serial port
%close all existing serial ports
snew=instrfind
if ~isempty(snew)
    fclose(snew)
end

%open the serial port
s=serial('COM4')
fopen(s);
s
get(s)

pause(1)


%%
%%set commands to myo


% lookup the first 8 chars of the device names
myo_device_name=myo_device_name(1:8);
rfduino_device_name=rfduino_device_name(1:8);
pause(0.1)
%end GAP discovery procedure call, else the dongle sends "1801: not in state for command"
fwrite(s,[0 0 6 4])
%start GAP discover procedure 
pause(0.1)
fwrite(s, [00 01 06 02 01])
pause(1)
addressMyo=[];
addressRFDuino=[];
read=[];
flushinput(s);
% lookup the address of the devices by Device names
while isempty(addressMyo) || isempty(addressRFDuino)
    %save GAP discovery
    read=strcat(read, fread(s)')
    %lookup Myo
    if isempty(addressMyo)
        idx=strfind(read,myo_device_name(1:8));
        if ~isempty(idx)
            if idx(1) >=12
                addressMyo=read(idx(1)-11:idx(1)-6)
                idx=[];
            end
        end
    else
        disp('Myo Found, waiting for RFDuino')
    end
    %lookup RFDuino
    if isempty(addressRFDuino) 
        idx=strfind(read,rfduino_device_name(1:8));
        if ~isempty(idx)
            if idx(1) >=12
                addressRFDuino=read(idx(1)-11:idx(1)-6)
                idx=[];
            end
        end
    else
        disp('RFDuino Found, waiting for Myo')
    end
end


%end GAP discovery procedure
fwrite(s,[0 0 6 4])
pause(0.1)

% maybe the names or adresses are upside down, careful! seems not


flushinput(s);
%Connect to RFID Reader
conRFDuino=[];
while isempty(conRFDuino)
    %connect to RFDuino
    fwrite(s,[00 15 06 03 addressRFDuino(1) addressRFDuino(2) addressRFDuino(3) addressRFDuino(4) addressRFDuino(5) addressRFDuino(6) 01 06 00 06 00 64 00 00 00])
    pause(0.1)
    %check connections to bluetooth and sets the number of the connected RFDuino
    fwrite(s, [00 00 00 06])
    pause(0.1)
    conRFDuino=fread(s)';
    
    %conRFDuino=conRFDuino(find(conRFDuino==229,1,'first')-2)
    conRFDuino=double(conRFDuino(strfind(conRFDuino,addressRFDuino)-2));
    if ~isempty(conRFDuino)
        conRFDuino=conRFDuino(1,1) %in case of 2 returned values of strfind
    end
end

pause(0.1)
%end GAP discovery procedure call, else the dongle sends "1801: not in state for command"
fwrite(s,[0 0 6 4])

pause(0.1)
%Connect to MYO
conMyo=[];
while isempty(conMyo)
    %connect to Myo
    fwrite(s,[00 15 06 03 107 235 93 19 54 231 00 06 00 06 00 64 00 00 00])
    pause(0.1)
    %check connections to bluetooth and sets the number of the connected myo
    fwrite(s, [00 00 00 06])
    pause(0.1)
    conMyo=fread(s)';
    
    %conMyo=conMyo(find(conMyo==107,1,'first')-2)
    %conMyo=conMyo(find(conMyo==[107 235 93 19 54 231],1,'first')-2)
    conMyo=double(conMyo(strfind(conMyo,addressMyo)-2));
    if ~isempty(conMyo)
        conMyo=conMyo(1,1) %in case of 2 returned values of strfind
    end
end



 pause(0.2)

 %tell Myo to send  filtered EMG and IMU data - no classification
fwrite(s, [00 09 04 05 conMyo 25 00 05 01 03 02 01 00])

pause(0.1)
 
 %Don’t let myo sleep, should be in code directly after connection established :
fwrite(s, [00 07 04 05 conMyo 25 00 03 09 00 01])

%Let Myo sleep when not used -> leads to disconnects when not moving:
%fwrite(s, [00 07 04 05 con 25 00 03 09 00 00])

pause(0.1)
%Check connection - short vibration
fwrite(s, [00 07 04 05 conMyo 25 00 03 03 01 01])

pause(0.1)
 
 %unknown
 %fwrite(s, [00 06 04 05 conMyo 18 00 02 01 00])
 
 
pause(0.2)
 
        %    fwrite(s, [00 05 04 05 conRFDuino 17 00 01 00]);
%subscribe RFduino notifications
% fwrite(s, [00 03 04 05 conRFDuino 01 00])
% pause(0.2)
% fwrite(s, [00 03 04 05 conRFDuino 00 01])
% pause(0.2)
% fwrite(s, [00 05 04 05 conRFDuino 00 17 01 00])
% pause(0.2)

% %subscribe RFduino indications
% fwrite(s, [00 03 04 05 conRFDuino 02 00])
% pause(0.2)
% 
% %subscribe RFduino notifications and indications
% fwrite(s, [00 03 04 05 conRFDuino 03 00])
% pause(0.2)
fwrite(s, [00 06 04 05 conRFDuino 15 00 02 03 00])
pause(0.2)
% fwrite(s, [00 05 04 05 conRFDuino 17 00 00 03])
% pause(0.2)
%fwrite(s, [00 03 04 05 conRFDuino 00 03])
%pause(0.2)
 
 
pause(0.2)
 
%subscribe imu data
fwrite(s, [00 06 04 05 conMyo 29 00 02 01 00])

pause(0.2)
% 
% %unknown, maybe gestures or arminformations or connectioninfos
% fwrite(s, [00 06 04 05 conMyo 18 00 02 01 00])



%subscribe emg data
fwrite(s, [00 06 04 05 conMyo 44 00 02 01 00])

pause(0.2) %pause commands needed here, else myo doesn't accept the commands!

%subscribe emg data
fwrite(s, [00 06 04 05 conMyo 47 00 02 01 00])

pause(0.2)

%subscribe emg data
fwrite(s, [00 06 04 05 conMyo 50 00 02 01 00])

pause(0.2)

%subscribe emg data
fwrite(s, [00 06 04 05 conMyo 53 00 02 01 00])

pause(0.2)

%set frequency of sensors???
%fwrite(s, [00 16 04 05 conMyo 25 00 12 02 10 02 01 01 200 00 100 20 50 00 00])
%fwrite(s, [00 16 04 05 conMyo 25 00 12 02 10 02 01 01 200 00 100 10 50 00 00])

%pause(0.1)

% %unknown, maybe gestures or arminformations or connectioninfos
% fwrite(s, [00 06 04 05 conMyo 36 00 02 01 00])
%2a 2d 30 33
%12 2c 2f 32 35 1d 24 

% % % % % % %enable emg data stream???
% % % % % % fwrite(s, [00 06 04 05 conMyo 40 00 02 01 00])
% % % % % % %fwrite(s, [00 06 04 05 conMyo 40 00 02 02 00])

pause(0.2)

%unknown
%fwrite(s, [00 06 04 05 conMyo 36 00 02 02 00])
pause(0.2)


fread(s);

%unknown
%fwrite(s, [00 03 04 04 00 17 00]) or 00 03 04 04 conMyo 17 00

    
%%
%%Parsing Preallocations and definitions
%IMU and EMG data per packet
imu=zeros(1,11); %timestamp, orientation (w,x,y,z), accel(x,y,z), gyro(x,y,z)
emg=zeros(1,17); %timestamp, emg samples old 1-8, emg samples new 1-8


%preallocate data for 1h...
%timestamp (approximatly/rough), orient w, orient x, orient y, orient z,
%accel x, accel y, accel z, gyro x, gyro y, gyro z, yaw, pitch, roll,
%linAccelX, linAccelY, linAccelZ
imuDataset=zeros(50*60*60,17); %46 Hz
%timestamp (approximatly/rough), emg 1-8
emgDataset=zeros(200*60*60,9); %182 Hz
emgHighpassDataset=zeros(200*60*60,9); %182 Hz
emgLowpassDataset=zeros(200*60*60,9); %182 Hz
%abs(emg)
emgAbsDataset=zeros(200*60*60,9); %182 Hz

imuCounter=0;
emgCounter=1;
readCommandCounter=0;
packet=[];
graspDetected=false;
debugrunningloopCounter=0;
detectedGraspCounter=2;
debugpacketCounter=0;
emgCollectedToCheck=-160; %let it run a little bit before first grasp detection
imuCollectedToCheck=-40;
graspChecksPerformed=10; %in case we want to check in the first run for the run before
lpAccel=[0 0 0]; 


%%
%%Grasp Detection Predefinitions

%column1: time, column2 grasp detected, column3: before time of grasp, column4 grasp, column5 lose grasp, 
%1 probability all .2 seconds
probabilities=zeros(size(imuDataset,1),5);
probabilities(:,1)=[0:200000:size(imuDataset,1)]';
graspDetectionMatrix=zeros(size(probabilities,1),25);
graspDetectionMatrix(:,1)=probabilities(:,1);
restDetectionMatrix=zeros(size(probabilities,1),4);
restDetectionMatrix(:,1)=probabilities(:,1);
walkDetectionMatrix=zeros(size(probabilities,1),5);
walkDetectionMatrix(:,1)=probabilities(:,1);
detectionTimestamps=zeros(size(probabilities,1),1);
emg6Features=zeros(size(imuDataset,1),20);
emg5Features=zeros(size(imuDataset,1),20);
emg4Features=zeros(size(imuDataset,1),20);
emg2Features=zeros(size(imuDataset,1),20);
emg3Features=zeros(size(imuDataset,1),20);
gyroXFeatures=zeros(size(imuDataset,1),20);
gyroYFeatures=zeros(size(imuDataset,1),20);
gyroZFeatures=zeros(size(imuDataset,1),20);
AccelXFeatures=zeros(size(imuDataset,1),20);
AccelYFeatures=zeros(size(imuDataset,1),20);
AccelZFeatures=zeros(size(imuDataset,1),20);
normalizationVector=zeros(1,size(graspDetectionMatrix,2));
objectDetectionMatrix=zeros(65535,10);
objectDetectionMatrix(:,1)=1:size(objectDetectionMatrix);


pause(2)

%%
%%running loop
tic
while true
    debugrunningloopCounter=debugrunningloopCounter+1;

    %Check for new Packet
    read=fread(s)'; 
    packetstarts=find(read==128);
    packetstartCounter=0;

    while true
        if packetstartCounter>=length(packetstarts)  %EVTL HIER -1
            packet=read(packetstarts(packetstartCounter):end);
            break;
        end
        debugpacketCounter=debugpacketCounter+1;
        %divide bluetoothreading into packets
        if packetstartCounter==0 
            packet(end+(1:(packetstarts(1)-1)))=read(1:(packetstarts(1)-1));
        else  
            packet=read(packetstarts(packetstartCounter):packetstarts(packetstartCounter+1)-1);
        end
        packetstartCounter=packetstartCounter+1;
        
        
        if length(packet) >2
            if packet(1)==128
                while length(packet) < (4+(packet(2))) && packetstartCounter <= length(packetstarts)-1
                    %disp(num2str(packet));
                    packet(end+(1:(packetstarts(packetstartCounter+1)-packetstarts(packetstartCounter))))=read(packetstarts(packetstartCounter):(packetstarts(packetstartCounter+1)-1));
                    %disp(num2str(packet));
                    packetstartCounter=packetstartCounter+1;  
                end
                if length(packet) < (4+(packet(2))) && packetstartCounter >= length(packetstarts)
                    %disp(num2str(packet))
                    packet(end+(1:length(read)-packetstarts(packetstartCounter)+1))=read(packetstarts(packetstartCounter):end);
                    %disp(num2str(packet))
                    disp('test2')
                    break;
                end
                if length(packet) <6
                    length(packet)
                    %disp(strcat('unknown packet type: ', num2str(packet)))
                    packet=[];
                    continue;
                end
            else
                packetstartCounter=packetstartCounter+1;
                disp(strcat('wrong packetstart', num2str(packet)))
                continue;
            end
        else
            packet(end+(1:(packetstarts(packetstartCounter+1)-packetstarts(packetstartCounter))))=read(packetstarts(packetstartCounter):(packetstarts(packetstartCounter+1)-1));
            packetstartCounter=packetstartCounter+1; 
            continue;
        end
        
        %Parse the Bluetooth data
        if packet(6) == 28 % IMU
            %conversion from bitstream (rMSB int16) to matlab number
            %sets 2 0...255 bytes to a -32768...32767 number (int16),
            %right MSB and convert it back to double (standard in matlab), than divide it by
            %the Myo scale
            imuCounter=imuCounter+1;
            
%IMU array: timestamp (approximatly/rough), orient w, orient x, orient y, orient z, accel x, accel y, accel z, gyro x, gyro y, gyro z, yaw, pitch, roll, linAccelX, linAccelY, linAccelZ
            imuDataset(imuCounter,1:11)=[imuCounter*20000 double(typecast([uint8(packet(10)), uint8(packet(11))],'int16'))/16384 double(typecast([uint8(packet(12)), uint8(packet(13))],'int16'))/16384 double(typecast([uint8(packet(14)), uint8(packet(15))],'int16'))/16384 double(typecast([uint8(packet(16)), uint8(packet(17))],'int16'))/16384 double(typecast([uint8(packet(18)), uint8(packet(19))],'int16'))/2048 double(typecast([uint8(packet(20)), uint8(packet(21))],'int16'))/2048 double(typecast([uint8(packet(22)), uint8(packet(23))],'int16'))/2048 double(typecast([uint8(packet(24)), uint8(packet(25))],'int16'))/16 double(typecast([uint8(packet(26)), uint8(packet(27))],'int16'))/16 double(typecast([uint8(packet(28)), uint8(packet(29))],'int16'))/16];
            [imuDataset(imuCounter,12) imuDataset(imuCounter,13) imuDataset(imuCounter,14)]= quat2angle([imuDataset(imuCounter,2), imuDataset(imuCounter,3), imuDataset(imuCounter,4), imuDataset(imuCounter,5)]);
            [lpAccel(:), imuDataset(imuCounter,15:17)]=lowAndHighpass(0.18, 0.022, imuDataset(imuCounter,6:8), lpAccel);
            imuCollectedToCheck=imuCollectedToCheck+1;

        elseif packet(6) == 43 || packet(6) == 52 || packet(6) == 46 || packet(6)==49 %EMG
            %2 emg samples will be send per packet. int8 format. Not
            %sure whiich sample is newer and which older, name was just
            %given random
            emgCounter=emgCounter+2; %if 2 emg samples are send +2
            emgDataset(emgCounter,:)=[emgCounter*5000 double(typecast(uint8(packet(10)), 'int8')), double(typecast(uint8(packet(11)), 'int8')), double(typecast(uint8(packet(12)), 'int8')), double(typecast(uint8(packet(13)), 'int8')), double(typecast(uint8(packet(14)), 'int8')), double(typecast(uint8(packet(15)), 'int8')), double(typecast(uint8(packet(16)), 'int8')), double(typecast(uint8(packet(17)), 'int8'))];
            emgDataset(emgCounter+1,:)=[(emgCounter+1)*5000  double(typecast(uint8(packet(18)), 'int8')), double(typecast(uint8(packet(19)), 'int8')), double(typecast(uint8(packet(20)), 'int8')), double(typecast(uint8(packet(21)), 'int8')), double(typecast(uint8(packet(22)), 'int8')), double(typecast(uint8(packet(23)), 'int8')), double(typecast(uint8(packet(24)), 'int8')), double(typecast(uint8(packet(25)), 'int8'))];
            emgHighpassDataset(emgCounter:emgCounter+1,1)=emgDataset(emgCounter:emgCounter+1, 1);
            emgLowpassDataset(emgCounter:emgCounter+1,1)=emgDataset(emgCounter:emgCounter+1, 1);
            [emgLowpassDataset(emgCounter-1, 2:8), emgHighpassDataset(emgCounter,2:8)]=lowAndHighpass(0.18, 0.022, emgDataset(emgCounter, 2:8), emgLowpassDataset(emgCounter-2, 2:8));
            [emgLowpassDataset(emgCounter, 2:8), emgHighpassDataset(emgCounter+1,2:8)]=lowAndHighpass(0.18, 0.022, emgDataset(emgCounter+1, 2:8), emgLowpassDataset(emgCounter-1, 2:8));
            emgAbsDataset(emgCounter:emgCounter+1,:)=abs(emgDataset(emgCounter:emgCounter+1,:));
            emgCollectedToCheck=emgCollectedToCheck+2;
        
        elseif packet(6)==14 %RFduino
            %get ID of reader request
            readerRequestID=typecast(uint8([packet(10) packet(11)]), 'uint16');
            %set the objectDetectionMatrix: timestamp of the receive of the reader packet
            objectDetectionMatrix(readerRequestID, 3)=imuDataset(imuCounter,1);
            %if tag found
            if length(packet)>=20
                %set the objectDetectionMatrix: tag
                beep
                objectDetectionMatrix(readerRequestID, 4:10)=[packet(14) packet(15) packet(16) packet(17) packet(18) packet(19) packet(20)];
            end
            %disp(strcat('RFduino reader packet: ', num2str(packet)))
            disp(strcat('ID: ', num2str(readerRequestID), ' Tag: ', num2str(objectDetectionMatrix(readerRequestID, 4:10)), ' timestampReceive-timestampSend: ', num2str(objectDetectionMatrix(readerRequestID, 3)- objectDetectionMatrix(readerRequestID, 2))))
        end

        
        packet=[];
    
    
        %check for grasp, 
    
        %Go in every 10 imu samples/40emg samples = 0.2 seconds
        if imuCollectedToCheck >=10 && emgCollectedToCheck >=40

            %Get the Fourier Features
            emg6Features(graspChecksPerformed,2:20)=calculateFeaturesFFT(emgDataset(emgCounter-49:emgCounter,7));
            emg5Features(graspChecksPerformed,2:20)=calculateFeaturesFFT(emgDataset(emgCounter-49:emgCounter,6));
            emg4Features(graspChecksPerformed,2:20)=calculateFeaturesFFT(emgDataset(emgCounter-49:emgCounter,5));
            emg2Features(graspChecksPerformed,2:20)=calculateFeaturesFFT(emgDataset(emgCounter-49:emgCounter,3));
            emg3Features(graspChecksPerformed,2:20)=calculateFeaturesFFT(emgDataset(emgCounter-49:emgCounter,4));
            gyroXFeatures(graspChecksPerformed,2:20)=calculateFeaturesFFT(imuDataset(imuCounter-49:imuCounter,9));
            gyroYFeatures(graspChecksPerformed,2:20)=calculateFeaturesFFT(imuDataset(imuCounter-49:imuCounter,10));
            gyroZFeatures(graspChecksPerformed,2:20)=calculateFeaturesFFT(imuDataset(imuCounter-49:imuCounter,11));
            AccelXFeatures(graspChecksPerformed, 2:20)=calculateFeaturesFFT(imuDataset(imuCounter-49:imuCounter,6));
            AccelYFeatures(graspChecksPerformed, 2:20)=calculateFeaturesFFT(imuDataset(imuCounter-49:imuCounter,7));
            AccelZFeatures(graspChecksPerformed, 2:20)=calculateFeaturesFFT(imuDataset(imuCounter-49:imuCounter,8));

            drawnow

            %check for resting
            %check for linAccelX not resting (before and lose)
            %avg magnitude in timedomain >0.01
            if mean(abs(imuDataset(imuCounter-10:imuCounter,15))) >0.01
                %for j=graspChecksPerformed:graspChecksPerformed+2
                    restDetectionMatrix(graspChecksPerformed,2)=1;
                %end
            end

            %check for linAccelY not resting (before and lose)
            %avg magnitude in timedomain <0.01
            if mean(abs(imuDataset(imuCounter-10:imuCounter,16))) >0.01
                %for j=graspChecksPerformed:graspChecksPerformed+2
                    restDetectionMatrix(graspChecksPerformed,3)=1;
                %end
            end

            %check for linAccelZ not resting (before and lose)
            %avg magnitude in timedomain <0.01
            if mean(abs(imuDataset(imuCounter-10:imuCounter,17))) >0.01
              %  for j=graspChecksPerformed:graspChecksPerformed+2
                    restDetectionMatrix(graspChecksPerformed,4)=1;
               % end
            end

            %check for walking
            %check for Accel X (holding)
            %avg energy in freqdomain not >35
            if mean(AccelXFeatures(graspChecksPerformed-9:graspChecksPerformed,17)) <35
                walkDetectionMatrix(graspChecksPerformed,2)=1;
            end

            %check for Accel X (holding)
            %avg cepstral 1 in freqdomain not >-1.7
            if mean(AccelXFeatures(graspChecksPerformed-9:graspChecksPerformed,6)) <-1.7
                walkDetectionMatrix(graspChecksPerformed,3)=1;
            end

            %check for Accel Y (holding)
            %avg entropy in freqdomain not >2.1
            if mean(AccelYFeatures(graspChecksPerformed-9:graspChecksPerformed,16)) <2.1
                walkDetectionMatrix(graspChecksPerformed,4)=1;
            end

            %check for Accel Z (holding)
            %avg cepstral 1 in freqdomain not >-1.7
            if mean(AccelZFeatures(graspChecksPerformed-9:graspChecksPerformed,6)) <-2
                walkDetectionMatrix(graspChecksPerformed,5)=1;
            end






            %check for grasping
            %check for gyroX (before) 
            %max > 20 (and max < 100?) in timedomain NOT ABS because of right moving direction      
            if max(imuDataset(imuCounter-10:imuCounter,9)) >14
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,13)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
            end


            %check for accelX+accelY (before and after) 
            %max > 0.25 in timedomain ... its absolute even without abs command   
            if ( max(imuDataset(imuCounter-9:imuCounter,6))-min(imuDataset(imuCounter-9:imuCounter,6)) + max(imuDataset(imuCounter-9:imuCounter,7))-min(imuDataset(imuCounter-9:imuCounter,7)) ) >0.2
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,14)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
                probabilities(graspChecksPerformed,5)=probabilities(graspChecksPerformed,5)+1;
            end

            %check for linAccelX (before)
            %max > 0.04 in timedomain ... directed, no abs!  
            if max(imuDataset(imuCounter-9:imuCounter,15)) >0.04
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,15)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
            end





            %all 0.22 seconds is a probability decision -> the probability before
            %and 0.5 seconds (5 after) should be +1 for grasp detected!
    %         if emg6Features(graspChecksPerformed,17) > 800%1200
            if emg6Features(graspChecksPerformed,17) > normalizationVector(2)*1.5
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,2)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
            end
            normalizationVector(2)=(normalizationVector(2)*9+emg6Features(graspChecksPerformed,17))/10;

    %         if emg5Features(graspChecksPerformed,17) > 800%1700
            if emg5Features(graspChecksPerformed,17) > normalizationVector(3)*1.5
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,3)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
                probabilities(graspChecksPerformed,5)=probabilities(graspChecksPerformed,5)+1;
            end
            normalizationVector(3)=(normalizationVector(3)*9+emg5Features(graspChecksPerformed,17))/10;

    %         if emg4Features(graspChecksPerformed,17) >800% 1700
            if emg4Features(graspChecksPerformed,17) >normalizationVector(4)*1.5
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,4)=1;
                end
                probabilities(graspChecksPerformed,5)=probabilities(graspChecksPerformed,5)+1;
             end
            normalizationVector(4)=(normalizationVector(4)*9+emg4Features(graspChecksPerformed,17))/10;

    %         if emg2Features(graspChecksPerformed,17) >800% 1700
            if emg2Features(graspChecksPerformed,17) >normalizationVector(5)*1.5
                probabilities(graspChecksPerformed,2)=probabilities(graspChecksPerformed,2)+1;
                graspDetectionMatrix(graspChecksPerformed,5)=1;
                probabilities(graspChecksPerformed,4)=probabilities(graspChecksPerformed,4)+1;
            end
            normalizationVector(5)=(normalizationVector(5)*9+emg2Features(graspChecksPerformed,17))/10;


    %         if emg3Features(graspChecksPerformed,17) >350% 400
            if emg3Features(graspChecksPerformed,17) >normalizationVector(6)*1.5
                probabilities(graspChecksPerformed,2)=probabilities(graspChecksPerformed,2)+1;
                graspDetectionMatrix(graspChecksPerformed,6)=1;
                probabilities(graspChecksPerformed,4)=probabilities(graspChecksPerformed,4)+1;
            end
            normalizationVector(6)=(normalizationVector(6)*9+emg3Features(graspChecksPerformed,17))/10;




            %check the time domain features

            %check for EMG1(holding)    PRETTY BAD FEATURE!!!! MAYBE
            %DELETE????????? Should be at grasp but maybe works much better
            %before grasp... so before and after?
            %%%%%%%%%%%%%%%%%%avg magnitude in timedomain >10
            %max > 5
    %         if max(emgAbsDataset(emgCounter-40:emgCounter,2)) >5
            if max(emgAbsDataset(emgCounter-40:emgCounter,2)) > normalizationVector(7)*1.5
    %             probabilities(graspChecksPerformed,2)=probabilities(graspChecksPerformed,2)+1;
    %             graspDetectionMatrix(graspChecksPerformed,7)=1;
    %             probabilities(graspChecksPerformed,4)=probabilities(graspChecksPerformed,4)+1;
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,7)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
                probabilities(graspChecksPerformed,5)=probabilities(graspChecksPerformed,5)+1;
            end
            %normalizationVector(7)=(normalizationVector(7)+max(emgAbsDataset(emgCounter-40:emgCounter,2))*3)/4;%normalizationvector calculation differ!
            normalizationVector(7)=max(emgAbsDataset(emgCounter-40:emgCounter,2));

            %check for EMG4 (holding)
            %avg magnitude in timedomain >10
    %         if mean(emgAbsDataset(emgCounter-40:emgCounter,5)) >10
            if mean(emgAbsDataset(emgCounter-40:emgCounter,5)) >normalizationVector(8)*1.5
                probabilities(graspChecksPerformed,2)=probabilities(graspChecksPerformed,2)+1;
                graspDetectionMatrix(graspChecksPerformed,8)=1;
                probabilities(graspChecksPerformed,4)=probabilities(graspChecksPerformed,4)+1;
            end
            normalizationVector(8)=(normalizationVector(8)*9+mean(emgAbsDataset(emgCounter-40:emgCounter,5)))/10;

            %check for EMG5 (before and lose)
            %avg magnitude in timedomain >10
    %         if mean(emgAbsDataset(emgCounter-40:emgCounter,6)) >10
            if mean(emgAbsDataset(emgCounter-40:emgCounter,6)) >normalizationVector(9)*1.5
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,9)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
                probabilities(graspChecksPerformed,5)=probabilities(graspChecksPerformed,5)+1;
            end
            normalizationVector(9)=(normalizationVector(9)*9+mean(emgAbsDataset(emgCounter-40:emgCounter,6)))/10;

            %check for EMG 6 (before and lose)
            %avg magnitude in timedomain >15

    %         if mean(emgAbsDataset(emgCounter-40:emgCounter,7)) >15
            if mean(emgAbsDataset(emgCounter-40:emgCounter,7)) >normalizationVector(10)*1.5
                for j=graspChecksPerformed:graspChecksPerformed+2
                    probabilities(j,2)=probabilities(j,2)+1;
                    graspDetectionMatrix(j,10)=1;
                end
                probabilities(graspChecksPerformed,3)=probabilities(graspChecksPerformed,3)+1;
                probabilities(graspChecksPerformed,5)=probabilities(graspChecksPerformed,5)+1;
            end
            normalizationVector(10)=(normalizationVector(10)*9+mean(emgAbsDataset(emgCounter-40:emgCounter,7)))/10;

            %check for EMG7 (holding)
            %avg magnitude in timedomain >10
    %         if mean(emgAbsDataset(emgCounter-40:emgCounter,8)) >10
            if mean(emgAbsDataset(emgCounter-40:emgCounter,8)) > normalizationVector(11)*1.5
                probabilities(graspChecksPerformed,2)=probabilities(graspChecksPerformed,2)+1;
                graspDetectionMatrix(graspChecksPerformed,11)=1;
                probabilities(graspChecksPerformed,4)=probabilities(graspChecksPerformed,4)+1;
            end
            normalizationVector(11)=(normalizationVector(11)*9+mean(emgAbsDataset(emgCounter-40:emgCounter,8)))/10;

            %check for EMG8 (holding)
            %avg magnitude in timedomain > 5
    %         if mean(emgAbsDataset(emgCounter-40:emgCounter,9)) >5
            if mean(emgAbsDataset(emgCounter-40:emgCounter,9)) > normalizationVector(12)*1.5
                probabilities(graspChecksPerformed,2)=probabilities(graspChecksPerformed,2)+1;
                graspDetectionMatrix(graspChecksPerformed,12)=1;
                probabilities(graspChecksPerformed,4)=probabilities(graspChecksPerformed,4)+1;
            end
            normalizationVector(12)=(normalizationVector(12)*9+mean(emgAbsDataset(emgCounter-40:emgCounter,9)))/10;







            %in case a grasp is detected send the number of detected grasp to object reader
            if sum(graspDetectionMatrix(graspChecksPerformed,2:15),2)>=9 && sum(walkDetectionMatrix(graspChecksPerformed,2:5),2)>=0 && sum(restDetectionMatrix(graspChecksPerformed,2:4),2)>=3 % to disable after a detection for a while: && imuDataset(imuCounter,1)-detectionTimestamps(detectedGraspCounter)>1000000

                     %%careful with sending 2 commands without pause
                     readCommandCounter=readCommandCounter+1;
                     readCommandCounterBytes=typecast(uint16(readCommandCounter), 'uint8');
                    fwrite(s, [00 06 04 05 conRFDuino 17 00 02 readCommandCounterBytes(1) readCommandCounterBytes(2)]);
                    detectionTimestamps(detectedGraspCounter+1)=imuDataset(imuCounter,1);
                    detectedGraspCounter=detectedGraspCounter+1;
                    objectDetectionMatrix(readCommandCounter, 2)=imuDataset(imuCounter,1);
                %end
            %else
            %    fwrite(s, [00 05 04 05 conRFDuino 17 00 01 00]);
            end


            %show graspDetectionMatrix
            disp(strcat('Features Detected: ', num2str(sum(graspDetectionMatrix(graspChecksPerformed,2:end),2)), ' Features: ', num2str(graspDetectionMatrix(graspChecksPerformed,2:15))))
            %disp(strcat('walk Detected: ', num2str(sum(walkDetectionMatrix(graspChecksPerformed,2:end),2)), ' Features: ', num2str(walkDetectionMatrix(graspChecksPerformed,2:5))))

            imuCollectedToCheck=0;
            emgCollectedToCheck=0;
            graspChecksPerformed=graspChecksPerformed+1;

        end

    end
    
    
    %debug timestamps
    %disp(strcat(' timediff imu: ', num2str(toc-imuDataset(imuCounter-1,1)/1000000), ' timediff emg: ', num2str(toc-emgDataset(emgCounter-1,1)/1000000 ), 'toc: ', num2str(toc), ' imu: ', num2str(imuDataset(imuCounter-1,1)/1000000), ' emg: ', num2str(emgDataset(emgCounter-1,1)/1000000)))
end

%%







%old code
% %break if a packet doesn't start with 128
%         if packet(1)~=128 
%             packetstartCounter=packetstartCounter+1;
%             disp(strcat('wrong packetstart', num2str(packet)))
%             continue;
%         end
%          if length(packet)<2
%              packet(end+(1:(packetstarts(packetstartCounter+1)-packetstarts(packetstartCounter))))=read(packetstarts(packetstartCounter):(packetstarts(packetstartCounter+1)-1));
%              packetstartCounter=packetstartCounter+1; 
%          end
%         %if a 128, which usually shows a new packet is inside a packet
%         %concatenate the next packet to it.
%         while length(packet)< (4 + packet(2))  && packetstartCounter >=length(packetstarts)-1
%             
%             %disp(num2str(packet));
%             packet(end+(1:(packetstarts(packetstartCounter+1)-packetstarts(packetstartCounter))))=read(packetstarts(packetstartCounter):(packetstarts(packetstartCounter+1)-1));
%             %disp(num2str(packet));
%             packetstartCounter=packetstartCounter+1;    
%         end
%         
%         %if not full packet inside 1 reading go to next reading and start
%         %in next running loop
%         if length(packet)< (4 + packet(2))  && packetstartCounter>=length(packetstarts)
%             %disp(num2str(packet))
%             packet(end+(1:length(read)-packetstarts(packetstartCounter)+1))=read(packetstarts(packetstartCounter):end);
%             %disp(num2str(packet))
%             disp('test2')
%             break;
%         end
% 
%         if length(packet) < 6
%             length(packet)
%             disp(strcat('unknown packet type: ', num2str(packet)))
%             packet=[];
%             continue;
%         end








%close all existing ports:
%snew=instrfind
%close(snew)

%close s:
%fclose(s)








        
        
        
%         Detections(:,1)=graspDetectionMatrix(:,1);
%         Detections(:,2)=sum(graspDetectionMatrix(:,2:12),2);
%         Detections=Detections(Detections(:,2)>=7,:);
%         DetectedFeatures=graspDetectionMatrix(ismember(graspDetectionMatrix(:,1), Detections(:,1)),:)



















%     if abs(toc-imuDataset(imuCounter,1))>1000000
%         disp('timestamp imu just too wrong!')
%     end
%     if abs(toc-emgDataset(emgCounter,1))>1000000
%         disp('timestamp emg just too wrong!')
%     end

    %I think this keeps the myo connected... has to be checked.
    %fwrite(s, [00 01 03 01 conMyo]);
    
    
    
    
    
    
%         if sum(walkDetectionMatrix(graspChecksPerformed,2:5),2)==0 && imuDataset(imuCounter,1)-detectionTimestamps(detectedGraspCounter)>2000000
%             %if sum(graspDetectionMatrix(graspChecksPerformed-2:graspChecksPerformed-1,14:23),2)<3
%                 fwrite(s, [00 07 04 05 conMyo 25 00 03 03 01 01]);
%                 detectionTimestamps(detectedGraspCounter+1)=imuDataset(imuCounter,1);
%                 detectedGraspCounter=detectedGraspCounter+1;
%             %end
%         end
    
    









% % packets=zeros(20000, 50);%debug
% % read = 0    %debug
% %    packetsNr=1;        %debug                                           %debug
% %    
% %    
% %    
% %    
% %     readOld=read;                                               %debug



%             disp (num2str(packetOld))%debug
%             disp (num2str(packet))%debug
%             disp (num2str(readOld))%debug
%             disp (num2str(read))%debug
%            length(packetstarts)%debug
%            packetstartCounter%debug
%             packetsNr%debug

    
%         
%         packets(packetsNr,1:length(packet))=packet;%debug
%         packetsNr=packetsNr+1;%debug
%         packetOld=packet;                                               %debug
%     
    
%visualization code


        
        
         %can slow down, only for debugging
         
% %          %for demonstration emg 1, 4, 6 and Accel X
% %          figure(fhandler)
% %          subplot(8,1,1)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,2))
% %          axis([emgCounter-2000 emgCounter -12 12])
% %          hold on;
% %          subplot(8,1,2)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,3))
% %          axis([emgCounter-2000 emgCounter -12 12])
% %          hold on;
% %          subplot(8,1,3)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,4))
% %          axis([emgCounter-2000 emgCounter -12 12])
% %          hold on;
% %          subplot(8,1,4)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,5))
% %          axis([emgCounter-2000 emgCounter -45 45])
% %          hold on;
% %          subplot(8,1,5)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,6))
% %          axis([emgCounter-2000 emgCounter -45 45])
% %          hold on;
% %          subplot(8,1,6)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,7))
% %          axis([emgCounter-2000 emgCounter -45 45])
% %          hold on;
% %          subplot(8,1,7)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,7))
% %          axis([emgCounter-2000 emgCounter -45 45])
% %          hold on;
% %          subplot(8,1,8)
% %          plot(emgCounter-45:emgCounter, emgDataset(emgCounter-45:emgCounter,8))
% %          axis([emgCounter-2000 emgCounter -12 12])
% %          hold on;
%          figure(fhandler+1)
%          subplot(1,3,1)
%          plot(imuCounter-13:imuCounter, imuDataset(imuCounter-13:imuCounter,6))
%          axis([imuCounter-100 imuCounter -2 2])
%          hold on;
%          subplot(1,3,2)
%          plot(imuCounter-13:imuCounter, imuDataset(imuCounter-13:imuCounter,7))
%          axis([imuCounter-100 imuCounter -2 2])
%          hold on;
%          subplot(1,3,3)
%          plot(imuCounter-13:imuCounter, imuDataset(imuCounter-13:imuCounter,8))
%          axis([imuCounter-100 imuCounter -2 2])
%          hold on;
         
         
%          %figure(linAccelXHandler)
% %          scatter(imuCounter-9:imuCounter,imuDataset(imuCounter-9:imuCounter,15),5,'b')
% %          axis([imuCounter-200 imuCounter -0.5 0.5])
% %         figure(linAccelYHandler)
%           figure(fhandler)
%           scatter(graspChecksPerformed,mean(AccelXFeatures(graspChecksPerformed-9:graspChecksPerformed,17)),5,'b')
%           axis([graspChecksPerformed-30 graspChecksPerformed -0 100])
%          figure(fhandler+1)
%           scatter(graspChecksPerformed,mean(AccelXFeatures(graspChecksPerformed-9:graspChecksPerformed,6)),5,'b')
%           axis([graspChecksPerformed-30 graspChecksPerformed -5 0])
%           figure(fhandler+2)
%           scatter(graspChecksPerformed, mean(AccelYFeatures(graspChecksPerformed-9:graspChecksPerformed,16)),5,'b')
%           axis([graspChecksPerformed-30 graspChecksPerformed -0 5])
%          figure(fhandler+3)
%           scatter(graspChecksPerformed,mean(AccelZFeatures(graspChecksPerformed-9:graspChecksPerformed,6)),5,'b')
%           axis([graspChecksPerformed-30 graspChecksPerformed -5 0])
%          plot(imuCounter-9:imuCounter,imuDataset(imuCounter-9:imuCounter,7))%,5,'b')
%          axis([imuCounter-200 imuCounter -2 2])
%          figure(fhandler+2)
%          scatter(imuCounter,(max(imuDataset(imuCounter-9:imuCounter,6))-min(imuDataset(imuCounter-9:imuCounter,6))) + (max(imuDataset(imuCounter-9:imuCounter,7))-min(imuDataset(imuCounter-9:imuCounter,7))))
%          axis([imuCounter-200 imuCounter -2 2])
% %          figure(fhandler+3)
% %          plot(imuCounter-9:imuCounter,abs(imuDataset(imuCounter-9:imuCounter,9))+abs(imuDataset(imuCounter-9:imuCounter,10))+abs(imuDataset(imuCounter-9:imuCounter,11)))%,5,'b')
% %          axis([imuCounter-200 imuCounter -80 80])
% %          figure(fhandler+4)
% %          plot(imuCounter-9:imuCounter,imuDataset(imuCounter-9:imuCounter,9))%,5,'b')
% %          axis([imuCounter-200 imuCounter -80 80])
% %         figure(linAccelZHandler)
% %          scatter(imuCounter-9:imuCounter,imuDataset(imuCounter-9:imuCounter,17),5,'b')
% %          axis([imuCounter-200 imuCounter -0.5 0.5])
%          %plot(emgCounter-39:emgCounter,emgLowpassDataset(emgCounter-39:emgCounter,5))
%          %axis([emgCounter-200 emgCounter -45 45])
% %          figure(fhandler)
% %          scatter(graspChecksPerformed,AccelZFeatures(graspChecksPerformed,14),5,'b')
%           figure(fhandler+1)
%           scatter(graspChecksPerformed,max(imuDataset(imuCounter-9:imuCounter,6))-min(imuDataset(imuCounter-9:imuCounter,6)) + max(imuDataset(imuCounter-9:imuCounter,7))-min(imuDataset(imuCounter-9:imuCounter,7)),5,'b')
%           figure(fhandler+2)
%           scatter(graspChecksPerformed,gyroZFeatures(graspChecksPerformed,3),5,'b')
%           figure(fhandler+3)
%           scatter(graspChecksPerformed,gyroZFeatures(graspChecksPerformed,4),5,'b')
%           figure(fhandler+4)
%           scatter(graspChecksPerformed,gyroZFeatures(graspChecksPerformed,5),5,'b')
%          %axis([emgCounter-200 emgCounter -45 45])
















% 
% 
% fhandler=figure('Name', 'EMG')
% subplot(8,1,1)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 1')
% subplot(8,1,2)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 2')
% subplot(8,1,3)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 3')
% subplot(8,1,4)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 4')
% subplot(8,1,5)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 5')
% subplot(8,1,6)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 6')
% subplot(8,1,7)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 7')
% subplot(8,1,8)
% hold on; grid on;
% xlabel('Reading'), ylabel('EMG 8')
% figure('Name', 'Accelerometer')
% subplot(1,3,1)
% hold on; grid on;
% xlabel('Reading'), ylabel('Accel X (g)')
% subplot(1,3,2)
% hold on; grid on;
% xlabel('Reading'), ylabel('Accel Y (g)')
% subplot(1,3,3)
% hold on; grid on;
% xlabel('Reading'), ylabel('Accel Z (g)')
% figure('Name', 'f3')
% hold on; grid on;
% figure('Name', 'f4')
% hold on; grid on;
% figure('Name', 'f5')
% hold on; grid on;