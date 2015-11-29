%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%function plot the PDF FFT over Time Domain

function plotFFTPDFs(yTime, yData, rows, columns, xTime, timeRange, xData, colorScheme, plotRange, xLabel, yLabel, timestamp, datasetName)
    yTime=yTime+timeRange(1,1); %sets time from sub log range to global log range
    nearestTimestamps=knnsearch(yTime(:,1), xTime); %find closest time indexes of both datasets
    %prepare the figure names and so on
    titleName=strcat(yLabel,' / ',xLabel);
    figureHandler=figure('Name',titleName);
    for i=1:plotRange(1)*plotRange(2)
        subplot(plotRange(1), plotRange(2), i);
        xlabel(xLabel); 
        ylabel(strcat(yLabel, num2str(i)));
        hold on; grid on;
    end
    
    fname=strcat(yLabel,'-',xLabel);
    %Change for filename forbidden signs
    fname=strrep(fname,'\',' and ');
    fname=strrep(fname,'/',' and ');
    %print the data
    showPDF(rows, columns, yTime, nearestTimestamps, timeRange, figureHandler, xData, yData, colorScheme, plotRange, fname, timestamp, datasetName);