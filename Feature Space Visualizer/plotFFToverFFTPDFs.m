%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%function plot the PDF FFT over FFT

function plotFFToverFFTPDFs(yTime, yData, rows, columns, xTime, timeRange, xData, colorScheme, plotRange, xLabel, yLabel, timestamp, datasetName)
    nearestTimestamps=knnsearch(yTime(:,1), xTime); %find closest time indexes of both datasets
    %prepare the figure names and so on
    titleName=strcat(yLabel,' / ',xLabel);
    figureHandler=figure('Name',titleName);
    for i=1:plotRange(1)*plotRange(2)
        subplot(plotRange(1), plotRange(2), i);
        xlabel(strcat(xLabel, num2str(i))); 
        ylabel(strcat(yLabel, num2str(i)));
        hold on; grid on;
    end
    
    fname=strcat(yLabel,'-',xLabel);
    %Change for filename forbidden signs
    fname=strrep(fname,'\',' and ');
    fname=strrep(fname,'/',' and ');
    %difference to showPDF: only 1 figure and it compares the same subplots
    %of the time figures!
    idx=1;
    for i=1:rows
        for j=1:columns-1
            endidx=find(yTime(:)>=yTime(1)+timeRange(i,j+1)-timeRange(1,1),1,'first');
            if ~isempty(endidx)
                    for k=1:size(yData,2)
                        subplot(plotRange(1),plotRange(2),k)
                        plotPDF(nearestTimestamps(idx:endidx), xData(:,k), yData(idx:endidx,k), colorScheme(j));
                    end
            end
            idx=endidx;
        end
        if i<rows
            endidx=find(yTime(:)>=yTime(1)+timeRange(i+1,1)-timeRange(1,1),1,'first');
                 for k=1:size(yData,2)
                     subplot(plotRange(1),plotRange(2),k)
                     plotPDF(nearestTimestamps(idx:endidx), xData(:,k), yData(idx:endidx,k), colorScheme(end));
                 end
             idx=endidx;
        end
        hgsave(strcat(datasetName, '-', timestamp, '\', fname, ' PDF'))
    end