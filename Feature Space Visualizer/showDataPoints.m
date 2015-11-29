%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%function plots the data points
%%input: rows of Timedata, columns of Timedata, timetable of Y data,
%%nearest Timestamps of X and Y data, Test Time Ranges, first Figure
%%Handler, x Data, y Data, colorScheme, subplotDims

function showDataPoints(rows, columns, timeY, nearestTimestamps, timeTestRange, figureHandlerStart, xData, yData, colorScheme, subPlotDims, fname, timestamp, datasetName)
    idx=1;
    for i=1:rows
        for j=1:columns-1
            endidx=find(timeY(:)>=timeY(1)+timeTestRange(i,j+1)-timeTestRange(1,1),1,'first');
            for l=1:size(xData,2)
                figure(figureHandlerStart+l-1)
                for k=1:size(yData,2)
                        subplot(subPlotDims(1),subPlotDims(2),k)
                        scatter(xData(nearestTimestamps(idx:endidx),l), yData(idx:endidx,k),10, colorScheme(j));
                end
            end
            idx=endidx;
        end
        if i<rows
            endidx=find(timeY(:)>=timeY(1)+timeTestRange(i+1,1)-timeTestRange(1,1),1,'first');
            for l=1:size(xData,2)
                figure(figureHandlerStart+l-1)
                for k=1:size(yData,2)
                        subplot(subPlotDims(1),subPlotDims(2),k)
                        scatter(xData(nearestTimestamps(idx:endidx),l), yData(idx:endidx,k),10, colorScheme(end));
                end
            end
            idx=endidx;
        end
        hgsave(strcat(datasetName, '-', timestamp, '\', fname, ' Datapoints'))
    end