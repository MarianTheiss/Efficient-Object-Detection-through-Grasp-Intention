%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%function shows Time Domain over Time domain PDF
function showPDF(rows, columns, timeY, nearestTimestamps, timeTestRange, figureHandlerStart, xData, yData, colorScheme, subPlotDims, fname, timestamp, datasetName)
    idx=1;
    for i=1:rows
        for j=1:columns-1
            endidx=find(timeY(:)>=timeY(1)+timeTestRange(i,j+1)-timeTestRange(1,1),1,'first');
            if ~isempty(endidx)
                for l=1:size(xData,2)
                    figure(figureHandlerStart+l-1)
                    for k=1:size(yData,2)
                        subplot(subPlotDims(1),subPlotDims(2),k)
                        plotPDF(nearestTimestamps(idx:endidx), xData(:,l), yData(idx:endidx,k), colorScheme(j));
                    end
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
                     plotPDF(nearestTimestamps(idx:endidx), xData(:,l), yData(idx:endidx,k), colorScheme(end));
                 end
             end
             idx=endidx;
        end
        hgsave(strcat(datasetName, '-', timestamp, '\', fname, ' PDF'))
    end