%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%The function plots bars 
function plotEventMarker(timeTestRange, height, rows)
    if size(timeTestRange,2)==5
        stem(timeTestRange(:,5)-timeTestRange(1,1), height*ones(1,rows), 'red')
        stem(timeTestRange(:,5)-timeTestRange(1,1), -height*ones(1,rows), 'red')
        stem(timeTestRange(:,2)-timeTestRange(1,1), height*ones(1,rows), 'm')
        stem(timeTestRange(:,2)-timeTestRange(1,1), -height*ones(1,rows), 'm')
        stem(timeTestRange(:,3)-timeTestRange(1,1), height*ones(1,rows), 'm')
        stem(timeTestRange(:,3)-timeTestRange(1,1), -height*ones(1,rows), 'm')
        stem(timeTestRange(:,4)-timeTestRange(1,1), height*ones(1,rows), 'm')
        stem(timeTestRange(:,4)-timeTestRange(1,1), -height*ones(1,rows), 'm')
    elseif size(timeTestRange,2)==2
        stem(timeTestRange(:,1)-timeTestRange(1,1), height*ones(1,rows), 'red')
        stem(timeTestRange(:,1)-timeTestRange(1,1), -height*ones(1,rows), 'red')
        stem(timeTestRange(:,2)-timeTestRange(1,1), height*ones(1,rows), 'red')
        stem(timeTestRange(:,2)-timeTestRange(1,1), -height*ones(1,rows), 'red')
    elseif size(timeTestRange,2)==1
        stem(timeTestRange(:,1)-timeTestRange(1,1), height*ones(1,rows), 'red')
        stem(timeTestRange(:,1)-timeTestRange(1,1), -height*ones(1,rows), 'red')
    elseif size(timeTestRange,2)==3
            if timeTestRange(1,2)==1
                stem(timeTestRange(1,1)-timeTestRange(1,1), height, 'm')
                stem(timeTestRange(1,1)-timeTestRange(1,1), -height, 'm')
            else
                stem(timeTestRange(1,1)-timeTestRange(1,1), height, 'red')
                stem(timeTestRange(1,1)-timeTestRange(1,1), -height, 'red')
            end
            for i=2:size(timeTestRange,1)
                if timeTestRange(i,3)>timeTestRange(i-1,3)                  
                    stem(timeTestRange(i,1)-timeTestRange(1,1), height, 'red')
                    stem(timeTestRange(i,1)-timeTestRange(1,1), -height, 'red')
                else 
                    stem(timeTestRange(i,1)-timeTestRange(1,1), height, 'm')
                    stem(timeTestRange(i,1)-timeTestRange(1,1), -height, 'm')
                end
            end
    end