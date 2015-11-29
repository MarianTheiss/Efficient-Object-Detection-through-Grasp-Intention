%%This script is made for the Master Thesis "Efficient Object Detection through Grasp Intention"
%%Author: Marian Theiss
%%function plots the data points
%%Function extracts Datapoints from figure

function [X,Y,name,nrows,ncols]=getDataFromFig(figurehandler,datasetName,timestamp)
    allAxesInFigure = findall(figurehandler,'type','axes');
    %get dimenstions of the subplot
    pos = cell2mat(get(allAxesInFigure,'position'));
    nrows = numel(unique(pos(:,1))); % same X positions
    ncols = numel(unique(pos(:,2))); % same Y positions
    %scatterdata is a child of all axes of all subplots of the figure
    for i=1:nrows*ncols
        hac=allchild(allAxesInFigure(i));
        %read out all scatter Points
        h=findobj(hac,'Type','patch');
        Xcell(:,i)=get(h,'XData');
        Ycell(:,i)=get(h,'YData');
        %Cells to matrix
        X(:,i)=cell2mat(Xcell(:,i));
        Y(:,i)=cell2mat(Ycell(:,i));
    end
    %Matrix is upside down and left to right -> flip it
    X=rot90(X,2);
    Y=rot90(Y,2);
    %get name of the figure
    name=get(figurehandler,'Name');
    %get dimenstions of the subplot
    pos = cell2mat(get(allAxesInFigure,'position'));
    nrows = numel(unique(pos(:,1))); % same X positions
    ncols = numel(unique(pos(:,2))); % same Y positions
    %save to variable... loading figures needs too much time!!!
    saveStruct={'name',name,'xData',X,'yData',Y};
    %Change for filename forbidden signs
    name=strrep(name,'\',' and ');
    name=strrep(name,'/',' and ');
    save(strcat(datasetName,'-',timestamp,'\',name,'.mat'), 'saveStruct');