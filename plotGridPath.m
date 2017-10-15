function [newStartC, newExpandedC, newPathL] = ...
    plotGridPath(ax, startC, expandedC, pathL, Grid, searchInfo)
%plotGridPath plots the input grid or plots a path on an already drawn grid
%or does both.
%    if input searchInfo is empty, only the grid will be drawn. Else, also
%    the path will be drawn. If grid is empty, the path will be drawn on
%    the figure that contains the grid. In this case, the handle to that
%    function (input ax) must not be empty.
%% Options
% ----------------------- colors -----------------------
gridColor     = [0.6 0.6 0.6]; %light gray
obstacleColor = [0.1 0.1 0.1]; %nearly black
startColor    = [0.5 0.5 1.0]; %pale blue
goalColor     = [1.0 0.5 0.5]; %pale red
pathColor     = [0.0 0.0 1.0]; %blue
expandedColor = [1.0 1.0 0.0]; %yellow
%if expandedColorMap is true, the color of the expanded cell will represent
%its relative g-value. the lower (closer to the search start) the more 
%yellow, the higher (closer to the search goal) the more cian.
expandedColorMap = true; 
% -------------------- other options --------------------
%if ticks is true, ticks numbers will be shown on each axis representing
%the cooredinates of the grid.
ticks = false;
%if gridLines is true a grid will be shown, otherwise no grid is drawn.
gridLines = false;
% -------------------- graph contents --------------------
plotExpanded = true; % allows expanded cells to be poltted.
plotPath = true;     % allows path line to be poltted.
%% Intialize
newStartC = [];
newExpandedC = [];
newPathL = [];
if gridLines
    gridStyle = '-';
else
    gridStyle = 'none';
end
%% If grid is available, delete old graphics (if any) and draw the grid
if ~isempty(Grid)
    [gridWidth, gridLength] = size(Grid.obstacles);
    %reset the figure, produce new one if no figure was not provided.
    if isempty(ax)
        figure 
        ax = gca;
    else
        cla(ax);
    end
    % Figure style and aesthetics
    axis equal;
    axis([0 gridWidth 0 gridLength])
    ax.XTick = 1:gridWidth;
    ax.YTick = 1:gridLength;
    ax.TickDir = 'in';
    if ~ticks
        ax.XTickLabel = {};
        ax.YTickLabel = {};
        ax.TickLength = [0.00 0.035];
    end
    ax.LineWidth = 1.0;
    ax.GridAlpha = 1.0;
    ax.GridColor = gridColor;
    ax.XColor    = gridColor;
    ax.YColor    = gridColor;
    ax.Box = 'on';
    if gridLines
        grid on;
    else
        grid off;
    end
    set(gcf,'color','w');
    % plot obstacle
    for x = 1:gridWidth
        for y = 1:gridLength
            if Grid.obstacles(x, y) == 1
                rectangle(ax, 'Position', [x-1,y-1,1,1], 'FaceColor', obstacleColor,...
                    'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
                hold on
            end
        end
    end
    % plot start and goal locations
    startC = rectangle(ax, 'Position', [Grid.start(1)-1, Grid.start(2)-1,1,1],...
        'FaceColor', startColor, 'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
    for x = 1:size(Grid.goal,1)
        rectangle(ax, 'Position', [Grid.goal(x,1)-1, Grid.goal(x,2)-1,1,1],...
            'FaceColor', goalColor, 'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
    end
    newStartC = startC;
end
%% If path is available, delete old graphics and plot this path
if ~isempty(searchInfo)
    %delete old lines and cells
    delete(startC);
    for c = expandedC
        delete(c);
    end
    delete(pathL);
    %plot expanded nodes
    if plotExpanded
        [gridWidth, gridLength] = size(searchInfo.expanded);
        if expandedColorMap
            colormap = searchInfo.g;
            colormap(~searchInfo.expanded)=0;
            colormap(colormap==Inf)=1;
            colormap=colormap./max(searchInfo.g(searchInfo.g < Inf));
        end
        newExpandedC = [];
        for x = 1:gridWidth
            for y = 1:gridLength
                if searchInfo.expanded(x, y)
                    if expandedColorMap
                        expandedColor = [1-colormap(x, y) 1 colormap(x, y)];
                    end
                    newExpandedC = [newExpandedC;...
                        rectangle(ax, 'Position', [x-1,y-1,1,1],...
                        'FaceColor', expandedColor,...
                        'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle)];
                    hold on
                end
            end
        end
    end
    %plot finish and start 
    Grid = searchInfo.grid;
    for x = 1:size(Grid.goal,1)
        rectangle(ax, 'Position', [Grid.goal(x,1)-1, Grid.goal(x,2)-1,1,1],...
            'FaceColor', goalColor, 'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
    end
    newStartC = rectangle(ax, 'Position', [Grid.start(1)-1, Grid.start(2)-1,1,1],...
        'FaceColor', startColor, 'EdgeColor', gridColor, 'LineWidth',1, 'LineStyle', gridStyle);
    %plot path
    if plotPath && searchInfo.success
        path = searchInfo.path;
        newPathL = plot(ax, path(:,1) - 0.5, path(:,2) - 0.5, '--',...
            'Color', pathColor, 'LineWidth', 2);
    end
end
drawnow;
end

