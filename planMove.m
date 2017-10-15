function [newStartC, newExpandedC, newPathL, newGrid, newSearchInfo, time, expanded] = ...
    planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlgorithm)
%planMove moves the robot 1 step (changes its start position). If there is
%no available plan, it plans then moves the robot. It also animates the
%movement if run in a loop.
%    It takes the searchInfo as well as the grid situation "after" the
%    movement. It then plots the planning result after movement and change
%    of the grid. If the input ax is empty, the function returns the time
%    spent on planning without animation.
%% Pauses Times (seconds)
planingTime = 2;    % time for showing the planning result (expanded cells).
moveTime    = 0.25;    % time between successive moves in the map.
newGridTime = 2;    % time for showing the new situation.
%% Initialize
newGrid       = grid;
newSearchInfo = searchInfo;
newStartC     = startC;
newExpandedC  = expandedC;
newPathL      = pathL;
time          = 0;
expanded      = 0;
%% If no plan before, or plan is not applicable any more:
% plan from scratch and graph the result
if isempty(searchInfo) || ~isequal(grid.goal,searchInfo.grid.goal)
    newSearchInfo = searchAlgorithm(grid, newSearchInfo);
    newSearchInfo = tracePath(newSearchInfo);
    if ~isempty(ax)
        [newStartC, newExpandedC, newPathL] = ...
            plotGridPath(ax, newStartC, newExpandedC, newPathL,...
            grid, newSearchInfo);
        pause(planingTime);
    end
    time = time + newSearchInfo.time;
    expanded = expanded + sum(sum(newSearchInfo.expanded));
end
%% If the a path was found and the robot has't arrived to the goal:
% move then plan for the grid situation after movement and plot the result.
if newSearchInfo.success && (size(newSearchInfo.path,1) > 1)
    newSearchInfo.path(1,:) = [];
    newGrid = defineStart(grid, newSearchInfo.path(1,:));
    newSearchInfo = searchAlgorithm(newGrid, newSearchInfo);
    if ~isempty(newSearchInfo.expanded)
        newSearchInfo = tracePath(newSearchInfo);
    end
    time = time + newSearchInfo.time;
    expanded = expanded + sum(sum(newSearchInfo.expanded));
    if ~isempty(ax)
        %if this was replanning: plot the new grid situation. and then plot
        %the plan.
        if ~isempty(searchInfo) &&...
                ~(isequal(grid.start, searchInfo.path(1,:)) &&...
                isequal(grid.obstacles,searchInfo.grid.obstacles))
            [newStartC, newExpandedC, newPathL] = ...
                plotGridPath(ax, newStartC, newExpandedC, newPathL,...
                newGrid, []);
            pause(newGridTime);
            [newStartC, newExpandedC, newPathL] = ...
                plotGridPath(ax, newStartC, newExpandedC, newPathL,...
                [], newSearchInfo);
            pause(planingTime);
            %else it is just movement (and improving previouse plans for anytime
            %planners: do not plot the grid, just the path and expanded cells.
        else
            % grid is not inputted to the plotGridPath function because no need
            % to redraw it.
            [newStartC, newExpandedC, newPathL] = ...
                plotGridPath(ax, newStartC, newExpandedC, newPathL,...
                [], newSearchInfo);
            pause(moveTime);
        end
    end
end
end

