clear all; close all;
searchAlgs = {@AStar, @AStarBack, @DStarLite, @DStarLite2, @ARAStarBack, @ADStar, @ADStar2};
% Make timeTest true to run time tests.
timeTest = true;
repetitions = 100; % number of repeated running times for time measurements.
%% The movement scenario
if timeTest
    reps = repetitions;
    ax = [];
else
    reps = 1;
    figure
    ax = gca;
end
times = [];
expands = [];
paths = [];
for j = 1:length(searchAlgs)
    time = 0;
    expand = 0;
    path = 0;
    for i = 1:reps
        grid = paperGrid;
        searchAlg = searchAlgs{j};
        startC = [];
        expandedC = [];
        pathL = [];
        searchInfo = [];
        % move for 2 steps
        [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
            planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
        time = time + time0;
        expand = expand + expanded;
        path = path + 1;
        % the grid is changed so replanning is needed
        grid = removeObstacle(grid, [4 6]);
        % move until the goal
        while size(searchInfo.path,1) > 1
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
    end
    times = [times time/reps];
    expands = [expands expand/reps];
    paths = [paths path/reps];
end
times
expands
paths