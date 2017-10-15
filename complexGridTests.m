clear all; close all;
searchAlgs = {@DStarLite2};
% Make timeTest true to run time tests.
timeTest = false;
repetitions = 50; % number of repeated running times for time measurements.
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
        grid = complexGrid;
        searchAlg = searchAlgs{j};
        startC = [];
        expandedC = [];
        pathL = [];
        searchInfo = [];
        % move for 3 steps
        for x = 1:2
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = removeObstacle(grid, [5 6]);
        % move for 8 steps
        for x = 1:8
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = addObstacle(grid, [23 1]);
        grid = addObstacle(grid, [23 2]);
        % move for 7 steps
        for x = 1:7
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = removeObstacle(grid, [21 13]);
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