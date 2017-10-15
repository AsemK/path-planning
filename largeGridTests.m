clear all; close all;
searchAlgs = {@DStarLite2}; 
% Make timeTest true to run time tests.
timeTest = false;
repetitions = 10; % number of repeated running times for time measurements.
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
        grid = largeGrid;
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
        grid = removeObstacle(grid, [10 6]);
        grid = removeObstacle(grid, [10 7]);
        % move for 20 steps
        for x = 1:20
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = addObstacle(grid, [14 19]);
        % move for 25 steps
        for x = 1:25
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = addObstacle(grid, [24 23]);
        grid = addObstacle(grid, [10 47]);
        grid = removeObstacle(grid, [21 36]);
        % move for 25 steps
        for x = 1:25
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = addObstacle(grid, [37 30]);
        grid = removeObstacle(grid, [95 8]);
        % move for 30 steps
        for x = 1:30
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = addObstacle(grid, [54 31]);
        % move for 20 steps
        for x = 1:20
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = removeObstacle(grid, [81 16]);
        grid = addObstacle(grid, [83 24]);
        % move for 20 steps
        for x = 1:20
            [startC, expandedC, pathL, grid, searchInfo, time0, expanded] = ...
                planMove(ax, startC, expandedC, pathL, grid, searchInfo, searchAlg);
            time = time + time0;
            expand = expand + expanded;
            path = path + 1;
        end
        % the grid is changed so replanning is needed
        grid = removeObstacle(grid, [87 22]);
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