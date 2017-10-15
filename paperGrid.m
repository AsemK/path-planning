function grid = paperGrid
%paperGrid returns the same grid seen in the Anytime D* paper
%   see https://www.cs.cmu.edu/~maxim/files/hsplanguide_icaps05ws.pdf
gridWidth  = 6;                             %number of cells in the x-direction.
gridLength = 7;                             %number of cells in the y-direction.
grid = createEmptyGrid(gridWidth, gridLength);
grid = defineStart(grid, [gridWidth 1]);
grid = addGoal(grid, [1 gridLength]);   
for x=1:5
    grid = addObstacle(grid, [x 1]);
    grid = addObstacle(grid, [x 3]);
    grid = addObstacle(grid, [x 6]);
end
grid = addObstacle(grid, [2 4]);
grid = addObstacle(grid, [2 5]);
grid = addObstacle(grid, [5 2]);
grid = removeObstacle(grid, [1 3]);
grid = removeObstacle(grid, [4 3]);
grid = removeObstacle(grid, [1 6]);
end

