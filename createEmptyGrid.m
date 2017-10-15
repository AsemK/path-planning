function grid = createEmptyGrid(gridWidth, gridLength)
%createEmptyGrid returns a new grid with no obstacles, start or goal
%positions.
%obstacles are represented by a binary matrix.
grid.obstacles = false(gridWidth, gridLength);
%empty starting position [x y].
grid.start = zeros(0,2);
%empty goal position of [x y].
grid.goal  = zeros(0,2);
end

