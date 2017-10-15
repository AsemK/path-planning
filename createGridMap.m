function grid = createGridMap(obstaclesMap)
%createEmptyGrid returns a new grid with obstacles corresponding to the
%input obstaclesMap binary matrix, and with empty start and goal positions.
%obstacles are represented by a binary matrix.
obstacles = obstaclesMap(end:-1:1,:);
obstacles = obstacles';
grid.obstacles = obstacles;
%empty starting position [x y].
grid.start = zeros(0,2);
%empty goal position of [x y].
grid.goal  = zeros(0,2);
end