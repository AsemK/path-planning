function b = isObstacle(grid, node)
%isObstacle returns 1 if the location is inside the grid and is an obstacle
%otherwise 0
[gridWidth, gridLength] = size(grid.obstacles);
if (node(1) > 0) && (node(1) <= gridWidth) && (node(2) > 0) && (node(2) <= gridLength)
    b = grid.obstacles(node(1), node(2));
else
    b = false;
end
end

