function newGrid = removeObstacle(grid, node)
%removeObstacle removes an obstacle from the grid. Does nothing if the desired
%location has no obstacle.
newGrid = grid;
if isObstacle(grid, node)
    newGrid.obstacles(node(1), node(2)) = false;
end
end

