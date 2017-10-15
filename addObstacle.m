function newGrid = addObstacle(grid, node)
%addObstacle adds an obstacle to the grid. Does nothing if the desired
%location has a goal or a start.
  newGrid = grid;
  if isFree(grid, node) && ~isequal(node, grid.start) &&...
     ~ismember(node, grid.goal, 'rows');
      newGrid.obstacles(node(1), node(2)) = true;
  end
end

