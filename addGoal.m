function newGrid = addGoal(grid, node)
%addGoal adds a goal on the grid. multiple goals can be added to one grid.
%does nothing if there is an obstacle in the desired position.
  newGrid = grid;
  if isFree(grid, node) && ~ismember(node, grid.goal, 'rows')
      newGrid.goal = [newGrid.goal; node];
  end
end