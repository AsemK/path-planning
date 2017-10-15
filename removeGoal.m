function newGrid = removeGoal(grid, node)
%removeGoal removes a goal from the grid. Does nothing if the desired
%location has no goal.
newGrid = grid;
[inGoals, loc] = ismember(node, grid.goal,'rows');
if inGoals
    grid.goal(loc, :) = [];
end
end

