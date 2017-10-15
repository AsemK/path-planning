function newGrid = defineStart(grid, node)
%defineStart sets the start position of the agent on the grid. does
%nothing if there is an obstacle in the desired position
newGrid = grid;
if isFree(grid, node)
    newGrid.start = node;
end
end

