function b = isFree(grid, node)
%isFree returns true if the location is inside the grid and free from
%obstacles, otherwise false
[gridWidth, gridLength] = size(grid.obstacles);
if (node(1) > 0) && (node(1) <= gridWidth) && (node(2) > 0) && (node(2) <= gridLength)
    b = ~grid.obstacles(node(1), node(2));
else
    b = false;
end
end

