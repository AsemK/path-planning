function c = cost8(grid, node1, node2)
%cost8 returns the cost of moving from a node1 to node2 in an 8-connected grid.
%   in an 8-connected grid, the sucessors and predecessors of a cell are
%   the 8-surrounding cells as long as they are not occupied by obstacles
%cost when one of the nodes is an obstacle is infinity.
if isObstacle(grid, node1) || isObstacle(grid, node2)
    c = Inf;
%cost of moving to an adjacent cell
elseif xor(abs(node1(1)-node2(1)) == 1, abs(node1(2)-node2(2)) == 1)
    c = 1;
%cost of moving diagonally
elseif and(abs(node1(1)-node2(1)) == 1, abs(node1(2)-node2(2)) == 1)
    c = 1.4142;
%cost when the nodes are not neighbors
else
    c = Inf;
end
end

