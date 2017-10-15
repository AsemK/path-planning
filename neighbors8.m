function nodes = neighbors8(grid, node)
%neighbors8 returns an array of the free neighbors of a node in an
%8-connected grid.
nodes = zeros(0,2);
for x = node(1)-1 : node(1)+1
    for y = [node(2)-1,node(2)+1]
        if isFree(grid, [x y])
            nodes = [nodes; [x y]];
        end
    end
end
for x = [node(1)-1,node(1)+1]
    if isFree(grid, [x node(2)])
        nodes = [nodes; [x node(2)]];
    end
end
end

