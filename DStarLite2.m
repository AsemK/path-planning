function newSearchInfo = DStarLite2(grid, searchInfo)
%DStarLite performs an optimized version of Dynamic A* Lite search
%    see: http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
%% Options:
h    = @hDiagonalDistance;  % the heuristic function
c    = @(node1, node2) cost8(grid, node1, node2); % the cost function
succ = @neighbors8; % the function returning the successors of a node
pred = @neighbors8; % the function returning the predecessors of a node
%% Nested Helping Functions
function k = key(node)
    minGRHS = min(g(node(1),node(2)), rhs(node(1),node(2)));
    % round the key so that comparisons are reliable.
    k = round([minGRHS + h(grid.start, node), minGRHS], 4);
end
function updateState(node)
    [inOpen, loc] = ismember(node, open(:, 3:4), 'rows');
    if g(node(1), node(2)) ~= rhs(node(1), node(2))
        if inOpen 
            open(loc,:) = [key(node), node];
        else
            open = [open; [key(node), node]];
        end
    else
        if inOpen 
            open(loc,:) = [];
        end
    end
end
% updates back pointer by inspecting successors
function updateBpSucc(node)
    suc = succ(grid, node);
    len = size(suc,1);
    vals = zeros(len,1);
    for x = 1:len
        vals(x) = g(suc(x,1), suc(x,2)) + c(node, suc(x,:));
    end
    [M, I] = min(vals);
    rhs(node(1), node(2)) = M;
    bp(node(1), node(2)) = {suc(I,:)};
    updateState(node);
end
function computeShortestPath
    % lexicographic sorting of the queue
    open     = sortrows(open);
    startKey = key(grid.start);    
    % lexicographical comparisons between two keys
    while (open(1,1) < startKey(1)) ||...
            ((open(1,1) == startKey(1)) && (open(1,2) < startKey(2))) ||...
            (g(grid.start(1),grid.start(2)) <...
            rhs(grid.start(1),grid.start(2)))
        s = open(1, 3:4); % the current considered cell.
        expanded(s(1), s(2)) = true;
        if g(s(1),s(2)) > rhs(s(1),s(2))
            g(s(1),s(2)) = rhs(s(1),s(2));
            open(1,:) = []; % pop the state.
            % itrate over all predecessors
            pre = pred(grid, s);
            for x = 1:size(pre,1)
                if rhs(pre(x,1), pre(x,2)) > g(s(1), s(2)) + c(pre(x,:), s)
                    rhs(pre(x,1), pre(x,2)) = g(s(1), s(2)) + c(pre(x,:), s);
                    bp(pre(x,1), pre(x,2)) = {s};
                    updateState(pre(x,:));
                end
            end
        else
            g(s(1),s(2)) = Inf;
            % itrate over all predecessors
            pre = pred(grid, s);
            for x = 1:size(pre,1)
                if isequal(cell2mat(bp(pre(x,1), pre(x,2))), s)
                    updateBpSucc(pre(x,:));
                end
            end
        end
        % keep the node with the minimum key at the top.
        open     = sortrows(open);   
        startKey = key(grid.start);
    end
    success = (rhs(grid.start(1), grid.start(2)) < Inf);
end
function saveData
% save the needed data.
    newSearchInfo.success  = success;
    newSearchInfo.goal     = grid.start;
    newSearchInfo.expanded = expanded;
    newSearchInfo.g        = g;
    newSearchInfo.rhs      = rhs;
    newSearchInfo.open     = open;
    newSearchInfo.bp       = bp;
    newSearchInfo.grid     = grid;
    newSearchInfo.time     = time;
end
if ~isempty(grid) && (isempty(searchInfo)...
        || ~isequal(grid.goal,searchInfo.grid.goal)...
        || ~isequal(grid.obstacles,searchInfo.grid.obstacles))
    [gridWidth, gridLength] = size(grid.obstacles);
    tic;
    success  = false;
    expanded = false(gridWidth, gridLength);
    % if no plan or goals has changed: plan from scratch
    if isempty(searchInfo) || ~isequal(grid.goal,searchInfo.grid.goal)
        % intialize the g and rhs values of all nodes to inf.
        g    = Inf(gridWidth, gridLength);
        rhs  = g;
        % insert all goal cells to the open queue.
        open = zeros(0,4);
        for r = 1 : size(grid.goal,1)
            rhs(grid.goal(r,1), grid.goal(r,2)) = 0;
            open = [open; [key(grid.goal(r,:)) grid.goal(r,:)]];
        end
        bp = cell(gridWidth, gridLength);
    % else if a change has occured in the grid: retrieve searchInfo and replan
    else
        g    = searchInfo.g;
        rhs  = searchInfo.rhs;
        open = searchInfo.open;
        % update the keys. The authors claim that this step can be skipped
        % but when applying their optimization, the result differs.
        for r=1:size(open,1)
            open(r,1:2) = key(open(r,3:4));
        end
        bp   = searchInfo.bp;
        % this loop might update nodes more than one time. needs to be
        % optimised in an actual implementation.
        for w = 1:gridWidth
            for l = 1:gridLength
                if grid.obstacles(w,l) ~= searchInfo.grid.obstacles(w,l)
                    if grid.obstacles(w, l)
                        changed = succ(grid, [w l]);
                        for r = 1:length(changed)
                            if ~ismember(changed(r,:), grid.goal, 'rows') && ...
                                    isequal(cell2mat(bp(changed(r,1),changed(r,2))), [w l])
                                updateBpSucc(changed(r,:));
                            end
                        end   
                    else
                        % since goal cells are not allowed to change, no
                        % check is done to ensure that [w l] is not a goal.
                        updateBpSucc([w l]);
                    end
                end
            end
        end
    end
    computeShortestPath;
    time = toc;
    saveData;
else
    newSearchInfo = searchInfo;
    newSearchInfo.time = 0;
    newSearchInfo.expanded = [];
    newSearchInfo.grid = grid;
end
end