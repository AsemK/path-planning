function newSearchInfo = AStarBack(grid, searchInfo)
%AStarBack performs a backward A* star search (from goal to start).
%% Options:
h    = @hDiagonalDistance; % the heuristic function
c    = @(node1, node2) cost8(grid, node1, node2); % the cost function
pred = @neighbors8; % the function returning the predecessors of a node
%% if no plan or goals has changed or a change has occured in the grid:
%  plan from scratch. (A* cannot replan)
if ~isempty(grid) && (isempty(searchInfo) ||...
        ~isequal(grid.goal,searchInfo.grid.goal) ||...
        ~isequal(grid.obstacles,searchInfo.grid.obstacles))
    %% Intializing
    tic;
    [gridWidth, gridLength] = size(grid.obstacles);
    success = false; 
    expanded = false(gridWidth, gridLength);
    % intialize the g-values of all nodes to inf.
    g = Inf(gridWidth, gridLength);
    % insert all goal nodes to the open queue.
    open = [];
    for x = 1 : size(grid.goal,1)
        g(grid.goal(x,1), grid.goal(x,2)) = 0;
        open = [open; [h(grid.start, grid.goal(x,:)) grid.goal(x,:)]];
    end
    %% Searching
    while ~isempty(open)
        [m, i] = min(open(:,1)); 
        s = open(i(1),2:3); % the current expanded state
        if isequal(s, grid.start)
            success = true;
            break;
        end
        expanded(s(1), s(2)) = true;
        open(i(1),:) = []; % pop state
        % iterate over predecessors
        pre = pred(grid, s);
        for x = 1:size(pre,1)
            if g(pre(x,1), pre(x,2)) > (g(s(1), s(2)) + c(pre(x,:), s))
                g(pre(x,1), pre(x,2)) = g(s(1), s(2)) + c(pre(x,:), s);
                bp(pre(x,1), pre(x,2)) = {s};
                key = g(pre(x,1), pre(x,2)) + h(grid.start, pre(x,:));
                % if the node was in open before update it
                [inOpen, loc] = ismember(pre(x,:), open(:,2:3),'rows');
                if inOpen
                    open(loc, 1) = key;
                % otherwise, insert it
                else
                    open = [open; [key pre(x,:)]];
                end
            end
        end
    end
    time = toc;
    %% Putting all the information in the searchInfo Struct
    newSearchInfo.success  = success;
    newSearchInfo.goal     = grid.start;
    newSearchInfo.bp       = bp;
    newSearchInfo.g        = g;
    newSearchInfo.expanded = expanded;
    newSearchInfo.grid     = grid;
    newSearchInfo.time     = time;
%% else: do nothing;
else
    newSearchInfo = searchInfo;
    newSearchInfo.time = 0;
    newSearchInfo.expanded = [];
    newSearchInfo.grid = grid;
end
end