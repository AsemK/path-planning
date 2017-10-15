function newSearchInfo = AStar(grid, searchInfo)
%AStar performs an A* star search.
%% Options:
h    = @hDiagonalDistance; % the heuristic function
c    = @(node1, node2) cost8(grid, node1, node2); % the cost function
succ = @neighbors8; % the function returning the successors of a node
%% if no plan or goals has changed or a change has occured in the grid: 
%  plan from scratch. (A* cannot replan)
if ~isempty(grid) && (isempty(searchInfo) ||...
        ~isequal(grid.goal,searchInfo.grid.goal) ||...
        ~isequal(grid.obstacles,searchInfo.grid.obstacles))
    %% Intializing
    tic
    [gridWidth, gridLength] = size(grid.obstacles);
    success  = false;
    goal     = zeros(0, 2);
    expanded = false(gridWidth, gridLength);
    % intialize the g-values of all nodes to inf.
    g = Inf(gridWidth, gridLength);
    g(grid.start(1), grid.start(2)) = 0;
    open = [h(grid.start, grid.goal) grid.start];
    %% Searching
    while ~isempty(open)
        [m, i] = min(open(:,1)); 
        s = open(i(1),2:3); % the current expanded state
        if ismember(s, grid.goal, 'rows')
            success = true;
            goal = s;
            break;
        end
        expanded(s(1), s(2)) = true;
        open(i(1),:) = []; % pop state
        % iterate over successors
        suc = succ(grid, s);
        for x = 1:size(suc,1)
            if g(suc(x,1), suc(x,2)) > (g(s(1), s(2)) + c(s, suc(x,:)))
                g(suc(x,1), suc(x,2)) = g(s(1), s(2)) + c(s, suc(x,:));
                bp(suc(x,1), suc(x,2)) = {s};
                key = g(suc(x,1), suc(x,2)) + h(suc(x,:), grid.goal);
                % if the node was in open before update it
                [inOpen, loc] = ismember(suc(x,:), open(:,2:3),'rows');
                if inOpen
                    open(loc, 1) = key;
                % otherwise, insert it
                else
                    open = [open; [key suc(x,:)]];
                end
            end
        end
    end
    time = toc;
    %% Putting all the information in the searchInfo Struct
    newSearchInfo.success  = success;
    newSearchInfo.goal     = goal;
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