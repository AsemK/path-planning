function newSearchInfo = ARAStarBack(grid, searchInfo)
%ARAStar performs a Backward (from goal to start) Anytime Replanning A* search
%    see: https://www.cs.cmu.edu/~maxim/files/hsplanguide_icaps05ws.pdf
%% Options:
h    = @hDiagonalDistance; % the heuristic function
c    = @(node1, node2) cost8(grid, node1, node2); % the cost function
pred = @neighbors8; % the function returning the predecessors of a node
Eps0 = 2.5;  % initial value of epsilon
dEps = 0.5; % amount of change in the epsilon values betweeen iterations
% when true, Eps is reset to its initial value whenever a change happens.
resetEps = true;
%% Nested Helping Function
function k = key(node)
    k = g(node(1),node(2)) + Eps.*h(grid.start, node);
end
function insertOpen(key, node)
    [inOpen, loc] = ismember(node, open(:,2:3), 'rows');
    if inOpen
        open(loc, :) = [key node];
    else
        open = [open; [key node]];
    end
end
function insertIncons(node)
    [inIncons, loc] = ismember(node, incons, 'rows');
    if inIncons
        incons(loc, :) = node;
    else
        incons = [incons; node];
    end
end
function improvePath
    open = sortrows(open);
    while ~isempty(open)
        s = open(1,2:3);
        if (key(grid.start) <= open(1,1))
            success = true;
            break;
        end
        open(1,:) = [];
        closed = [closed; s];
        expanded(s(1), s(2)) = true;
        % iterate over predecessors
        pre = pred(grid, s);
        for x = 1:size(pre,1)
            if g(pre(x,1),pre(x,2)) > (c(pre(x,:), s) + g(s(1),s(2)))
                g(pre(x,1),pre(x,2)) = c(pre(x,:), s) + g(s(1),s(2));
                bp(pre(x,1), pre(x,2)) = {s};
                if ~ismember(pre(x,:), closed, 'rows')
                    insertOpen(key(pre(x,:)), pre(x,:));
                else
                    insertIncons(pre(x,:));
                end
            end
        end
        open = sortrows(open);
    end
end
function saveData
    % save the needed data.
    newSearchInfo.success  = success; %true when a path is found the goal.
    newSearchInfo.goal     = grid.start;
    % expanded matrix indicates if a node was popped out of the queue
    newSearchInfo.expanded = expanded;
    newSearchInfo.g        = g;
    newSearchInfo.Eps      = Eps;
    %open is the queue containing the nodes to be explored (corresponds
    %to U in the paper) ordered in a lexicographic way.
    newSearchInfo.open     = open;
    newSearchInfo.incons   = incons;
    % back pointers to trace the shortest path.
    newSearchInfo.bp       = bp;
    newSearchInfo.grid     = grid;
    newSearchInfo.time     = time;
end
%% if no plan or goals has changed or a change has occured in the grid: 
%  plan from scratch. (ARA* cannot replan)
if ~isempty(grid) && (isempty(searchInfo)...
        || ~isequal(grid.goal,searchInfo.grid.goal)...
        || ~isequal(grid.obstacles,searchInfo.grid.obstacles))
    [gridWidth, gridLength] = size(grid.obstacles);
    % initialize data
    tic;
    success  = false;
    expanded = false(gridWidth, gridLength);
    % intialize the g values of all nodes to inf. (this can be avoided)
    g        = Inf(gridWidth, gridLength);
    if ~isempty(searchInfo) && ~resetEps
        if searchInfo.Eps - dEps >= 1.0
            Eps  = searchInfo.Eps - dEps;
        else
            Eps  = 1.0;
        end
    else
        Eps      = Eps0;
    end
    open     = zeros(0,3);
    closed   = zeros(0,2);
    incons   = zeros(0,2);
    bp       = cell(gridWidth, gridLength);
    %insert all goal cells to the open queue.
    for r = 1 : size(grid.goal,1)
        g(grid.goal(r,1), grid.goal(r,2)) = 0;
        open = [open; [key(grid.goal(r,:)) grid.goal(r,:)]];
    end
    improvePath;
    time = toc;
    saveData;
%% else if the grid as it is: improve the current plan
elseif ~isempty(grid) && searchInfo.success && (searchInfo.Eps > 1.0)
    [gridWidth, gridLength] = size(grid.obstacles);
    % retreive search data
    tic;
    success  = true;
    expanded = false(gridWidth, gridLength);
    g        = searchInfo.g;
    if searchInfo.Eps - dEps >= 1.0
        Eps  = searchInfo.Eps - dEps;
    else
        Eps  = 1.0;
    end
    open     = searchInfo.open;
    closed   = zeros(0,2);
    incons   = searchInfo.incons;
    bp       = searchInfo.bp;
    % insert cells in incons to open
    for r=1:size(incons,1)
        insertOpen(0, incons(r,:));
    end
    % update keys in open
    for r=1:size(open,1)
        open(r,1) = key(open(r,2:3));
    end
    % update keys in open
    improvePath;
    time = toc;
    saveData;
%% else: do nothing
else
    newSearchInfo = searchInfo;
    newSearchInfo.time = 0;
    newSearchInfo.expanded = [];
    newSearchInfo.grid = grid;
end
end