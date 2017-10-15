function newSearchInfo = ADStar2(grid, searchInfo)
%ADStar2 performs an optimised version of Anytime Dynamic A* search
%    see: www.csd.uoc.gr/~hy475/papers2006/berg_jur_van_den_2006_1.pdf
%% Options:
h    = @hDiagonalDistance; % the heuristic function
c    = @(node1, node2) cost8(grid, node1, node2); % the cost function
succ = @neighbors8; % the function returning the successors of a node
pred = @neighbors8; % the function returning the predecessors of a node
Eps0 = 2.5;  % initial value of epsilon
dEps = 0.5; % amount of change in the epsilon values betweeen iterations
% when true, Eps is reset to its initial value whenever a change happens.
resetEps = false; 
%% Nested Helping Functions
function k = key(node)
    if g(node(1),node(2)) > rhs(node(1),node(2))
        k = [rhs(node(1),node(2))+Eps.*h(grid.start, node),...
            rhs(node(1),node(2))];
    else
        k = [g(node(1),node(2)) + h(grid.start, node),...
            g(node(1),node(2))];
    end
    k = round(k, 4); % round the key so that comparisons are reliable.
end
function insertOpen(key, node)
    [inOpen, loc] = ismember(node, open(:,3:4), 'rows');
    if inOpen
        open(loc, :) = [key node];
    else
        open = [open; [key node]];
    end
end
function insertIncons(node)
    if ~ismember(node, incons, 'rows')
        incons = [incons; node];
    end
end
function updateSetMembership(node)
    if g(node(1), node(2)) ~= rhs(node(1), node(2))
        if ~ismember(node, closed,'rows')
            insertOpen(key(node), node);
        else
            insertIncons(node);
        end
    else
        [inOpen, loc] = ismember(node, open(:,3:4), 'rows');
        if inOpen
            open(loc,:) = [];
        else
            [inIncons, loc] = ismember(node, incons, 'rows');
            if inIncons
                incons(loc,:) = [];
            end
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
    updateSetMembership(node);
end
function computePath
    % lexicographic sorting of the queue
    open     = sortrows(open);
    startKey = key(grid.start);
    % lexicographical comparisons between two keys
    while (open(1,1) < startKey(1)) ||...
            ((open(1,1) == startKey(1)) && (open(1,2) < startKey(2))) ||...
            (g(grid.start(1),grid.start(2)) <...
            rhs(grid.start(1),grid.start(2)))
        s = open(1, 3:4); % the current considered state.
        open(1,:) = [];   % remove s from Open
        if g(s(1),s(2)) > rhs(s(1),s(2))
            g(s(1),s(2)) = rhs(s(1),s(2));
            closed = [closed; s];
            expanded(s(1), s(2)) = true;
            % itrate over all predecessors
            pre = pred(grid, s);
            for x = 1:size(pre,1)
                if rhs(pre(x,1), pre(x,2)) > g(s(1), s(2)) + c(pre(x,:), s)
                    rhs(pre(x,1), pre(x,2)) = g(s(1), s(2)) + c(pre(x,:), s);
                    bp(pre(x,1), pre(x,2)) = {s};
                    updateSetMembership(pre(x,:));
                end
            end
        else
            g(s(1),s(2)) = Inf;
            updateSetMembership(s);
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
    newSearchInfo.success  = success; %true when a path is found the goal.
    newSearchInfo.goal     = grid.start;
    % expanded matrix indicates if a node was popped out of the queue
    newSearchInfo.expanded = expanded;
    newSearchInfo.g        = g;
    newSearchInfo.rhs      = rhs;
    newSearchInfo.Eps      = Eps;
    % open is the queue containing the nodes to be explored (corresponds
    % to U in the paper) ordered in a lexicographic way.
    newSearchInfo.open     = open;
    newSearchInfo.incons   = incons;
    % back pointers to trace the shortest path.
    newSearchInfo.bp       = bp;
    newSearchInfo.grid     = grid;
    newSearchInfo.time     = time;
end
[gridWidth, gridLength] = size(grid.obstacles);
%% if no plan or goals has changed: plan from scratch
if ~isempty(grid) && (isempty(searchInfo) ||...
                      ~isequal(grid.goal,searchInfo.grid.goal))
    tic;
    success  = false;
    expanded = false(gridWidth, gridLength);
    %intialize the g and rhs values of all nodes to inf.
    g        = Inf(gridWidth, gridLength);
    rhs      = g;
    Eps      = Eps0;
    open     = zeros(0,4);
    closed   = zeros(0,2);
    incons   = zeros(0,2);
    bp       = cell(gridWidth, gridLength);
    % insert all goal cells to the open queue.
    for r = 1 : size(grid.goal,1)
        rhs(grid.goal(r,1), grid.goal(r,2)) = 0;
        open = [open; [key(grid.goal(r,:)) grid.goal(r,:)]];
    end
    computePath;
    time = toc;
    saveData;
%% else if a change has occured in or eps > 1: replan in anytime way.
elseif ~isempty(grid) && ((searchInfo.success && (searchInfo.Eps > 1.0))...
        || ~isequal(grid.obstacles,searchInfo.grid.obstacles))
    % retrieve Data
    tic;
    success  = false;
    expanded = false(gridWidth, gridLength);
    g        = searchInfo.g;
    rhs      = searchInfo.rhs;
    Eps      = searchInfo.Eps;
    open     = searchInfo.open;
    closed   = zeros(0,2);
    incons   = searchInfo.incons;
    bp       = searchInfo.bp;
    if ~isequal(grid.obstacles,searchInfo.grid.obstacles)
        % this loop might update nodes more than one time. needs to be
        % optimised in an actual implementation.
        % also, itrating over the whole map just for two or three changed costs
        % is pointless.
        for w = 1:gridWidth
            for l = 1:gridLength
                if grid.obstacles(w,l) ~= searchInfo.grid.obstacles(w,l)
                    if grid.obstacles(w, l)
                        changed = succ(grid, [w l]);
                        for r = 1:length(changed)
                            if ~ismember(changed(r,:), grid.goal, 'rows') && ...
                                    isequal(cell2mat(bp(changed(r,1), changed(r,2))), [w l])
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
        if resetEps
            Eps = Eps0;
        elseif Eps - dEps > 1.0
            Eps = Eps - dEps;
        else
            Eps = 1.0;
        end
    elseif Eps - dEps > 1.0
        Eps = Eps - dEps;
    else
        Eps = 1.0;
    end
    % insert cells in incons to open
    for r=1:size(incons,1)
        insertOpen([0 0], incons(r,:));
    end
    % update keys in open
    for r=1:size(open,1)
        open(r,1:2) = key(open(r,3:4));
    end
    computePath;
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