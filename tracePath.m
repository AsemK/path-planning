function newSearchInfo = tracePath(searchInfo)
%tracePath takes the searchInfo struct with back pointer (pb) and goal and
%returns the path as an array of nodes.
if searchInfo.success
    path = [searchInfo.goal];
    bp = searchInfo.bp;
    if isequal(searchInfo.goal, searchInfo.grid.start)
        while ~ismember(path(end,:), searchInfo.grid.goal, 'rows');
            path = [path; cell2mat(bp(path(end,1),path(end,2)))];
        end
    else
        while ~isequal(path(end,:), searchInfo.grid.start)
            path = [path; cell2mat(bp(path(end,1),path(end,2)))];
        end
        path = path(end:-1:1,:);
    end
else
    path = zeros(0,2);
end
searchInfo.path = path;
newSearchInfo = searchInfo;
end

