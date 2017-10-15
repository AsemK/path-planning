function h = hDiagonalDistance(start, goal)
%hMaximumDistance returns the maximum of x and y distances between start
%and goal nodes. works for single and multiple goals
%   see http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
D1 = 1;       %straight distance
D2 = 1.4142;  %diagonal distance (sqrt(2))
dx = abs(start(1) - goal(:,1));
dy = abs(start(2) - goal(:,2));
h  = min(dx + dy + (D2 - 2*D1).*min(dx,dy));
end

