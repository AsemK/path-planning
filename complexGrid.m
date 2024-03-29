function grid = complexGrid
%complexGrid returns a 15*26 grid with multiple goals
obstacles = [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
             0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
             0 0 0 1 1 1 1 0 0 1 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0;...
             0 0 0 1 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0;...
             0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;...
             0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;...
             0 0 0 1 0 0 1 1 1 1 1 1 1 0 0 1 1 1 1 1 0 0 1 0 0 0;...
             0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0;...
             0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0;...
             0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 1 0 0 1 0 0 0;...
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;...
             0 0 1 1 1 1 1 0 0 1 1 1 1 1 1 1 0 0 0 1 0 0 1 1 1 1;...
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0;...
             0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0];
grid = createGridMap(obstacles);     
grid = defineStart(grid, [1 1]);
grid = addGoal(grid, [26 15]); 
grid = addGoal(grid, [26 1]); 