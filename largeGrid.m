function grid = largeGrid
%largGrid returns a larg grid containing 5445 cells
%   the grid was originaly created by the maze generator shown here:
%   https://rosettacode.org/wiki/Maze_generation#MATLAB_.2F_Octave
%   it was then edited to allow multiple paths to the goals.
M = load('largeGrid.txt');
grid = createGridMap(M);
grid = defineStart(grid, [1 1]);
grid = addGoal(grid, [99 55]);
grid = addGoal(grid, [95 1 ]); 
grid = addGoal(grid, [85 45]); 
grid = addGoal(grid, [85 27]); 
end
