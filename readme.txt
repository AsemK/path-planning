Planning and Replanning in Dynamic Configuration Spaces
A MATLAB simulation package
by Asem Khattab 
(a.a.m.f.khattab@student.utwente.nl)
written in DLR, Braunschweig, Germany
September 2017

Inclludded Files:
- grid formation functions
  + createEmptyGrid
  + createGridMap
- grid modification functions
  + defineStart
  + addGoal
  + addObstacle
  + removeGoal
  + removeObstacle
- grid verification functions
  + isFree
  + isObstacle
- grids
  + paperGrid
  + complexGrid
  + largeGrid
- search functions
  + AStar
  + AStarBack
  + ARAStarBack
  + DStarLite
  + DStarLite2
  + ADStar
  + ADStar2
  + tracePath
- 8-connected grid functions
  + neighbors8
  + cost8
- heuristics
  + hDiagonalDistance
- graphing and animation functions
  + plotGridPath
  + planMove
- executable test scripts
  + paperGridTests
  + complexGridTests
  + largeGridTests