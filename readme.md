# Planning and Replanning in Dynamic Configuration Spaces

A MATLAB simulation package for various path planning techniques to show their behavior when the environment changes.

<p align="center">
  <img src="https://user-images.githubusercontent.com/37188590/153775518-63a25287-5da9-41e3-90a4-f8a08785c162.png">
</p>

Used in the research paper [Static and Dynamic Path Planning Using Incremental Heuristic Search](https://arxiv.org/abs/1804.07276).

written in DLR, Braunschweig, Germany

## Included Files:

### grid formation functions

- createEmptyGrid
- createGridMap

### grid modification functions

- defineStart
- addGoal
- addObstacle
- removeGoal
- removeObstacle

### grid verification functions

- isFree
- isObstacle

### grids

- paperGrid
- complexGrid
- largeGrid

### search functions

- AStar
- AStarBack
- ARAStarBack
- DStarLite
- DStarLite2
- ADStar
- ADStar2
- tracePath

### 8-connected grid functions

- neighbors8
- cost8

### heuristics

- hDiagonalDistance

### graphing and animation functions

- plotGridPath
- planMove

### executable test scripts

- paperGridTests
- complexGridTests
- largeGridTests
