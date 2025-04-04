
# ME5413 Homework 3: Planning

> Autonomous Mobile Robotics · AY2024/25 Sem 2  
> Author: Yvelle 

---

## Introduction

This project is the implementation of **Homework 3** for ME5413: Autonomous Mobile Robotics. The main objective is to explore and compare various **global path planning algorithms** on a real-world-like grid map (VivoCity Level 2), with a **human footprint constraint (≥0.3m radius)**.

---

##  Directory Structure

```
.
├── homework3.ipynb                 # Main notebook with all Task 1 & 2 logic
├── Homework_3_Planning.pdf         # Report file
├── README.md                       # Project overview and instructions
├── result/                         # Output folder for path images and summaries
│   ├── astar_heuristics/
│   ├── task1/
│   │   └── greedy_bfs/
│   ├── hybrid_astar/
│   ├── rrt/
│   ├── rrt_star/
│   └── task2/
│       └── tsp/
├── third_party/
│   └── PathPlanning/              # Reference algorithms from PythonRobotics
│       ├── AStar/
│       ├── Dijkstra/
│       ├── GreedyBestFirstSearch/
│       ├── RRT/
│       ├── RRTStar/
│       ├── HybridAStar/
│       ├── DubinsPath/, ReedsSheppPath/
│       ├── PotentialFieldPlanning/
│       └── ...                    # Many more planning algorithms

```

> `third_party/PathPlanning` is based on the [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) project for reference and experimentation purposes.

---

## Task 1: Global Planning

Implemented and tested the following planning methods between **all 5 key locations** on the map:

- - [x]  A* (with 8-connected neighbors)
- - [x] Dijkstra (A* with zero heuristic)
- - [x] Greedy Best-First Search
- - [x] Hybrid A*
- - [x] RRT and RRT\*

All algorithms are footprint-aware (using kernel size matching ≥0.3m), and results are visualized.

### Output

- Per-pair planned path overlays
- Distance matrices (in meters)
- Number of visited cells
- Runtime statistics
- Summary plots saved under:
  - `result/astar_heuristics/...`
  - `result/task1/greedy_bfs/...`
  - `result/rrt/`, `result/hybrid_astar/`, etc.

---

## Task 2: Travelling Shopper Problem (TSP)

Using the **distance table from Task 1**, we solved the shortest complete visiting tour (return to start) using:

- - [x] Brute-force (Exhaustive TSP)
- - [x] Nearest Neighbor Heuristic
- - [x] Dynamic Programming (Held-Karp)
- - [x] NetworkX TSP Approximation

### Visualization

- Actual path segments are rendered using A* computed paths (not straight lines)
- Final tour is shown with `result/task2/tsp/*.png` images
- Summary includes total distance and visual route comparison

---

## How to Run

1. Launch Jupyter:
   ```bash
   jupyter notebook homework3.ipynb
```

2. Step through cells in order (Task 1 first, then Task 2).

3. Ensure dependencies:
```
pip install numpy matplotlib pandas networkx scipy imageio
```

---

## Notes

- All grid maps are 1000x1000 grayscale images, with `255 = free`, `0 = occupied`.
    
- Each cell = `0.2m x 0.2m`; hence distances are scaled accordingly.
    
- A footprint-aware safety kernel is applied in all planning methods.
    

---

## References

- [PythonRobotics by Atsushi Sakai](https://github.com/AtsushiSakai/PythonRobotics) (used in `third_party/PathPlanning/`)
    
- [NetworkX TSP Solver](https://networkx.org/)
    
- NUS ME5413 Homework 3 Assignment
    

---
