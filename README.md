# Trailblazer

Trailblazer is a C++ graphical application that implements three foundational graph algorithms:

- **Dijkstra's Algorithm** for shortest paths
- **A* Search Algorithm** for efficient pathfinding
- **Kruskal's Algorithm** for maze generation using minimum spanning trees

## 🌄 Features

- Generate random terrains (small to huge)
- Visualize elevation-based maps with interactive pathfinding
- Run A* or Dijkstra between two points
- View algorithm states in real-time (visited, frontier, and final path)
- Generate and explore random mazes using Kruskal's algorithm
- Load custom world files for testing

## 🧠 Algorithms Implemented

### Dijkstra's Algorithm
Finds the shortest path between two points using a priority queue and uniform cost search.

### A* Search
Optimized shortest path search using heuristics (estimated distance to goal).

### Kruskal's Algorithm
Generates random mazes via minimum spanning trees using disjoint-set (union-find).

## 🎮 Usage

- Click two points to find paths.
- Choose algorithm from dropdown.
- Generate random terrain or maze.
- View final cost and color-coded search areas.

## 🛠 Tech Stack

- **Language**: C++
- **GUI**: Stanford C++ Graphics Libraries

## 📁 Folder Structure

- `Trailblazer.cpp` – Main logic for A*, Dijkstra, and Kruskal
- `GUI/` – Graphical components
- `res/` – Terrain/world files
