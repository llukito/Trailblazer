/*
 * File: Trailblazer.cpp
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include <random.h>

using namespace std;

/*
* Helper function for shortestPath, which is for adding neighbours.
* We had to pass almost every element, but i think it is worth it
* cause it makes is better to read and better to debug
*/
void addNeighbours(Loc curr, Loc end, Grid<double>& world, Set<Loc>& green, Map<Loc, double>& dist,
    double costFn(Loc from, Loc to, Grid<double>& world), 
    Map<Loc, Loc>& parents, TrailblazerPQueue<Loc>& pq, 
    double heuristic(Loc start, Loc end, Grid<double>& world)) {

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0)continue; // we don't stay at same Loc
            int newRow = curr.row + i;
            int newCol = curr.col + j;
            if (!world.inBounds(newRow, newCol))continue;
            Loc newNode = makeLoc(newRow, newCol);
            if (green.contains(newNode)) { // we can't find shorter routes for green ones
                continue;
            }
            double newCost = dist[curr] + costFn(curr, newNode, world);
            if (!dist.containsKey(newNode)) { // node is gray
                colorCell(world, newNode, YELLOW);
                dist[newNode] = newCost;
                parents[newNode] = curr;
                pq.enqueue(newNode, newCost + heuristic(newNode, end, world));
            }
            else if (dist[newNode] > newCost) {
                dist[newNode] = newCost;
                parents[newNode] = curr;
                pq.decreaseKey(newNode, newCost + heuristic(newNode, end, world));
            }
        }
    }
}

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.
 */
Vector<Loc>
shortestPath(Loc start,
    Loc end,
    Grid<double>& world,
    double costFn(Loc from, Loc to, Grid<double>& world),
    double heuristic(Loc start, Loc end, Grid<double>& world)) {

    // check if start or end are valid nodes 
    if (!world.inBounds(start.row, start.col) ||
        !world.inBounds(end.row, end.col)) {
        error("Start or end nodes are out of bounds");
    }

    // our priority queue
    TrailblazerPQueue<Loc> pq;
    pq.enqueue(start, heuristic(start, end, world));

    // will remember shortests routes in dist
    Map<Loc, double> dist;
    dist[start] = 0;

    // will remmeber parents of nodes
    Map<Loc, Loc> parents;

    // will store already shortest path nodes
    Set<Loc> green;

    colorCell(world, start, YELLOW);

    while (!pq.isEmpty()) {
        Loc curr = pq.dequeueMin();
        colorCell(world, curr, GREEN);
        green.add(curr);
        if (curr == end) {
            break;
        }
        addNeighbours(curr, end, world, green, dist, costFn, parents, pq, heuristic);
    }

    // if did not reach it
    if (!green.contains(end)) {
        error("Nodes can't be reached");
    }
    // reconstruct path
    Vector<Loc> result;
    Loc current = end;
    while (true) {
        if (current == start)break;
        result.push_back(current);
        current = parents[current];
    }
    result.push_back(start);
    reverse(result.begin(), result.end());
    return result;
}

/*
* This function searches for root, like starting
* parent LUCA :)
*/
Loc find(Map<Loc, Loc>& parent, Loc x) {
    if (parent[x] != x) {
        parent[x] = find(parent, parent[x]);
    }
    return parent[x];
}

/*
* This function merges two clusters if they are not
* in the same one already. We use ranks to make sure
* we do less job
*/
void unionSets(Map<Loc, Loc>& parent, Map<Loc, int>& rank, Loc a, Loc b) {
    Loc rootA = find(parent, a);
    Loc rootB = find(parent, b);
    if (rootA != rootB) {
        if (rank[rootA] < rank[rootB]) {
            parent[rootA] = rootB;
        }
        else if (rank[rootA] > rank[rootB]) {
            parent[rootB] = rootA;
        }
        else {
            parent[rootB] = rootA;
            rank[rootA]++;
        }
    }
}

Set<Edge> createMaze(int numRows, int numCols) {
    Map<Loc, Loc> parent;
    Map<Loc, int> rank;
    int totalCells = numRows * numCols;

    TrailblazerPQueue<Edge> pq;
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            Loc curr = makeLoc(r, c);
            parent[curr] = curr;
            rank[curr] = 0;
            if (r + 1 < numRows) {
                pq.enqueue(makeEdge(curr, makeLoc(r + 1, c)), randomReal(0, 1));
            }
            if (c + 1 < numCols) {
                pq.enqueue(makeEdge(curr, makeLoc(r, c + 1)), randomReal(0, 1));
            }
        }
    }

    Set<Edge> result;
    // second check ensures early exit cause tree with n nodes will
    // always have n-1 edges. Once we get that, rest edges are unnecessary
    while (!pq.isEmpty() && result.size() < totalCells - 1) {
        Edge e = pq.dequeueMin();
        if (find(parent, e.start) != find(parent, e.end)) {
            unionSets(parent,rank, e.start, e.end);
            result.add(e);
        }
    }

    return result;
}
