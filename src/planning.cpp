#include "planning.h"
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>

using namespace std;

struct Node {
  int x, y;
  double g, h;
  Node *parent;
};

struct Compare {
  bool operator()(Node *a, Node *b) {
    return (a->g + a->h) > (b->g + b->h);
  }
};

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return hypot(x2 - x1, y2 - y1);
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  priority_queue<Node *, vector<Node *>, Compare> open;
  map<pair<int, int>, double> gscore;
  map<pair<int, int>, Node *> allNodes;

  Node *startNode = new Node{start.first, start.second, 0.0,
                             heuristic(start.first, start.second, goal.first,
                                       goal.second),
                             nullptr};
  open.push(startNode);
  gscore[start] = 0.0;
  allNodes[start] = startNode;

  vector<pair<int, int>> directions = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                       {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    Node *curr = open.top();
    open.pop();

    if (curr->x == goal.first && curr->y == goal.second) {
      vector<pair<int, int>> path;
      while (curr) {
        path.push_back({curr->x, curr->y});
        curr = curr->parent;
      }
      reverse(path.begin(), path.end());
      return path;
    }

    for (auto d : directions) {
      int nx = curr->x + d.first;
      int ny = curr->y + d.second;

      if (!isvalid(nx, ny))
        continue;

      double tentative_g = curr->g + heuristic(curr->x, curr->y, nx, ny);
      if (!gscore.count({nx, ny}) || tentative_g < gscore[{nx, ny}]) {
        Node *neighbor = new Node{nx, ny, tentative_g,
                                  heuristic(nx, ny, goal.first, goal.second),
                                  curr};
        open.push(neighbor);
        gscore[{nx, ny}] = tentative_g;
        allNodes[{nx, ny}] = neighbor;
      }
    }
  }

  return {}; // no path found
}
