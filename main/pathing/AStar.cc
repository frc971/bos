#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

struct Vector2 {
    double x, y;
};

struct Node {
    Vector2
    double g, h;
    Node* parent;
    
    Node(int x_, int y_, double g_ = 0, double h_ = 0, Node* parent_ = nullptr)
        : x(x_), y(y_), g(g_), h(h_), parent(parent_) {}
    
    double f() const { return g + h; }
};

// Comparison for priority queue
struct CompareNode {
    bool operator()(Node* a, Node* b) { return a->f() > b->f(); }
};

// --------------------- Simple A* ---------------------
vector<Vector2> AStar(const vector<vector<int>>& grid, Vector2 start, Vector2 goal) {
    int rows = grid.size(), cols = grid[0].size();
    auto heuristic = [](Vector2 a, Vector2 b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    };
    
    priority_queue<Node*, vector<Node*>, CompareNode> openSet;
    vector<vector<bool>> closedSet(rows, vector<bool>(cols, false));
    
    Node* startNode = new Node(start.x, start.y, 0, heuristic(start, goal));
    openSet.push(startNode);

    Node* endNode = nullptr;
    
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};
    
    while (!openSet.empty()) {
        Node* current = openSet.top(); openSet.pop();
        if (closedSet[current->x][current->y]) continue;
        closedSet[current->x][current->y] = true;
        
        if (current->x == goal.x && current->y == goal.y) {
            endNode = current;
            break;
        }
        
        for (int i = 0; i < 4; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];
            if (nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue;
            if (grid[nx][ny] != 0 || closedSet[nx][ny]) continue;
            
            double gNew = current->g + 1; // cost of moving
            Node* neighbor = new Node(nx, ny, gNew, heuristic({nx, ny}, goal), current);
            openSet.push(neighbor);
        }
    }
    
    // Reconstruct path
    vector<Vector2> path;
    while (endNode != nullptr) {
        path.push_back({(double)endNode->x, (double)endNode->y});
        endNode = endNode->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

