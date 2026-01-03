#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

const int GRID_W = 10;
const int GRID_H = 10;
const int CELL_SIZE = 50;

class Node {
public:
    int x = 0;
    int y = 0;
    bool walkable = true;
    Node* parent = nullptr;
    float g = INFINITY;
    cv::Scalar color = { 200, 200, 200 };
};

cv::Rect nodeRect(const Node& n) {
    return cv::Rect(n.x * CELL_SIZE, n.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
}

void drawGrid(cv::Mat& canvas, const std::vector<std::vector<Node>>& grid) {
    for (int y = 0; y < GRID_H; ++y) {
        for (int x = 0; x < GRID_W; ++x) {
            const Node& n = grid[y][x];
            cv::rectangle(canvas, nodeRect(n), n.color, cv::FILLED);
            cv::rectangle(canvas, nodeRect(n), { 50, 50, 50 }, 1);
        }
    }
}

void makeObstacle(std::vector<std::vector<Node>>& grid, int row, int col) {
    grid[row][col].walkable = false;
    grid[row][col].color = { 0, 0, 0 };
}

std::vector<Node*> reconstructPath(Node* target) {
    std::vector<Node*> path;
    Node* current = target;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void BFS(std::vector<std::vector<Node>>& grid, Node* start, Node* target) {
    std::priority_queue<std::pair<float, Node*>, std::vector<std::pair<float, Node*>>, std::greater<std::pair<float, Node*>>> openList;
    start->g = 0.0f;
    start->color = { 0, 255, 0 };
    target->color = { 255, 0, 0 };
    openList.push(std::make_pair(start->g, start));

    std::vector<std::pair<int, int>> dirs = { {0,-1},{0,1},{-1,0},{1,0},{-1,-1},{-1,1},{1,-1},{1,1} };

    while (!openList.empty()) {
        std::pair<float, Node*> currentPair = openList.top();
        openList.pop();
        Node* current = currentPair.second;
        if (current == target) return;

        for (const std::pair<int, int>& d : dirs) {
            int nx = current->x + d.first;
            int ny = current->y + d.second;
            if (nx < 0 || nx >= GRID_W || ny < 0 || ny >= GRID_H) continue;
            Node* neighbor = &grid[ny][nx];
            if (!neighbor->walkable) continue;
            float dx = static_cast<float>(d.first);
            float dy = static_cast<float>(d.second);
            float cost = std::sqrt(dx * dx + dy * dy);
            float tentativeG = current->g + cost;
            if (tentativeG < neighbor->g) {
                neighbor->g = tentativeG;
                neighbor->parent = current;
                openList.push(std::make_pair(neighbor->g, neighbor));
            }
        }
    }
}

void constructLinePath(cv::Mat& canvas, std::vector<Node*> path) {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int x1 = path[i]->x * CELL_SIZE + CELL_SIZE / 2;
        int y1 = path[i]->y * CELL_SIZE + CELL_SIZE / 2;
        int x2 = path[i + 1]->x * CELL_SIZE + CELL_SIZE / 2;
        int y2 = path[i + 1]->y * CELL_SIZE + CELL_SIZE / 2;
        cv::line(canvas, cv::Point(x1, y1), cv::Point(x2, y2), {0, 0, 0}, 3);
        
    }
}

int main() {
    std::cout << "hi" << std::endl;
    
    std::vector<std::vector<Node>> grid(GRID_H, std::vector<Node>(GRID_W));
    for (int y = 0; y < GRID_H; ++y) {
        for (int x = 0; x < GRID_W; ++x) {
            grid[y][x].x = x;
            grid[y][x].y = y;
        }
    }

    makeObstacle(grid, 2, 3);
    makeObstacle(grid, 5, 6);
    makeObstacle(grid, 7, 1);

    Node* start = &grid[0][0];
    Node* target = &grid[9][9];

    BFS(grid, start, target);

    std::vector<Node*> path = reconstructPath(target);
    for (Node* n : path) {
        if (n != start && n != target) {
            n->color = { 0, 0, 255 };
        }
    }

    std::cout << "test" << std::endl;
    std::cout.flush();
    
    cv::Mat canvas(GRID_H * CELL_SIZE, GRID_W * CELL_SIZE, CV_8UC3, { 255, 255, 255 });
    drawGrid(canvas, grid);
    constructLinePath(canvas, path);
    cv::imshow("BFS", canvas);
    cv::waitKey(0);
    return 0;
}