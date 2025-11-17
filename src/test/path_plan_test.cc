#include <vector>
#include <iostream>

const int WIDTH = 100;
const int HEIGHT = 60;

std::vector<std::vector<int>> createField() {
    return std::vector<std::vector<int>>(HEIGHT, std::vector<int>(WIDTH, 0));
}

void addCircle(std::vector<std::vector<int>>& field, int cx, int cy, int radius) {
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            int dx = j - cx;
            int dy = i - cy;
            if (dx*dx + dy*dy <= radius*radius) {
                field[i][j] = 1;
            }
        }
    }
}

void printField(std::vector<std::vector<int>>& field) {
    for (auto row : field) {
        for (int cell : row) {
            if (cell == 0) {
                std::cout << ".";
            } else if (cell == 1) {
                std::cout << "#";
            }
        }
        std::cout << "\n";
    }
}

int main() {
    auto field = createField();

    addCircle(field, 50, 30, 8);

    printField(field);

    return 0;   
}