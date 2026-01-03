#include <algorithm>
#include <cstdlib>
#include <functional>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

typedef struct Edge {
  int x1;
  int y1;
  int x2;
  int y2;
  double weight;
} edge_t;

typedef struct Node {
  cv::Vec3b color;
  double magnitude;
  double magnitude_high;
  double magnitude_low;
  double x_grad;
  double y_grad;
  double x_grad1;
  double y_grad1;
  double x_grad2;
  double y_grad2;
  uint children;
  Node* parent;
} node_t;

Node& GetParent(Node& curr) {
  if (curr.parent == nullptr) {
    return curr;
  }
  return GetParent(*curr.parent);
}

inline int get_value(unsigned char* buffer, uint x, uint y, uint step) {
  return static_cast<int>(buffer[y * step + x]);
}

cv::Vec3b GenerateRandomColor() {
  // Generate random values between 0 and 255 for Blue, Green, and Red channels
  int blue = std::rand() % 256;   // Random value for Blue channel (0 to 255)
  int green = std::rand() % 256;  // Random value for Green channel (0 to 255)
  int red = std::rand() % 256;    // Random value for Red channel (0 to 255)

  // Return a Scalar representing a color in BGR format
  return cv::Vec3b(blue, green, red);
}

int main() {
  cv::Mat image = cv::imread("apriltag.png");

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // unsigned char* buffer = gray.data;
  // size_t step = gray.step;

  cv::Mat grad_x, grad_y;
  Sobel(gray, grad_x, CV_64F, 1, 0, 3);
  Sobel(gray, grad_y, CV_64F, 0, 1, 3);

  uint edges_length = (gray.rows - 1) * (gray.cols - 1) * 2;
  edge_t* edges = static_cast<edge_t*>(malloc(sizeof(edge_t) * edges_length));

  uint matrix_length = (gray.rows) * (gray.cols);
  node_t* graph = static_cast<node_t*>(malloc(sizeof(node_t) * matrix_length));

  for (int i = 0; i < gray.rows; ++i) {
    for (int j = 0; j < gray.cols; ++j) {
      const double norm =
          std::hypot(grad_x.at<double>(i, j), grad_y.at<double>(i, j));
      if (norm != 0) {
        grad_x.at<double>(i, j) /= norm;
        grad_y.at<double>(i, j) /= norm;
      }

      graph[i * gray.cols + j] = node_t{.color = GenerateRandomColor(),
                                        .magnitude = norm,
                                        .magnitude_high = norm,
                                        .magnitude_low = norm,
                                        .x_grad = grad_x.at<double>(i, j),
                                        .y_grad = grad_y.at<double>(i, j),
                                        .x_grad1 = grad_x.at<double>(i, j),
                                        .y_grad1 = grad_y.at<double>(i, j),
                                        .x_grad2 = grad_x.at<double>(i, j),
                                        .y_grad2 = grad_y.at<double>(i, j),
                                        .children = 1,
                                        .parent = nullptr};
    }
  }

  for (int i = 0; i < gray.rows - 1; ++i) {
    for (int j = 0; j < gray.cols - 1; ++j) {
      // Dot product
      // higher = more simmilar
      {
        const double weight =
            grad_x.at<double>(i, j) * grad_x.at<double>(i, j + 1) +
            grad_y.at<double>(i, j) * grad_y.at<double>(i, j + 1);
        edges[2 * (i * gray.cols + j)] =
            edge_t{.x1 = i, .y1 = j, .x2 = i, .y2 = j + 1, .weight = weight};
      }
      {
        const double weight =
            grad_x.at<double>(i, j) * grad_x.at<double>(i + 1, j) +
            grad_y.at<double>(i, j) * grad_y.at<double>(i + 1, j);

        edges[2 * (i * gray.cols + j) + 1] =
            edge_t{.x1 = i, .y1 = j, .x2 = i + 1, .y2 = j, .weight = weight};
      }
    }
  }

  std::sort(edges, edges + edges_length,
            [](edge_t a, edge_t b) { return a.weight > b.weight; });

  const double kmagnitude = 120000000;
  const double kgradient = 10;
  for (size_t i = 0; i < edges_length; ++i) {
    node_t& node1 = graph[edges[i].x1 * gray.cols + edges[i].y1];
    node_t& node2 = graph[edges[i].x2 * gray.cols + edges[i].y2];

    node_t& node1_parent = GetParent(node1);
    node_t& node2_parent = GetParent(node2);
    if (&node1_parent == &node2_parent) {
      continue;
    }

    const double curr_magnitude_cost =
        std::min(node1_parent.magnitude_high - node1_parent.magnitude_low,
                 node1_parent.magnitude_high - node1_parent.magnitude_low) +
        kmagnitude / (node1_parent.children + node2_parent.children);

    const double new_magnitude_cost =
        std::max(node1_parent.magnitude_high, node2_parent.magnitude_high) -
        std::min(node1_parent.magnitude_low, node2_parent.magnitude_low);

    if (new_magnitude_cost >= curr_magnitude_cost) {
      continue;
    }

    const double curr_gradient_cost =
        -std::max(node1_parent.x_grad1 * node1_parent.x_grad2 +
                      node1_parent.y_grad1 * node1_parent.y_grad2,
                  node2_parent.x_grad1 * node2_parent.x_grad2 +
                      node2_parent.y_grad1 * node2_parent.y_grad2) +
        kgradient / (node1_parent.children + node2_parent.children);

    double x_grad1 = node1_parent.x_grad1;
    double x_grad2 = node2_parent.x_grad1;
    double y_grad1 = node1_parent.y_grad1;
    double y_grad2 = node2_parent.y_grad1;
    //node1grad1 + node2grad1
    double new_gradient_cost = node1_parent.x_grad1 * node2_parent.x_grad1 +
                               node1_parent.y_grad1 * node2_parent.y_grad1;

    //node1grad1 + node2grad2
    const double new_gradient_cost2 =
        node1_parent.x_grad1 * node2_parent.x_grad2 +
        node1_parent.y_grad1 * node2_parent.y_grad2;
    if (new_gradient_cost2 < new_gradient_cost) {
      new_gradient_cost = new_gradient_cost2;
      x_grad1 = node1_parent.x_grad1;
      x_grad2 = node2_parent.x_grad2;
      y_grad1 = node1_parent.y_grad1;
      y_grad2 = node2_parent.y_grad2;
    }

    //node1grad2 + node2grad1
    const double new_gradient_cost3 =
        node1_parent.x_grad2 * node2_parent.x_grad1 +
        node1_parent.y_grad2 * node2_parent.y_grad1;
    if (new_gradient_cost3 < new_gradient_cost) {
      new_gradient_cost = new_gradient_cost3;
      x_grad1 = node1_parent.x_grad2;
      y_grad1 = node1_parent.y_grad2;
      y_grad2 = node2_parent.y_grad1;
      x_grad2 = node2_parent.x_grad1;
    }

    //node1grad2 + node2grad2
    const double new_gradient_cost4 =
        node1_parent.x_grad2 * node2_parent.x_grad2 +
        node1_parent.y_grad2 * node2_parent.y_grad2;
    if (new_gradient_cost4 < new_gradient_cost) {
      new_gradient_cost = new_gradient_cost4;
      x_grad1 = node1_parent.x_grad2;
      x_grad2 = node2_parent.x_grad2;
      y_grad1 = node1_parent.y_grad2;
      y_grad2 = node2_parent.y_grad2;
    }

    new_gradient_cost = -new_gradient_cost;

    if (new_magnitude_cost >= curr_magnitude_cost) {
      std::cout << new_magnitude_cost << " " << curr_magnitude_cost << "\n";
      std::cout << "failed magnitude check\n";
      continue;
    }
    if (new_gradient_cost >= curr_gradient_cost) {
      std::cout << new_gradient_cost << " " << curr_gradient_cost << "\n";
      std::cout << "failed gradient check\n";
      continue;
    }

    //join
    node1_parent.parent = &node2_parent;

    node2_parent.magnitude_high =
        std::max(node2_parent.magnitude_high, node1_parent.magnitude_high);
    node2_parent.magnitude_low =
        std::min(node2_parent.magnitude_low, node1_parent.magnitude_low);
    node2_parent.x_grad1 = x_grad1;
    node2_parent.y_grad1 = y_grad1;
    node2_parent.x_grad2 = x_grad2;
    node2_parent.y_grad2 = y_grad2;
    node2_parent.children += node1_parent.children;

    std::cout << node2_parent.children << "\n";
    std::cout << curr_magnitude_cost << "\n";
    std::cout << new_magnitude_cost << "\n";
    std::cout << curr_gradient_cost << "\n";
    std::cout << new_gradient_cost << "\n";
    std::cout << "----------------\n";
  }

  cv::Mat union_image = image.clone();
  for (int i = 0; i < union_image.rows; ++i) {
    for (int j = 0; j < union_image.cols; ++j) {
      union_image.at<cv::Vec3b>(i, j) =
          GetParent(graph[i * union_image.cols + j]).color;
    }
  }

  free(edges);
  free(graph);

  cv::imshow("image", image);
  cv::imshow("gray", gray);
  cv::imshow("grad_x", grad_x);
  cv::imshow("grad_y", grad_y);
  cv::imshow("union_image", union_image);
  cv::waitKey(0);
}
