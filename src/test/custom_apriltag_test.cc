#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "src/utils/log.h"

using edge_t = struct Edge {
  int x1 = -1;
  int y1 = -1;
  int x2 = -1;
  int y2 = -1;
  double weight;
};

using node_t = struct Node {
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
  double x_start;
  double x_end;

  double x;
  double y;
  double x_sum;
  double y_sum;
  double x_grad_sum;
  double y_grad_sum;

  double y_start = -1;
  double y_end = -1;
  double slope = -1;
  double bias = -1;

  double top = 0;
  double bottom = 0;

  uint children;
  Node* parent = nullptr;
  bool is_quad = false;
};

using box_t = struct Box {
  Node* segment1;
  Node* segment2;
  Node* segment3;
  Node* segment4;
};

using point_t = struct Point {
  double x;
  double y;
};

using vector_t = struct Vector {
  double x;
  double y;
};

auto GetPoint(const node_t& segment1, const node_t& segment2) -> point_t {
  const double x =
      (segment2.bias - segment1.bias) / (segment1.slope - segment2.slope);
  const double y = segment1.slope * x + segment1.bias;
  return {x, y};
}

auto DrawPoint(const point_t& point, const cv::Mat& image) -> void {
  cv::circle(image,
             cv::Point(static_cast<uint>(point.y), static_cast<uint>(point.x)),
             1, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
}

auto IsNeighbor(node_t* a, node_t* b) -> bool {
  const double squared_distance =
      (a->x_end - b->x_start) * (a->x_end - b->x_start) +
      (a->y_end - b->y_start) * (a->y_end - b->y_start);
  if (squared_distance > 50) {
    return false;
  }
  return true;
}

auto FindQuads(node_t* curr, std::vector<node_t*> quad,
               const std::vector<node_t*>& segments) -> std::vector<node_t*> {
  quad.push_back(curr);
  if (quad.size() == 4) {
    return IsNeighbor(curr, quad[0]) ? quad : std::vector<node_t*>{};
  }
  for (size_t i = 0; i < segments.size(); i++) {
    if (!segments[i]->is_quad && IsNeighbor(curr, segments[i])) {
      std::vector<node_t*> result = FindQuads(segments[i], quad, segments);
      if (result.size() == 4) {
        return result;
      }
    }
  }
  return {};
}

auto GetParent(node_t& curr) -> node_t& {
  if (curr.parent == nullptr) {
    return curr;
  }
  return GetParent(*curr.parent);
}

inline auto get_value(unsigned char* buffer, uint x, uint y, uint step) -> int {
  return static_cast<int>(buffer[y * step + x]);
}

auto GenerateRandomColor() -> cv::Vec3b {
  // Generate random values between 0 and 255 for Blue, Green, and Red channels
  int blue = std::rand() % 256;   // Random value for Blue channel (0 to 255)
  int green = std::rand() % 256;  // Random value for Green channel (0 to 255)
  int red = std::rand() % 256;    // Random value for Red channel (0 to 255)

  // Return a Scalar representing a color in BGR format
  return {static_cast<unsigned char>(blue), static_cast<unsigned char>(green),
          static_cast<unsigned char>(red)};
}

auto DrawLine(node_t& node, cv::Mat& mat) -> void {
  cv::line(
      mat,
      cv::Point(static_cast<int>(node.y_start), static_cast<int>(node.x_start)),
      cv::Point(static_cast<int>(node.y_end), static_cast<int>(node.x_end)),
      cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

  cv::circle(mat, cv::Point(node.y_start, node.x_start), 3,
             cv::Scalar(255, 0, 0), -1, cv::LINE_AA);

  cv::circle(mat, cv::Point(node.y_end, node.x_end), 3, cv::Scalar(0, 255, 0),
             -1, cv::LINE_AA);
}

auto main() -> int {
  cv::Mat image = cv::imread("apriltag2.png");
  cv::GaussianBlur(image, image, cv::Size(0, 0), 0.8);

  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  // unsigned char* buffer = gray.data;
  // size_t step = gray.step;

  cv::Mat grad_x, grad_y;
  Sobel(gray, grad_x, CV_64F, 1, 0, 3);
  Sobel(gray, grad_y, CV_64F, 0, 1, 3);

  size_t edges_length = (gray.rows - 1) * (gray.cols - 1) * 2;
  auto* edges = new edge_t[edges_length];

  size_t graph_length = gray.rows * gray.cols;
  auto* graph = new node_t[graph_length];

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
                                        .x_start = static_cast<double>(i),
                                        .x_end = static_cast<double>(i),
                                        .x = static_cast<double>(i),
                                        .y = static_cast<double>(j),
                                        .x_sum = static_cast<double>(i),
                                        .y_sum = static_cast<double>(j),
                                        .x_grad_sum = grad_x.at<double>(i, j),
                                        .y_grad_sum = grad_y.at<double>(i, j),
                                        .children = 1,
                                        .parent = nullptr};
    }
  }

  int idx = 0;
  for (int i = 0; i < gray.rows - 1; ++i) {
    for (int j = 0; j < gray.cols - 1; ++j) {
      assert(static_cast<size_t>(i * gray.cols + j) < graph_length);
      // Dot product
      // higher = more simmilar
      {
        const double weight =
            grad_x.at<double>(i, j) * grad_x.at<double>(i, j + 1) +
            grad_y.at<double>(i, j) * grad_y.at<double>(i, j + 1);
        edges[idx] =
            edge_t{.x1 = i, .y1 = j, .x2 = i, .y2 = j + 1, .weight = weight};
        idx++;
      }
      {
        const double weight =
            grad_x.at<double>(i, j) * grad_x.at<double>(i + 1, j) +
            grad_y.at<double>(i, j) * grad_y.at<double>(i + 1, j);

        edges[idx] =
            edge_t{.x1 = i, .y1 = j, .x2 = i + 1, .y2 = j, .weight = weight};
        idx++;
      }
    }
  }
  assert(static_cast<size_t>(idx) == edges_length);

  std::sort(edges, edges + edges_length,
            [](edge_t a, edge_t b) { return a.weight > b.weight; });

  const double kmagnitude = 12000000;
  const double kgradient = 20;
  for (size_t i = 0; i < edges_length; ++i) {
    CHECK(static_cast<size_t>(edges[i].x1 * gray.cols + edges[i].y1) <
          graph_length)
        << edges[i].x1 << " " << edges[i].y1 << " " << i << std::endl;
    CHECK(static_cast<size_t>(edges[i].x2 * gray.cols + edges[i].y2) <
          graph_length);
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
      continue;
    }
    if (new_gradient_cost >= curr_gradient_cost) {
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
    node2_parent.x_sum += node1_parent.x_sum;
    node2_parent.y_sum += node1_parent.y_sum;
    node2_parent.x_grad_sum += node1_parent.x_grad_sum;
    node2_parent.y_grad_sum += node1_parent.y_grad_sum;
    node2_parent.x_start = std::min(node2_parent.x_start, node1_parent.x_start);
    node2_parent.x_end = std::max(node2_parent.x_end, node1_parent.x_end);
    node2_parent.children += node1_parent.children;
  }

  cv::Mat magnitude;
  cv::magnitude(grad_x, grad_y, magnitude);
  magnitude /= 2;

  cv::Mat union_image = image.clone();
  for (int i = 0; i < union_image.rows; ++i) {
    for (int j = 0; j < union_image.cols; ++j) {
      node_t& parent = GetParent(graph[i * union_image.cols + j]);
      if (parent.parent == nullptr && parent.children > 100) {
        union_image.at<cv::Vec3b>(i, j) = parent.color;
      }
    }
  }

  // Fit line segments
  std::vector<node_t*> segments;
  for (size_t i = 0; i < graph_length; ++i) {
    node_t& node = graph[i];
    node_t& parent = GetParent(node);
    parent.top += (node.x - parent.x_sum / parent.children) *
                  (node.y - parent.y_sum / parent.children);
    parent.bottom += (node.x - (parent.x_sum / parent.children)) *
                     (node.x - (parent.x_sum - parent.children));
    if (node.parent == nullptr && node.children > 100) {
      segments.push_back(&node);
    }
  }

  for (auto& segment : segments) {
    segment->slope = segment->bottom == 0 ? 0 : segment->top / segment->bottom;
    // segment->slope = segment->top / segment->bottom;
    const double x_bar = segment->x_sum / segment->children;
    const double y_bar = segment->y_sum / segment->children;
    segment->bias = y_bar - segment->slope * x_bar;
    segment->y_start = segment->bias + segment->slope * segment->x_start;
    segment->y_end = segment->bias + segment->slope * segment->x_end;
    if (segment->x_grad_sum < 0) {
      std::swap(segment->x_start, segment->x_end);
      std::swap(segment->y_start, segment->y_end);
    }
  }

  cv::Mat segment_image = image.clone();
  for (auto& segment : segments) {
    cv::line(segment_image,
             cv::Point(static_cast<int>(segment->y_start),
                       static_cast<int>(segment->x_start)),
             cv::Point(static_cast<int>(segment->y_end),
                       static_cast<int>(segment->x_end)),
             cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    cv::circle(segment_image, cv::Point(segment->y_start, segment->x_start), 3,
               cv::Scalar(255, 0, 0), -1, cv::LINE_AA);

    cv::circle(segment_image, cv::Point(segment->y_end, segment->x_end), 3,
               cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
  }

  cv::Mat quad_image = image.clone();
  std::vector<box_t> boxes;
  for (size_t i = 0; i < segments.size(); i++) {
    if (segments[i]->is_quad) {
      continue;
    }
    std::vector<node_t*> quad = FindQuads(segments[i], {}, segments);
    if (quad.size() == 4) {
      boxes.push_back(box_t{quad[0], quad[1], quad[2], quad[3]});
      for (auto& i : quad) {
        i->is_quad = true;
      }
    }
  }

  LOG(INFO) << boxes.size();

  for (auto& box : boxes) {
    DrawLine(*box.segment1, quad_image);
    DrawLine(*box.segment2, quad_image);
    DrawLine(*box.segment3, quad_image);
    DrawLine(*box.segment4, quad_image);
  }

  cv::Mat decode_image = image.clone();
  const uint ktag_pixels = 8;
  for (auto& box : boxes) {
    point_t p0 = GetPoint(*box.segment1, *box.segment2);
    point_t p1 = GetPoint(*box.segment2, *box.segment3);
    point_t p2 = GetPoint(*box.segment3, *box.segment4);
    point_t p3 = GetPoint(*box.segment3, *box.segment4);
    DrawPoint(p0, decode_image);
    DrawPoint(p1, decode_image);
    DrawPoint(p2, decode_image);
    DrawPoint(p3, decode_image);
    vector_t i_vector{.x = (p2.x - p1.x) / ktag_pixels,
                      .y = (p2.y - p1.y) / ktag_pixels};
    vector_t j_vector{
        .x = (p1.x - p0.x) / ktag_pixels,
        .y = (p1.y - p0.y) / ktag_pixels,
    };
    for (int i = 0; i < ktag_pixels; ++i) {
      for (int j = 0; j < ktag_pixels; ++j) {
        double x = p1.x + ((i + 0.5) * i_vector.x - (j + 0.5) * j_vector.x);
        double y = p1.y + ((i + 0.5) * i_vector.y - (j + 0.5) * j_vector.y);
        cv::circle(decode_image,
                   cv::Point(static_cast<uint>(y), static_cast<uint>(x)), 1,
                   cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
      }
    }
    break;
  }
  cv::imshow("grad_y", grad_y);
  cv::imshow("magnitude", magnitude);
  cv::imshow("union_image", union_image);
  cv::imshow("segment_image", segment_image);
  cv::imshow("quad_image", quad_image);
  cv::imshow("decode_image", decode_image);

  cv::imwrite("union_image.png", union_image);
  cv::imwrite("segment_image.png", segment_image);

  delete[] edges;
  delete[] graph;

  cv::waitKey(0);
}
