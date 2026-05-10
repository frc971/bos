#include <gtest/gtest.h>
#include <XAD/XAD.hpp>
#include <algorithm>
#include <utility>

double x0 = 1.0;
double x1 = 1.5;
double x2 = 1.3;
double x3 = 1.2;

template <typename T>
auto f2(const T& x0_in, const T& x1_in, const T& x2_in, const T& x3_in)
    -> std::pair<T, T> {
  T y1 = x0_in * x1_in + x2_in;
  T y2 = x0_in + x2_in * x3_in;
  return {y1, y2};
}
TEST(XadTest, Add) {  // NOLINT

  using mode = xad::fwd<double>;
  using AD = mode::active_type;

  AD x0_ad = x0;
  AD x1_ad = x1;
  AD x2_ad = x2;
  AD x3_ad = x3;

  AD sum = x0_ad + x1_ad + x2_ad + x3_ad;
  EXPECT_DOUBLE_EQ(sum.value(), x0 + x1 + x2 + x3);
}

TEST(XadTest, Vector) {  // NOLINT
  // tape and active data type for 1st order adjoint computation with 2-vector mode
  using mode = xad::adj<double, 2>;
  using tape_type = mode::tape_type;
  using AD = mode::active_type;

  // initialize tape
  tape_type tape;

  // set independent variables
  AD x0_ad = x0;
  AD x1_ad = x1;
  AD x2_ad = x2;
  AD x3_ad = x3;

  // and register them
  tape.registerInput(x0_ad);
  tape.registerInput(x1_ad);
  tape.registerInput(x2_ad);
  tape.registerInput(x3_ad);

  // start recording derivatives
  tape.newRecording();

  // run function with 2 outputs
  std::pair<AD, AD> y = f2(x0_ad, x1_ad, x2_ad, x3_ad);

  // register and seed adjoint of outputs
  tape.registerOutput(y.first);
  tape.registerOutput(y.second);
  y.first.setAdjoint({1.0, 0.0});
  y.second.setAdjoint({0.0, 1.0});

  // compute all other adjoints
  tape.computeAdjoints();

  // output results
  std::cout << "y1 = " << value(y.first) << "\n"
            << "y2 = " << value(y.second) << "\n"
            << "\nfirst order derivatives of y1:\n"
            << "dy1/dx0 = " << x0_ad.getAdjoint()[0] << "\n"
            << "dy1/dx1 = " << x1_ad.getAdjoint()[0] << "\n"
            << "dy1/dx2 = " << x2_ad.getAdjoint()[0] << "\n"
            << "dy1/dx3 = " << x3_ad.getAdjoint()[0] << "\n"
            << "\nfirst order derivatives of y2:\n"
            << "dy2/dx0 = " << x0_ad.getAdjoint()[1] << "\n"
            << "dy2/dx1 = " << x1_ad.getAdjoint()[1] << "\n"
            << "dy2/dx2 = " << x2_ad.getAdjoint()[1] << "\n"
            << "dy2/dx3 = " << x3_ad.getAdjoint()[1] << "\n";
}
