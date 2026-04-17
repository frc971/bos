#include <gtest/gtest.h>

TEST(FailingTest, ShouldFail) {

  int a = "hi";
  EXPECT_EQ(a, 5);
  EXPECT_EQ(1, 2);
}