#include <gtest/gtest.h>

TEST(FailingTest, ShouldFail) {

  int a = "hi";
  int b = "pls work (fail) already";
  EXPECT_EQ(a, 5);
}