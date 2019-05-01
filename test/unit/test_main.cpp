#include <gtest/gtest.h>


// Use a bespoke main() to set a few flags for Gtest
int main(int argc, char** argv) {
  ::testing::InitGoogleTest( &argc, argv);

  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  return RUN_ALL_TESTS();
}
