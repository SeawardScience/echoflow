#include <gtest/gtest.h>
#include "grid_map_filters.hpp"

TEST(echoflow, test_compute_sequential_mean)
{
    ASSERT_EQ(3, 1+2);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
