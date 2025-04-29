#include <gtest/gtest.h>

TEST(echoflow, test_add_to_radar_grid_map) 
{
    ASSERT_EQ(3, 1+2);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}