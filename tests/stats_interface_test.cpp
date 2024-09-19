//
// Created by Yinghao Qin on 19/09/2024.
//

#include "gtest/gtest.h"
#include "stats_interface.hpp"

using namespace ::testing;


// Test case for empty data
TEST(StatsInterfaceTest, CalculateStatisticalIndicators_EmptyData) {
    std::vector<double> data = {};
    Indicators result = StatsInterface::calculate_statistical_indicators(data);

    EXPECT_EQ(result.min, 0.0);  // Default value
    EXPECT_EQ(result.max, 0.0);  // Default value
    EXPECT_EQ(result.avg, 0.0);  // Default value
    EXPECT_EQ(result.std, 0.0);  // Default value
    EXPECT_EQ(result.size, 0);   // No data
}

// Test case for multiple elements
TEST(StatsInterfaceTest, CalculateStatisticalIndicators_MultipleElements) {
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    Indicators result = StatsInterface::calculate_statistical_indicators(data);

    EXPECT_DOUBLE_EQ(result.min, 1.0);
    EXPECT_DOUBLE_EQ(result.max, 5.0);
    EXPECT_DOUBLE_EQ(result.avg, 3.0);
    EXPECT_NEAR(result.std, 1.5811, 0.0001);  // Standard deviation with some precision tolerance
    EXPECT_EQ(result.size, 5);
}

TEST(StatsInterfaceTest, CreateDirectoriesIfNotExists) {
    std::string directory_path = kStatsPath + "/algorithm" + "/instance" + "/seed";
    bool result = StatsInterface::create_directories_if_not_exists(directory_path);

    EXPECT_TRUE(result);
    EXPECT_TRUE(fs::exists(directory_path));

    // Clean up
    fs::remove_all(kStatsPath);
}

TEST(StatsInterfaceTEst, StatsForMultipleTrials) {
    std::string directory_path = kStatsPath + "/algorithm" + "/instance" + "/seed";
    StatsInterface::create_directories_if_not_exists(directory_path);

    std::string file_path = kStatsPath + "/algorithm" + "/instance" + "/stats.csv";
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};

    StatsInterface::stats_for_multiple_trials(file_path, data);

    EXPECT_TRUE(fs::exists(file_path));

    // Clean up
    fs::remove_all(kStatsPath);
}