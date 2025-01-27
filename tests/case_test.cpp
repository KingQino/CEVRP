//
// Created by Yinghao Qin on 13/09/2024.
//

#include "gtest/gtest.h"
#include "case.hpp"

using namespace ::testing;

class CaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(1, file_name_);
    }

    void TearDown() override {
        delete instance;
    }

    Case* instance{};
};

TEST_F(CaseTest, Init) {
    vector<int> customerOne2CustomerListNearToFar = {2, 6, 5, 3, 7, 4, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};

    SCOPED_TRACE("Case init...");
    EXPECT_EQ(instance->customer_ids_.size(), 21);
    EXPECT_EQ(instance->station_ids_.size(), 8);
    EXPECT_EQ(instance->sorted_nearby_customers[1][0], 2);
    EXPECT_EQ(instance->sorted_nearby_customers[1][1], 6);
    EXPECT_EQ(instance->sorted_nearby_customers[1][2], 5);
    EXPECT_EQ(instance->sorted_nearby_customers[1][instance->num_customer_ - 2], 21);
    EXPECT_EQ(instance->best_station_[1][13], 24);
    EXPECT_EQ(instance->best_station_[0][21], 22);
    EXPECT_EQ(instance->best_station_[5][6], 29);
    EXPECT_EQ(instance->max_evals_, 750000);
    EXPECT_EQ(instance->get_customer_demand_(2), 700);
    EXPECT_TRUE(instance->is_charging_station(0));
    EXPECT_TRUE(instance->is_charging_station(22));
    EXPECT_TRUE(instance->is_charging_station(29));
    EXPECT_FALSE(instance->is_charging_station(13));
    EXPECT_FALSE(instance->is_charging_station(30));
}

TEST_F(CaseTest, GetBestAndFeasibleStation) {

    int station_01 = instance->get_best_and_feasible_station(0, 19, 78.33333333333334);
    int station_02 = instance->get_best_and_feasible_station(19, 2, 69.38906142333418);
    int station_03 = instance->get_best_and_feasible_station(0, 17, 78.33333333333334);

    EXPECT_EQ(station_01, 22);
    EXPECT_EQ(station_02, 23);
    EXPECT_EQ(station_03, 23);
}

TEST_F(CaseTest, TotalDistanceAndDemandSum) {
    vector<vector<int>> routes = {
            {0, 21, 19, 17, 20, 0},
            {0, 18, 15, 14, 12, 16, 0},
            {0, 6, 8, 10, 7, 5, 3, 1, 0},
            {0, 2, 9, 4, 11, 13, 0}
    };

    double total_distance = instance->compute_total_distance(routes);
    vector<int> demand_sum_per_route = instance->compute_demand_sum_per_route(routes);

    vector<int> truth_demand_sum_per_route = {6000, 5500, 5900, 5100};

    EXPECT_EQ(total_distance, 553.4851515212215);
    EXPECT_EQ(demand_sum_per_route, truth_demand_sum_per_route);
}