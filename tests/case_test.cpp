//
// Created by Yinghao Qin on 13/09/2024.
//

#include "gtest/gtest.h"
#include "case.hpp"

using namespace ::testing;

class CaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        string fileName = "E-n22-k4.evrp";
        instance = new Case(1, fileName);
    }

    void TearDown() override {
        delete instance;
    }

    Case* instance{};
};

TEST_F(CaseTest, Init) {
    vector<int> customerOne2CustomerListNearToFar = {2, 6, 5, 3, 7, 4, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};

    SCOPED_TRACE("Case init...");
    ASSERT_EQ(instance->customers.size(), 21);
    ASSERT_EQ(instance->stations.size(), 8);
    ASSERT_EQ(instance->customerClusterMap[1], customerOne2CustomerListNearToFar);
    ASSERT_EQ(instance->bestStation[1][13], 24);
    ASSERT_EQ(instance->bestStation[0][21], 22);
    ASSERT_EQ(instance->bestStation[5][6], 29);
    ASSERT_EQ(instance->maxEvals, 750000);
    ASSERT_EQ(instance->get_customer_demand(2), 700);
    ASSERT_TRUE(instance->is_charging_station(0));
    ASSERT_TRUE(instance->is_charging_station(22));
    ASSERT_TRUE(instance->is_charging_station(29));
    ASSERT_FALSE(instance->is_charging_station(13));
    ASSERT_FALSE(instance->is_charging_station(30));
}

TEST_F(CaseTest, GetBestAndFeasibleStation) {

    int station_01 = instance->get_best_and_feasible_station(0, 19, 78.33333333333334);
    int station_02 = instance->get_best_and_feasible_station(19, 2, 69.38906142333418);
    int station_03 = instance->get_best_and_feasible_station(0, 17, 78.33333333333334);

    ASSERT_EQ(station_01, 22);
    ASSERT_EQ(station_02, 23);
    ASSERT_EQ(station_03, 23);
}

TEST_F(CaseTest, TotalDistanceAndDemandSum) {
    vector<vector<int>> routes = {
            {0, 21, 19, 17, 20, 0},
            {0, 18, 15, 14, 12, 16, 0},
            {0, 6, 8, 10, 7, 5, 3, 1, 0},
            {0, 2, 9, 4, 11, 13, 0}
    };

    double total_distance = instance->compute_total_distance(routes);
    vector<int> demand_sum = instance->compute_demand_sum(routes);

    vector<int> truth_demand_sum = {6000, 5500, 5900, 5100};

    ASSERT_EQ(total_distance, 553.4851515212215);
    ASSERT_EQ(demand_sum, truth_demand_sum);
}