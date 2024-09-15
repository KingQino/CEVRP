//
// Created by Yinghao Qin on 14/09/2024.
//

#include "gtest/gtest.h"
#include "utils.hpp"

using namespace ::testing;
using namespace std;

class UtilsTest : public Test {
protected:
    void SetUp() override {
        string fileName = "E-n22-k4.evrp";
        instance = new Case(1, fileName);
        rng = std::default_random_engine(0);;
    }

    void TearDown() override {
        delete instance;
    }

    Case* instance{};
    std::default_random_engine rng;
};


TEST_F(UtilsTest, PrinsSplit) {
    vector<vector<int>> result = prins_split(    instance->customers, *instance);

    EXPECT_EQ(result.size(), 5);
    ASSERT_EQ(result[0], vector<int>({19, 20, 21}));
}

TEST_F(UtilsTest, Hien) {
    vector<vector<int>> routes = hien_clustering(*instance, rng);

    std::size_t size_1 = 0;
    for (const auto& route : routes) {
        size_1 += route.size();
//        cout << "Route: ";
//        for (const auto& customer : route) {
//            cout << customer << " ";
//        }
//        cout << endl;
    }
//    cout << endl;


    hien_balancing(routes, *instance, rng);
    std::size_t size_2 = 0;
    for (const auto& route : routes) {
        size_2 += route.size();
//        cout << "Route: ";
//        for (const auto& customer : route) {
//            cout << customer << " ";
//        }
//        cout << endl;
    }
//    cout << endl;

    // Assert
    ASSERT_EQ(size_1, instance->customerNumber);
    ASSERT_EQ(size_2, instance->customerNumber);
}

TEST_F(UtilsTest, RoutesConstructor) {
    vector<vector<int>> routes_x = routes_constructor_with_split(*instance, rng);
    vector<vector<int>> routes_y = routes_constructor_with_hien_method(*instance, rng);
    vector<vector<int>> routes_z = routes_constructor_with_direct_encoding(*instance, rng);

    EXPECT_EQ(routes_x[0][0], 0);
    EXPECT_EQ(routes_x[-1][-1], 0);
    EXPECT_EQ(routes_y[0][0], 0);
    EXPECT_EQ(routes_y[-1][-1], 0);
    EXPECT_EQ(routes_z[0][0], 0);
    EXPECT_EQ(routes_z[-1][-1], 0);
}

TEST_F(UtilsTest, TwoOptForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 8, 0},
            {0, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum = {6000, 5500, 5900, 5100};
    shared_ptr<Individual> ind = std::make_shared<Individual>(8, 22, routes, 485.60155200568835, demand_sum);

    two_opt_for_individual(*ind, *instance);

    EXPECT_EQ(ind->upper_cost, 385.2853879430639);
}

TEST_F(UtilsTest, TwoOptStarForIndividual) {
    vector<vector<int>> routes = {
            {0, 9, 7, 5, 2, 1, 6, 8, 0},
            {0, 10, 3, 4, 11, 13, 0},
            {0, 19, 21, 20, 17, 0},
            {0, 12, 15, 18, 14, 16, 0}
    };
    vector<int> demand_sum = {5700, 5300, 6000, 5500};
    shared_ptr<Individual> ind = std::make_shared<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum);

    double fit_prev = ind->upper_cost;
    bool updated = two_opt_star_for_individual(*ind, *instance);
    double fit_cur = ind->upper_cost;

    EXPECT_LT(fit_cur, fit_prev);
    EXPECT_TRUE(updated);
}