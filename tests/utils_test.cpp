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