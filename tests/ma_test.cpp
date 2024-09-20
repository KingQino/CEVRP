//
// Created by Yinghao Qin on 20/09/2024.
//

#include "gtest/gtest.h"
#include "ma.hpp"

using namespace ::testing;
using namespace std;

class MaTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(1, file_name_);
        ma = new Ma(instance, 1);
    }

    void TearDown() override {
        delete ma;
        delete instance;
    }

    Case* instance{};
    Ma* ma{};
};


TEST_F(MaTest, AdmitOneImmigrant) {
    shared_ptr<Individual> ind_ptr = ma->admit_one_immigrant(instance->customers_);

    EXPECT_EQ(ind_ptr->num_routes, 5);
    EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, 569.47817556079303);
    EXPECT_EQ(ind_ptr->num_nodes_per_route[0], 5);
}