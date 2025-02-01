//
// Created by Yinghao Qin on 20/01/2025.
//

#include "gtest/gtest.h"
#include "lahc.hpp"

using namespace ::testing;
using namespace std;

class LahcTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(1, 20, file_name_);
        lahc = new Lahc(instance, 1, 0, false);
    }

    void TearDown() override {
        delete lahc;
        delete instance;
    }

    Case* instance{};
    Lahc* lahc{};
};

TEST_F(LahcTest, Run) {

    lahc->run();

    EXPECT_TRUE(true);
}