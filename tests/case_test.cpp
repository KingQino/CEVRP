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

}