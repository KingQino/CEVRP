//
// Created by Yinghao Qin on 14/09/2024.
//

#ifndef CEVRP_UTILS_HPP
#define CEVRP_UTILS_HPP

#include <iostream>
#include <vector>
#include <random>
#include <cstring>
#include <numeric>
#include <memory>
#include "case.hpp"

using namespace std;

#define INFEASIBLE 1'000'000'000L


// used in 2-opt*, define the hash relationship to make sure one-to-one mapping between route pairs
struct pair_hash
{
    size_t operator() (pair<int, int> const & apair) const {
        return apair.first * 256 + apair.second;
    }
};

// split - transfer a chromosome into upper-solution
vector<vector<int>> prins_split(const vector<int>& chromosome, Case& instance);
vector<vector<int>> hien_clustering(const Case& instance, std::default_random_engine& rng);
void hien_balancing(vector<vector<int>>& routes, const Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_split(Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_hien_method(const Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_direct_encoding(const Case& instance, std::default_random_engine& rng);



#endif //CEVRP_UTILS_HPP
