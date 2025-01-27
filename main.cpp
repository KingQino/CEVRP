#include <iostream>
#include <thread>
#include "magic_enum.hpp"
#include "case.hpp"
#include "ma.hpp"
#include "lahc.hpp"


using namespace std;
using namespace magic_enum;

#define MAX_TRIALS 10

enum class Algorithm { CBMA, LAHC };

void run_algorithm(const int run, const Algorithm algorithm, const string& file_name, const bool enable_logging, const int stop_criteria_option, std::vector<double>& perfOfTrials) {
    Case* instance = new Case(run, file_name);

    switch (algorithm) {
        case Algorithm::CBMA: {
            Ma* ma = new Ma(instance, run, stop_criteria_option, enable_logging);
            ma->run();
            perfOfTrials[run - 1] = ma->global_best->lower_cost;
            delete ma;
            break;
        }

        case Algorithm::LAHC: {
            Lahc* lahc = new Lahc(instance, run, stop_criteria_option, enable_logging);
            lahc->run();
            perfOfTrials[run - 1] = lahc->global_best->lower_cost;
            delete lahc;
            break;
        }
    }

    delete instance;
}



int main(int argc, char *argv[]) {
    int run;

    if (argc != 6) {
        cerr << "Usage: " << argv[0] << " <algorithm: 0-CBMA, 1-LAHC> <problem_instance_filename> <enable_logging: 0-no, 1-yes> <stop_criteria: 0 max-evals, 1 max-time> <multithreading: 0 no, 1 yes>" << endl;
        return 1;
    }

    Algorithm algorithm = static_cast<Algorithm>(std::stoi(argv[1]));
    string file_name(argv[2]);
    bool enable_logging = std::stoi(argv[3]) != 0;
    int stop_criteria_option = std::stoi(argv[4]);
    int is_activate_multi_threading = std::stoi(argv[5]);

    vector<double> perfOfTrials(MAX_TRIALS);
    if (is_activate_multi_threading == 1) {
        std::vector<std::thread> threads;

        for (run = 2; run <= MAX_TRIALS; ++run) {
            threads.emplace_back(run_algorithm, run, algorithm, file_name, enable_logging, stop_criteria_option, std::ref(perfOfTrials));
        }
        run_algorithm(1, algorithm, file_name, enable_logging, stop_criteria_option, perfOfTrials);

        for (auto& thread : threads) {
            thread.join();
        }
    } else {
        run_algorithm(1, algorithm, file_name, enable_logging, stop_criteria_option, perfOfTrials);
    }

    string stats_file_path = kStatsPath + "/" + static_cast<string>(enum_name(algorithm)) + "/" +
            file_name.substr(0, file_name.find('.'));

    StatsInterface::create_directories_if_not_exists(stats_file_path);
    StatsInterface::stats_for_multiple_trials(stats_file_path + "/" + "stats." + file_name,perfOfTrials);

    return 0;
}
