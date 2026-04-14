#pragma once

#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <functional>
#include <stdexcept>
#include <iostream>

struct Pose { double r[9]; double t[3]; };

/**
 * @namespace utils
 * Encapsulates utility functions to prevent name collisions in larger projects.
 */
namespace utils {

    /**
     Providing loadBaro and loadPoses utility functions for loading data from dataset
     */

    std::map<int64_t, double> loadBaro(const std::string& path) {
        std::map<int64_t, double> baro;
        std::ifstream f(path);
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string tok;
            int64_t t; double z;
            std::getline(ss, tok, ','); t = std::stoll(tok);
            std::getline(ss, tok, ','); z = std::stod(tok);
            baro[t] = z;
            }
    return baro;
    }


    std::map<int64_t, Pose> loadPoses(const std::string& path) {
        std::map<int64_t, Pose> poses;
        std::ifstream f(path);
        std::string line;
        std::getline(f, line); // skip header
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string tok;
            int64_t t; Pose p;
            std::getline(ss, tok, ','); t = std::stoll(tok);
            for (int i = 0; i < 9; i++) { std::getline(ss, tok, ','); p.r[i] = std::stod(tok); }
            for (int i = 0; i < 3; i++) { std::getline(ss, tok, ','); p.t[i] = std::stod(tok); }
            poses[t] = p;
                }
        return poses;
    }

} // namespace utils