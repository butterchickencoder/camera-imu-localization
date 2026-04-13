#pragma once

#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <functional>
#include <stdexcept>
#include <iostream>

/**
 * @namespace utils
 * Encapsulates utility functions to prevent name collisions in larger projects.
 */
namespace utils {

    /**
     * @brief A robust, generic CSV loader for robotics datasets.
     * 
     * @tparam T The data type being loaded (e.g., Eigen::Vector3d, double, or Pose).
     * @param path The filesystem path to the .csv file.
     * @param parser A lambda function: (std::stringstream& ss) -> std::pair<int64_t, T>.
     *               The lambda defines how to turn a single line of text into data.
     * 
     * @return std::map<int64_t, T> A map containing timestamps as keys and sensor data as values.
     * @throws std::runtime_error if the file cannot be opened.
     */
    template <typename T>
    std::map<int64_t, T> loadCSV(const std::string& path, 
                                 std::function<std::pair<int64_t, T>(std::stringstream&)> parser) {
        std::map<int64_t, T> data;
        std::ifstream file(path);

        if (!file.is_open()) {
            throw std::runtime_error("CRITICAL: Could not open file: " + path);
        }

        std::string line;
        // Skip the header line 
        if (!std::getline(file, line)) {
            return data; 
        }

        int line_count = 1; // Tracked for precise error reporting
        while (std::getline(file, line)) {
            line_count++;
            if (line.empty()) continue;

            std::stringstream ss(line);
            try {
                // Execute the user-defined parsing logic
                auto result = parser(ss);
                data[result.first] = result.second;
            } catch (const std::exception& e) {
                // We log the error and line number, but keep processing other lines.
                // This prevents one bad sensor reading from crashing a long simulation.
                std::cerr << "[CSV Utils] Warning: Error on line " << line_count 
                          << " in file '" << path << "' -> " << e.what() << std::endl;
            }
        }

        return data;
    }

} // namespace utils