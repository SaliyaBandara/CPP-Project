#include "../include/Simulation.h"
#include "../include/Config.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <fstream>
#include <stdexcept>
#include <algorithm> // Include for std::clamp

Config loadConfig() {
    Config cfg;
    return cfg;
}

// Renders the particle simulation state to the console using ASCII characters.
void renderASCII(const std::vector<std::unique_ptr<Particle>> &particles, double fieldSize, const Config &cfg) {
    // Initialize a 2D grid to store particle counts per cell.
    std::vector<std::vector<int>> gridCounts(cfg.grid_height, std::vector<int>(cfg.grid_width, 0));

    // Iterate through particles and map them to grid cells.
    for (const auto &particle : particles) {
        double x = particle->getX();
        double y = particle->getY();

        // Calculate grid column and row based on particle position.
        int col = static_cast<int>((x + fieldSize / 2) * cfg.grid_width / fieldSize);
        int row = static_cast<int>((y + fieldSize / 2) * cfg.grid_height / fieldSize);

        // Clamp grid coordinates to ensure they are within bounds.
        col = std::max(0, std::min(col, cfg.grid_width - 1));  // Ensure col is within bounds [0, grid_width - 1]
        row = std::max(0, std::min(row, cfg.grid_height - 1)); // Ensure row is within bounds [0, grid_height - 1]

        // Increment the count for the corresponding grid cell.
        gridCounts[row][col]++;
    }

    // Clear the console and move cursor to the top-left.
    std::cout << "\033[2J\033[H";

    // Draw the top border of the grid.
    std::cout << '+' << std::string(cfg.grid_width, '-') << "+\n";

    // Iterate through the grid rows to render the particle density.
    for (int i = 0; i < cfg.grid_height; ++i) {
        std::cout << '|'; // Draw the left border.
        // Iterate through the grid columns.
        for (int j = 0; j < cfg.grid_width; ++j) {
            int count = gridCounts[i][j];
            // Render character based on particle count (density).
            if (count == 0) {
                std::cout << ' '; // Empty cell.
            } else {
                // Clamp density level and look up the corresponding character.
                int level = std::min(count, cfg.max_density_level);
                auto it = cfg.density_map.find(level);
                std::cout << (it != cfg.density_map.end() ? it->second : ' '); // Use mapped char or space.
            }
        }
        std::cout << "|\n"; // Draw the right border and newline.
    }

    // Draw the bottom border of the grid.
    std::cout << '+' << std::string(cfg.grid_width, '-') << "+\n";
    // Flush the output buffer to ensure the grid is displayed immediately.
    std::cout << std::flush;
}

// Main entry point of the application.
int main() {
    try {
        Config config = loadConfig();
        Simulation simulation(config);
        simulation.start();

        // Calculate the desired time per frame based on target FPS.
        const double FRAME_TIME = 1.0 / config.target_fps;

        // Main simulation loop, continues as long as there are particles.
        while (simulation.getParticleCount() > 0) {
            auto frameStart = std::chrono::high_resolution_clock::now();
            simulation.step();
            renderASCII(simulation.getParticles(), config.field_size, config);

            // Record the end time of the frame.
            auto frameEnd = std::chrono::high_resolution_clock::now();
            // Calculate the duration of the frame processing.
            auto frameDuration = std::chrono::duration<double>(frameEnd - frameStart).count();

            // If the frame processed faster than the target frame time, pause to maintain FPS.
            if (frameDuration < FRAME_TIME) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(FRAME_TIME - frameDuration));
            }

            // Periodically print simulation statistics.
            static int frameCount = 0;
            if (++frameCount % 30 == 0) { // Print stats every 30 frames.
                auto now = std::chrono::high_resolution_clock::now();
                static auto lastStatTime = now; // Track time since last stats print.
                auto elapsed = std::chrono::duration<double>(now - lastStatTime).count();
                // Calculate actual FPS over the last 30 frames.
                double actualFps = (elapsed > 1e-6) ? (30.0 / elapsed) : 0.0;
                lastStatTime = now;

                // Output statistics.
                std::cout << "\nParticles: " << simulation.getParticleCount()
                          << " | Energy: " << simulation.getTotalEnergy()
                          << " | FPS: " << actualFps << std::endl;
            }
        }

        // Stop the simulation threads.
        simulation.stop();
        std::cout << "Simulation ended. All particles escaped.\n";
    } catch (const std::exception &e) {
        // Catch and report any standard exceptions.
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Indicate failure.
    }
    return 0; // Indicate successful execution.
}