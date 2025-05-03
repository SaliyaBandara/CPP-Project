#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <thread>
#include <iostream>
Simulation::Simulation(const Config &config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads) {
    initializeParticles(config);
    threadManager->start();
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config &config) {
    std::random_device rd;
    std::mt19937 gen(config.random_seed ? config.random_seed : rd());
    std::uniform_real_distribution<> dis(-fieldSize / 2, fieldSize / 2);
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0);

    size_t count = config.num_particles;
    for (size_t i = 0; i < count; ++i) {
        auto particle = std::make_unique<Particle>(
            dis(gen), dis(gen),
            config.initial_energy,
            config.particle_radius,
            config.max_energy);
        particle->setVelocity(vel_dis(gen), vel_dis(gen));
        particles.push_back(std::move(particle));
    }
}

void Simulation::setContainmentField(std::unique_ptr<ContainmentField> field) {
    containmentField = std::move(field);
}

void Simulation::start() {
    running = true;
}

void Simulation::stop() {
    running = false;
    if (threadManager && threadManager->isRunning()) {
        threadManager->stop();
    }
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step() {
    threadManager->addTask([this]() { removeEscapedParticles(); });
    threadManager->addTask([this]() { applyForces(timeStep); });
    if (std::rand() % 3 != 0) {
        threadManager->addTask([this]() { handleCollisions(); });
    }
    threadManager->waitForCompletion();
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {

    std::vector<bool> shouldRemove(particles.size(), false);
    const size_t particleCount = particles.size();
    const size_t batchSize = std::max(size_t(1), particleCount / numThreads);

    for (size_t i = 0; i < particleCount; i += batchSize) {
        const size_t end = std::min(i + batchSize, particleCount);

        threadManager->addTask([this, i, end, &shouldRemove]() {
            for (size_t idx = i; idx < end; ++idx)
            {
                shouldRemove[idx] = !containmentField->isParticleContained(*particles[idx]);
            } });
    }

    threadManager->waitForCompletion();

    std::vector<std::unique_ptr<Particle>> remainingParticles;
    remainingParticles.reserve(particleCount);

    for (size_t i = 0; i < particleCount; ++i) {
        if (!shouldRemove[i]) {
            remainingParticles.push_back(std::move(particles[i]));
        }
    }

    particles.swap(remainingParticles);
}

size_t Simulation::getParticleCount() const {
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>> &Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    double total = 0.0;
    for (const auto &particle : particles) {
        total += particle->getEnergy();
    }
    return total;
}

void Simulation::setNumThreads(size_t newNumThreads) {
    numThreads = newNumThreads;
    threadManager->setNumThreads(newNumThreads);
}

size_t Simulation::getNumThreads() const {
    return numThreads;
}

void Simulation::updatePositions(double dt) {

    const size_t particleCount = particles.size();
    const size_t batchSize = std::max(size_t(1), particleCount / numThreads);

    for (size_t i = 0; i < particleCount; i += batchSize) {
        const size_t end = std::min(i + batchSize, particleCount);

        threadManager->addTask([this, i, end, dt]() {
            for (size_t idx = i; idx < end; ++idx)
            {
                auto &particle = particles[idx];
                double x = particle->getX() + particle->getVX() * dt;
                double y = particle->getY() + particle->getVY() * dt;
                particle->setPosition(x, y);
            } });
    }

    threadManager->waitForCompletion();
}

void Simulation::handleCollisions() {

    const size_t particleCount = particles.size();
    const size_t batchSize = std::max(size_t(1), particleCount / (numThreads * 2));

    for (size_t i = 0; i < particleCount; i += batchSize) {
        const size_t end = std::min(i + batchSize, particleCount);

        threadManager->addTask([this, i, end, particleCount]() {
            for (size_t idx = i; idx < end; ++idx)
            {
                for (size_t j = idx + 1; j < particleCount; ++j)
                {
                    if (particles[idx]->isColliding(*particles[j]))
                    {
                        particles[idx]->collide(*particles[j]);
                    }
                }
            } });
    }

    threadManager->waitForCompletion();
}

void Simulation::applyForces(double dt) {

    const size_t particleCount = particles.size();
    const size_t batchSize = std::max(size_t(1), particleCount / numThreads);

    for (size_t i = 0; i < particleCount; i += batchSize) {
        const size_t end = std::min(i + batchSize, particleCount);

        threadManager->addTask([this, i, end, dt]() {
            for (size_t idx = i; idx < end; ++idx)
            {
                auto &particle = particles[idx];
                // Get current position
                double x = particle->getX();
                double y = particle->getY();
                // Calculate distance from center
                double distance = std::sqrt(x * x + y * y);

                // Get force from containment field
                double force = containmentField->getContainmentForce(*particle);

                // Calculate acceleration components (towards center)
                double ax = -x / (distance + 1e-10) * force; // Avoid division by zero
                double ay = -y / (distance + 1e-10) * force; // Avoid division by zero

                // Calculate new velocity using Euler integration
                double vx = particle->getVX() + ax * dt;
                double vy = particle->getVY() + ay * dt;

                // Update particle velocity
                particle->setVelocity(vx, vy);
            } });
    }

    threadManager->waitForCompletion();
}