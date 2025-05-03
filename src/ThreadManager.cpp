#include "../include/ThreadManager.h"
#include <stdexcept>

ThreadManager::ThreadManager(size_t numThreads)
    : numThreads(numThreads), running(false) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    if (running) // Prevent starting if already running
        return;

    running = true;
    threads.clear();                // Clear any previous threads
    threadLoads.resize(numThreads); // Resize load tracking vector

    // Create and launch worker threads
    for (size_t i = 0; i < numThreads; ++i) {
        threadLoads[i] = 0; // Initialize load for this thread
        threads.emplace_back(&ThreadManager::workerThread, this, i);
    }
}

// Stops the worker threads gracefully.
void ThreadManager::stop() {
    if (!running) // Prevent stopping if not running
        return;

    {
        std::lock_guard<std::mutex> lock(taskMutex);
        running = false;            // Signal threads to stop
        taskCondition.notify_all(); // Wake up all waiting threads
    }

    // Wait for all threads to finish execution
    for (auto &thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    threads.clear(); // Remove thread objects
}

// Adds a task to the queue.
void ThreadManager::addTask(std::function<void()> task) {
    if (!running) // Don't add tasks if not running
        return;

    {
        std::lock_guard<std::mutex> lock(taskMutex);
        taskQueue.push(std::move(task)); // Add task to the queue
    }

    taskCondition.notify_one(); // Notify one waiting thread
}

// Checks if the thread manager is currently running.
bool ThreadManager::isRunning() const {
    return running;
}

// Returns the configured number of threads.
size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

// Sets a new number of threads (restarts the pool if running).
void ThreadManager::setNumThreads(size_t newNumThreads) {
    if (newNumThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }

    if (running) {
        stop(); // Stop existing threads
        numThreads = newNumThreads;
        start(); // Start with the new number of threads
    } else {
        numThreads = newNumThreads; // Just update the number if not running
    }
}

// Returns the current number of tasks in the queue.
size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex); // Lock required for queue access
    return taskQueue.size();
}

// Waits until the task queue is empty and all active threads have finished.
void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(completionMutex);
    // Loop until no tasks are pending and no threads are actively processing
    while (getTaskCount() > 0 || activeThreads > 0) {
        // Sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Returns the number of threads currently processing a task.
size_t ThreadManager::getActiveThreadCount() const {
    // Note: activeThreads is atomic, no lock needed for read
    return activeThreads;
}

// Processes the next available task from the queue (intended for single-threaded execution if needed).
void ThreadManager::processNextTask() {
    std::function<void()> task;

    {
        std::unique_lock<std::mutex> lock(taskMutex);
        if (taskQueue.empty()) // Do nothing if queue is empty
            return;

        task = std::move(taskQueue.front()); // Get the next task
        taskQueue.pop();
    }

    if (task) {
        activeThreads++; // Increment active thread count
        task();          // Execute the task
        activeThreads--; // Decrement active thread count
    }
}

// The main function executed by each worker thread.
void ThreadManager::workerThread(size_t threadId) {
    while (running) { // Loop until stop() is called
        std::function<void()> task;

        {
            std::unique_lock<std::mutex> lock(taskMutex);
            // Wait until there's a task or the manager is stopping
            taskCondition.wait(lock, [this] { return !taskQueue.empty() || !running; });

            // Exit if stopping and the queue is empty
            if (!running && taskQueue.empty()) {
                return;
            }

            // Get a task if available
            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
                threadLoads[threadId]++; // Increment load counter for this thread
            }
        }

        // Execute the task if one was retrieved
        if (task) {
            activeThreads++; // Mark thread as active
            task();          // Run the task
            activeThreads--; // Mark thread as inactive
        }
    }
}