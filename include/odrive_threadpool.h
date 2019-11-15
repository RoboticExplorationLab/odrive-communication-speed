#ifndef ODRIVE_THREADPOOL_H
#define ODRIVE_THREADPOOL_H

#include <cstddef>     // for size_t
#include <functional>  // for the function template used in the schedule signature
#include <thread>      // for thread
#include <map>
#include <condition_variable>
#include <mutex>
#include "odrive_c_sdk.h"

class ODriveThreadPool
{
public:
    ODriveThreadPool();
    void addOdriveThread(ODrive_t odrive);
    void schedule(ODrive_t odrive, const std::function<void(void)> &thunk);
    void wait();
    ~ODriveThreadPool();

private:
    struct thread_t
    {
        std::thread odrive_thread;            // worker thread handle
        bool idle;                // indicates whether the thread is idle
        std::function<void(void)> job; // thunk for the thread to execute
        std::mutex jobLock;
        std::condition_variable_any jobCond;
    };

    bool running;
    std::map<ODrive_t, thread_t*> threads;
    size_t idleThreads;                         // number of currently idle threads
    std::condition_variable_any idleCond;       // condition variable for idleThreads
    std::mutex idleLock;                        // mutex for guarding idleThreads

    void odriveThread(ODrive_t odrive);

    ODriveThreadPool(const ODriveThreadPool &original) = delete;
    ODriveThreadPool &operator=(const ODriveThreadPool &rhs) = delete;
};

#endif