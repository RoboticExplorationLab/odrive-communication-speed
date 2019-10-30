#include "odrive_threadpool.h"

using namespace std;

ODriveThreadPool::ODriveThreadPool() : threads(), idleCond(), idleLock()
{
    running = true;
    idleThreads = 0;
}

void ODriveThreadPool::addOdriveThread(ODrive_t odrive)
{
    thread odrive_thread([this](ODrive_t odrive) {
        odriveThread(odrive);
    }, odrive);
    mutex jobLock;
    condition_variable_any jobCond;
    struct thread_t s = {odrive_thread, true, NULL, jobLock, jobCond};
    threads.emplace(odrive, s);

    lock_guard<mutex> lgr(idleLock);
    idleThreads++;
}

void ODriveThreadPool::odriveThread(ODrive_t odrive)
{
    while (running) {
        if (!threads.count(odrive)) continue;
        threads[odrive].jobLock.lock();
        threads[odrive].jobCond.wait(threads[odrive].jobLock, [this, odrive] { return threads[odrive].job != NULL || !running; });
        threads[odrive].jobLock.unlock();
        if (!running) break;

        threads[odrive].job();
        threads[odrive].job = NULL;
        lock_guard<mutex> lgr(idleLock);
        threads[odrive].idle = true;
        idleThreads++;
        idleCond.notify_one();
    }
}
