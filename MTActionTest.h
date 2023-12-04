
#include <sys/time.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nodelet/nodelet.h>
#include <carplanner_msgs/MTTestAction.h>
#include "threadpool.hpp"
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <numeric>

#ifdef __APPLE__
#include "pthread.h"
#include <mach/clock.h>
#include <mach/mach.h>
#define SetThreadName(x) pthread_setname_np(x);
#else
#include <sys/prctl.h>
#define SetThreadName(x) prctl(PR_SET_NAME,x,0,0,0);
#endif

#define NUM_THREADS 10
#define MAX_SERVER_TIME 1.0

#define USE_SIMPLE_ACTIONS
#define USE_MANY_SERVERS

inline double Tic() {

    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec  + 1e-6 * (tv.tv_usec);
}

inline double Toc(double dTic) {
    return Tic() - dTic;
}