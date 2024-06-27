
/**
 * @file WorkerThread.hpp WorkerThread class declarations
 *
 * WorkerThread defines a std::thread which runs the do_work()
 * function as well as methods to start and stop that thread.
 * This file is intended to help reduce code duplication for the common
 * task of starting and stopping threads. As in artdaq, std::thread may
 * be replaced by boost::thread to allow setting the stack size at a
 * later date if that functionality is found to be necessary.
 *
 * This is part of the DUNE DAQ Application Framework, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */

#ifndef WORKERTHREAD_HPP_
#define WORKERTHREAD_HPP_

#include <functional>
#include <future>
#include <list>
#include <memory>
#include <string>

namespace cib
{
  namespace utilities
  {

    /**
     * @brief WorkerThread contains a thread which runs the do_work()
     * function
     *
     * WorkerThread runs a given function in a std::thread and allows that
     * work function to be started and stopped via the
     * start_working_thread() and stop_working_thread() methods,
     * respectively. The work function takes a std::atomic<bool>&  which
     * indicates whether the thread should continue running, so a typical
     * implementation of a work function is:
     *
     * @code
     * void do_work(std::atomic<bool>& running_flag){
     *   while(running_flag.load()){
     *    // do something ...
     *   }
     * }
     * @endcode
     *
     * If your do_work function is a class member, you will need to wrap
     * it with std::bind or a lambda to bind the implicit 'this' argument,
     * eg
     *
     * @code
     * class MyClass {
     * public:
     *   MyClass()
     *     : helper_(std::bind(MyClass::do_work, this, std::placeholders::_1))
     *   {}
     *   void do_work(std::atomic<bool>& running_flag) { ... }
     *   WorkerThread helper_;
     * };
     * @endcode
     */
    class WorkerThread final
    {
    public:
      /**
       * @brief WorkerThread Constructor
       * @param do_work Function to be executed in the thread
       *
       * This constructor sets the defaults for the thread control variables
       */
      explicit WorkerThread(std::function<void(std::atomic<bool>&)> do_work);

      /**
       * @brief Start the working thread (which executes the do_work() function)
       * @throws ThreadingIssue if the thread is already running
       */
      void start_working_thread(const std::string& name = "noname");
      /**
       * @brief Stop the working thread
       * @throws ThreadingIssue If the thread has not yet been started
       * @throws ThreadingIssue If the thread is not in the joinable state
       * @throws ThreadingIssue If an exception occurs during thread join
       */
      void stop_working_thread();

      /**
       * @brief Determine if the thread is currently running
       * @return Whether the thread is currently running
       */
      bool thread_running() const { return m_thread_running.load(); }

      WorkerThread(const WorkerThread&) = delete;            ///< WorkerThread is not copy-constructible
      WorkerThread& operator=(const WorkerThread&) = delete; ///< WorkerThread is not copy-assginable
      WorkerThread(WorkerThread&&) = delete;                 ///< WorkerThread is not move-constructible
      WorkerThread& operator=(WorkerThread&&) = delete;      ///< WorkerThread is not move-assignable
    private:
      std::atomic<bool> m_thread_running;
      std::unique_ptr<std::thread> m_working_thread;
      std::function<void(std::atomic<bool>&)> m_do_work;
    };

  } /* namespace utilities */
} /* namespace cib */

#endif /* WORKERTHREAD_HPP_ */
