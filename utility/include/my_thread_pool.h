#ifndef _THREAD_POOL_H_
#define _THREAD_POOL_H_

#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>

class MyThreadPool
{
    public:
        /// @name Type Define
        /// @{
        /// @}
        /// @name Life Circle
        /// @{
        MyThreadPool(int num_threads=8);
        ~MyThreadPool();
        /// @}
        /// @name Operators
        /// @{
        /// @}
        /// @name Operations
        /// @{

        template<class F, class... Args>
        void enqueue(F&& f, Args&&... args)
        {
            std::function<void()> task(std::bind(std::forward<F>(f), std::forward<Args>(args)... ));
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_tasks.emplace(std::move(task));
                m_num_of_working_threads++;
            }
            m_cv.notify_one();
        }

        void join();
        
        /// @}
        /// @name Access
        /// @{
        /// @}
        /// @name Inquiry
        /// @{
        size_t num_of_threads();
        /// @}
    protected:
        /// @name Protected Static Member Variables
        /// @{
        /// @}
        /// @name Protected Member Variables
        /// @{
        /// @}
        /// @name Protected Operatiors
        /// @{
        /// @}
        /// @name Protected Operations
        /// @{
        /// @}
        /// @name Protected Access
        /// @{
        /// @}
        /// @name Protected Inquiry
        /// @{
        /// @}
    private:
        /// @name Private Static Member Variables
        /// @{
        /// @}
        /// @name Private Member Variables
        /// @{
        std::vector<std::thread>               m_threads;
        std::queue<std::function<void()>>      m_tasks;
        std::mutex                             m_mutex;
        std::condition_variable                m_cv;

        size_t                                 m_num_of_threads;
        size_t                                 m_num_of_working_threads;
        bool                                   m_threads_pool_stop;
        /// @}
        /// @name Private Operatiors
        /// @{
        /// @}
        /// @name Private Operations
        /// @{
        bool end_all_threads();
        bool start_all_threads();
        /// @}
        /// @name Private Access
        /// @{
        /// @}
        /// @name Private Inquiry
        /// @{
        /// @}
};


#endif  