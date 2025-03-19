#include "my_thread_pool.h"

/// @brief public:
/// @name Type Define
/// @{
/// @}
/// @name Life Circle
/// @{
MyThreadPool::MyThreadPool(int num_threads):m_threads_pool_stop(false),m_num_of_threads(num_threads),m_num_of_working_threads(0)
{
    start_all_threads();
}

MyThreadPool::~MyThreadPool()
{
    end_all_threads();
}
/// @}
/// @name Operators
/// @{
/// @}
/// @name Operations
/// @{

void MyThreadPool::join()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this](){
        return m_num_of_working_threads == 0 && m_tasks.empty();
    });
    // end_all_threads();
    // start_all_threads();
}

/// @}
/// @name Access
/// @{
/// @}
/// @name Inquiry
/// @{

size_t MyThreadPool::num_of_threads()
{
    return m_num_of_threads;
}

/// @}

/// @brief protected:
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

/// @brief private:
/// @name Private Static Member Variables
/// @{
/// @}
/// @name Private Member Variables
/// @{
/// @}
/// @name Private Operatiors
/// @{
/// @}
/// @name Private Operations
/// @{
bool MyThreadPool::end_all_threads()
{
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_threads_pool_stop = true;
    }
    m_cv.notify_all();
    for(auto &thread:m_threads)
        thread.join();
    return true;
}

bool MyThreadPool::start_all_threads()
{
    if(m_threads_pool_stop)
        m_threads_pool_stop = false;
    m_threads.clear();
    for(size_t i=0;i<m_num_of_threads;++i)
    {
        m_threads.emplace_back([this](){
            while(true){
                std::unique_lock<std::mutex> lock(m_mutex);
                m_cv.wait(lock, [this](){
                    return !this->m_tasks.empty() || this->m_threads_pool_stop;
                });

                if(this->m_tasks.empty() && this->m_threads_pool_stop)
                    break;

                std::function<void()> task(std::move(this->m_tasks.front()));
                this->m_tasks.pop();
                lock.unlock();

                task();

                lock.lock();
                m_num_of_working_threads--;
                if(m_num_of_working_threads == 0 && m_tasks.empty())
                    m_cv.notify_all();
                lock.unlock();
            };
        });
    }
    return true;
}
/// @}
/// @name Private Access
/// @{
/// @}
/// @name Private Inquiry
/// @{
/// @}






