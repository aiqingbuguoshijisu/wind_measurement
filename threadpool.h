#pragma once
//创建线程池类
#include<future>
#include<thread>
#include<queue>
#include <functional>

class ThreadPool {
public:
    explicit ThreadPool(size_t numThreads = std::thread::hardware_concurrency()) 
        : stop(false) {
        // 创建指定数量的工作线程
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        // 等待任务或停止信号
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        
                        if (stop && tasks.empty()) {
                            return;
                        }
                        
                        // 取出任务
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    // 执行任务
                    task();
                }
            });
        }
    }

    // 提交任务的通用模板函数
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type> {
        
        using return_type = typename std::result_of<F(Args...)>::type;
        
        // 创建任务包装器
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
        std::future<return_type> result = task->get_future();
        
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            if (stop) {
                throw std::runtime_error("ThreadPool已停止，无法添加新任务");
            }
            
            // 将任务添加到队列
            tasks.emplace([task]() { (*task)(); });
        }
        
        condition.notify_one();
        return result;
    }

    // 析构函数
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        
        condition.notify_all();
        
        for (std::thread &worker : workers) {
            worker.join();
        }
    }

    // 获取当前任务队列大小
    size_t getQueueSize() const {
        std::unique_lock<std::mutex> lock(queueMutex);
        return tasks.size();
    }

    // 获取线程数量
    size_t getThreadCount() const {
        return workers.size();
    }

private:
    std::vector<std::thread> workers;           // 工作线程
    std::queue<std::function<void()>> tasks;    // 任务队列
    
    mutable std::mutex queueMutex;              // 队列互斥锁
    std::condition_variable condition;          // 条件变量
    bool stop;                                  // 停止标志
};