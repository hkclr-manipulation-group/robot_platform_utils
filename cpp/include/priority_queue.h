#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <queue>
#include <chrono>
#include <vector>
#include <memory>
#include <iostream>

namespace robot::platform {
    template <typename T>
    class PriorityQueue{
        public:
            struct QueueData {
                int id = -1;
                int priority = 0;
                uint64_t seq = 0;
                long received_at = 0;
                T data;
            };

            struct QueueDataCompare {
                bool operator()(const std::unique_ptr<QueueData>& lhs, const std::unique_ptr<QueueData>& rhs) const {
                    if (lhs->priority != rhs->priority) {
                        return lhs->priority < rhs->priority; 
                    }
                    if (lhs->seq != rhs->seq) {
                        return lhs->seq < rhs->seq; 
                    }
                    return lhs->received_at < rhs->received_at; 
                }
            };

            void push(std::unique_ptr<QueueData> data);
            std::unique_ptr<QueueData> pop();
            bool empty() const;
            void clear();
            size_t size() const;
            void print() const;
        private:
            std::priority_queue<std::unique_ptr<QueueData>, std::vector<std::unique_ptr<QueueData>>, QueueDataCompare> queue_;
    };

    template <typename T>
    void PriorityQueue<T>::push(std::unique_ptr<QueueData> data){
        queue_.push(std::move(data));
    }

    template <typename T>
    std::unique_ptr<typename PriorityQueue<T>::QueueData> PriorityQueue<T>::pop(){
        if (queue_.empty()) return nullptr;
        std::unique_ptr<QueueData> top = std::move(const_cast<std::unique_ptr<QueueData>&>(queue_.top()));
        queue_.pop();
        return top;
    }

    template <typename T>
    bool PriorityQueue<T>::empty() const{
        return queue_.empty();
    }

    template <typename T>
    void PriorityQueue<T>::clear(){
        queue_ = std::priority_queue<std::unique_ptr<QueueData>, std::vector<std::unique_ptr<QueueData>>, QueueDataCompare>();
    }

    template <typename T>
    size_t PriorityQueue<T>::size() const{
        return queue_.size();
    }

    template <typename T>
    void PriorityQueue<T>::print() const{
        std::priority_queue<std::unique_ptr<QueueData>, std::vector<std::unique_ptr<QueueData>>, QueueDataCompare> temp_queue = queue_;
        int queue_order = 0;
        while (!temp_queue.empty()){
            std::unique_ptr<QueueData> qd = std::move(temp_queue.top());
            std::cout << "queue order: " << queue_order << " id: " << qd->id << " priority: " << qd->priority << " seq: " << qd->seq << " received_at: " << qd->received_at << std::endl;
            queue_order++;
            temp_queue.pop();
        }
    }

} // namespace robot::platform
#endif