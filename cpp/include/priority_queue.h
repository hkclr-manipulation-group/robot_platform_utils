#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <queue>
#include <chrono>
#include <vector>
#include <memory>
#include <iostream>
#include <algorithm>

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
            void removeById(int target_id);
        private:
            std::vector<std::unique_ptr<QueueData>> queue_;
    };

    template <typename T>
    void PriorityQueue<T>::push(std::unique_ptr<QueueData> data){
        queue_.push_back(std::move(data));
        // Keeps the vector arranged as a max-heap
        std::push_heap(queue_.begin(), queue_.end(), QueueDataCompare());
    }

    template <typename T>
    std::unique_ptr<typename PriorityQueue<T>::QueueData> PriorityQueue<T>::pop(){
        if (queue_.empty()) return nullptr;
        std::pop_heap(queue_.begin(), queue_.end(), QueueDataCompare());
        std::unique_ptr<QueueData> top = std::move(queue_.back());
        queue_.pop_back();
        return top;
    }

    template <typename T>
    bool PriorityQueue<T>::empty() const{
        return queue_.empty();
    }

    template <typename T>
    void PriorityQueue<T>::clear(){
        queue_.clear();
    }

    template <typename T>
    size_t PriorityQueue<T>::size() const{
        return queue_.size();
    }

    template <typename T>
    void PriorityQueue<T>::print() const{
        if (queue_.empty()) {
            std::cout << "Queue is empty." << std::endl;
            return;
        }
        std::vector<const QueueData*> temp_ptrs;
        temp_ptrs.reserve(queue_.size());
        for (const auto& item : queue_) {
            temp_ptrs.push_back(item.get());
        }
        std::sort(temp_ptrs.begin(), temp_ptrs.end(), [](const QueueData* lhs, const QueueData* rhs) {
            if (lhs->priority != rhs->priority) return lhs->priority > rhs->priority; 
            if (lhs->seq != rhs->seq) return lhs->seq > rhs->seq; 
            return lhs->received_at > rhs->received_at; 
        });
        int queue_order = 0;
        for (const auto* qd : temp_ptrs) {
            std::cout << "queue order: " << queue_order 
                      << " id: " << qd->id 
                      << " priority: " << qd->priority 
                      << " seq: " << qd->seq 
                      << " received_at: " << qd->received_at << std::endl;
            queue_order++;
        }
    }

    template <typename T>
    void PriorityQueue<T>::removeById(int target_id) {
        if (queue_.empty()) return;
        queue_.erase(
            std::remove_if(queue_.begin(), queue_.end(), 
                [target_id](const std::unique_ptr<QueueData>& item) {
                    return item->id == target_id;
                }
            ), 
            queue_.end()
        );

        std::make_heap(queue_.begin(), queue_.end(), QueueDataCompare());
    }

} // namespace robot::platform
#endif