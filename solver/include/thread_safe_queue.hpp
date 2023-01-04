#ifndef THREAD_SAFE_QUEUE_HPP
#define THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <mutex>
#include <queue>
#include <type_traits>

template <typename T>
using isSupportedType =
    std::enable_if_t<std::is_default_constructible_v<T> &&
                     std::is_move_assignable_v<T>>;

template <typename Item>
class ThreadSafeQueue
{
    public:
        template <typename Item_ = Item, typename = isSupportedType<Item_>>
        ThreadSafeQueue(){}

        void push(Item value)
        {
            {
                std::scoped_lock lock{_mtx};
                _queue.push(std::move(value));
            }
            _cond.notify_one();
        }

        void pop(Item& value)
        {
            std::unique_lock lock{_mtx};
            _cond.wait(lock, [this]{return !_queue.empty();});
            value = _queue.front();
            _queue.pop();
        }

        bool empty() const
        {
            std::scoped_lock lock{_mtx};
            return _queue.empty();
        }

    private:
        mutable std::mutex _mtx;
        std::condition_variable _cond;
        std::queue<Item> _queue;
};

#endif // THREAD_SAFE_QUEUE_HPP