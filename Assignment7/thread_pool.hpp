#pragma once
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <condition_variable>

class thread_pool {
	using task_type = std::function<void()>;

	int size;
	std::vector<bool> working;
	std::vector<std::unique_ptr<std::thread>> workers;
	std::queue<task_type> tasks;
	std::mutex tasks_mutex;
	std::condition_variable condition;
	bool stop;
	int active_threads;

public:
	explicit thread_pool(int size) : size(size), working(size, false), workers(size), stop(false), active_threads(0) {
		for (int i = 0; i < size; ++i) {
			workers[i] = std::make_unique<std::thread>(&thread_pool::worker_thread, this, i);
		}
	}

	~thread_pool() {
		stop = true;
		condition.notify_all();
		for (int i = 0; i < size; ++i) {
			if (workers[i]->joinable()) {
				workers[i]->join();
			}
		}
	}

	template <typename Func, typename... Args>
	void add_task(Func&& func, Args&&... args) {
		std::unique_lock lock(tasks_mutex);
		tasks.emplace([=]() { func(args...); });
		condition.notify_one();
	}

	void wait() {
		std::unique_lock lock(tasks_mutex);
		condition.wait(lock, [&] {
			return tasks.empty() && active_threads == 0;
		});
	}

private:
	void worker_thread(int idx) {
		while (!stop) {
			task_type task;
			{
				std::unique_lock<std::mutex> lock(tasks_mutex);
				condition.wait(lock, [&] {
					return !tasks.empty() || stop;
				});
				if (stop) return;
				task = std::move(tasks.front());
				tasks.pop();
				working[idx] = true;
				++active_threads;
			}

			task();

			{
				std::unique_lock lock(tasks_mutex);
				--active_threads;
				working[idx] = false;
				if (tasks.empty() && active_threads == 0) {
					condition.notify_one();
				}
			}
		}
	}
};
