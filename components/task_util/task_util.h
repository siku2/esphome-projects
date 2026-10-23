#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <deque>
#include <utility>

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome::task_util {

/// Hands items from any task to the owner's loop().
template<typename T> class LoopInbox {
 public:
  LoopInbox(Component *owner, size_t capacity) : owner_(owner), capacity_(capacity) {}

  template<typename U> bool post(U &&item) {
    {
      LockGuard guard(this->mutex_);
      if (this->items_.size() >= this->capacity_)
        return false;
      this->items_.push_back(std::forward<U>(item));
    }
    this->owner_->enable_loop_soon_any_context();
    App.wake_loop_threadsafe();
    return true;
  }

  template<typename F> void drain(F &&handler) {
    while (true) {
      T item{};
      {
        LockGuard guard(this->mutex_);
        if (this->items_.empty())
          return;
        item = std::move(this->items_.front());
        this->items_.pop_front();
      }
      handler(std::move(item));
    }
  }

  template<typename P> void remove_if(P &&predicate) {
    LockGuard guard(this->mutex_);
    this->items_.erase(std::remove_if(this->items_.begin(), this->items_.end(), predicate), this->items_.end());
  }

 private:
  Component *owner_;
  size_t capacity_;
  Mutex mutex_;
  std::deque<T> items_;
};

/// Hands items from loop() to a worker task.
template<typename T> class TaskInbox {
 public:
  explicit TaskInbox(size_t capacity) : capacity_(capacity) {}

  void set_task(TaskHandle_t task) { this->task_ = task; }

  template<typename U> bool post(U &&item) {
    if (this->task_ == nullptr)
      return false;
    {
      LockGuard guard(this->mutex_);
      if (this->items_.size() >= this->capacity_)
        return false;
      this->items_.push_back(std::forward<U>(item));
    }
    xTaskNotifyGive(this->task_);
    return true;
  }

  bool pop(T &out) {
    LockGuard guard(this->mutex_);
    if (this->items_.empty())
      return false;
    out = std::move(this->items_.front());
    this->items_.pop_front();
    return true;
  }

  void wait() { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); }

 private:
  size_t capacity_;
  TaskHandle_t task_{nullptr};
  Mutex mutex_;
  std::deque<T> items_;
};

}  // namespace esphome::task_util
