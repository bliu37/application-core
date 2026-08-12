/******************************************************************************
* Copyright 2023 The Apollo Authors. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*****************************************************************************/

#pragma once
#include <unordered_map>
#include <map>
#include <list>

namespace apollo {
namespace tile_map_images_creator {

template<typename T>
class LRUcache {
public:

    /**@brief 更新LRU缓存值，如果元素存在则只移动优先级
     * @param matrix_index[in]:需要更新的元素
     * @return 存储空间已满，且插入元素在cache中不存在则返回false，其余返回true
     * */
    bool Update(const T& index);

    /**@brief 从Cache中弹出最近最早未被使用的元素
     * @param erased_index[out]:被移除的index值
     * @return 如果cache存储空间为零则返回false，其余返回true */
    bool Pop(T& index);

    /**@brief 初始化cache，所有内容会被清零
     * @param capacity: LRU cache存储容量上限
     * */
    void Init(int capacity);

    int Size() {
        return size_;
    }
private:

    bool insert(const T& index);
    bool erase(const T& index);

    int size_, capacity_;
    std::list<T> elements_;
    std::map<T, typename std::list<T>::iterator> element_position_memory_;
};

template<typename T>
void LRUcache<T>::Init(int capacity) {
    size_ = 0;
    capacity_ = capacity;
    elements_.clear();
    element_position_memory_.clear();
}

template<typename T>
bool LRUcache<T>::Update(const T& index) {
    if (element_position_memory_.count(index)) {
        erase(index);
        insert(index);
        return true;
    }
    return insert(index);
}

template<typename T>
bool LRUcache<T>::insert(const T& index) {
    if (size_ == capacity_) {
        return false;
    }
    elements_.push_back(index);
    typename std::list<T>::iterator pointer = elements_.end();
    element_position_memory_[index] = --pointer;
    size_++;
    return true;
}

template<typename T>
bool LRUcache<T>::Pop(T &index) {
    if (size_ == 0) {
        return false;
    }
    T index_erase = elements_.front();
    erase(index_erase);
    index = index_erase;
    return true;
}

template<typename T>
bool LRUcache<T>::erase(const T &index) {
    if (size_ == 0) {
        return false;
    }
    typename std::list<T>::iterator erase_position = element_position_memory_[index];
    elements_.erase(erase_position);
    element_position_memory_.erase(index);
    --size_;
    return true;
}

}
}