#include <algorithm>

#include "Utils/APQueue.h"

APQueue::APQueue(size_t graph_size)
    : open_map(graph_size, std::vector<ApexPathPairPtr>()) {

    std::make_heap(this->heap.begin(), this->heap.end(), this->more_than);
}

bool APQueue::empty() {
    return this->heap.empty();
}

ApexPathPairPtr APQueue::top() {
    return this->heap.back();
}

ApexPathPairPtr APQueue::pop() {
    // Pop from min heap
    std::pop_heap(this->heap.begin(), this->heap.end(), this->more_than);
    ApexPathPairPtr pp = this->heap.back();
    this->heap.pop_back();

    // Remove from open map
    std::vector<ApexPathPairPtr> &relevant_pps = this->open_map[pp->id];
    for (auto iter = relevant_pps.begin(); iter != relevant_pps.end(); ++iter) {
        if (pp == *iter) {
            relevant_pps.erase(iter);
            break;
        }
    }

    return pp;
}

void APQueue::insert(ApexPathPairPtr &pp) {
    // Insert to min heap
    this->heap.push_back(pp);
    std::push_heap(this->heap.begin(), this->heap.end(), this->more_than);

    // Insert to open map
    this->open_map[pp->id].push_back(pp);
}

std::vector<ApexPathPairPtr> &APQueue::get_open_pps(size_t id) {
	return this->open_map[id];
}
