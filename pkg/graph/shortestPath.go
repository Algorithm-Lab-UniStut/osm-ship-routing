package graph

import (
	"container/heap"
)

type PriorityQueueItem struct {
	itemId      int // node id of this item
	priority    int // distance from origin to this node
	predecessor int // node id of the predecessor
	index       int // index of the item in the heap
}

// A PriorityQueue implements the heap.Interface and hold PriorityQueueItems
type PriorityQueue []*PriorityQueueItem

func (h PriorityQueue) Len() int {
	return len(h)
}

func (h PriorityQueue) Less(i, j int) bool {
	// MinHeap implementation
	return h[i].priority < h[j].priority
}

func (h PriorityQueue) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index, h[j].index = i, j
}

func (h *PriorityQueue) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*PriorityQueueItem)
	pqItem.index = n
	*h = append(*h, pqItem)
}

func (h *PriorityQueue) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *PriorityQueue) update(pqItem *PriorityQueueItem, newPriority int) {
	pqItem.priority = newPriority
	heap.Fix(h, pqItem.index)
}
