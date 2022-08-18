package queue

import (
	"container/heap"
)

type PriorityQueueItem struct {
	ItemId      int // node id of this item
	Priority    int // distance from origin to this node
	Predecessor int // node id of the predecessor
	Index       int // index of the item in the heap
}

// A PriorityQueue implements the heap.Interface and hold PriorityQueueItems
type PriorityQueue []*PriorityQueueItem

func NewPriorityQueueItem(itemId int, priority int, predecessor int) *PriorityQueueItem {
	return &PriorityQueueItem{ItemId: itemId, Priority: priority, Predecessor: predecessor, Index: -1}
}

func NewPriorityQueue(initialItem *PriorityQueueItem) *PriorityQueue {
	pq := make(PriorityQueue, 0)
	heap.Init(&pq)
	if initialItem != nil {
		heap.Push(&pq, initialItem)
	}
	return &pq
}

func (h PriorityQueue) Len() int {
	return len(h)
}

func (h PriorityQueue) Less(i, j int) bool {
	// MinHeap implementation
	return h[i].Priority < h[j].Priority
}

func (h PriorityQueue) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].Index, h[j].Index = i, j
}

func (h *PriorityQueue) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*PriorityQueueItem)
	pqItem.Index = n
	*h = append(*h, pqItem)
}

func (h *PriorityQueue) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.Index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *PriorityQueue) Update(pqItem *PriorityQueueItem, newPriority int) {
	pqItem.Priority = newPriority
	heap.Fix(h, pqItem.Index)
}
