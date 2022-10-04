package queue

import (
	"container/heap"
)

type Item struct {
	ItemId      int // node id of this item
	Priority    int // distance from origin to this node
	Predecessor int // node id of the predecessor
	Index       int // index of the item in the heap
}

// A Queue implements the heap.Interface and hold PriorityQueueItems
type Queue []*Item

func NewQueueItem(itemId int, priority int, predecessor int) *Item {
	return &Item{ItemId: itemId, Priority: priority, Predecessor: predecessor, Index: -1}
}

func NewQueue(initialItem *Item) *Queue {
	pq := make(Queue, 0)
	heap.Init(&pq)
	if initialItem != nil {
		heap.Push(&pq, initialItem)
	}
	return &pq
}

func (h Queue) Len() int {
	return len(h)
}

func (h Queue) Less(i, j int) bool {
	// MinHeap implementation
	return h[i].Priority < h[j].Priority
}

func (h Queue) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].Index, h[j].Index = i, j
}

func (h *Queue) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*Item)
	pqItem.Index = n
	*h = append(*h, pqItem)
}

func (h *Queue) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.Index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *Queue) Update(pqItem *Item, newPriority int) {
	pqItem.Priority = newPriority
	heap.Fix(h, pqItem.Index)
}
