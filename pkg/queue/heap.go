package queue

import (
	"container/heap"
	"strings"
)

type MinHeap[T Priorizable] struct {
	Queue   PriorityQueue // hold the priority queue
	Storage []T           // may or may not contain the items in the priority queue. Indexable directly by the ID
}

func NewMinHeap[T Priorizable](items []T) *MinHeap[T] {
	h := &MinHeap[T]{Storage: items}
	h.Queue = make(PriorityQueue, len(items))
	for i, item := range items {
		h.Queue[i] = item
		item.SetIndex(i)
	}
	heap.Init(&h.Queue)
	return h
}

type Priorizable interface {
	Priority() int
	Index() int
	SetIndex(index int)
	String() string
}

// Implements heap.Interface
type PriorityQueue []Priorizable

func (q PriorityQueue) Len() int           { return len(q) }
func (q PriorityQueue) Less(i, j int) bool { return q[i].Priority() < q[j].Priority() }
func (q PriorityQueue) Swap(i, j int) {
	q[i], q[j] = q[j], q[i]
	q[i].SetIndex(i)
	q[j].SetIndex(j)
}
func (q *PriorityQueue) Push(item any) {
	n := len(*q)
	pqItem := item.(Priorizable)
	pqItem.SetIndex(n)
	*q = append(*q, pqItem)
}
func (q *PriorityQueue) Pop() any {
	old := *q
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.SetIndex(-1) // for safety
	*q = old[:n-1]
	return item
}

func (h *MinHeap[T]) Len() int      { return h.Queue.Len() }
func (h *MinHeap[T]) Push(item T)   { heap.Push(&h.Queue, item) }
func (h *MinHeap[T]) Pop() T        { return heap.Pop(&h.Queue).(T) }
func (h *MinHeap[T]) Update(item T) { heap.Fix(&h.Queue, item.Index()) }
func (h *MinHeap[T]) Peek() T       { return h.Queue[0].(T) }
func (h *MinHeap[T]) PeekAt(index int) T {
	if index >= h.Len() {
		panic("index out of bounds")
	}
	return h.Queue[index].(T)
}
func (h *MinHeap[T]) Remove(index int) { heap.Remove(&h.Queue, index) }
func (h *MinHeap[T]) String() string {
	var sb strings.Builder
	for i := 0; i < h.Len(); i++ {
		item := h.PeekAt(i)
		sb.WriteString(item.String())
	}
	return sb.String()
}
