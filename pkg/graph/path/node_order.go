package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

type OrderItem struct {
	nodeId             graph.NodeId
	edgeDifference     int
	processedNeighbors int
	index              int
}

// NodeOrder implements the heap.Interface to hold the PriorityQueue
type NodeOrder []*OrderItem

func NewOrderItem(nodeId graph.NodeId) *OrderItem {
	return &OrderItem{nodeId: nodeId, index: -1}
}

func NewNodeOrder(initialItem *OrderItem) *NodeOrder {
	pq := make(NodeOrder, 0)
	heap.Init(&pq)
	if initialItem != nil {
		heap.Push(&pq, initialItem)
	}
	return &pq
}

func (o OrderItem) Priority() int { return o.edgeDifference + o.processedNeighbors }

func (h NodeOrder) Len() int { return len(h) }

func (h NodeOrder) Less(i, j int) bool { return h[i].Priority() < h[j].Priority() }

func (h NodeOrder) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index, h[j].index = i, j
}

func (h *NodeOrder) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*OrderItem)
	pqItem.index = n
	*h = append(*h, pqItem)
}

func (h *NodeOrder) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *NodeOrder) update(pqItem *OrderItem, edgeDifference, processedNeighbors int) {
	pqItem.edgeDifference = edgeDifference
	pqItem.processedNeighbors = processedNeighbors
	heap.Fix(h, pqItem.index)
}

func (h *NodeOrder) Peek() interface{} {
	return (*h)[0]
}
