package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

type PathItem interface {
	Id() graph.NodeId          // node id of this item in the graph
	Priority() int             // distance to origin of this node
	Predecessor() graph.NodeId // node id of the predecessor
	Index() int                // internal usage
	SetIndex(i int)            // set the index
	SetPriority(i int)
}

type DijkstraItem struct {
	nodeId      graph.NodeId // node id of this item in the graph
	distance    int          // distance to origin of this node
	heuristic   int          // estimated distance from node to destination
	predecessor graph.NodeId // node id of the predecessor
	index       int          // internal usage
}

// MinPath implements the heap.Interface to hold the PriorityQueue
type MinPath []*DijkstraItem

func NewDijkstraItem(nodeId graph.NodeId, distance int, predecessor graph.NodeId, heuristic int) *DijkstraItem {
	return &DijkstraItem{nodeId: nodeId, distance: distance, predecessor: predecessor, index: -1, heuristic: heuristic}
}

func NewMinPath(initialItem *DijkstraItem) *MinPath {
	pq := make(MinPath, 0)
	heap.Init(&pq)
	if initialItem != nil {
		heap.Push(&pq, initialItem)
	}
	return &pq
}

func (d DijkstraItem) Priority() int { return d.distance + d.heuristic }

/*
func (d DijkstraItem) Id() graph.NodeId          { return d.nodeId }
func (d DijkstraItem) Predecessor() graph.NodeId { return d.predecessor }
func (d DijkstraItem) Index() int                { return d.index }
func (d *DijkstraItem) SetIndex(i int)           { d.index = i }
func (d *DijkstraItem) SetPriority(priority int) { d.distance = priority }
*/

func (h MinPath) Len() int {
	return len(h)
}

func (h MinPath) Less(i, j int) bool {
	// MinHeap implementation
	return h[i].Priority() < h[j].Priority()
}

func (h MinPath) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index, h[j].index = i, j
}

func (h *MinPath) Push(item interface{}) {
	n := len(*h)
	pqItem := item.(*DijkstraItem)
	pqItem.index = n
	*h = append(*h, pqItem)
}

func (h *MinPath) Pop() interface{} {
	old := *h
	n := len(old)
	pqItem := old[n-1]
	old[n-1] = nil
	pqItem.index = -1 // for safety
	*h = old[0 : n-1]
	return pqItem
}

func (h *MinPath) update(pqItem *DijkstraItem, distance int) {
	pqItem.distance = distance
	heap.Fix(h, pqItem.index)
}
