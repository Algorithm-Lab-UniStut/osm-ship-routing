package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type Dijkstra struct {
	g               graph.Graph
	dijkstraItems   []*queue.PriorityQueueItem
	pqPops          int
	pqUpdates       int
	edgeRelaxations int
}

func NewDijkstra(g graph.Graph) *Dijkstra {
	return &Dijkstra{g: g}
}

func (d *Dijkstra) ComputeShortestPath(origin, destination int) int {
	d.dijkstraItems = make([]*queue.PriorityQueueItem, d.g.NodeCount(), d.g.NodeCount())
	originItem := queue.NewPriorityQueueItem(origin, 0, -1) //{ItemId: origin, Priority: 0, Predecessor: -1, Index: -1}
	d.dijkstraItems[origin] = originItem

	pq := make(queue.PriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, d.dijkstraItems[origin])

	d.pqPops = 0
	d.pqUpdates = 0
	d.edgeRelaxations = 0
	for len(pq) > 0 {
		currentPqItem := heap.Pop(&pq).(*queue.PriorityQueueItem)
		currentNodeId := currentPqItem.ItemId
		d.pqPops++

		for _, arc := range d.g.GetArcsFrom(currentNodeId) {
			d.edgeRelaxations++
			successor := arc.Destination()

			if d.dijkstraItems[successor] == nil {
				newPriority := d.dijkstraItems[currentNodeId].Priority + arc.Cost()
				pqItem := queue.NewPriorityQueueItem(successor, newPriority, currentNodeId) //{ItemId: successor, Priority: newPriority, Predecessor: currentNodeId, Index: -1}
				d.dijkstraItems[successor] = pqItem
				heap.Push(&pq, pqItem)
				d.pqUpdates++
			} else {
				if updatedDistance := d.dijkstraItems[currentNodeId].Priority + arc.Cost(); updatedDistance < d.dijkstraItems[successor].Priority {
					pq.Update(d.dijkstraItems[successor], updatedDistance)
					d.pqUpdates++
					d.dijkstraItems[successor].Predecessor = currentNodeId
				}
			}
		}

		if currentNodeId == destination {
			break
		}
	}

	length := -1 // by default a non-existing path has length -1
	if d.dijkstraItems[destination] != nil {
		length = d.dijkstraItems[destination].Priority
	}
	return length
}

func (d *Dijkstra) GetPath(origin, destination graph.NodeId) []graph.NodeId {
	path := make([]graph.NodeId, 0) // by default, a non-existing path is an empty slice
	if d.dijkstraItems[destination] != nil {
		for nodeId := destination; nodeId != -1; nodeId = d.dijkstraItems[nodeId].Predecessor {
			path = append([]int{nodeId}, path...)
		}
	}
	return path
}

func (d *Dijkstra) GetSearchSpace() []*DijkstraItem {
	panic("not implemented")
}

func (d *Dijkstra) GetPqPops() int {
	return d.pqPops
}

func (d *Dijkstra) GetPqUpdates() int {
	return d.pqUpdates
}

func (d *Dijkstra) GetEdgeRelaxations() int {
	return d.edgeRelaxations
}
