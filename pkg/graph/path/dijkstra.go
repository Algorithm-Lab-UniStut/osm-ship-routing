package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type Dijkstra struct {
	g graph.Graph
}

func NewDijkstra(g graph.Graph) Dijkstra {
	return Dijkstra{g: g}
}

func (d Dijkstra) GetPath(origin, destination int) ([]int, int) {
	dijkstraItems := make([]*queue.PriorityQueueItem, d.g.NodeCount(), d.g.NodeCount())
	originItem := queue.NewPriorityQueueItem(origin, 0, -1) //{ItemId: origin, Priority: 0, Predecessor: -1, Index: -1}
	dijkstraItems[origin] = originItem

	pq := make(queue.PriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, dijkstraItems[origin])

	for len(pq) > 0 {
		currentPqItem := heap.Pop(&pq).(*queue.PriorityQueueItem)
		currentNodeId := currentPqItem.ItemId

		for _, arc := range d.g.GetArcsFrom(currentNodeId) {
			successor := arc.Destination()

			if dijkstraItems[successor] == nil {
				newPriority := dijkstraItems[currentNodeId].Priority + arc.Cost()
				pqItem := queue.NewPriorityQueueItem(successor, newPriority, currentNodeId) //{ItemId: successor, Priority: newPriority, Predecessor: currentNodeId, Index: -1}
				dijkstraItems[successor] = pqItem
				heap.Push(&pq, pqItem)
			} else {
				if updatedDistance := dijkstraItems[currentNodeId].Priority + arc.Cost(); updatedDistance < dijkstraItems[successor].Priority {
					pq.Update(dijkstraItems[successor], updatedDistance)
					dijkstraItems[successor].Predecessor = currentNodeId
				}
			}
		}

		if currentNodeId == destination {
			break
		}
	}

	length := -1           // by default a non-existing path has length -1
	path := make([]int, 0) // by default, a non-existing path is an empty slice
	if dijkstraItems[destination] != nil {
		length = dijkstraItems[destination].Priority
		for nodeId := destination; nodeId != -1; nodeId = dijkstraItems[nodeId].Predecessor {
			path = append([]int{nodeId}, path...)
		}
	}
	return path, length
}
