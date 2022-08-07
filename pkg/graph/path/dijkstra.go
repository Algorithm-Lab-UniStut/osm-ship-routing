package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type Dijkstra struct{}

func (d Dijkstra) ShortestPath(g graph.Graph, origin, destination int) ([]int, int) {
	dijkstraItems := make([]*queue.PriorityQueueItem, g.NodeCount(), g.NodeCount())
	originItem := queue.PriorityQueueItem{ItemId: origin, Priority: 0, Predecessor: -1, Index: -1}
	dijkstraItems[origin] = &originItem

	pq := make(queue.PriorityQueue, 0)
	heap.Init(&pq)
	heap.Push(&pq, dijkstraItems[origin])

	for len(pq) > 0 {
		currentPqItem := heap.Pop(&pq).(*queue.PriorityQueueItem)
		currentNodeId := currentPqItem.ItemId

		for _, edge := range g.GetEdgesFrom(currentNodeId) {
			successor := edge.To

			if dijkstraItems[successor] == nil {
				newPriority := dijkstraItems[currentNodeId].Priority + edge.Distance
				pqItem := queue.PriorityQueueItem{ItemId: successor, Priority: newPriority, Predecessor: currentNodeId, Index: -1}
				dijkstraItems[successor] = &pqItem
				heap.Push(&pq, &pqItem)
			} else {
				if updatedDistance := dijkstraItems[currentNodeId].Priority + edge.Distance; updatedDistance < dijkstraItems[successor].Priority {
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
