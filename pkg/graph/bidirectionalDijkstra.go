package graph

import (
	"container/heap"
	"math"
)

type BidirectionalDijkstra struct{}

func (bd BidirectionalDijkstra) ShortestPath(g Graph, origin, destination int) ([]int, int) {
	forwardDijkstraItems := make([]*PriorityQueueItem, g.NodeCount(), g.NodeCount())
	originItem := PriorityQueueItem{itemId: origin, priority: 0, predecessor: -1, index: -1}
	forwardDijkstraItems[origin] = &originItem

	backwardDijkstraItems := make([]*PriorityQueueItem, g.NodeCount(), g.NodeCount())
	destinationItem := PriorityQueueItem{itemId: destination, priority: 0, predecessor: -1, index: -1}
	backwardDijkstraItems[destination] = &destinationItem

	forwardPq := make(PriorityQueue, 0)
	heap.Init(&forwardPq)
	heap.Push(&forwardPq, forwardDijkstraItems[origin])

	backwardPq := make(PriorityQueue, 0)
	heap.Init(&backwardPq)
	heap.Push(&backwardPq, backwardDijkstraItems[destination])

	distance := math.MaxInt
	connectionNode := 0

	for len(forwardPq) > 0 && len(backwardPq) > 0 {
		currentForwardPqItem := heap.Pop(&forwardPq).(*PriorityQueueItem)
		currentForwardNodeId := currentForwardPqItem.itemId
		currentBackwardPqItem := heap.Pop(&backwardPq).(*PriorityQueueItem)
		currentBackwardNodeId := currentBackwardPqItem.itemId

		if forwardDijkstraItems[currentForwardNodeId].priority+backwardDijkstraItems[currentBackwardNodeId].priority >= distance {
			break
		}

		for _, edge := range g.GetEdgesFrom(currentForwardNodeId) {
			successor := edge.To

			if forwardDijkstraItems[successor] == nil {
				newPriority := forwardDijkstraItems[currentForwardNodeId].priority + edge.Distance
				pqItem := PriorityQueueItem{itemId: successor, priority: newPriority, predecessor: currentForwardNodeId, index: -1}
				forwardDijkstraItems[successor] = &pqItem
				heap.Push(&forwardPq, &pqItem)
			} else {
				if updatedDistance := forwardDijkstraItems[currentForwardNodeId].priority + edge.Distance; updatedDistance < forwardDijkstraItems[successor].priority {
					forwardPq.update(forwardDijkstraItems[successor], updatedDistance)
					forwardDijkstraItems[successor].predecessor = currentForwardNodeId
				}
			}

			if connection := backwardDijkstraItems[successor]; connection != nil && forwardDijkstraItems[currentForwardNodeId].priority+edge.Distance+connection.priority < distance {
				distance = forwardDijkstraItems[currentForwardNodeId].priority + edge.Distance + connection.priority
				forwardDijkstraItems[successor].predecessor = currentForwardNodeId
				connectionNode = successor
			}
		}

		if currentForwardNodeId == destination {
			break
		}

		for _, edge := range g.GetEdgesFrom(currentBackwardNodeId) {
			successor := edge.To

			if backwardDijkstraItems[successor] == nil {
				newPriority := backwardDijkstraItems[currentBackwardNodeId].priority + edge.Distance
				pqItem := PriorityQueueItem{itemId: successor, priority: newPriority, predecessor: currentBackwardNodeId, index: -1}
				backwardDijkstraItems[successor] = &pqItem
				heap.Push(&backwardPq, &pqItem)
			} else {
				if updatedDistance := backwardDijkstraItems[currentBackwardNodeId].priority + edge.Distance; updatedDistance < backwardDijkstraItems[successor].priority {
					backwardPq.update(backwardDijkstraItems[successor], updatedDistance)
					backwardDijkstraItems[successor].predecessor = currentBackwardNodeId
				}
			}

			if connection := forwardDijkstraItems[successor]; connection != nil && backwardDijkstraItems[currentBackwardNodeId].priority+edge.Distance+connection.priority < distance {
				distance = backwardDijkstraItems[currentBackwardNodeId].priority + edge.Distance + connection.priority
				backwardDijkstraItems[successor].predecessor = currentBackwardNodeId
				connectionNode = successor
			}
		}

		if currentBackwardNodeId == origin {
			break
		}
	}

	length := -1
	path := make([]int, 0)

	if distance < math.MaxInt {
		length = distance
		for nodeId := connectionNode; nodeId != -1; nodeId = forwardDijkstraItems[nodeId].predecessor {
			path = append([]int{nodeId}, path...)
		}
		// connectionNode may be doubled
		for nodeId := connectionNode; nodeId != -1; nodeId = backwardDijkstraItems[nodeId].predecessor {
			path = append(path, nodeId)
		}
	}
	return path, length
}
