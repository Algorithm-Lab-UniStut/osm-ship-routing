package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type UniversalDijkstra struct {
	// check if pointers are needed/better
	g            graph.Graph
	visitedNodes []graph.NodeId
	heuristic    []int
	path         []int                      // contains the shortest path of the last calculation
	settledNodes []*queue.PriorityQueueItem // search space, maybe make this a pointer?
}

func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	// heuristic is initially "nil"
	return &UniversalDijkstra{g: g, visitedNodes: make([]graph.NodeId, g.NodeCount())}
}

func (dijkstra *UniversalDijkstra) GetShortestPath(origin, destination graph.NodeId) int {
	//activeNodes := make([]graph.Node, 1, 1)
	dijkstra.visitedNodes = nil
	dijkstra.path = nil
	nodes := make([]*queue.PriorityQueueItem, dijkstra.g.NodeCount(), dijkstra.g.NodeCount())
	settledNodes := make([]bool, dijkstra.g.NodeCount(), dijkstra.g.NodeCount())

	startNode := queue.NewPriorityQueueItem(origin, 0, -1)
	nodes[origin] = startNode
	pq := queue.NewPriorityQueue(startNode)

	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*queue.PriorityQueueItem)
		settledNodes[currentNode.ItemId] = true
		dijkstra.visitedNodes = append(dijkstra.visitedNodes, currentNode.ItemId)

		if destination != -1 && settledNodes[destination] == true {
			break
		}

		for _, arc := range dijkstra.g.GetArcsFrom(currentNode.ItemId) {
			successor := arc.Destination()
			if nodes[successor] == nil {
				newPriority := currentNode.Priority + arc.Cost()
				nextNode := queue.NewPriorityQueueItem(successor, newPriority, currentNode.ItemId)
				nodes[successor] = nextNode
				heap.Push(pq, nextNode)
			} else {
				if updatedCost := currentNode.Priority + arc.Cost(); updatedCost < nodes[successor].Priority {
					pq.Update(nodes[successor], updatedCost)
					nodes[successor].Predecessor = currentNode.ItemId
				}
			}
		}

	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		dijkstra.settledNodes = nodes
	} else if nodes[destination] == nil {
		// no valid path found
		return -1
	}
	path := make([]int, 0)
	length := nodes[destination].Priority
	for nodeId := destination; nodeId != -1; nodeId = nodes[nodeId].Predecessor {
		path = append(path, nodeId)
	}
	// reverse path (to create the correct direction)
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	dijkstra.path = path

	return length
}

func (dijkstra *UniversalDijkstra) SetHeuristic(heuristic []int) {
	dijkstra.heuristic = heuristic
}

func (dijkstra *UniversalDijkstra) GetPath(origin, destination int) ([]int, int) {
	length := dijkstra.GetShortestPath(origin, destination)
	if dijkstra.path == nil {
		return make([]int, 0), length
	}
	return dijkstra.path, length
}
