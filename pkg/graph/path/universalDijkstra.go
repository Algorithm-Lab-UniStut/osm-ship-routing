package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/queue"
)

type UniversalDijkstra struct {
	// check if pointers are needed/better
	g            graph.Graph
	visitedNodes []bool
	searchSpace  []*queue.PriorityQueueItem // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	heuristic    []int
}

func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	// heuristic is initially "nil"
	return &UniversalDijkstra{g: g, visitedNodes: make([]bool, g.NodeCount())}
}

func (d *UniversalDijkstra) InitializeSearch() {
	d.visitedNodes = make([]bool, d.g.NodeCount())
	d.searchSpace = make([]*queue.PriorityQueueItem, d.g.NodeCount())
}

func (d *UniversalDijkstra) SettleNode(node *queue.PriorityQueueItem) {
	d.searchSpace[node.ItemId] = node
	d.visitedNodes[node.ItemId] = true
}

func (d *UniversalDijkstra) RelaxEdges(node *queue.PriorityQueueItem, pq *queue.PriorityQueue) {
	for _, arc := range d.g.GetArcsFrom(node.ItemId) {
		successor := arc.Destination()
		if d.searchSpace[successor] == nil {
			newPriority := node.Priority + arc.Cost()
			nextNode := queue.NewPriorityQueueItem(successor, newPriority, node.ItemId)
			d.searchSpace[successor] = nextNode
			heap.Push(pq, nextNode)
		} else {
			if updatedCost := node.Priority + arc.Cost(); updatedCost < d.searchSpace[successor].Priority {
				pq.Update(d.searchSpace[successor], updatedCost)
				d.searchSpace[successor].Predecessor = node.ItemId
			}
		}
	}
}

func (dijkstra *UniversalDijkstra) GetShortestPath(origin, destination graph.NodeId) int {
	dijkstra.InitializeSearch()
	pq := queue.NewPriorityQueue(queue.NewPriorityQueueItem(origin, 0, -1))

	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*queue.PriorityQueueItem)
		dijkstra.SettleNode(currentNode)

		if destination != -1 && currentNode.ItemId == destination {
			break
		}

		dijkstra.RelaxEdges(currentNode, pq)
	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		//dijkstra.settledNodes = nodes
		return 0
	}
	if dijkstra.searchSpace[destination] == nil {
		// no valid path found
		return -1
	}
	length := dijkstra.searchSpace[destination].Priority
	return length
}

func (dijkstra *UniversalDijkstra) SetHeuristic(heuristic []int) {
	dijkstra.heuristic = heuristic
}

func (dijkstra *UniversalDijkstra) GetPath(origin, destination int) ([]int, int) {
	length := dijkstra.GetShortestPath(origin, destination)
	if destination == -1 {
		// path to each node was calculated
		// return nothing
		return make([]int, 0), 0
	}
	if length == -1 {
		// no path found
		return make([]int, 0), length
	}
	path := make([]int, 0)
	for nodeId := destination; nodeId != -1; nodeId = dijkstra.searchSpace[nodeId].Predecessor {
		path = append(path, nodeId)
	}
	// reverse path (to create the correct direction)
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}

	return path, length
}
