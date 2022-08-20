package path

import (
	"container/heap"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

type UniversalDijkstra struct {
	// check if pointers are needed/better
	g                       graph.Graph
	visitedNodes            []bool
	searchSpace             []*DijkstraItem // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	distances               []int           // TODO: distance values for each node. Is this necessary?
	origin                  graph.NodeId    // the origin of the current search
	destination             graph.NodeId    // the distinaiton of the current search
	useHeuristic            bool            // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional           bool            // flag indicating if search shuld be done from both sides
	bidirectionalConnection *BidirectionalConnection
	considerArcFlags        bool
	costUpperBound          int
	maxNumSettledNodes      int
}

type BidirectionalConnection struct {
	nodeId      graph.NodeId
	predecessor graph.NodeId
	successor   graph.NodeId
	distance    int
}

func NewUniversalDijkstra(g graph.Graph, useHeuristic bool) *UniversalDijkstra {
	// heuristic is initially "nil"
	return &UniversalDijkstra{g: g, useHeuristic: useHeuristic, bidirectionalConnection: nil}
}

func NewBidirectionalConnection(nodeId, predecessor, successor graph.NodeId, distance int) *BidirectionalConnection {
	return &BidirectionalConnection{nodeId: nodeId, predecessor: predecessor, successor: successor, distance: distance}
}

func (d *UniversalDijkstra) InitializeSearch(origin, destination graph.NodeId) {
	d.visitedNodes = make([]bool, d.g.NodeCount())
	d.searchSpace = make([]*DijkstraItem, d.g.NodeCount())
	d.origin = origin
	d.destination = destination
	if d.bidirectional {
		d.bidirectionalConnection = nil
	}
}

func (d *UniversalDijkstra) SettleNode(node *DijkstraItem) {
	d.searchSpace[node.nodeId] = node
	d.visitedNodes[node.nodeId] = true
}

func (d *UniversalDijkstra) RelaxEdges(node *DijkstraItem, pq *MinPath) {
	for _, arc := range d.g.GetArcsFrom(node.nodeId) {
		successor := arc.Destination()
		if d.bidirectional && d.searchSpace[successor] != nil && d.searchSpace[successor].searchDirection != node.searchDirection {
			// store potential connection node, needed for later
			// this is a "real" copy, not just a pointer since it get changed now
			connection := d.searchSpace[successor]
			connectionPredecessor := node.nodeId
			connectionSuccessor := connection.predecessor
			var con *BidirectionalConnection
			if node.searchDirection == FORWARD {
				con = NewBidirectionalConnection(connection.nodeId, connectionPredecessor, connectionSuccessor, node.distance+arc.Cost()+connection.distance)
			} else {
				con = NewBidirectionalConnection(connection.nodeId, connectionSuccessor, connectionPredecessor, node.distance+arc.Cost()+connection.distance)
			}
			if d.bidirectionalConnection == nil || con.distance < d.bidirectionalConnection.distance {
				d.bidirectionalConnection = con
			}
		}
		if d.searchSpace[successor] == nil {
			cost := node.distance + arc.Cost()
			heuristic := 0
			if d.useHeuristic {
				heuristic = d.g.EstimateDistance(successor, d.destination)
			}
			nextNode := NewDijkstraItem(successor, cost, node.nodeId, heuristic, node.searchDirection)
			d.searchSpace[successor] = nextNode
			heap.Push(pq, nextNode)
		} else {
			if updatedPriority := node.distance + arc.Cost() + d.searchSpace[successor].heuristic; updatedPriority < d.searchSpace[successor].Priority() {
				pq.update(d.searchSpace[successor], node.distance+arc.Cost())
				d.searchSpace[successor].predecessor = node.nodeId
				d.searchSpace[successor].searchDirection = node.searchDirection
			}
		}
	}
}

func (dijkstra *UniversalDijkstra) GetShortestPath(origin, destination graph.NodeId) int {
	dijkstra.InitializeSearch(origin, destination)
	if dijkstra.useHeuristic && dijkstra.bidirectional {
		panic("AStar doesn't work bidirectional so far.")
	}
	if destination == -1 && dijkstra.bidirectional {
		panic("Can't use bidirectional search with no specified destination")
	}
	heuristic := 0
	if dijkstra.useHeuristic {
		heuristic = dijkstra.g.EstimateDistance(origin, dijkstra.destination)
	}
	pq := NewMinPath(NewDijkstraItem(origin, 0, -1, heuristic, FORWARD))

	// for bidirectional algorithm
	if dijkstra.bidirectional {
		heap.Push(pq, NewDijkstraItem(destination, 0, -1, heuristic, BACKWARD))
	}

	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*DijkstraItem)
		dijkstra.SettleNode(currentNode)
		if dijkstra.bidirectionalConnection != nil && currentNode.nodeId == dijkstra.bidirectionalConnection.nodeId {
			// node with lowest priority is the current connection node
			// -> every edge increases cost/priority
			// -> this has to be the shortest path
			break
		}

		if destination != -1 {
			if currentNode.searchDirection == FORWARD && currentNode.nodeId == destination {
				break
			} else if dijkstra.bidirectional && currentNode.searchDirection == BACKWARD && currentNode.nodeId == origin {
				// not necessary?
				break
			}
		}

		dijkstra.RelaxEdges(currentNode, pq)
	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		//dijkstra.settledNodes = nodes
		return 0
	}

	if dijkstra.bidirectional {
		if dijkstra.bidirectionalConnection == nil {
			// no valid path found
			return -1
		}
		length := dijkstra.bidirectionalConnection.distance
		return length
	}

	if dijkstra.searchSpace[destination] == nil {
		// no valid path found
		return -1
	}
	length := dijkstra.searchSpace[destination].distance
	return length
}

func (dijkstra *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	dijkstra.useHeuristic = useHeuristic
}

func (dijkstra *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	dijkstra.bidirectional = bidirectional
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
	path := dijkstra.extractPath(origin, destination)

	return path, length
}

func (d *UniversalDijkstra) extractPath(origin, destination int) []int {
	path := make([]int, 0)
	if d.bidirectional {
		for nodeId := d.bidirectionalConnection.predecessor; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		slice.ReverseIntInPlace(path)
		path = append(path, d.bidirectionalConnection.nodeId)
		for nodeId := d.bidirectionalConnection.successor; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
	} else {
		for nodeId := destination; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		// reverse path (to create the correct direction)
		slice.ReverseIntInPlace(path)
	}
	return path
}

func (d *UniversalDijkstra) GetSearchSpace() []graph.Node {
	searchSpace := make([]graph.Node, 0)
	for i := 0; i < len(d.visitedNodes); i++ {
		if d.visitedNodes[i] {
			dijkstraItem := d.searchSpace[i]
			node := d.g.GetNode(dijkstraItem.nodeId)
			searchSpace = append(searchSpace, node)
		}
	}
	return searchSpace
}

func (d *UniversalDijkstra) SetConsiderArcFlags(considerArcFlags bool) {
	d.considerArcFlags = considerArcFlags
}

func (d *UniversalDijkstra) SetCostUpperBound(costUpperBound int) {
	d.costUpperBound = costUpperBound
}

func (d *UniversalDijkstra) SetMaxNumSettledNodes(maxNumSettledNodes int) {
	d.maxNumSettledNodes = maxNumSettledNodes
}
