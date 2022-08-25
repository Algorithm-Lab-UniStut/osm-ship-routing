package path

import (
	"container/heap"
	"fmt"
	"math"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

type UniversalDijkstra struct {
	// check if pointers are needed/better
	g            graph.Graph
	visitedNodes []graph.NodeId  // TODO: think about making this slice only store the visited node ids. Benefit: Less space needed. Loss: slice needs to get increases for each new node
	searchSpace  []*DijkstraItem // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	//distances               []int           // TODO: distance values for each node. Is this necessary?
	origin                  graph.NodeId // the origin of the current search
	destination             graph.NodeId // the distinaiton of the current search
	pathLength              int          // length of the computed path (-1 if no path found)
	useHeuristic            bool         // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional           bool         // flag indicating if search shuld be done from both sides
	bidirectionalConnection *BidirectionalConnection
	considerArcFlags        bool
	costUpperBound          int
	maxNumSettledNodes      int
	debugLevel              int
}

type BidirectionalConnection struct {
	nodeId      graph.NodeId
	predecessor graph.NodeId
	successor   graph.NodeId
	distance    int
}

func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	ud := &UniversalDijkstra{g: g, visitedNodes: make([]graph.NodeId, 0), searchSpace: make([]*DijkstraItem, g.NodeCount()) /*distances: make([]int, g.NodeCount()), */, bidirectionalConnection: nil, costUpperBound: math.MaxInt, maxNumSettledNodes: math.MaxInt, pathLength: -1}
	/*for i := range ud.distances {
		ud.distances[i] = -1
	}*/
	return ud
}

func NewBidirectionalConnection(nodeId, predecessor, successor graph.NodeId, distance int) *BidirectionalConnection {
	return &BidirectionalConnection{nodeId: nodeId, predecessor: predecessor, successor: successor, distance: distance}
}

func (d *UniversalDijkstra) InitializeSearch(origin, destination graph.NodeId) {
	if d.debugLevel >= 1 {
		fmt.Printf("visited nodes: %v\n", d.visitedNodes)
	}
	for _, previouslyVisitedNode := range d.visitedNodes {
		//d.distances[previouslyVisitedNode] = -1
		d.searchSpace[previouslyVisitedNode] = nil
		for _, arc := range d.g.GetArcsFrom(previouslyVisitedNode) {
			// also reset the potential other enqueued nodes
			d.searchSpace[arc.Destination()] = nil
		}
	}
	for i := 0; i < len(d.searchSpace); i++ {
		if d.searchSpace[i] != nil {
			// remove later
			fmt.Printf("%v node is not nil, is %+v\n", i, d.searchSpace[i])
			panic("domain initially not nil.")
		}
	}
	d.visitedNodes = make([]graph.NodeId, 0)
	d.origin = origin
	d.destination = destination
	if d.bidirectional {
		d.bidirectionalConnection = nil
	}
}

func (d *UniversalDijkstra) SettleNode(node *DijkstraItem) {
	if d.debugLevel >= 1 {
		fmt.Printf("Settle Node %v\n", node.NodeId)
	}
	d.searchSpace[node.NodeId] = node
	//d.distances[node.nodeId] = node.distance
	if !slice.Contains(d.visitedNodes, node.NodeId) {
		// would be good if finding a better way here (not iterating over the whole slice)
		// maybe make visitedNodes a Set (implemented with a map)
		d.visitedNodes = append(d.visitedNodes, node.NodeId)

	}
}

func (d *UniversalDijkstra) RelaxEdges(node *DijkstraItem, pq *MinPath) {
	if d.debugLevel >= 1 {
		fmt.Printf("Relax Edges for node %v\n", node.NodeId)
	}
	for _, arc := range d.g.GetArcsFrom(node.NodeId) {
		if d.considerArcFlags && !arc.ArcFlag() {
			// ignore this arc
			continue
		}
		if d.debugLevel >= 1 {
			fmt.Printf("Relax Edge %v -> %v", node.NodeId, arc.Destination())
		}
		successor := arc.Destination()
		if d.bidirectional && d.searchSpace[successor] != nil && d.searchSpace[successor].searchDirection != node.searchDirection {
			// store potential connection node, needed for later
			// this is a "real" copy, not just a pointer since it get changed now
			connection := d.searchSpace[successor]
			connectionPredecessor := node.NodeId
			connectionSuccessor := connection.predecessor
			var con *BidirectionalConnection
			if node.searchDirection == FORWARD {
				con = NewBidirectionalConnection(connection.NodeId, connectionPredecessor, connectionSuccessor, node.distance+arc.Cost()+connection.distance)
			} else {
				con = NewBidirectionalConnection(connection.NodeId, connectionSuccessor, connectionPredecessor, node.distance+arc.Cost()+connection.distance)
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
			nextNode := NewDijkstraItem(successor, cost, node.NodeId, heuristic, node.searchDirection)
			d.searchSpace[successor] = nextNode
			heap.Push(pq, nextNode)
		} else {
			if updatedPriority := node.distance + arc.Cost() + d.searchSpace[successor].heuristic; updatedPriority < d.searchSpace[successor].Priority() {
				pq.update(d.searchSpace[successor], node.distance+arc.Cost())
				d.searchSpace[successor].predecessor = node.NodeId
				d.searchSpace[successor].searchDirection = node.searchDirection
			}
		}
	}
}

func (dijkstra *UniversalDijkstra) ComputeShortestPath(origin, destination graph.NodeId) int {
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
	originItem := NewDijkstraItem(origin, 0, -1, heuristic, FORWARD)
	pq := NewMinPath(originItem)
	// Initialize
	dijkstra.SettleNode(originItem)

	// for bidirectional algorithm
	if dijkstra.bidirectional {
		destinationItem := NewDijkstraItem(destination, 0, -1, heuristic, BACKWARD)
		heap.Push(pq, destinationItem)
		// Initialize
		dijkstra.SettleNode(destinationItem)
	}

	numSettledNodes := 0
	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*DijkstraItem)
		dijkstra.SettleNode(currentNode)
		numSettledNodes++
		if dijkstra.costUpperBound < currentNode.Priority() || dijkstra.maxNumSettledNodes < numSettledNodes {
			// Each following node exeeds the max allowed cost or the number of allowed nodes is reached
			// Stop search
			dijkstra.pathLength = -1
			return -1
		}
		if dijkstra.bidirectionalConnection != nil && dijkstra.bidirectionalConnection.distance < currentNode.Priority() {
			// node with lowest priority is the current connection node
			// -> every edge increases cost/priority
			// -> this has to be the shortest path --> wrong, if one edge is (really) long
			// Correction: current node (with lowest possible distance) is already greater than connection distance
			// -> connection has to be the lowest one
			break
		}

		if destination != -1 {
			if currentNode.searchDirection == FORWARD && currentNode.NodeId == destination {
				break
			} else if dijkstra.bidirectional && currentNode.searchDirection == BACKWARD && currentNode.NodeId == origin {
				// not necessary?
				break
			}
		}

		dijkstra.RelaxEdges(currentNode, pq)
	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		//dijkstra.settledNodes = nodes
		dijkstra.pathLength = 0
		return 0
	}

	if dijkstra.bidirectional {
		if dijkstra.bidirectionalConnection == nil {
			// no valid path found
			dijkstra.pathLength = -1
			return -1
		}
		length := dijkstra.bidirectionalConnection.distance
		dijkstra.pathLength = length
		return length
	}

	if dijkstra.searchSpace[destination] == nil {
		// no valid path found
		dijkstra.pathLength = -1
		return -1
	}
	length := dijkstra.searchSpace[destination].distance
	dijkstra.pathLength = length
	return length
}

func (dijkstra *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	dijkstra.useHeuristic = useHeuristic
}

func (dijkstra *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	dijkstra.bidirectional = bidirectional
}

func (dijkstra *UniversalDijkstra) GetPath(origin, destination int) []int {
	if destination == -1 {
		// path to each node was calculated
		// return nothing
		return make([]int, 0)
	}
	if dijkstra.pathLength == -1 {
		// no path found
		return make([]int, 0)
	}
	path := dijkstra.extractComputedPath(origin, destination)

	return path
}

func (d *UniversalDijkstra) extractComputedPath(origin, destination int) []int {
	path := make([]int, 0)
	if d.bidirectional {
		if d.debugLevel >= 1 {
			fmt.Printf("con: %v, pre: %v, suc: %v\n", d.bidirectionalConnection.nodeId, d.bidirectionalConnection.predecessor, d.bidirectionalConnection.successor)
		}
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

func (d *UniversalDijkstra) GetSearchSpace() []*DijkstraItem {
	searchSpace := make([]*DijkstraItem, len(d.visitedNodes))
	for i, visitedNode := range d.visitedNodes {
		searchSpace[i] = d.searchSpace[visitedNode]
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

func (d *UniversalDijkstra) SetDebugLevel(level int) {
	d.debugLevel = level
}
