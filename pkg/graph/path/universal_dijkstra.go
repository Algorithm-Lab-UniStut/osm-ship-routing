package path

import (
	"container/heap"
	"fmt"
	"math"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

// UniversalDijkstra implements various path finding algorithms which all are based on Dijkstra.
// it can be used for plain Dijsktra, Bidirectional search, A* (and maybe more will come).
// Implements the Navigator Interface.
type UniversalDijkstra struct {
	// check if pointers are needed/better
	g                       graph.Graph
	visitedNodes            []bool                   // Array which indicates if a node (defined by index) was visited in the forward search
	backwardVisitedNodes    []bool                   // Array which indicates if a node (defined by index) was visited in the backward search
	searchSpace             []*DijkstraItem          // search space, a map really reduces performance. If node is also visited, this can be seen as "settled"
	backwardSearchSpace     []*DijkstraItem          // search space for the backward search
	origin                  graph.NodeId             // the origin of the current search
	destination             graph.NodeId             // the distination of the current search
	useHeuristic            bool                     // flag indicating if heuristic (remaining distance) should be used (AStar implementation)
	bidirectional           bool                     // flag indicating if search should be done from both sides
	useHotStart             bool                     // flag indicating if the previously cached results should get used
	ignoreNodes             []bool                   // store for each node ID if it is ignored. A map would also be viable (for performance aspect) to achieve this
	bidirectionalConnection *BidirectionalConnection // contains the connection between the forward and backward search (if done bidirecitonal). If no connection is found, this is nil
	pqPops                  int                      // store the amount of Pops which were performed on the priority queue for the computed search
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

// Create a new Dijkstra instance with the given graph g
func NewUniversalDijkstra(g graph.Graph) *UniversalDijkstra {
	return &UniversalDijkstra{g: g, costUpperBound: math.MaxInt, maxNumSettledNodes: math.MaxInt, ignoreNodes: make([]bool, g.NodeCount()), origin: -1, destination: -1}
}

// Creates a new item which describes a connection between a forward and a backward search
// The nodeId is the id of the node which connects both searches, the predecessor represents the node of the forward search and the successor the node of the backward search.
// The distance is the length needed from both searches to reach this node
func NewBidirectionalConnection(nodeId, predecessor, successor graph.NodeId, distance int) *BidirectionalConnection {
	return &BidirectionalConnection{nodeId: nodeId, predecessor: predecessor, successor: successor, distance: distance}
}

// Compute the shortest path from the origin to the destination.
// It returns the length of the found path
// If no path was found, it returns -1
// If the path to all possible target is calculated (set target to -1), it returns 0
func (d *UniversalDijkstra) ComputeShortestPath(origin, destination graph.NodeId) int {
	if d.useHotStart && origin == d.origin && !d.bidirectional && destination >= 0 && d.visitedNodes[destination] {
		// hot start
		if destination < 0 {
			panic("Can't use Hot start when calculating path to everywhere")
		}
		if d.bidirectional {
			panic("Can't use Hot start for bidirectional search")
		}

		if origin == d.origin && d.visitedNodes[destination] {
			// TODO think about storing/using old priority queue to not recalculate the first settled nodes
			if d.debugLevel >= 1 {
				fmt.Printf("Using hot start %v -> %v, distance is %v\n", origin, destination, d.searchSpace[destination].distance)
			}
			//d.destination = destination
			d.pqPops = 0
			//d.bidirectionalConnection = nil
			return d.searchSpace[destination].distance
		}
	}
	if d.useHeuristic && d.bidirectional {
		panic("AStar doesn't work bidirectional so far.")
	}
	if destination < 0 && d.bidirectional {
		panic("Can't use bidirectional search with no specified destination")
	}

	d.initializeSearch(origin, destination)
	heuristic := 0
	if d.useHeuristic {
		heuristic = d.g.EstimateDistance(origin, d.destination)
	}
	originItem := NewDijkstraItem(origin, 0, -1, heuristic, FORWARD)
	pq := NewMinPath(originItem)
	// Initialize
	d.settleNode(originItem)

	// for bidirectional algorithm
	if d.bidirectional {
		destinationItem := NewDijkstraItem(destination, 0, -1, 0, BACKWARD)
		heap.Push(pq, destinationItem)
		// Initialize
		d.settleNode(destinationItem)
	}

	numSettledNodes := 0
	for pq.Len() > 0 {
		currentNode := heap.Pop(pq).(*DijkstraItem)
		d.pqPops++
		if d.debugLevel >= 1 {
			fmt.Printf("Settling node %v, direction: %v, distance %v\n", currentNode.NodeId, currentNode.searchDirection, currentNode.distance)
		}
		d.settleNode(currentNode)
		numSettledNodes++
		if d.costUpperBound < currentNode.Priority() || d.maxNumSettledNodes < numSettledNodes {
			// Each following node exeeds the max allowed cost or the number of allowed nodes is reached
			// Stop search
			if d.debugLevel >= 1 {
				fmt.Printf("Exceeded limits - cost upper bound: %v, current cost: %v, max settled nodes: %v, current settled nodes: %v\n", d.costUpperBound, currentNode.Priority(), d.maxNumSettledNodes, numSettledNodes)
			}
			return -1
		}
		if !d.considerArcFlags && d.bidirectionalConnection != nil && d.isFullySettled(currentNode.NodeId) {
			// node with lowest priority is the current connection node
			// -> every edge increases cost/priority
			// -> this has to be the shortest path --> wrong, if one edge is (really) long
			// Correction: current node (with lowest possible distance) is already greater than connection distance
			// -> connection has to be the lowest one
			// -> Wrong, this condition just made a "normal" Dijkstra out of the Bidirectional Dijkstra
			// Correction: check if the node was already settled in both directions, this is a connection
			// the connection item contains the information, which one is the real connection (this has not to be the current one)
			break
		}
		if d.considerArcFlags && d.bidirectionalConnection != nil && currentNode.Priority() > d.bidirectionalConnection.distance {
			// exit condition for contraction hierarchies
			// if path is directed, it is not enough to test if node is settled from both sides, since the direction can block to reach the node from one side
			// for normal Dijkstra, this would force that the search space is in the bidirectional search as big as unidirecitonal search
			// check if the current visited node has already a higher priority (distance) than the connection. If this is the case, no lower connection can get found
			break
		}

		if destination != -1 {
			if currentNode.searchDirection == FORWARD && currentNode.NodeId == destination {
				break
			} else if d.bidirectional && currentNode.searchDirection == BACKWARD && currentNode.NodeId == origin {
				// not necessary? - should be catched in bidirectionalConnection
				// appearently this can happen (at least for contraction hierarchies when calculated bidirecitonal)
				// -> first path/conneciton is found which has higher distance than possible (directed) path from destinaiton to source
				break
			}
		}

		d.relaxEdges(currentNode, pq)
	}

	if d.debugLevel >= 1 {
		fmt.Printf("Finished search\n")
	}

	if destination == -1 {
		// calculated every distance from source to each possible target
		//dijkstra.settledNodes = nodes
		return 0
	}

	if d.bidirectional {
		if d.bidirectionalConnection == nil {
			// no valid path found
			return -1
		}
		length := d.bidirectionalConnection.distance
		return length
	}

	if d.searchSpace[destination] == nil {
		// no valid path found
		if d.debugLevel >= 1 {
			fmt.Printf("No path found\n")
		}
		return -1
	}

	if d.debugLevel >= 1 {
		fmt.Printf("Found path with distance %v\n", d.searchSpace[destination].distance)
	}

	return d.searchSpace[destination].distance
}

// Get the path of a previous computation. This contains the nodeIds which lie on the path from source to destination
func (d *UniversalDijkstra) GetPath(origin, destination int) []int {
	if destination == -1 {
		// path to each node was calculated
		// return nothing
		return make([]int, 0)
	}
	path := make([]int, 0)
	if d.bidirectional {
		if d.bidirectionalConnection == nil {
			// no path found
			return make([]int, 0)
		}
		if d.debugLevel >= 1 {
			fmt.Printf("con: %v, pre: %v, suc: %v\n", d.bidirectionalConnection.nodeId, d.bidirectionalConnection.predecessor, d.bidirectionalConnection.successor)
		}
		for nodeId := d.bidirectionalConnection.predecessor; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		slice.ReverseIntInPlace(path)
		path = append(path, d.bidirectionalConnection.nodeId)
		for nodeId := d.bidirectionalConnection.successor; nodeId != -1; nodeId = d.backwardSearchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
	} else {
		if d.searchSpace[destination] == nil {
			// no path found
			return make([]int, 0)
		}
		for nodeId := destination; nodeId != -1; nodeId = d.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		// reverse path (to create the correct direction)
		slice.ReverseIntInPlace(path)
	}

	return path
}

// Returns the search space of a previous computation. This contains all items which were settled.
func (d *UniversalDijkstra) GetSearchSpace() []*DijkstraItem {
	visitedNodes := 0
	for _, visited := range d.visitedNodes {
		if visited {
			visitedNodes++
		}
	}
	for _, visited := range d.backwardVisitedNodes {
		if visited {
			visitedNodes++
		}
	}
	searchSpace := make([]*DijkstraItem, visitedNodes)
	i := 0
	for nodeId, visited := range d.visitedNodes {
		if visited {
			searchSpace[i] = d.searchSpace[nodeId]
			i++
		}
	}
	for nodeId, visited := range d.backwardVisitedNodes {
		if visited {
			searchSpace[i] = d.backwardSearchSpace[nodeId]
			i++
		}
	}
	return searchSpace
}

// Initialize a new search
// This resets the search space and visited nodes (and all other leftovers of a previous search)
func (d *UniversalDijkstra) initializeSearch(origin, destination graph.NodeId) {
	if d.debugLevel >= 99 {
		var visitedNodes []graph.NodeId
		for node, visited := range d.visitedNodes {
			if visited {
				visitedNodes = append(visitedNodes, node)
			}
		}
		fmt.Printf("visited nodes: %v\n", visitedNodes)
	}
	d.searchSpace = make([]*DijkstraItem, d.g.NodeCount())
	d.backwardSearchSpace = make([]*DijkstraItem, d.g.NodeCount())
	d.visitedNodes = make([]bool, d.g.NodeCount())
	d.backwardVisitedNodes = make([]bool, d.g.NodeCount())
	d.origin = origin
	d.destination = destination
	d.pqPops = 0
	d.bidirectionalConnection = nil
}

// Settle the given node item
func (d *UniversalDijkstra) settleNode(node *DijkstraItem) {
	searchSpace, visitedNodes := d.searchSpace, d.visitedNodes
	if node.searchDirection == BACKWARD {
		searchSpace, visitedNodes = d.backwardSearchSpace, d.backwardVisitedNodes
	}
	searchSpace[node.NodeId] = node
	visitedNodes[node.NodeId] = true
}

// Check whether the given node item is settled in both search directions
func (d *UniversalDijkstra) isFullySettled(nodeId graph.NodeId) bool {
	visited := d.visitedNodes[nodeId]
	backwardVisited := d.backwardVisitedNodes[nodeId]
	return visited && backwardVisited
}

// Relax the Edges for the given node item and add the new nodes to the MinPath priority queue
func (d *UniversalDijkstra) relaxEdges(node *DijkstraItem, pq *MinPath) {
	for _, arc := range d.g.GetArcsFrom(node.NodeId) {
		if d.considerArcFlags && !arc.ArcFlag() {
			// ignore this arc
			if d.debugLevel >= 1 {
				fmt.Printf("Ignore Edge %v -> %v\n", node.NodeId, arc.Destination())
			}
			continue
		}
		if d.ignoreNodes[arc.Destination()] {
			// ignore this node
			if d.debugLevel >= 1 {
				fmt.Printf("Ignore Edge %v -> %v, because target is in ignore list\n", node.NodeId, arc.Destination())
			}
			continue
		}

		if d.debugLevel >= 1 {
			fmt.Printf("Relax Edge %v -> %v\n", node.NodeId, arc.Destination())
		}
		successor := arc.Destination()
		searchSpace, inverseSearchSpace := d.searchSpace, d.backwardSearchSpace
		if node.searchDirection == BACKWARD {
			searchSpace, inverseSearchSpace = inverseSearchSpace, searchSpace
		}
		if d.bidirectional && inverseSearchSpace[successor] != nil {
			// store potential connection node, needed for later
			// this is a "real" copy, not just a pointer since it get changed now
			connection := inverseSearchSpace[successor]
			connectionPredecessor := node.NodeId
			connectionSuccessor := connection.predecessor
			if node.searchDirection == BACKWARD {
				// Default direction is forward
				// if it is backward, switch successor and predecessor
				connectionPredecessor, connectionSuccessor = connectionSuccessor, connectionPredecessor
			}
			con := NewBidirectionalConnection(connection.NodeId, connectionPredecessor, connectionSuccessor, node.distance+arc.Cost()+connection.distance)
			if d.bidirectionalConnection == nil || con.distance < d.bidirectionalConnection.distance {
				d.bidirectionalConnection = con
			}
		}
		if searchSpace[successor] == nil {
			cost := node.distance + arc.Cost()
			heuristic := 0
			if d.useHeuristic {
				heuristic = d.g.EstimateDistance(successor, d.destination)
			}
			nextNode := NewDijkstraItem(successor, cost, node.NodeId, heuristic, node.searchDirection)
			searchSpace[successor] = nextNode
			heap.Push(pq, nextNode)
		} else {
			if updatedPriority := node.distance + arc.Cost() + searchSpace[successor].heuristic; updatedPriority < searchSpace[successor].Priority() {
				pq.update(searchSpace[successor], node.distance+arc.Cost())
				searchSpace[successor].predecessor = node.NodeId
			}
		}
	}
}

// Specify whether a heuristic for path finding (AStar) should be used
func (d *UniversalDijkstra) SetUseHeuristic(useHeuristic bool) {
	d.useHeuristic = useHeuristic
}

// Specify wheter the search should be done in both directions
func (d *UniversalDijkstra) SetBidirectional(bidirectional bool) {
	d.bidirectional = bidirectional
}

// SPecify if the arc flags of the Arcs should be considered.
// If set to false, Arcs with a negative flag will be ignored by the path finding
func (d *UniversalDijkstra) SetConsiderArcFlags(considerArcFlags bool) {
	d.considerArcFlags = considerArcFlags
}

// Set the upper cost for a valid path from source to target
func (d *UniversalDijkstra) SetCostUpperBound(costUpperBound int) {
	d.costUpperBound = costUpperBound
}

// Set the maximum number of nodes that can get settled before the search is terminated
func (d *UniversalDijkstra) SetMaxNumSettledNodes(maxNumSettledNodes int) {
	d.maxNumSettledNodes = maxNumSettledNodes
}

func (d *UniversalDijkstra) SetIgnoreNodes(nodes []graph.NodeId) {
	d.ignoreNodes = make([]bool, d.g.NodeCount())
	for _, node := range nodes {
		d.ignoreNodes[node] = true
	}
	// invalidate previously calculated results
	d.visitedNodes = make([]bool, d.g.NodeCount())
}

func (d *UniversalDijkstra) SetHotStart(useHotStart bool) {
	d.useHotStart = useHotStart
}

// Returns the amount of priority queue/heap pops which werer performed during the search
func (d *UniversalDijkstra) GetPqPops() int { return d.pqPops }

// Set the debug level to show different debug messages.
// If it is 0, no debug messages are printed
func (d *UniversalDijkstra) SetDebugLevel(level int) {
	d.debugLevel = level
}
