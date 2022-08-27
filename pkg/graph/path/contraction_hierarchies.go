package path

import (
	"bufio"
	"container/heap"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strings"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

// Provides the precomputation and query for shortest paths with the use of Contraction Hierarchies.
// Implements the Navigator Interface.
type ContractionHierarchies struct {
	g            graph.Graph        // graph to work on (for precomputation, this ha sto be a graph.DynamicGraph)
	dg           graph.DynamicGraph // Dynamic graph which is used for precomputation (to add shortcuts)
	dijkstra     *UniversalDijkstra // the dijkstra algorithm to perform the searches
	nodeOrdering []graph.NodeId     // the node ordering (in which order the nodes were contracted)
	orderOfNode  []int              // the order of the node ("reverse" node ordering). At which position the specified node was contracted

	// decide for one
	//shortcutMap map[graph.NodeId]map[graph.NodeId]graph.NodeId // a map of the shortcuts (from/source -> to/target -> via)
	shortcuts []Shortcut // array which contains all shortcuts

	addedShortcuts       map[int]int // debug information - stores the number of how many nodes introduced the specified amount of shortcuts. Key is the number of shortcuts, value is how many introduced them
	debugLevel           int         // the debug level - used for printing some informaiton
	graphFilename        string      // the filename were the file gets stored
	shortcutsFilename    string      // the filename were the shourtcuts gets stored
	nodeOrderingFilename string      // the filname were the node ordering gets stored
}

// Describes a shortcut.
// It contains the information of the source and target node and via which node this shortcut is spanned
type Shortcut struct {
	source graph.NodeId // the source node
	target graph.NodeId // the target node
	via    graph.NodeId // over which node this shortcut is spanned
}

// Create a new Contraciton Hierarchy.
// Before a query can get executed, the Precomputation has to be done
func NewContractionHierarchies(g graph.Graph, dijkstra *UniversalDijkstra) *ContractionHierarchies {
	return &ContractionHierarchies{g: g, dijkstra: dijkstra, graphFilename: "contracted_graph.fmi", shortcutsFilename: "shortcuts.txt", nodeOrderingFilename: "node_ordering.txt"}
}

// Create a new Contraction Hierarchy which is already initialized with the shortcuts and node ordering.
// This can directly start a new query
func NewContractionHierarchiesInitialized(g graph.Graph, dijkstra *UniversalDijkstra, shortcuts []Shortcut, nodeOrdering []int) *ContractionHierarchies {
	ch := NewContractionHierarchies(g, dijkstra)
	ch.SetShortcuts(shortcuts)
	ch.SetNodeOrdering(nodeOrdering)
	return ch
}

// Do the precomputation of the Contractin Hierarchies.
// This adds new Edges in the graph by adding shortcuts. The result will be a modified graph, a collection of shortcuts and the calculated node ordering.
// If givenNodeOrder is not nil, the OrderOption oo are ignored.
// givenNodeOrder predefines the order of the nodes.
// oo defines how the node ordering will be calculated.
func (ch *ContractionHierarchies) Precompute(givenNodeOrder []int, oo OrderOptions) {
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetBidirectional(false)
	ch.dijkstra.SetUseHeuristic(false) // TODO test true
	// maybe test without maxNumSettledNodes
	ch.dijkstra.SetMaxNumSettledNodes(20) // TODO maybe test 60 (or something else)
	ch.addedShortcuts = make(map[int]int)
	//ch.shortcutMap = make(map[graph.NodeId]map[graph.NodeId]graph.NodeId)
	ch.shortcuts = make([]Shortcut, 0)
	ch.nodeOrdering = make([]int, ch.g.NodeCount())
	ch.orderOfNode = make([]int, ch.g.NodeCount())
	for i := range ch.orderOfNode {
		ch.orderOfNode[i] = -1
	}
	if givenNodeOrder == nil && !oo.IsValid() {
		panic("Order Options are not valid")
	}
	dg, ok := ch.g.(graph.DynamicGraph)
	if !ok {
		panic("Given Graph is no dynamic graph. Adding Arcs will not be possible")
	}
	ch.dg = dg
	ch.dg.EnableAllArcs()
	if givenNodeOrder != nil {
		// ignore the order options
		// -> just overwrite them
		oo = MakeOrderOptions().SetDynamic(false).SetRandom(false).SetEdgeDifference(false).SetProcessedNeighbors(false).SetRandom(false)
	}
	if ch.debugLevel >= 1 {
		fmt.Printf("Compute Node Ordering\n")
	}
	pq := ch.computeInitialNodeOrder(givenNodeOrder, oo)
	if ch.debugLevel >= 1 {
		fmt.Printf("%v\n", pq)
		fmt.Printf("Contract Nodes\n")
	}
	ch.contractNodes(pq, oo)
	if ch.debugLevel >= 1 {
		fmt.Printf("Shortcuts:\n")
		for shortcutOrder, addedShortcuts := range ch.addedShortcuts {
			fmt.Printf("%v Shortcuts: %v x\n", shortcutOrder, addedShortcuts)
		}
	}
}

// Compute the shortest path for the given query (from origin to destination node).
// It returns the length of the path.
// If no path was found, -1 is returned
func (ch *ContractionHierarchies) ComputeShortestPath(origin, destination graph.NodeId) int {
	ch.dijkstra.SetMaxNumSettledNodes(math.MaxInt)
	ch.dijkstra.SetCostUpperBound(math.MaxInt)
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetBidirectional(true)
	ch.dijkstra.SetUseHeuristic(false)
	ch.matchArcsWithNodeOrder()
	length := ch.dijkstra.ComputeShortestPath(origin, destination)
	return length
}

// Get the computed path.
// A slice is returned which contains the node IDs in order from source to target
func (ch *ContractionHierarchies) GetPath(origin, destination graph.NodeId) []int {
	// TODO: this probably gets more efficient with other data structures (store shortcuts as map -> faster access)
	path := ch.dijkstra.GetPath(origin, destination)
	if ch.debugLevel == 1 {
		fmt.Println(path)
	}
	for i := 0; i < len(path)-1; i++ {
		source := path[i]
		target := path[i+1]
		/*
			via, exists := ch.shortcutMap[source][target]
			if exists {
				path = slice.InsertInt(path, i+1, via)
				if ch.debugLevel == 1 {
					fmt.Printf("Added node %v -> %v -> %v\n", source, via, target)
				}
				i-- // reevaluate, if the source has a shortcut to the currently added node
			}
		*/
		for _, sc := range ch.shortcuts {
			if sc.source == source && sc.target == target {
				path = slice.InsertInt(path, i+1, sc.via)
				if ch.debugLevel == 1 {
					fmt.Printf("Added node %v -> %v -> %v\n", source, sc.via, target)
				}
				i-- // reevaluate, if the source has a shortcut to the currently added node
			}
		}

	}
	return path
}

// Get the search space of the path finding query
// S slice is returned which contains all settled nodes of the query (containing the search information, e.g. distance to source node, which search direction was used for this item, ...)
func (ch *ContractionHierarchies) GetSearchSpace() []*DijkstraItem {
	return ch.dijkstra.GetSearchSpace()
}

// Compute an initial node order. If givenNodeOrder is not nil, the OrderOption oo are ignored.
// givenNodeOrder predefines the order of the nodes.
// oo defines how the node ordering will be calculated.
// It returns the calculated node order in a priority queue
func (ch *ContractionHierarchies) computeInitialNodeOrder(givenNodeOrder []int, oo OrderOptions) *NodeOrder {
	var pq *NodeOrder
	if givenNodeOrder != nil {
		order := make(NodeOrder, ch.g.NodeCount())
		for i := 0; i < ch.g.NodeCount(); i++ {
			order[i] = NewOrderItem(givenNodeOrder[i])
			order[i].edgeDifference = i // needed?
		}
		pq = &order
		heap.Init(pq)
	} else if oo.IsRandom() {
		nodeOrdering := make([]int, ch.g.NodeCount())
		for i := range nodeOrdering {
			nodeOrdering[i] = i
		}
		rand.Seed(time.Now().UnixNano()) // completely random
		rand.Shuffle(len(nodeOrdering), func(i, j int) { nodeOrdering[i], nodeOrdering[j] = nodeOrdering[j], nodeOrdering[i] })

		order := make(NodeOrder, ch.g.NodeCount())
		for i := 0; i < ch.g.NodeCount(); i++ {
			order[i] = NewOrderItem(nodeOrdering[i])
			//order[i].edgeDifference = i
		}
		pq = &order
		heap.Init(pq)
	} else {
		pq = NewNodeOrder(nil)
		for i := 0; i < ch.g.NodeCount(); i++ {
			shortcuts, incidentArcs, processedNeighbors := ch.contractNode(i, true)
			orderItem := NewOrderItem(i)
			if oo.ConsiderEdgeDifference() {
				orderItem.edgeDifference = shortcuts - incidentArcs
			}
			if oo.ConsiderProcessedNeighbors() {
				orderItem.processedNeighbors = processedNeighbors
			}
			if ch.debugLevel >= 2 {
				fmt.Printf("Add node %6v, edge difference: %3v, processed neighbors: %3v\n", i, shortcuts-incidentArcs, processedNeighbors)
			}
			heap.Push(pq, orderItem)
		}
	}
	return pq
}

// Contract the nodes based on the given order.
// The OrderOptions oo define, if and how the nodeOrder can get updated dynamically.
func (ch *ContractionHierarchies) contractNodes(order *NodeOrder, oo OrderOptions) {
	if order.Len() != ch.g.NodeCount() {
		// this is a rudimentary test, if the ordering could be valid.
		// However, it misses to test if every id appears exactly once
		panic("Node ordering not valid")
	}
	level := 0
	intermediateUpdates := 0
	for order.Len() > 0 {
		pqItem := heap.Pop(order).(*OrderItem)
		neededShortcuts, edges, processedNeighbors := ch.contractNode(pqItem.nodeId, true)
		edgeDifference := neededShortcuts - edges
		if oo.ConsiderEdgeDifference() {
			pqItem.edgeDifference = edgeDifference
		}
		if oo.ConsiderProcessedNeighbors() {
			pqItem.processedNeighbors = processedNeighbors
		}
		if !oo.IsDynamic() || order.Len() == 0 || pqItem.Priority() <= order.Peek().(*OrderItem).Priority() {
			// always stick to initially computed order or this is still the smallest edge difference
			if ch.orderOfNode[pqItem.nodeId] >= 0 {
				panic("Node was already ordered?")
			}
			ch.orderOfNode[pqItem.nodeId] = level
			ch.nodeOrdering[level] = pqItem.nodeId
			ch.contractNode(pqItem.nodeId, false)
			if ch.debugLevel >= 1 {
				fmt.Printf("Level %6v - Contract Node %6v, heap: %6v, sc: %3v, edges: %3v, ed: %3v pn: %3v, prio: %3v, updates: %6v\n", level, pqItem.nodeId, order.Len(), neededShortcuts, edges, edgeDifference, processedNeighbors, edgeDifference+processedNeighbors, intermediateUpdates)
			}
			intermediateUpdates = 0
			level++
		} else {
			if ch.debugLevel == 2 {
				fmt.Printf("Update order\n")
			}
			heap.Push(order, pqItem)
			intermediateUpdates++
		}
		if oo.IsPeriodic() && level%100 == 0 {
			for i := 0; i < order.Len(); i++ {
				orderItem := order.PeekAt(i).(*OrderItem)
				sc, edges, pn := ch.contractNode(orderItem.nodeId, true)
				ed := sc - edges
				if oo.ConsiderEdgeDifference() {
					orderItem.edgeDifference = ed
				}
				if oo.ConsiderProcessedNeighbors() {
					orderItem.processedNeighbors = pn
				}
				// wrong here, has this directly can influence the ordering and results in the same item could be taken twice
				//order.update(orderItem, ed, pn)
			}
			// reestablish node order
			heap.Init(order)
			// not informative here?
			intermediateUpdates++
		}
	}
}

// Contract the node given by nodeId and add shortcuts if necessary.
// if computeEdgeDifferenceOnly is true, the node is not contracted but only the necessary shortcuts are calculated.
// This returns 3 values:
//	- number of added/needed shortcuts
//	- number of (active) incident arcs to that node (arcs from nodes which are not contracted, yet)
//	- number of already contracted neighbors
func (ch *ContractionHierarchies) contractNode(nodeId graph.NodeId, computeEdgeDifferenceOnly bool) (int, int, int) {
	if ch.debugLevel >= 2 && !computeEdgeDifferenceOnly {
		fmt.Printf("Contract Node %v\n", nodeId)
	}
	shortcuts := 0
	contractedNeighbors := 0
	arcs := ch.g.GetArcsFrom(nodeId)
	incidentArcsAmount := len(arcs)
	if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
		fmt.Printf("Incident arcs %v\n", incidentArcsAmount)
	}
	ch.disableArcsForNode(nodeId)
	runtime := 0
	computations := 0
	for _, arc := range arcs {
		source := arc.Destination()
		if ch.isNodeContracted(source) {
			if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
				fmt.Printf("source %v already processed\n", source)
			}
			contractedNeighbors++
			continue
		}
		for _, otherArc := range ch.g.GetArcsFrom(nodeId) {
			target := otherArc.Destination()
			if source == target {
				// no need to process this path
				continue
			}
			if ch.isNodeContracted(target) {
				if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
					fmt.Printf("target %v already processed\n", target)
				}
				//contractedNeighbors++
				continue
			}
			if ch.debugLevel >= 2 && !computeEdgeDifferenceOnly {
				fmt.Printf("testing for shortcut %v -> %v\n", source, target)
			}
			maxCost := arc.Cost() + otherArc.Cost()
			ch.dijkstra.SetCostUpperBound(maxCost)
			start := time.Now()
			length := ch.dijkstra.ComputeShortestPath(source, target)
			elapsed := time.Since(start)
			runtime += int(elapsed.Nanoseconds())
			computations++

			if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
				fmt.Printf("Length: %v, cost via node: %v\n", length, maxCost)
			}
			if length == -1 || length > maxCost {
				// add shortcut, since the path via this node is the fastest
				// without this node, the target is either not reachable or the path is longer
				shortcuts++
				if !computeEdgeDifferenceOnly {
					ch.addShortcut(source, target, nodeId, maxCost)
				}
			}

		}
	}
	if computeEdgeDifferenceOnly {
		// reset temporarily disabled arcs
		ch.enableArcsForNode(nodeId)
	} else {
		_, exists := ch.addedShortcuts[shortcuts]
		if !exists {
			ch.addedShortcuts[shortcuts] = 1
		} else {
			ch.addedShortcuts[shortcuts]++
		}
	}

	if ch.debugLevel >= 1 && computations > 0 {
		fmt.Printf("Dijkstra Runtime: %v ns * %v\n", float64(runtime/computations), computations)
	}

	// Fix incident arcs: Remove number of already contracted neighbors
	incidentArcsAmount -= contractedNeighbors

	if ch.debugLevel == 2 && !computeEdgeDifferenceOnly {
		fmt.Printf("Added Shortcuts: %v, incident Arcs: %v, contractedNeighbors: %v\n", shortcuts/2, incidentArcsAmount, contractedNeighbors)
	}
	// number of shortcuts is doubled, since we have two arcs for each each (because symmetric graph)
	return shortcuts / 2, incidentArcsAmount, contractedNeighbors
}

// Adds a shortcut to the graph from source to target with length cost which is spanned over node defined by via.
// This adds a new arc in the graph.
func (ch *ContractionHierarchies) addShortcut(source, target, via graph.NodeId, cost int) {
	ch.dg.AddArc(source, target, cost)
	sc := Shortcut{source: source, target: target, via: via}
	ch.shortcuts = append(ch.shortcuts, sc)
	// maybe this map is not so a good idea
	/*
		if ch.shortcutMap[source] == nil {
			ch.shortcutMap[source] = make(map[graph.NodeId]graph.NodeId)
		}
		ch.shortcutMap[source][target] = nodeId
	*/
	if ch.debugLevel == 2 {
		fmt.Printf("Add shortcut %v %v %v %v\n", source, target, via, cost)
	}
}

// Enable all arcs for the node given by nodeId.
func (ch *ContractionHierarchies) enableArcsForNode(nodeId graph.NodeId) {
	ch.g.SetArcFlags(nodeId, true)
	if ch.debugLevel == 2 {
		fmt.Printf("enable arcs of node %v\n", nodeId)
	}
}

// Disable all arcs for the node given by nodeId.
func (ch *ContractionHierarchies) disableArcsForNode(nodeId graph.NodeId) {
	ch.g.SetArcFlags(nodeId, false)
	if ch.debugLevel == 2 {
		fmt.Printf("disable arcs of node %v\n", nodeId)
	}
}

func (ch *ContractionHierarchies) matchArcsWithNodeOrder() {
	for source := range ch.g.GetNodes() {
		for _, arc := range ch.g.GetArcsFrom(source) {
			target := arc.Destination()
			arc.SetArcFlag(ch.orderOfNode[source] < ch.orderOfNode[target])
		}
	}
}

// checks whether the node given by nodeId is already contracted.
func (ch *ContractionHierarchies) isNodeContracted(node graph.NodeId) bool {
	// orderOfNode gets initialized with -1
	// if the entry is greater than that, there was already a level assigned
	return ch.orderOfNode[node] >= 0
}

// Set the shortcuts by an already available list.
// This is used when one has already a contracted graph and one need to define which arcs are shortcuts.
func (ch *ContractionHierarchies) SetShortcuts(shortcuts []Shortcut) {
	ch.shortcuts = shortcuts
	/*
		ch.shortcutMap = make(map[graph.NodeId]map[graph.NodeId]graph.NodeId)
		for _, v := range shortcuts {
			source := v.source
			target := v.target
			via := v.via
			if ch.shortcutMap[source] == nil {
				ch.shortcutMap[source] = make(map[graph.NodeId]graph.NodeId)
			}
			ch.shortcutMap[source][target] = via
		}
	*/
}

// Get the calculated shortcuts.
func (ch *ContractionHierarchies) GetShortcuts() []Shortcut {
	return ch.shortcuts
	/*
		sc := make([]Shortcut, 0)
		for source, shortcuts := range ch.shortcutMap {
			for target, via := range shortcuts {
				sc = append(sc, Shortcut{source: source, target: target, via: via})
			}
		}
		return sc
	*/
}

// Set the node ordering by an already available list.
// This is used when one has already a contracted graph and one need to define in which order the nodes were contracted.
//the index of the list reflects to the node id, the value to the level/position, when the node was contracted.
func (ch *ContractionHierarchies) SetNodeOrdering(nodeOrdering []int) {
	ch.nodeOrdering = nodeOrdering
	ch.orderOfNode = make([]int, len(nodeOrdering))
	for i, nodeId := range nodeOrdering {
		ch.orderOfNode[nodeId] = i
	}
}

// Set the debug level.
func (ch *ContractionHierarchies) SetDebugLevel(level int) {
	ch.debugLevel = level
}

// Write the graph to a file
func (ch *ContractionHierarchies) WriteGraph() {
	graph.WriteFmi(ch.g, ch.graphFilename)
}

// Write the shortcuts to a file
func (ch *ContractionHierarchies) WriteShortcuts() {
	file, cErr := os.Create(ch.shortcutsFilename)

	if cErr != nil {
		log.Fatal(cErr)
	}

	var sb strings.Builder

	for _, v := range ch.GetShortcuts() {
		shortcut := fmt.Sprintf("%v %v %v\n", v.source, v.target, v.via)
		sb.WriteString(shortcut)
	}

	writer := bufio.NewWriter(file)
	writer.WriteString(sb.String())
	writer.Flush()
}

// Write the node ordering to a file
func (ch *ContractionHierarchies) WriteNodeOrdering() {
	file, cErr := os.Create(ch.nodeOrderingFilename)

	if cErr != nil {
		log.Fatal(cErr)
	}

	var sb strings.Builder
	for _, v := range ch.nodeOrdering {
		order := fmt.Sprintf("%v\n", v)
		sb.WriteString(order)
	}
	writer := bufio.NewWriter(file)
	writer.WriteString(sb.String())
	writer.Flush()
}

// Write the contraciotn resutl (graph, shortcuts, node ordering) to a file
func (ch *ContractionHierarchies) WriteContractionResult() {
	ch.WriteGraph()
	ch.WriteShortcuts()
	ch.WriteNodeOrdering()
}

// Read a shortcuts file and return the list of shortcuts
func ReadShortcutFile(filename string) []Shortcut {
	file, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return ConvertToShortcuts(string(file))
}

// Parse a string to a list of shortcuts
func ConvertToShortcuts(shortcutsString string) []Shortcut {
	scanner := bufio.NewScanner(strings.NewReader(shortcutsString))

	shortcuts := make([]Shortcut, 0)
	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 || line[0] == '#' {
			// skip empty lines and comments
			continue
		}
		var source, target, via graph.NodeId
		fmt.Scanf(line, "%d %d %d", source, target, via)
		shortcuts = append(shortcuts, Shortcut{source: source, target: target, via: via})
	}
	return shortcuts
}

// Read a node ordering file and return the node order
func ReadNodeOrderingFile(filename string) []int {
	file, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return ConvertToNodeOrdering(string(file))
}

// Parse a string to a lsit which specifies the node order
func ConvertToNodeOrdering(nodeOrderingString string) []int {
	scanner := bufio.NewScanner(strings.NewReader(nodeOrderingString))

	nodeOrdering := make([]int, 0)
	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 || line[0] == '#' {
			// skip empty lines and comments
			continue
		}
		var nodeId graph.NodeId
		fmt.Scanf(line, "%d", nodeId)
		nodeOrdering = append(nodeOrdering, nodeId)
	}
	return nodeOrdering
}
