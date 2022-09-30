package path

import (
	"bufio"
	"container/heap"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

// Provides the precomputation and query for shortest paths with the use of Contraction Hierarchies.
// Implements the Navigator Interface.
type ContractionHierarchies struct {
	g                  graph.Graph        // graph to work on (for precomputation, this ha sto be a graph.DynamicGraph)
	dg                 graph.DynamicGraph // Dynamic graph which is used for precomputation (to add shortcuts)
	dijkstra           *UniversalDijkstra // the dijkstra algorithm to perform the searches
	nodeOrdering       [][]graph.NodeId   // the node ordering (in which order the nodes were contracted)
	orderOfNode        []int              // the order of the node ("reverse" node ordering). At which position the specified node was contracted
	contractedNodes    []graph.NodeId     // contains the IDs of the contracted nodes
	orderItems         []*OrderItem
	pqOrder            *NodeOrder
	contractionWorkers []*UniversalDijkstra

	contractionLevelLimit float64 // percentage, how many nodes should get contracted

	// decide for one, currently both are needed (but probably could get rid of the slice)
	shortcuts   []Shortcut                                     // array which contains all shortcuts
	shortcutMap map[graph.NodeId]map[graph.NodeId]graph.NodeId // map of the shortcuts (from/source -> to/target -> via)

	addedShortcuts       map[int]int // debug information - stores the number of how many nodes introduced the specified amount of shortcuts. Key is the number of shortcuts, value is how many introduced them
	debugLevel           int         // the debug level - used for printing some informaiton
	graphFilename        string      // the filename were the file gets stored
	shortcutsFilename    string      // the filename were the shourtcuts gets stored
	nodeOrderingFilename string      // the filname were the node ordering gets stored

	// For some debuging
	initialTime       time.Time
	runtime           []time.Duration
	shortcutCounter   []int
	milestones        []float64
	milestoneIndex    int
	milestoneFilename string

	// search items needed for (manual) path calculation
	visitedNodes         []bool
	backwardVisitedNodes []bool
	searchSpace          []*DijkstraItem
	backwardSearchSpace  []*DijkstraItem
	connection           graph.NodeId
	pqPops               int
	pqUpdates            int
	relaxedEdges         int
	relaxationAttempts   int
}

// Describes a shortcut.
// It contains the information of the source and target node and via which node this shortcut is spanned
type Shortcut struct {
	source graph.NodeId // the source node
	target graph.NodeId // the target node
	via    graph.NodeId // over which node this shortcut is spanned
	cost   int          // cost of the shortcut
}

// Contrains the result of a (computed, virtual) node contraction
type ContractionResult struct {
	nodeId              graph.NodeId // the node which gets contracted
	shortcuts           []Shortcut   // the necessary shortcuts
	incidentEdges       int          // number of (active) incident edges
	contractedNeighbors int          // number of contracted neighbors
}

type PathFindingOptions struct {
	Manual        bool
	UseHeuristic  bool
	StallOnDemand int
	SortArcs      bool
}

type ContractionOptions struct {
	Bidirectional      bool
	UseHeuristic       bool
	HotStart           bool
	MaxNumSettledNodes int
	ContractionLimit   float64
	ContractionWorkers int
}

func MakeDefaultContractionOptions() ContractionOptions {
	// TODO test bidirectional=true, useHeuristic=true, maxNumSettledNodes=60 (or something else)
	return ContractionOptions{Bidirectional: false, UseHeuristic: false, HotStart: true, MaxNumSettledNodes: math.MaxInt, ContractionLimit: 100, ContractionWorkers: 1}
}

func MakeDefaultPathFindingOptions() PathFindingOptions {
	return PathFindingOptions{Manual: false, UseHeuristic: false, StallOnDemand: 2, SortArcs: false}
}

// Create a new Contraction Hierarchy.
// Before a query can get executed, the Precomputation has to be done
func NewContractionHierarchies(g graph.Graph, dijkstra *UniversalDijkstra, options ContractionOptions) *ContractionHierarchies {
	// just add one worker by default
	cw := make([]*UniversalDijkstra, 0, options.ContractionWorkers)
	for i := 0; i < options.ContractionWorkers; i++ {
		worker := NewUniversalDijkstra(g)
		worker.SetHotStart(options.HotStart)
		worker.SetBidirectional(options.Bidirectional)
		worker.SetUseHeuristic(options.UseHeuristic)
		worker.SetMaxNumSettledNodes(options.MaxNumSettledNodes)
		cw = append(cw, worker)

	}

	return &ContractionHierarchies{g: g, dijkstra: dijkstra, contractionWorkers: cw, contractionLevelLimit: options.ContractionLimit, graphFilename: "contracted_graph.fmi", shortcutsFilename: "shortcuts.txt", nodeOrderingFilename: "node_ordering.txt", milestoneFilename: "milestones.txt"}
}

// Create a new Contraction Hierarchy which is already initialized with the shortcuts and node ordering.
// This can directly start a new query
func NewContractionHierarchiesInitialized(g graph.Graph, dijkstra *UniversalDijkstra, shortcuts []Shortcut, nodeOrdering [][]int, pathFindingOptions PathFindingOptions) *ContractionHierarchies {
	ch := NewContractionHierarchies(g, dijkstra, MakeDefaultContractionOptions()) // use default contraction options (they are not used anyway)
	ch.SetShortcuts(shortcuts)
	ch.SetNodeOrdering(nodeOrdering)
	ch.matchArcsWithNodeOrder()
	ch.ShortestPathSetup(pathFindingOptions)
	return ch
}

// Do the precomputation of the Contraction Hierarchies.
// This adds new Edges in the graph by adding shortcuts. The result will be a modified graph, a collection of shortcuts and the calculated node ordering.
// If givenNodeOrder is not nil, the OrderOption oo are ignored.
// givenNodeOrder predefines the order of the nodes.
// oo defines how the node ordering will be calculated.
func (ch *ContractionHierarchies) Precompute(givenNodeOrder []int, oo OrderOptions) {
	ch.initialTime = time.Now()
	ch.milestoneIndex = 0

	ch.addedShortcuts = make(map[int]int)
	ch.shortcuts = make([]Shortcut, 0)
	ch.nodeOrdering = make([][]int, ch.g.NodeCount()) // TODO this is maybe to big (when multiple nodes are on the same level)
	ch.orderOfNode = make([]int, ch.g.NodeCount())
	ch.contractedNodes = make([]graph.NodeId, 0, ch.g.NodeCount()) // TODO maybe use custom struct with fixed length slice (bool) and an int, indicating how many nodes are "contracted" (true)
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
	if givenNodeOrder != nil {
		// ignore the order options
		// -> just overwrite them
		oo = MakeOrderOptions().SetLazyUpdate(false).SetRandom(false).SetEdgeDifference(false).SetProcessedNeighbors(false).SetRandom(false)
	}
	if ch.debugLevel >= 1 {
		log.Printf("Compute Node Ordering\n")
	}
	ch.pqOrder = ch.computeInitialNodeOrder(givenNodeOrder, oo)

	if ch.debugLevel >= 3 {
		log.Printf("Initial computed order:\n%v\n", ch.pqOrder)
	}
	if ch.milestones != nil && ch.milestones[0] == 0 {
		runtime := time.Since(ch.initialTime)
		milestone := ch.milestones[ch.milestoneIndex]
		ch.runtime[ch.milestoneIndex] = runtime
		ch.shortcutCounter[ch.milestoneIndex] = 0

		f, err := os.OpenFile(ch.milestoneFilename, os.O_APPEND|os.O_WRONLY|os.O_CREATE, 0600)
		if err != nil {
			panic(err)
		}

		defer f.Close()
		if _, err = f.WriteString(fmt.Sprintf("Milestone %05.2f %% - Runtime: %6.3f s, difference: %.3f s, total Shortcuts: %5v, added Shortcuts: %5v\n", milestone, float64(runtime.Microseconds())/1000000, float64(runtime.Microseconds())/1000000, 0, 0)); err != nil {
			panic(err)
		}
		ch.milestoneIndex++
	}

	if ch.debugLevel >= 1 {
		log.Printf("Contract Nodes\n")
	}
	ch.contractNodes(oo, givenNodeOrder != nil)
	if ch.debugLevel >= 1 {
		log.Printf("Shortcuts:\n")
		shortcutOrder := make([]int, 0, len(ch.addedShortcuts))
		for shortcutAmount := range ch.addedShortcuts {
			shortcutOrder = append(shortcutOrder, shortcutAmount)
		}
		sort.Ints(shortcutOrder)
		for _, amount := range shortcutOrder {
			log.Printf("%v x %v Shortcuts\n", ch.addedShortcuts[amount], amount)
		}
	}
	// store the computed shortcuts in the map
	ch.SetShortcuts(ch.shortcuts)
	// match arcs with node order
	ch.matchArcsWithNodeOrder()

	for i, m := range ch.milestones {
		runtime := ch.runtime[i]
		timeDif := ch.runtime[i]
		if i > 0 {
			timeDif -= ch.runtime[i-1]
		}
		totalShortcuts := ch.shortcutCounter[i]
		previousShortcuts := ch.shortcutCounter[0]
		if i > 0 {
			previousShortcuts = ch.shortcutCounter[i-1]
		}
		addedDifference := totalShortcuts - previousShortcuts
		if ch.debugLevel >= 1 {
			log.Printf("Milestone %05.2f %% - Runtime: %6.3f s, difference: %.3f s, total Shortcuts: %5v, added Shortcuts: %5v\n", m, float64(runtime.Microseconds())/1000000, float64(timeDif.Microseconds())/1000000, totalShortcuts, addedDifference)
		}
	}
}

// Setup ch to compute the shortest path
func (ch *ContractionHierarchies) ShortestPathSetup(options PathFindingOptions) {
	// set fix options for CH search
	ch.dijkstra.SetCostUpperBound(math.MaxInt)
	ch.dijkstra.SetMaxNumSettledNodes(math.MaxInt)
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetIgnoreNodes(nil)
	ch.dijkstra.SetHotStart(false)

	// set tuning options
	ch.dijkstra.SetBidirectional(!options.Manual)
	ch.dijkstra.SetUseHeuristic(options.UseHeuristic)
	ch.dijkstra.SetStallOnDemand(options.StallOnDemand)
	ch.SetSortArcs(options.SortArcs)
}

// Compute the shortest path for the given query (from origin to destination node).
// It returns the length of the path.
// If no path was found, -1 is returned
func (ch *ContractionHierarchies) ComputeShortestPath(origin, destination graph.NodeId) int {
	if ch.debugLevel >= 1 {
		log.Printf("Compute path from %v to %v\n", origin, destination)
	}

	if ch.debugLevel >= 3 {
		log.Printf("Node ordering: %v\n", ch.nodeOrdering)
	}

	if ch.dijkstra.bidirectional {
		return ch.dijkstra.ComputeShortestPath(origin, destination)
	}
	// if path should not get calculated bidirectional, the following will get executed
	// compute shortest path manually since two unidirectinoal dijkstras were used
	ch.dijkstra.ComputeShortestPath(origin, -1)
	ch.visitedNodes = ch.dijkstra.visitedNodes
	ch.searchSpace = ch.dijkstra.searchSpace
	ch.pqPops = ch.dijkstra.GetPqPops()
	ch.pqUpdates = ch.dijkstra.GetPqUpdates()
	ch.relaxedEdges = ch.dijkstra.GetEdgeRelaxations()
	ch.relaxationAttempts = ch.dijkstra.GetRelaxationAttempts()

	ch.dijkstra.ComputeShortestPath(destination, -1)
	ch.backwardVisitedNodes = ch.dijkstra.visitedNodes
	ch.backwardSearchSpace = ch.dijkstra.searchSpace
	ch.pqPops += ch.dijkstra.GetPqPops()
	ch.pqUpdates += ch.dijkstra.GetPqUpdates()
	ch.relaxedEdges += ch.dijkstra.GetEdgeRelaxations()
	ch.relaxationAttempts += ch.dijkstra.GetRelaxationAttempts()
	ch.connection = -1
	shortestLength := math.MaxInt
	for nodeId := 0; nodeId < ch.g.NodeCount(); nodeId++ {
		if ch.visitedNodes[nodeId] && ch.backwardVisitedNodes[nodeId] {
			length := ch.searchSpace[nodeId].distance + ch.backwardSearchSpace[nodeId].distance
			if length < shortestLength {
				shortestLength = length
				ch.connection = nodeId
			}
		}
	}
	if ch.connection == -1 {
		return -1
	}

	return shortestLength
}

// Get the computed path.
// A slice is returned which contains the node IDs in order from source to target
func (ch *ContractionHierarchies) GetPath(origin, destination graph.NodeId) []int {
	// TODO: this probably gets more efficient with other data structures (store shortcuts as map -> faster access)
	if ch.connection == -1 {
		return make([]int, 0)
	}
	path := make([]int, 0)
	if ch.dijkstra.bidirectional {
		path = ch.dijkstra.GetPath(origin, destination)
	} else {
		// compute path manually, since two unidirectional dijkstras were used
		for nodeId := ch.searchSpace[ch.connection].predecessor; nodeId != -1; nodeId = ch.searchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
		slice.ReverseIntInPlace(path)
		path = append(path, ch.connection)
		for nodeId := ch.backwardSearchSpace[ch.connection].predecessor; nodeId != -1; nodeId = ch.backwardSearchSpace[nodeId].predecessor {
			path = append(path, nodeId)
		}
	}
	if ch.debugLevel >= 1 {
		log.Printf("Path with shortcuts: %v\n", path)
	}
	for i := 0; i < len(path)-1; i++ {
		source := path[i]
		target := path[i+1]

		if via, exists := ch.shortcutMap[source][target]; exists {
			path = slice.InsertInt(path, i+1, via)
			if ch.debugLevel >= 2 {
				log.Printf("Added node %v -> %v -> %v\n", source, via, target)
			}
			i-- // reevaluate, if the source has a shortcut to the currently added node
		}
		/*
			for _, sc := range ch.shortcuts {
				if sc.source == source && sc.target == target {
					path = slice.InsertInt(path, i+1, sc.via)
					if ch.debugLevel == 1 {
						log.Printf("Added node %v -> %v -> %v\n", source, sc.via, target)
					}
					i-- // reevaluate, if the source has a shortcut to the currently added node
				}
			}
		*/

	}
	return path
}

// Get the search space of the path finding query
// S slice is returned which contains all settled nodes of the query (containing the search information, e.g. distance to source node, which search direction was used for this item, ...)
func (ch *ContractionHierarchies) GetSearchSpace() []*DijkstraItem {
	if ch.dijkstra.bidirectional {
		return ch.dijkstra.GetSearchSpace()
	}
	// compute search space manually, since two unidirectional dijkstras were used
	searchSpace := make([]*DijkstraItem, 0)
	for i := 0; i < ch.g.NodeCount(); i++ {
		if ch.visitedNodes[i] {
			searchSpace = append(searchSpace, ch.searchSpace[i])
		} else if ch.backwardVisitedNodes[i] {
			searchSpace = append(searchSpace, ch.backwardSearchSpace[i])
		}
	}
	return searchSpace
}

// Get the number of pq pops (from the priority queue)
func (ch *ContractionHierarchies) GetPqPops() int {
	if ch.dijkstra.bidirectional {
		return ch.dijkstra.GetPqPops()
	}
	// use manual computed pq pops when calculated the path manually
	return ch.pqPops
}

// Get the number of the pq updates
func (ch *ContractionHierarchies) GetPqUpdates() int {
	if ch.dijkstra.bidirectional {
		return ch.dijkstra.GetPqUpdates()
	}
	// use manual computed pq updates when calculated the path manually
	return ch.pqUpdates
}

// Get the number of relaxed edges
func (ch *ContractionHierarchies) GetEdgeRelaxations() int {
	if ch.dijkstra.bidirectional {
		return ch.dijkstra.GetEdgeRelaxations()
	}
	// use manual computed pq updates when calculated the path manually
	return ch.relaxedEdges
}

// Get the number for how many edges the relaxation was tested (but maybe early terminated)
func (ch *ContractionHierarchies) GetRelaxationAttempts() int {
	if ch.dijkstra.bidirectional {
		return ch.dijkstra.GetRelaxationAttempts()
	}
	// use manual computed pq updates when calculated the path manually
	return ch.relaxationAttempts
}

// get the used graph
func (ch *ContractionHierarchies) GetGraph() graph.Graph {
	return ch.g
}

// Compute an initial node order. If givenNodeOrder is not nil, the OrderOption oo are ignored.
// givenNodeOrder predefines the order of the nodes.
// oo defines how the node ordering will be calculated.
// It returns the calculated node order in a priority queue
func (ch *ContractionHierarchies) computeInitialNodeOrder(givenNodeOrder []int, oo OrderOptions) *NodeOrder {
	var pq *NodeOrder
	ch.orderItems = make([]*OrderItem, ch.g.NodeCount())

	if givenNodeOrder != nil {
		order := make(NodeOrder, ch.g.NodeCount())

		for i := 0; i < ch.g.NodeCount(); i++ {
			orderItem := NewOrderItem(givenNodeOrder[i])
			orderItem.edgeDifference = i // set edge difference for maintaining different priority
			orderItem.index = i
			order[i] = orderItem
			ch.orderItems[orderItem.nodeId] = orderItem
		}

		pq = &order
	} else if oo.IsRandom() {
		nodeOrdering := make([]int, ch.g.NodeCount())

		for i := range nodeOrdering {
			nodeOrdering[i] = i
		}

		rand.Seed(time.Now().UnixNano()) // completely random
		rand.Shuffle(len(nodeOrdering), func(i, j int) { nodeOrdering[i], nodeOrdering[j] = nodeOrdering[j], nodeOrdering[i] })

		order := make(NodeOrder, ch.g.NodeCount())

		for i := 0; i < ch.g.NodeCount(); i++ {
			orderItem := NewOrderItem(nodeOrdering[i])
			orderItem.index = i
			order[i] = orderItem
			ch.orderItems[orderItem.nodeId] = orderItem
		}

		pq = &order
	} else {
		order := make(NodeOrder, ch.g.NodeCount())

		nodes := make([]graph.NodeId, ch.g.NodeCount())
		for i := range nodes {
			nodes[i] = i
		}
		contractionResults := ch.computeNodeContractionParallel(nodes, nil, true)

		for i, result := range contractionResults {
			nodeId := result.nodeId
			edgeDifference := len(result.shortcuts) - result.incidentEdges
			contractedNeighbors := result.contractedNeighbors

			if !oo.ConsiderEdgeDifference() {
				edgeDifference = 0
			}

			if !oo.ConsiderProcessedNeighbors() {
				contractedNeighbors = 0
			}

			item := NewOrderItem(nodeId)
			item.edgeDifference = edgeDifference
			item.processedNeighbors = contractedNeighbors
			item.index = i

			if ch.debugLevel >= 2 {
				log.Printf("Add node %6v, edge difference: %3v, processed neighbors: %3v\n", nodeId, edgeDifference, contractedNeighbors)
			}

			order[i] = item
			ch.orderItems[nodeId] = item
		}
		pq = &order
	}

	heap.Init(pq)
	return pq
}

// compute an independent set from the node order (pqOrder).
func (ch *ContractionHierarchies) computeIndependentSet(ignorePriority bool) []graph.NodeId {
	priority := ch.pqOrder.Peek().(*OrderItem).Priority()
	if ignorePriority {
		priority = math.MaxInt
	}

	independentSet := make([]graph.NodeId, 0)
	forbiddenNodes := make([]bool, ch.g.NodeCount())
	increasedPriority := false
	ignoredNode := false

	for i := 0; i < ch.pqOrder.Len(); i++ {
		item := ch.pqOrder.PeekAt(i).(*OrderItem)
		if forbiddenNodes[item.nodeId] == true {
			ignoredNode = true
			continue
		}
		if priority < item.Priority() {
			increasedPriority = true
		}
		if increasedPriority && ignoredNode {
			break
		}
		independentSet = append(independentSet, item.nodeId)
		forbiddenNodes[item.nodeId] = true
		for _, arc := range ch.g.GetArcsFrom(item.nodeId) {
			forbiddenNodes[arc.To] = true
		}
	}

	return independentSet
}

// update the node order for the given nodes by computing a virtual contraction for each given node
func (ch *ContractionHierarchies) updateOrderForNodes(nodes []graph.NodeId, oo OrderOptions) {
	// collect all nodes which have to get updates
	// TODO maybe remove this precheck
	updateNodes := make([]graph.NodeId, 0, len(nodes))
	for _, node := range nodes {
		if !ch.isNodeContracted(node) {
			updateNodes = append(updateNodes, node)
		}
	}

	// add "current node" to the ignore list, because it is not contracted, yet (what is the basis for the ignore list)
	contractionResult := ch.computeNodeContractionParallel(updateNodes, nil, true)
	log.Printf("Computed contraction results\n")
	for _, result := range contractionResult {
		nodeId := result.nodeId
		edgeDifference := len(result.shortcuts) - result.incidentEdges
		contractedNeighbors := result.contractedNeighbors

		if !oo.ConsiderEdgeDifference() {
			edgeDifference = 0
		}

		if !oo.ConsiderProcessedNeighbors() {
			contractedNeighbors = 0
		}

		item := ch.orderItems[nodeId]
		oldPrio := item.Priority()
		oldPos := item.index

		ch.pqOrder.update(item, edgeDifference, contractedNeighbors)

		newPrio := item.Priority()
		newPos := item.index
		if ch.debugLevel >= 2 {
			log.Printf("Updating node %v. old priority: %v, new priority: %v, old position: %v, new position: %v\n", nodeId, oldPrio, newPrio, oldPos, newPos)
		}
	}
}

// Compute contraction result for given nodes.
// Returns the result of the (virtual) contraction.
func (ch *ContractionHierarchies) computeNodeContractionParallel(nodes []graph.NodeId, ignoreList []graph.NodeId, ignoreCurrentNode bool) []*ContractionResult {
	numJobs := len(nodes)
	numWorkers := len(ch.contractionWorkers)

	jobs := make(chan graph.NodeId, numJobs)
	results := make(chan *ContractionResult, numJobs)

	// create workers
	for i := 0; i < numWorkers; i++ {
		go func(worker *UniversalDijkstra) {
			for nodeId := range jobs {
				if ch.debugLevel >= 3 {
					log.Printf("Contract Node %7v\n", nodeId)
				}

				var ignoreNodes []graph.NodeId

				if len(ignoreList) > 0 || ignoreCurrentNode {
					// make a "hard" copy to handle different cases in the different goroutines
					ignoreNodes = make([]graph.NodeId, len(ch.contractedNodes))
					copy(ignoreNodes, ch.contractedNodes)
					if len(ignoreList) > 0 {
						ignoreNodes = append(ignoreNodes, ignoreList...)
					}
					if ignoreCurrentNode {
						ignoreNodes = append(ignoreNodes, nodeId)
					}
				} else {
					// just use the same (unchanged) slice for all goroutines
					ignoreNodes = ch.contractedNodes
				}

				// Recalculate shortcuts, incident edges and processed neighbors
				// TODO may not be necessary when updating the neighbors with every contraction -> shortcuts need to get cached (maybe bad for RAM?)
				cr := ch.computeNodeContraction(nodeId, ignoreNodes, worker)

				results <- cr
			}
		}(ch.contractionWorkers[i])
	}

	// fill jobs
	for i := 0; i < numJobs; i++ {
		nodeId := nodes[i]
		jobs <- nodeId
	}
	close(jobs)

	contractionResults := make([]*ContractionResult, numJobs)
	for i := 0; i < numJobs; i++ {
		contractionResults[i] = <-results
	}

	return contractionResults
}

// Contract the nodes based on the given order.
// The OrderOptions oo define, if and how the nodeOrder can get updated dynamically.
func (ch *ContractionHierarchies) contractNodes(oo OrderOptions, fixedOrder bool) {
	if ch.pqOrder.Len() != ch.g.NodeCount() {
		// this is a rudimentary test, if the ordering could be valid.
		// However, it misses to test if every id appears exactly once
		panic("Node ordering not valid")
	}

	level := 0
	intermediateUpdates := 0
	shortcutCounter := 0
	newShortcuts := 0

	for ch.pqOrder.Len() > 0 && (float64(len(ch.contractedNodes))/float64(ch.g.NodeCount()))*100 <= ch.contractionLevelLimit {
		updateNodes := make([]graph.NodeId, 0)
		var nodes []graph.NodeId
		if fixedOrder {
			// stick to initial order
			// only contract one by one
			nodes = []graph.NodeId{ch.pqOrder.Peek().(*OrderItem).nodeId}
		} else {
			nodes = ch.computeIndependentSet(false) // TODO check for which order option this cat get true
		}

		if oo.IsLazyUpdate() {
			newShortcuts = 0

			ignoreList := make([]graph.NodeId, len(nodes))
			copy(ignoreList, nodes)
			contractionResults := ch.computeNodeContractionParallel(nodes, ignoreList, true) // Ignore nodes for current level (they are not contracted, yet)
			// TODO this gets instable when ignoreList is equal to nodes, but this should be the correct calculation?
			priorityThreshold := math.MaxInt
			if ch.pqOrder.Len() > len(contractionResults) {
				priorityThreshold = ch.pqOrder.PeekAt(len(contractionResults)).(*OrderItem).Priority()
			}

			deniedContractionItems := make([]*OrderItem, 0, len(contractionResults))
			contractedNodes := make([]graph.NodeId, 0, len(nodes))
			ch.nodeOrdering[level] = make([]graph.NodeId, 0, len(nodes))
			affectedNeighbors := make([]graph.NodeId, 0)

			candidateShortcuts := make([]Shortcut, 0)
			for _, cr := range contractionResults {
				item := ch.orderItems[cr.nodeId]

				if ch.debugLevel >= 3 {
					log.Printf("Test contraction of Node %v\n", item.nodeId)
				}

				heap.Remove(ch.pqOrder, item.index)

				if oo.ConsiderEdgeDifference() {
					item.edgeDifference = len(cr.shortcuts) - cr.incidentEdges
				}

				if oo.ConsiderProcessedNeighbors() {
					item.processedNeighbors = cr.contractedNeighbors
				}

				if ch.pqOrder.Len() == 0 || item.Priority() <= priorityThreshold {
					// always stick to initially computed order or this is still the smallest edge difference
					if ch.orderOfNode[item.nodeId] >= 0 {
						panic("Node was already ordered?")
					}
					ch.orderOfNode[item.nodeId] = level
					contractedNodes = append(contractedNodes, item.nodeId)
					ch.contractedNodes = append(ch.contractedNodes, item.nodeId)
					if ch.debugLevel >= 3 {
						log.Printf("Contract node %v\n", item.nodeId)
					}
					//ch.addShortcuts(cr.shortcuts) // avoid recomputation of shortcuts. Just add the previously calculated shortcuts
					candidateShortcuts = append(candidateShortcuts, cr.shortcuts...)

					// update neighbors -> fill neighbor list
					// TODO maybe needs rework here (contracted nodes grow in this loop)
					// should not make a difference because of independent set
					if oo.UpdateNeighbors() {
						// collect all nodes which have to get updates
						for _, arc := range ch.g.GetArcsFrom(item.nodeId) {
							destination := arc.To
							if !ch.isNodeContracted(destination) {
								affectedNeighbors = append(affectedNeighbors, destination)
							}
						}
					}
				} else {
					if ch.debugLevel >= 3 {
						log.Printf("Update order\n")
					}
					deniedContractionItems = append(deniedContractionItems, item)
				}
			}

			if len(contractedNodes) > 0 {
				ch.nodeOrdering[level] = contractedNodes
				level++
				intermediateUpdates = 0
			}

			finalShortcuts := make([]Shortcut, 0, len(candidateShortcuts))
			for i := 0; i < len(candidateShortcuts); i++ {
				shortcut := candidateShortcuts[i]
				keep := true
				for j := 0; j < len(candidateShortcuts); j++ {
					otherShortcut := candidateShortcuts[j]
					if shortcut.source == otherShortcut.source && shortcut.target == otherShortcut.target && /*shortcut.via != otherShortcut.via &&*/ shortcut.cost >= otherShortcut.cost {
						if i > j {
							// when equivalent, only use the first shortcut
							keep = false
						}
					}
				}
				if keep {
					finalShortcuts = append(finalShortcuts, shortcut)
				}
			}
			ch.addShortcuts(finalShortcuts)
			newShortcuts += len(finalShortcuts)

			if len(deniedContractionItems) > 0 {
				intermediateUpdates++
				for _, item := range deniedContractionItems {
					heap.Push(ch.pqOrder, item)
				}
			}

			updateNodes = affectedNeighbors

			// TODO maybe combine this somehow with parallelComputation (but only give a single node)
			// But shortcuts should then get cached

			// Recalculate shortcuts, incident edges and processed neighbors
			// TODO may not be necessary when updateing the neighbors with every contraction

		} else {
			// no lazy update -> either all best nocdes of the independent set are calculated or fixed order
			ch.nodeOrdering[level] = make([]graph.NodeId, len(nodes))
			for i, nodeId := range nodes {
				ch.orderOfNode[nodeId] = level
				ch.nodeOrdering[level][i] = nodeId
				ch.contractedNodes = append(ch.contractedNodes, nodeId)
				heap.Remove(ch.pqOrder, ch.orderItems[nodeId].index)
			}

			contractionResults := ch.computeNodeContractionParallel(nodes, nil, false)

			candidateShortcuts := make([]Shortcut, 0)
			// TODO check if is better to use slice (fixed length) everywhere (-> boolean slices instead of "limited" nodeId slice)
			neighborsMap := make(map[graph.NodeId]struct{})
			for _, result := range contractionResults {
				candidateShortcuts = append(candidateShortcuts, result.shortcuts...)
				// remove duplicates
				for _, arc := range ch.g.GetArcsFrom(result.nodeId) {
					if !ch.isNodeContracted(arc.To) {
						neighborsMap[arc.To] = struct{}{}
					}
				}
			}

			// remove duplicate shortcuts (shortcuts which are found from both middle nodes. However, both nodes ignore each other so there is a different path. Only one path should remain)
			// This does a little bit of redunant work, since the shortcuts of the same contracted node don't need to get compared
			// to avoid this, store them in separate lists (maybe TODO)
			// a second slice can get avoided, when removing inplace (maybe TODO)
			finalShortcuts := make([]Shortcut, 0, len(candidateShortcuts))
			for i := 0; i < len(candidateShortcuts); i++ {
				shortcut := candidateShortcuts[i]
				keep := true
				for j := 0; j < len(candidateShortcuts); j++ {
					otherShortcut := candidateShortcuts[j]
					if shortcut.source == otherShortcut.source && shortcut.target == otherShortcut.target && /*shortcut.via != otherShortcut.via &&*/ shortcut.cost >= otherShortcut.cost {
						if i > j {
							// when equivalent, only use the first shortcut
							keep = false
						}
					}
				}
				if keep {
					finalShortcuts = append(finalShortcuts, shortcut)
				}
			}
			ch.addShortcuts(finalShortcuts)

			affectedNeighbors := make([]graph.NodeId, 0, len(neighborsMap))
			for node := range neighborsMap {
				affectedNeighbors = append(affectedNeighbors, node)
			}

			updateNodes = affectedNeighbors
			newShortcuts = len(finalShortcuts)

			intermediateUpdates = 0
			level++
		}

		shortcutCounter += newShortcuts
		if ch.milestones != nil && ch.milestoneIndex < len(ch.milestones) && float64(len(ch.contractedNodes))/float64(len(ch.orderOfNode)) > ch.milestones[ch.milestoneIndex]/100 {
			runtime := time.Since(ch.initialTime)
			timeDif := runtime
			if ch.milestoneIndex > 0 {
				timeDif -= ch.runtime[ch.milestoneIndex-1]
			}
			milestone := ch.milestones[ch.milestoneIndex]
			ch.runtime[ch.milestoneIndex] = runtime
			ch.shortcutCounter[ch.milestoneIndex] = shortcutCounter
			totalShortcuts := ch.shortcutCounter[ch.milestoneIndex]
			previousShortcuts := ch.shortcutCounter[0]
			if ch.milestoneIndex > 0 {
				previousShortcuts = ch.shortcutCounter[ch.milestoneIndex-1]
			}
			addedDifference := totalShortcuts - previousShortcuts

			f, err := os.OpenFile(ch.milestoneFilename, os.O_APPEND|os.O_WRONLY|os.O_CREATE, 0600)
			if err != nil {
				panic(err)
			}

			defer f.Close()
			if _, err = f.WriteString(fmt.Sprintf("Milestone %05.2f %% - Runtime: %6.3f s, difference: %.3f s, total Shortcuts: %5v, added Shortcuts: %5v\n", milestone, float64(runtime.Microseconds())/1000000, float64(timeDif.Microseconds())/1000000, totalShortcuts, addedDifference)); err != nil {
				panic(err)
			}

			ch.milestoneIndex++
		}

		// update all nodes or neighbors (if required)
		if oo.IsPeriodic() && level%100 == 0 {
			ch.updateFullContractionOrder(oo)
		} else if oo.UpdateNeighbors() {
			ch.updateOrderForNodes(updateNodes, oo)
		}
	}
	ch.liftUncontractedNodes()
}

// Set all uncontracted nodes to highest level
func (ch *ContractionHierarchies) liftUncontractedNodes() {
	for i := range ch.orderOfNode {
		if ch.orderOfNode[i] == -1 {
			ch.orderOfNode[i] = math.MaxInt
		}
	}
}

// update the whole node order (pqOrder) by performing a virtual contraction for each remaining node
func (ch *ContractionHierarchies) updateFullContractionOrder(oo OrderOptions) {
	nodes := make([]graph.NodeId, ch.pqOrder.Len())
	for i := 0; i < ch.pqOrder.Len(); i++ {
		orderItem := ch.pqOrder.PeekAt(i).(*OrderItem)
		nodes[i] = orderItem.nodeId
	}
	ch.updateOrderForNodes(nodes, oo)
	// TODO Verify if this would be more efficient
	// heap.Init(pqOrder)
}

// Contract the node given by nodeId and add shortcuts if necessary.
// if computeEdgeDifferenceOnly is true, the node is not contracted but only the necessary shortcuts are calculated.
// This returns 3 values:
//	- number of added/needed shortcuts
//	- number of (active) incident arcs to that node (arcs from nodes which are not contracted, yet)
//	- number of already contracted neighbors
func (ch *ContractionHierarchies) computeNodeContraction(nodeId graph.NodeId, ignoreNodes []graph.NodeId, contractionWorker *UniversalDijkstra) *ContractionResult {
	shortcuts := make([]Shortcut, 0)
	contractedNeighbors := 0

	contractionWorker.SetIgnoreNodes(ignoreNodes)
	contractionWorker.SetDebugLevel(ch.dijkstra.debugLevel) // copy debug level

	var runtime time.Duration = 0
	computations := 0

	arcs := ch.dg.GetArcsFrom(nodeId)
	incidentArcsAmount := len(arcs)
	if ch.debugLevel >= 4 {
		log.Printf("Incident arcs %v\n", incidentArcsAmount)
	}
	for i := 0; i < len(arcs); i++ {
		arc := arcs[i]
		source := arc.Destination()
		if ch.isNodeContracted(source) {
			if ch.debugLevel >= 4 {
				log.Printf("source %v already processed\n", source)
			}
			contractedNeighbors++
			continue
		}
		for j := i + 1; j < len(arcs); j++ {
			otherArc := arcs[j]
			target := otherArc.Destination()
			if source == target {
				// source shouldn't be the same like target
				panic("This should not happen")
			}
			if ch.isNodeContracted(target) {
				if ch.debugLevel >= 4 {
					log.Printf("target %v already processed\n", target)
				}
				//contractedNeighbors++
				continue
			}

			if ch.debugLevel >= 4 {
				log.Printf("testing for shortcut %v -> %v\n", source, target)
			}
			maxCost := arc.Cost() + otherArc.Cost()
			contractionWorker.SetCostUpperBound(maxCost)
			start := time.Now()
			length := contractionWorker.ComputeShortestPath(source, target)
			elapsed := time.Since(start)
			runtime += elapsed
			computations++

			if ch.debugLevel >= 4 {
				log.Printf("Length: %v, cost via node: %v, pq pops: %v\n", length, maxCost, ch.dijkstra.GetPqPops())
			}
			if length == -1 || length > maxCost {
				// add shortcut, since the path via this node is the fastest
				// without this node, the target is either not reachable or the path is longer
				if ch.debugLevel >= 4 {
					log.Printf("Shortcut %v -> %v via %v needed\n", source, target, nodeId)
				}
				shortcut := Shortcut{source: source, target: target, via: nodeId, cost: maxCost}
				// add reverse shortcut since this is only computed once
				// only calculate source -> target once, don't calculate target -> source
				reverseShortcut := Shortcut{source: target, target: source, via: nodeId, cost: maxCost}
				shortcuts = append(shortcuts, []Shortcut{shortcut, reverseShortcut}...)
			}
		}
	}

	if ch.debugLevel >= 3 && computations > 0 {
		log.Printf("Dijkstra Runtime: %v us * %v, node %v\n", float64(int(runtime.Nanoseconds())/computations)/1000, computations, nodeId)
	}

	// Fix incident arcs: Remove number of already contracted neighbors
	incidentArcsAmount -= contractedNeighbors

	// number of shortcuts is doubled, since we have two arcs for each each (because symmetric graph)
	// TODO maybe divide len(shortcuts) by 2 when using them
	contractionResult := &ContractionResult{nodeId: nodeId, shortcuts: shortcuts, incidentEdges: 2 * incidentArcsAmount, contractedNeighbors: contractedNeighbors}
	return contractionResult
}

// Add the shortcuts to the graph (by adding new arcs)
func (ch *ContractionHierarchies) addShortcuts(shortcuts []Shortcut) {
	for _, sc := range shortcuts {
		ch.addShortcut(sc.source, sc.target, sc.via, sc.cost)
	}
	_, exists := ch.addedShortcuts[len(shortcuts)]
	if !exists {
		ch.addedShortcuts[len(shortcuts)] = 1
	} else {
		ch.addedShortcuts[len(shortcuts)]++
	}
}

// Adds a shortcut to the graph from source to target with length cost which is spanned over node defined by via.
// This adds a new arc in the graph.
func (ch *ContractionHierarchies) addShortcut(source, target, via graph.NodeId, cost int) {
	defer func() {
		if r := recover(); r != nil {
			fmt.Printf("from: %v, to: %v, via: %v, cost: %v\n", source, target, via, cost)
			fmt.Printf("Level - source: %v, target: %v, via: %v\n", ch.orderOfNode[source], ch.orderOfNode[target], ch.orderOfNode[via])
			panic("Terminating.")
		}
	}()
	if ch.debugLevel >= 3 {
		log.Printf("Add shortcut %v %v %v %v\n", source, target, via, cost)
	}
	if ch.orderOfNode[source] != -1 || ch.orderOfNode[target] != -1 {
		panic("Edge Node already contracted")
	}
	added := ch.dg.AddArc(source, target, cost)
	if added {
		sc := Shortcut{source: source, target: target, via: via}
		ch.shortcuts = append(ch.shortcuts, sc)
	}
	// maybe this map is not so a good idea
	/*
		if ch.shortcutMap[source] == nil {
			ch.shortcutMap[source] = make(map[graph.NodeId]graph.NodeId)
		}
		ch.shortcutMap[source][target] = nodeId
	*/
}

// Enable all arcs for the node given by nodeId.
func (ch *ContractionHierarchies) enableArcsForNode(nodeId graph.NodeId) {
	ch.g.SetArcFlags(nodeId, true)
	if ch.debugLevel >= 3 {
		log.Printf("enable arcs of node %v\n", nodeId)
	}
}

// Disable all arcs for the node given by nodeId.
func (ch *ContractionHierarchies) disableArcsForNode(nodeId graph.NodeId) {
	ch.g.SetArcFlags(nodeId, false)
	if ch.debugLevel >= 3 {
		log.Printf("disable arcs of node %v\n", nodeId)
	}
}

// Enable arcs which point to a node with higher level. Disable all other ones
func (ch *ContractionHierarchies) matchArcsWithNodeOrder() {
	for source := range ch.g.GetNodes() {
		for _, arc := range ch.g.GetArcsFrom(source) {
			target := arc.Destination()
			arc.SetArcFlag(ch.orderOfNode[source] < ch.orderOfNode[target] || ch.orderOfNode[target] == math.MaxInt)
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

	// map: source -> target -> via
	ch.shortcutMap = make(map[graph.NodeId]map[graph.NodeId]graph.NodeId)
	for _, sc := range ch.shortcuts {
		// TODO maybe check if it is nil, instead of exists
		if _, exists := ch.shortcutMap[sc.source]; !exists {
			ch.shortcutMap[sc.source] = make(map[graph.NodeId]graph.NodeId)
		}
		/*
			if via, exists := ch.shortcutMap[sc.source][sc.target]; exists {
				// TODO Add panic for testing. But should be no problem
				panic(fmt.Sprintf("Shortcut already exists. Old connector: %v, new connector: %v", via, sc.via))
			}
		*/
		ch.shortcutMap[sc.source][sc.target] = sc.via
	}
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
func (ch *ContractionHierarchies) SetNodeOrdering(nodeOrdering [][]int) {
	ch.orderOfNode = make([]int, ch.g.NodeCount())

	// initialize order of node
	for i := range ch.orderOfNode {
		// TODO think about setting it directly to match.MaxInt
		// This would save a second iteration (liftUncontractedNodes)
		ch.orderOfNode[i] = -1
	}

	for i, nodeIds := range nodeOrdering {
		for _, nodeId := range nodeIds {
			ch.orderOfNode[nodeId] = i
		}
	}

	ch.liftUncontractedNodes()
}

// Set if the arcs should be sorted?
// TODO
// flag indicating if the arcs are sorted (first enabled arcs, then disabled arcs)
func (ch *ContractionHierarchies) SetSortArcs(flag bool) {
	if flag {
		ch.g.SortArcs()
		ch.dijkstra.SortedArcs(true)
	}
}

// Set the debug level
func (ch *ContractionHierarchies) SetDebugLevel(level int) {
	ch.debugLevel = level
}

// Set the precomputation milestones (which are worth a log message)
func (ch *ContractionHierarchies) SetPrecomputationMilestones(milestones []float64) {
	ch.milestones = milestones
	ch.milestoneIndex = 0
	ch.runtime = make([]time.Duration, len(milestones))
	ch.shortcutCounter = make([]int, len(milestones))
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
	for _, cascadedNodeOrdering := range ch.nodeOrdering {
		for _, v := range cascadedNodeOrdering {
			order := fmt.Sprintf("%v ", v)
			sb.WriteString(order)
		}
		order := fmt.Sprintf("\n")
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
		fmt.Sscanf(line, "%d %d %d", &source, &target, &via)
		sc := Shortcut{source: source, target: target, via: via}
		shortcuts = append(shortcuts, sc)
	}
	return shortcuts
}

// Read a node ordering file and return the node order
func ReadNodeOrderingFile(filename string) [][]int {
	file, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return ConvertToNodeOrdering(string(file))
}

// Parse a string to a lsit which specifies the node order
func ConvertToNodeOrdering(nodeOrderingString string) [][]int {
	scanner := bufio.NewScanner(strings.NewReader(nodeOrderingString))

	nodeOrdering := make([][]int, 0)
	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 || line[0] == '#' {
			// skip empty lines and comments
			continue
		}
		stringNodeIds := strings.Fields(line)
		nodeIds := make([]graph.NodeId, len(stringNodeIds))
		for i, sid := range stringNodeIds {
			id, err := strconv.Atoi(sid)
			if err != nil {
				log.Fatal(err)
			}
			nodeIds[i] = id
		}
		nodeOrdering = append(nodeOrdering, nodeIds)
	}
	return nodeOrdering
}
