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

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

type ContractionHierarchies struct {
	g                    graph.Graph
	dijkstra             *UniversalDijkstra
	nodeOrdering         []graph.NodeId
	orderOfNode          []int
	shortcuts            []Shortcut
	addedShortcuts       map[int]int
	debugLevel           int
	orderOptions         OrderOptions
	graphFilename        string
	shortcutsFilename    string
	nodeOrderingFilename string
}

type Shortcut struct {
	source graph.NodeId
	target graph.NodeId
	via    graph.NodeId
}

type OrderOptions byte

const (
	dynamicOrder OrderOptions = 1 << iota
	considerEdgeDifference
	considerProcessedNeighbors
)

func NewContractionHierarchies(g graph.Graph, dijkstra *UniversalDijkstra) *ContractionHierarchies {
	return &ContractionHierarchies{g: g, dijkstra: dijkstra, graphFilename: "contracted_graph.fmi", shortcutsFilename: "shortcuts.txt", nodeOrderingFilename: "node_ordering.txt"}
}

func NewContractionHierarchiesInitialized(g graph.Graph, dijkstra *UniversalDijkstra, shortcuts []Shortcut, nodeOrdering []int) *ContractionHierarchies {
	ch := NewContractionHierarchies(g, dijkstra)
	ch.SetShortcuts(shortcuts)
	ch.SetNodeOrdering(nodeOrdering)
	return ch
}

func (ch *ContractionHierarchies) SetShortcuts(shortcuts []Shortcut) {
	ch.shortcuts = shortcuts
}

func (ch *ContractionHierarchies) ReadShortcutFile() []Shortcut {
	file, err := os.ReadFile(ch.shortcutsFilename)
	if err != nil {
		log.Fatal(err)
	}
	return ch.ConvertToShortcuts(string(file))
}

func (ch *ContractionHierarchies) ConvertToShortcuts(shortcutsString string) []Shortcut {
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

func (ch *ContractionHierarchies) SetNodeOrdering(nodeOrdering []int) {
	ch.nodeOrdering = nodeOrdering
}

func (ch *ContractionHierarchies) ReadNodeOrderingFile() []int {
	file, err := os.ReadFile(ch.shortcutsFilename)
	if err != nil {
		log.Fatal(err)
	}
	return ch.ConvertToNodeOrdering(string(file))
}

func (ch *ContractionHierarchies) ConvertToNodeOrdering(nodeOrderingString string) []int {
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

func (ch *ContractionHierarchies) ComputeNodeOrdering() {
	ch.computeRandomNodeOrdering()
}
func (ch *ContractionHierarchies) computeRandomNodeOrdering() {
	ch.nodeOrdering = make([]int, ch.g.NodeCount())
	for i := range ch.nodeOrdering {
		ch.nodeOrdering[i] = i
	}
	//rand.Seed(time.Now().UnixNano()) // completely random
	rand.Shuffle(len(ch.nodeOrdering), func(i, j int) { ch.nodeOrdering[i], ch.nodeOrdering[j] = ch.nodeOrdering[j], ch.nodeOrdering[i] })
}

func (ch *ContractionHierarchies) ContractNode(nodeId graph.NodeId, computeEdgeDifferenceOnly bool) (int, int) {
	ch.dijkstra.SetMaxNumSettledNodes(20)
	if ch.debugLevel == 1 {
		fmt.Printf("Contract Node %v\n", nodeId)
	}
	shortcutsForNode := 0
	contractedNeighbors := 0
	incidentArcs := len(ch.g.GetArcsFrom(nodeId))
	ch.disableArcsForNode(nodeId)
	for _, arc := range ch.g.GetArcsFrom(nodeId) {
		source := arc.Destination()
		if ch.isNodeAlreadyProcessed(source) {
			if ch.debugLevel == 1 {
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
			if ch.isNodeAlreadyProcessed(target) {
				if ch.debugLevel == 1 {
					fmt.Printf("target %v already processed\n", target)
				}
				//contractedNeighbors++
				continue
			}
			if ch.debugLevel == 1 {
				fmt.Printf("testing for shortcut %v -> %v\n", source, target)
			}
			maxCost := arc.Cost() + otherArc.Cost()
			ch.dijkstra.SetCostUpperBound(maxCost)
			ch.dijkstra.SetConsiderArcFlags(true)
			_, length := ch.dijkstra.GetPath(source, target)
			if length == -1 || length > maxCost {
				// add shortcut, since the path via this node is the fastest
				// without this node, the target is either not reachable or the path is longer
				shortcutsForNode++
				if !computeEdgeDifferenceOnly {
					shortcut := graph.NewEdge(target, source, maxCost)
					g, ok := ch.g.(graph.DynamicGraph)
					if !ok {
						panic("Adding edge not possible, Make sure to provide an dynamic graph")
					}
					g.AddEdge(*shortcut)
					ch.shortcuts = append(ch.shortcuts, Shortcut{source: source, target: target, via: nodeId})
					if ch.debugLevel == 1 {
						fmt.Printf("Add shortcut %v %v %v %v\n", source, target, maxCost, nodeId)
					}
				}
			}

		}
	}
	if computeEdgeDifferenceOnly {
		// reset temporarily disabled arcs
		ch.enableArcsForNode(nodeId)
	}

	if !computeEdgeDifferenceOnly {
		_, exists := ch.addedShortcuts[shortcutsForNode]
		if !exists {
			ch.addedShortcuts[shortcutsForNode] = 1
		} else {
			ch.addedShortcuts[shortcutsForNode]++
		}
	}
	// number of shortcuts is doubled, since we have two arcs for each each (because symmetric graph)
	return shortcutsForNode/2 - incidentArcs, contractedNeighbors
}

func (ch *ContractionHierarchies) ContractNodes() {
	if ch.nodeOrdering == nil {
		panic("Node ordering not set")
	}
	if len(ch.nodeOrdering) != ch.g.NodeCount() {
		// this is a rudimentary test, if the ordering could be valid.
		// However, it misses to test if every id appears exactly once
		panic("Node ordering not valid")
	}
	ch.addedShortcuts = make(map[int]int)
	ch.shortcuts = make([]Shortcut, 0)
	ch.orderOfNode = make([]int, ch.g.NodeCount())
	for i := 0; i < ch.g.NodeCount(); i++ {
		for _, v := range ch.g.GetArcsFrom(i) {
			if !v.ArcFlag() {
				panic("Arc Flags are initially not completely true")
			}
		}
	}
	for i := range ch.g.GetNodes() {
		nodeId := ch.nodeOrdering[i]
		ch.orderOfNode[nodeId] = i
		ch.ContractNode(nodeId, false)
	}
	fmt.Println(ch.addedShortcuts)
}

func (ch *ContractionHierarchies) precompute() {
	ch.addedShortcuts = make(map[int]int)
	ch.shortcuts = make([]Shortcut, 0)
	ch.orderOfNode = make([]int, ch.g.NodeCount())
	ch.nodeOrdering = make([]int, ch.g.NodeCount())
	for i := 0; i < ch.g.NodeCount(); i++ {
		for _, v := range ch.g.GetArcsFrom(i) {
			if !v.ArcFlag() {
				panic("Arc Flags are initially not completely true")
			}
		}
	}
	//dynamicOrdering := true
	pq := NewNodeOrder(nil)
	for i := 0; i < ch.g.NodeCount(); i++ {
		ed, processedNeighbors := ch.ContractNode(i, true)
		orderItem := NewOrderItem(i)
		orderItem.edgeDifference = ed
		orderItem.processedNeighbors = processedNeighbors
		heap.Push(pq, orderItem)
	}
	position := 0
	for pq.Len() > 0 {
		pqItem := heap.Pop(pq).(*OrderItem)
		currentEdgeDifference, currentProcessedNeighbors := ch.ContractNode(pqItem.nodeId, true)
		if /*!dynamicOrdering ||*/ pq.Len() == 0 || currentEdgeDifference+currentProcessedNeighbors <= (*pq)[0].edgeDifference+(*pq)[0].processedNeighbors {
			// always stick to initially computed order or this is still the smallest edge difference
			ch.ContractNode(pqItem.nodeId, false)
			ch.orderOfNode[pqItem.nodeId] = position
			ch.nodeOrdering[position] = pqItem.nodeId
			position++
		} else {
			pqItem.edgeDifference = currentEdgeDifference
			pqItem.processedNeighbors = currentProcessedNeighbors
			heap.Push(pq, pqItem)
		}
	}
}

func (ch *ContractionHierarchies) computeShortestPath(origin, destination graph.NodeId) int {
	ch.dijkstra.SetMaxNumSettledNodes(math.MaxInt)
	ch.dijkstra.SetCostUpperBound(math.MaxInt)
	ch.dijkstra.SetConsiderArcFlags(true)
	ch.dijkstra.SetBidirectional(true)
	ch.disableArcsAccordingToNodeOrder()
	path, length := ch.dijkstra.GetPath(origin, destination)
	fmt.Println(path)
	return length
}

func (ch *ContractionHierarchies) GetPath(origin, destination graph.NodeId) ([]int, int) {
	// TODO: change interface:
	// two methods: compute path, get path
	// this function has to replace the shortcuts
	// TODO: this probably gets more efficient with other data structuers (store shortcuts as map -> faster access)
	path, length := ch.dijkstra.GetPath(origin, destination)
	for i := 0; i < len(path)-1; i++ {
		fmt.Println("length", len(path)-1)
		source := path[i]
		target := path[i+1]
		for _, sc := range ch.shortcuts {
			if sc.source == source && sc.target == target {
				path = slice.InsertInt(path, i+1, sc.via)
				fmt.Printf("Added node %v -> %v -> %v\n", source, sc.via, target)
				i-- // reevaluate, if the source has a shortcut to the currently added node
			}
		}
	}
	return path, length
}

func (ch *ContractionHierarchies) GetSearchSpace() []graph.Node {
	return ch.dijkstra.GetSearchSpace()
}

func (ch *ContractionHierarchies) IsShortcut(edge *graph.Edge) bool {
	// TODO is this necessary
	panic("not implemented")
}

func (ch *ContractionHierarchies) disableArcsForNode(nodeId graph.NodeId) {
	for _, arc := range ch.g.GetArcsFrom(nodeId) {
		// disable the outgoing arc
		if ch.debugLevel == 1 {
			fmt.Printf("disable arc %v -> %v\n", nodeId, arc.Destination())
		}
		arc.SetArcFlag(false)
		for _, otherArc := range ch.g.GetArcsFrom(arc.Destination()) {
			if otherArc.Destination() == nodeId {
				// disable the incoming arc
				//otherArc.SetArcFlag(false)
			}
		}
	}
}

func (ch *ContractionHierarchies) enableArcsForNode(nodeId graph.NodeId) {
	for _, arc := range ch.g.GetArcsFrom(nodeId) {
		// disable the outgoing arc
		if ch.debugLevel == 1 {
			fmt.Printf("disable arc %v -> %v\n", nodeId, arc.Destination())
		}
		arc.SetArcFlag(true)
		for _, otherArc := range ch.g.GetArcsFrom(arc.Destination()) {
			if otherArc.Destination() == nodeId {
				// disable the incoming arc
				//otherArc.SetArcFlag(false)
			}
		}
	}
}

func (ch *ContractionHierarchies) resetArcFlags() {
	for i := range ch.g.GetNodes() {
		for _, arc := range ch.g.GetArcsFrom(i) {
			arc.SetArcFlag(true)
		}
	}
}

func (ch *ContractionHierarchies) disableArcsAccordingToNodeOrder() {
	for source := range ch.g.GetNodes() {
		for _, arc := range ch.g.GetArcsFrom(source) {
			target := arc.Destination()
			arc.SetArcFlag(ch.orderOfNode[source] < ch.orderOfNode[target])
		}
	}
}

func (ch *ContractionHierarchies) isNodeAlreadyProcessed(nodeId graph.NodeId) bool {
	// check whether the arcs of the node are already disabled
	// TODO: other options: check if the nodeId is on a lower position in the nodeOrdering array than the node which gets contracted
	for _, arc := range ch.g.GetArcsFrom(nodeId) {
		if arc.ArcFlag() {
			return false
		}
	}
	return true
	/*
		if !contractionNode.arc.ArcFlag() && false {
			// already processed this arc (from a previous contraction)
			// ignore this arc
			// TODO: is this necessary? or does only the outgoing arcs have to get deactivated (since then, the "next" node can't reach anything)
			fmt.Printf("source arc flag %v -> %v not set\n", nodeId, arc.Destination())
			continue
		}
	*/
}

func (ch *ContractionHierarchies) WriteGraph() {
	graph.WriteFmi(ch.g, ch.graphFilename)
}

func (ch *ContractionHierarchies) WriteShortcuts() {
	file, cErr := os.Create(ch.shortcutsFilename)

	if cErr != nil {
		log.Fatal(cErr)
	}

	var sb strings.Builder
	for _, v := range ch.shortcuts {
		shortcut := fmt.Sprintf("%v %v %v\n", v.source, v.target, v.via)
		sb.WriteString(shortcut)
	}
	writer := bufio.NewWriter(file)
	writer.WriteString(sb.String())
	writer.Flush()
}

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

func (ch *ContractionHierarchies) WriteContractionResult() {
	ch.WriteGraph()
	ch.WriteShortcuts()
	ch.WriteNodeOrdering()
}
