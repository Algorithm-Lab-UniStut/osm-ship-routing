package path

import (
	"testing"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

const cuttableGraph = `13
42
#Nodes
0 0 0
1 0 2
2 1 1
3 1 2
4 2 0
5 2 1
6 2 2
7 3 0
8 3 1
9 3 3
10 5 0
11 4 1
12 5 2
#Edges
0 1 3
0 2 4
0 4 7
1 0 3
1 2 5
1 3 2
2 0 4
2 1 5
2 3 2
2 5 1
3 1 2
3 2 2
3 6 5
4 0 7
4 5 4
4 7 6
5 2 1
5 4 4
5 6 3
5 8 1
6 3 5
6 5 3
6 9 7
7 4 6
7 8 3
7 10 5
8 5 1
8 7 3
8 9 3
8 11 1
9 6 7
9 8 3
9 12 4
10 7 5
10 11 2
10 12 4
11 8 1
11 10 2
11 12 3
12 9 4
12 10 4
12 11 3
`

func TestGivenNodeOrdering(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg)
	ch := NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
	nodeOrdering := []int{0, 1, 10, 12, 7, 4, 9, 3, 6, 5, 8, 11, 2}
	//ch.SetDebugLevel(1)
	ch.Precompute(nodeOrdering, MakeOrderOptions())
	if len(ch.nodeOrdering) != len(nodeOrdering) {
		t.Errorf("noder ordering length does not match")
	}
	for i, cascadedNodeOrdering := range ch.nodeOrdering {
		if len(cascadedNodeOrdering) != 1 {
			// in each hierarchy, only one level should be present (when node order is given)
			t.Errorf("Node ordering is weird.\n")
		}
		if nodeOrdering[i] != cascadedNodeOrdering[0] {
			t.Errorf("ordering at position %v does not match\n", i)
		}
	}
}

func TestContractGraph(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg)
	dijkstra.SetDebugLevel(1)
	ch := NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
	ch.SetDebugLevel(1)
	nodeOrdering := []int{0, 1, 10, 12, 7, 4, 9, 3, 6, 5, 8, 11, 2}
	ch.Precompute(nodeOrdering, MakeOrderOptions())
	for i, cascadedNodeOrdering := range ch.nodeOrdering {
		if len(cascadedNodeOrdering) != 1 {
			// in each hierarchy, only one level should be present (when node order is given)
			t.Errorf("Node ordering is weird.\n")
		}
		if nodeOrdering[i] != cascadedNodeOrdering[0] {
			t.Errorf("node ordering was changed.\n")
		}
	}
	if len(ch.GetShortcuts())/2 != 2 {
		t.Errorf("wrong number of nodes shortcuttet.\n")
	}
	if len(ch.addedShortcuts) > 2 {
		t.Errorf("wrong number of shortcuts.\n")
	}
	zeroShortcuts := ch.addedShortcuts[0]
	if zeroShortcuts != 11 {
		t.Errorf("wrong number of 0 shortcuts\n")
	}
	twoShortcuts := ch.addedShortcuts[1]
	if twoShortcuts != 2 {
		t.Errorf("wrong number of 2 shortcuts\n")
	}
	if ch.g.ArcCount() != 46 {
		t.Errorf("wrong number of Arcs added")
	}
}

func TestPathFinding(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg)
	source, target := 0, 12
	l := dijkstra.ComputeShortestPath(source, target)
	p := dijkstra.GetPath(source, target)
	ch := NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
	nodeOrdering := []int{0, 1, 10, 12, 7, 4, 9, 3, 6, 5, 8, 11, 2}
	ch.Precompute(nodeOrdering, MakeOrderOptions())
	ch.ShortestPathSetup(MakeDefaultPathFindingOptions())
	length := ch.ComputeShortestPath(source, target)
	if l != length {
		t.Errorf("Length do not match")
	}
	path := ch.GetPath(source, target)
	if len(p) != len(path) || p[0] != path[0] || p[len(p)-1] != path[len(path)-1] {
		t.Errorf("computed SP do not match")
	}

	alg = graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra = NewUniversalDijkstra(alg)
	ch = NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
	nodeOrdering = []int{1, 5, 9, 4, 3, 11, 10, 6, 8, 2, 7, 0, 12}
	ch.Precompute(nodeOrdering, MakeOrderOptions())
	ch.ShortestPathSetup(MakeDefaultPathFindingOptions())
	length = ch.ComputeShortestPath(source, target)
	if l != length {
		t.Errorf("Length do not match")
	}
	path = ch.GetPath(source, target)
	if len(p) != len(path) || p[0] != path[0] || p[len(p)-1] != path[len(path)-1] {
		t.Errorf("computed SP do not match. Shortcuts: %v", ch.GetShortcuts())
	}
}

func TestPrecompute(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg)
	ch := NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
	ch.SetDebugLevel(3)
	ch.Precompute(nil, MakeOrderOptions().SetLazyUpdate(false).SetEdgeDifference(true).SetProcessedNeighbors(true).SetUpdateNeighbors(true))
}

func TestContractionHierarchies(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg)
	source, target := 0, 12
	l := dijkstra.ComputeShortestPath(source, target)
	p := dijkstra.GetPath(source, target)
	dijkstra.SetDebugLevel(1)
	ch := NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
	ch.SetDebugLevel(2)
	ch.Precompute(nil, MakeOrderOptions().SetLazyUpdate(false).SetEdgeDifference(true).SetProcessedNeighbors(true).SetUpdateNeighbors(true))
	ch.ShortestPathSetup(MakeDefaultPathFindingOptions())
	length := ch.ComputeShortestPath(source, target)
	if length != l {
		t.Errorf("Length does not match")
	}
	if length != 10 {
		t.Errorf("Wrong length")
	}
	path := ch.GetPath(source, target)
	if len(p) != len(path) || p[0] != path[0] || p[len(p)-1] != path[len(path)-1] {
		t.Errorf("computed SP do not match")
	}
}

func TestRandomContraction(t *testing.T) {
	referenceAlg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	referenceDijkstra := NewUniversalDijkstra(referenceAlg)
	source, target := 0, 12
	l := referenceDijkstra.ComputeShortestPath(source, target)
	p := referenceDijkstra.GetPath(source, target)
	for i := 0; i < 100; i++ {
		alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
		dijkstra := NewUniversalDijkstra(alg)
		//dijkstra.SetDebugLevel(3)
		ch := NewContractionHierarchies(alg, dijkstra, MakeDefaultContractionOptions())
		//ch.SetDebugLevel(3)
		ch.Precompute(nil, MakeOrderOptions().SetLazyUpdate(false).SetRandom(true))
		ch.ShortestPathSetup(MakeDefaultPathFindingOptions())
		length := ch.ComputeShortestPath(source, target)
		if length != l {
			t.Errorf("Length does not match - Is: %v. Should: %v", length, l)
		}
		path := ch.GetPath(source, target)
		if len(p) != len(path) || p[0] != path[0] || p[len(p)-1] != path[len(path)-1] {
			t.Errorf("computed SP do not match. Shortcuts: %v", ch.GetShortcuts())
		}
	}
}
