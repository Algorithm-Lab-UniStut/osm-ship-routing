package path

import (
	"fmt"
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

func TestNodeOrdering(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg, false)
	ch := NewContractionHierarchies(alg, dijkstra)
	nodeOrdering := []int{0, 1, 10, 12, 7, 4, 9, 3, 6, 5, 8, 11, 2}
	ch.nodeOrdering = nodeOrdering
	if len(ch.nodeOrdering) != len(nodeOrdering) {
		t.Errorf("noder ordering length does not match")
	}
	for i := range ch.nodeOrdering {
		if ch.nodeOrdering[i] != nodeOrdering[i] {
			t.Errorf("ordering at position %v does not match", i)
		}
	}
}

func TestContractGraph(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg, false)
	ch := NewContractionHierarchies(alg, dijkstra)
	nodeOrdering := []int{0, 1, 10, 12, 7, 4, 9, 3, 6, 5, 8, 11, 2}
	ch.debugLevel = 0
	ch.Precompute(nodeOrdering)
	fmt.Printf("node ordering: %v\n", ch.nodeOrdering)
	fmt.Printf("shortcuts: %v\n", ch.GetShortcuts())
	if len(ch.addedShortcuts) != 2 {
		t.Errorf("wrong number of nodes shortcuttet.\n")
	}
	zeroShortcuts, ok := ch.addedShortcuts[0]
	if !ok || zeroShortcuts != 11 {
		t.Errorf("wrong number of 0 shortcuts\n")
	}
	twoShortcuts, ok := ch.addedShortcuts[2]
	if twoShortcuts != 2 {
		t.Errorf("wrong number of 2 shortcuts\n")
	}
	//fmt.Println(ch.g.AsString())
	if ch.g.ArcCount() != 46 {
		t.Errorf("wrong number of Arcs added")
	}
	//ch.WriteContractionResult()
}

func TestPathFinding(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg, false)
	source, target := 0, 12
	l := dijkstra.ComputeShortestPath(source, target)
	p := dijkstra.GetPath(source, target)
	ch := NewContractionHierarchies(alg, dijkstra)
	nodeOrdering := []int{0, 1, 10, 12, 7, 4, 9, 3, 6, 5, 8, 11, 2}
	//ch.nodeOrdering = nodeOrdering
	ch.Precompute(nodeOrdering)
	length := ch.ComputeShortestPath(source, target)
	if l != length {
		t.Errorf("Length do not match")
	}
	fmt.Println(ch.GetSearchSpace())
	path := ch.GetPath(source, target)
	if len(p) != len(path) || p[0] != path[0] || p[len(p)-1] != path[len(path)-1] {
		t.Errorf("computed SP do not match")
	}
}

func TestPrecompute(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg, false)
	ch := NewContractionHierarchies(alg, dijkstra)
	ch.debugLevel = 0
	ch.Precompute(nil)
}

func TestContractionHierarchies(t *testing.T) {
	alg := graph.NewAdjacencyListFromFmiString(cuttableGraph)
	dijkstra := NewUniversalDijkstra(alg, false)
	source, target := 0, 12
	l := dijkstra.ComputeShortestPath(source, target)
	p := dijkstra.GetPath(source, target)
	ch := NewContractionHierarchies(alg, dijkstra)
	ch.Precompute(nil)
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
	fmt.Println(path)
}
