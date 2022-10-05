package path

import (
	"fmt"
	"log"
	"os"
	"testing"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

const graphFmi = `10
26
# nodes
0 0 0
1 0 1
2 0 2
3 1 0
4 1 1
5 1 2
6 2 0
7 2 1
8 2 2
9 3 3
# edges
0 1 1
0 3 1
1 0 1
1 2 1
1 4 1
2 1 1
2 5 1
3 0 1
3 4 1
3 6 1
4 1 1
4 3 1
4 5 1
4 7 1
5 2 1
5 4 1
5 8 1
6 3 1
6 7 1
7 4 1
7 6 1
7 8 1
8 5 1
8 7 1
8 9 1
9 8 1`

func TestPlainDijkstra(t *testing.T) {
	aag := graph.NewAdjacencyArrayFromFmiString(graphFmi)
	d := NewUniversalDijkstra(aag)
	length := d.ComputeShortestPath(0, 9)
	path := d.GetPath(0, 9)
	pathReference := []int{0, 1, 4, 5, 8, 9}
	lengthReference := 5
	if length != lengthReference {
		t.Errorf("length is %v. Should be %v\n", length, lengthReference)
	}
	if len(path) != len(pathReference) {
		t.Errorf("path has wrong length. Is %v, should be %v\n", len(path), len(pathReference))
	}
	for i, v := range pathReference {
		if path[i] != v {
			t.Errorf("path at position %v has wrong value. Is %v, should be %v\n", i, path[i], v)
		}
	}

}

func TestAStarDijkstra(t *testing.T) {
	aag := graph.NewAdjacencyArrayFromFmiString(graphFmi)
	d := NewUniversalDijkstra(aag)
	astar := NewUniversalDijkstra(aag)
	astar.SetUseHeuristic(true)
	length := d.ComputeShortestPath(0, 9)
	path := d.GetPath(0, 9)
	astarLength := astar.ComputeShortestPath(0, 9)
	astarPath := astar.GetPath(0, 9)
	if length != astarLength {
		t.Errorf("Length does not match. Is %v, should be %v", astarLength, length)
	}
	if len(astarPath) != len(path) {
		t.Errorf("Path has wrong length. Is %v, should be %v", len(astarPath), len(path))
	}
	if astarPath[0] != path[0] || astarPath[len(astarPath)-1] != path[len(path)-1] {
		t.Errorf("First or last element do not math.\nDijkstra: %v %v\nAStar: %v %v", path[0], path[len(path)-1], astarPath[0], astarPath[len(astarPath)-1])
	}
}

func TestBidirectionalDijkstra(t *testing.T) {
	aag := graph.NewAdjacencyArrayFromFmiString(graphFmi)
	d := NewUniversalDijkstra(aag)
	bidijkstra := NewUniversalDijkstra(aag)
	bidijkstra.SetBidirectional(true)
	length := d.ComputeShortestPath(0, 9)
	path := d.GetPath(0, 9)
	bidijkstraLength := bidijkstra.ComputeShortestPath(0, 9)
	bidijkstraPath := bidijkstra.GetPath(0, 9)
	if length != bidijkstraLength {
		t.Errorf("Length does not match. Is %v, should be %v", bidijkstraLength, length)
	}
	if len(bidijkstraPath) != len(path) {
		t.Errorf("Path has wrong length. Is %v, should be %v", len(bidijkstraPath), len(path))
	}
	if bidijkstraPath[0] != path[0] || bidijkstraPath[len(bidijkstraPath)-1] != path[len(path)-1] {
		t.Errorf("First or last element do not math.\nDijkstra: %v %v\nAStar: %v %v", path[0], path[len(path)-1], bidijkstraPath[0], bidijkstraPath[len(bidijkstraPath)-1])
	}
}

func BenchmarkPlainDijkstra(b *testing.B) {
	dirname, err := os.UserHomeDir()
	if err != nil {
		log.Fatal(err)
	}

	targets := [][4]int{{494605, 285854, 10997302, 480},
		{604036, 168163, 11261529, 487},
		{174605, 4274, 9693631, 426},
		{507581, 318527, 4354181, 187},
		{408389, 73715, 21980178, 961},
		{660648, 48223, 16820403, 720},
		{179257, 318504, 9257630, 411},
		{113983, 113598, 15285174, 651},
		{102268, 269489, 10849334, 474},
		{683277, 146270, 12195462, 527}}
	aag := graph.NewAdjacencyArrayFromFmiFile(dirname + "/osm/ocean_equi_4.fmi")
	dijkstra := NewUniversalDijkstra(aag)
	for n := 0; n < b.N; n++ {
		pqPops := 0
		invalidLengths := make([][2]int, 0)
		invalidResults := make([]int, 0)
		invalidHops := make([][3]int, 0)
		for i, target := range targets {
			origin := target[0]
			destination := target[1]
			referenceLength := target[2]
			referenceHops := target[3]

			length := dijkstra.ComputeShortestPath(origin, destination)
			pqPops += dijkstra.GetPqPops()
			path := dijkstra.GetPath(origin, destination)

			if length != referenceLength {
				invalidLengths = append(invalidLengths, [2]int{i, length - referenceLength})
			}
			if length > -1 && (path[0] != origin || path[len(path)-1] != destination) {
				invalidResults = append(invalidResults, i)
			}
			if referenceHops != len(path) {
				invalidHops = append(invalidHops, [3]int{i, len(path), referenceHops})
			}
		}
		fmt.Printf("Average pq pops: %d\n", pqPops/len(targets))
		fmt.Printf("%v/%v invalid path lengths.\n", len(invalidLengths), len(targets))

		for i, testCase := range invalidLengths {
			fmt.Printf("%v: Case %v (%v -> %v) has invalid length. Difference: %v\n", i, testCase[0], targets[testCase[0]][0], targets[testCase[0]][1], testCase[1])
		}
		fmt.Printf("%v/%v invalid Result (source/target).\n", len(invalidResults), len(targets))
		for i, result := range invalidResults {
			fmt.Printf("%v: Case %v (%v -> %v) has invalid result\n", i, result, targets[result][0], targets[result][1])
		}
		fmt.Printf("%v/%v invalid hops number.\n", len(invalidHops), len(targets))
		for i, hops := range invalidHops {
			testcase := hops[0]
			actualHops := hops[1]
			referenceHops := hops[2]
			fmt.Printf("%v: Case %v (%v -> %v) has invalid #hops. Has: %v, reference: %v, difference: %v\n", i, testcase, targets[testcase][0], targets[testcase][1], actualHops, referenceHops, actualHops-referenceHops)
		}
	}
}

func BenchmarkContractionHierarchies(b *testing.B) {
	dirname, err := os.UserHomeDir()
	if err != nil {
		log.Fatal(err)
	}

	targets := [][4]int{{494605, 285854, 10997302, 480},
		{604036, 168163, 11261529, 487},
		{174605, 4274, 9693631, 426},
		{507581, 318527, 4354181, 187},
		{408389, 73715, 21980178, 961},
		{660648, 48223, 16820403, 720},
		{179257, 318504, 9257630, 411},
		{113983, 113598, 15285174, 651},
		{102268, 269489, 10849334, 474},
		{683277, 146270, 12195462, 527}}
	aag := graph.NewAdjacencyArrayFromFmiFile(dirname + "/osm/big_contracted_graph.fmi")
	shortcuts := ReadShortcutFile(dirname + "/osm/big_shortcuts.txt")
	nodeOrdering := ReadNodeOrderingFile(dirname + "/osm/big_node_ordering.txt")
	dijkstra := NewUniversalDijkstra(aag)
	ch := NewContractionHierarchiesInitialized(aag, dijkstra, shortcuts, nodeOrdering, MakeDefaultPathFindingOptions())
	for _, target := range targets {
		origin := target[0]
		destination := target[1]
		b.Run(fmt.Sprintf("Target: %v", target), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				ch.ComputeShortestPath(origin, destination)
			}
		})
	}
}
