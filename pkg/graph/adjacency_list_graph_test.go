package graph

import (
	"testing"
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

func TestGraphReading(t *testing.T) {
	alg := NewAdjacencyListFromFmiString(cuttableGraph)
	if alg.AsString() != cuttableGraph {
		t.Errorf("Graph wrongly parsed\n")
	}
}
