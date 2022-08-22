package graph

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
)

// fmi parse states
const (
	PARSE_NODE_COUNT = iota
	PARSE_EDGE_COUNT = iota
	PARSE_NODES      = iota
	PARSE_EDGES      = iota
)

func WriteFmi(g Graph, filename string) {
	file, cErr := os.Create(filename)

	if cErr != nil {
		log.Fatal(cErr)
	}

	graphAsString := g.AsString()
	writer := bufio.NewWriter(file)
	writer.WriteString(graphAsString)
	writer.Flush()
}

func NewAdjacencyListFromFmiString(fmi string) *AdjacencyListGraph {
	scanner := bufio.NewScanner(strings.NewReader(fmi))

	numNodes := 0
	numParsedNodes := 0

	alg := AdjacencyListGraph{}
	id2index := make(map[int]int)

	parseState := PARSE_NODE_COUNT
	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 {
			// skip empty lines
			continue
		} else if line[0] == '#' {
			// skip comments
			continue
		}

		switch parseState {
		case PARSE_NODE_COUNT:
			if val, err := strconv.Atoi(line); err == nil {
				numNodes = val
				parseState = PARSE_EDGE_COUNT
			}
		case PARSE_EDGE_COUNT:
			parseState = PARSE_NODES
		case PARSE_NODES:
			var id int
			var lat, lon float64
			fmt.Sscanf(line, "%d %f %f", &id, &lat, &lon)
			id2index[id] = alg.NodeCount()
			alg.AddNode(Node{Lon: lon, Lat: lat})
			numParsedNodes++
			if numParsedNodes == numNodes {
				parseState = PARSE_EDGES
			}
		case PARSE_EDGES:
			var from, to, distance int
			fmt.Sscanf(line, "%d %d %d", &from, &to, &distance)
			edge := NewEdge(id2index[to], id2index[from], distance)
			alg.AddEdge(*edge)
		}
	}

	if alg.NodeCount() != numNodes {
		// cannot check edge count because ocean.fmi contains duplicates, which are removed during import
		panic("Invalid parsing result")
	}

	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}

	return &alg
}

func NewAdjacencyListFromFmiFile(filename string) *AdjacencyListGraph {
	fmi, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return NewAdjacencyListFromFmiString(string(fmi))
}

func NewAdjacencyArrayFromFmiString(fmi string) *AdjacencyArrayGraph {
	alg := NewAdjacencyListFromFmiString(fmi)
	aag := NewAdjacencyArrayFromGraph(alg)
	return aag
}

func NewAdjacencyArrayFromFmiFile(filename string) *AdjacencyArrayGraph {
	fmi, err := os.ReadFile(filename)
	if err != nil {
		log.Fatal(err)
	}
	return NewAdjacencyArrayFromFmiString(string(fmi))
}
