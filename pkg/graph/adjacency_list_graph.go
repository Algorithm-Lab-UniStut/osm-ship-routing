package graph

import (
	"fmt"
	"strings"

	geo "github.com/natevvv/osm-ship-routing/pkg/geometry"
)

// Implementation for dynamic graphs
type AdjacencyListGraph struct {
	Nodes    []geo.Point // The nodes of the graph
	Edges    [][]Arc     // The Arcs of the graph. The first slice specifies to which the arc belongs
	arcCount int         // the number of arcs in the graph
}

// Return the node for the given id
func (alg *AdjacencyListGraph) GetNode(id NodeId) *geo.Point {
	if id < 0 || id >= alg.NodeCount() {
		panic(id)
	}
	return &alg.Nodes[id]
}

// Return all nodes of the graph
func (alg *AdjacencyListGraph) GetNodes() []geo.Point {
	return alg.Nodes
}

// Get the arcs for the given node
func (alg *AdjacencyListGraph) GetArcsFrom(id NodeId) []Arc {
	if id < 0 || id >= alg.NodeCount() {
		panic(id)
	}
	return alg.Edges[id]
}

func (alg *AdjacencyListGraph) SortArcs() {
	for i := 0; i < alg.NodeCount(); i++ {
		for j := 0; j < len(alg.Edges[i]); j++ {
			if !alg.Edges[i][j].ArcFlag() {
				for k := j + 1; k < len(alg.Edges[i]); k++ {
					if alg.Edges[i][k].ArcFlag() {
						// swap arcs
						alg.Edges[i][j], alg.Edges[i][k] = alg.Edges[i][k], alg.Edges[i][j]
						break
					}
				}
			}
		}
	}
}

// Return the number of total nodes
func (alg *AdjacencyListGraph) NodeCount() int {
	return len(alg.Nodes)
}

// Return the numebr of total arcs
func (alg *AdjacencyListGraph) ArcCount() int {
	return alg.arcCount
}

// Return a human readable string of the graph
func (alg *AdjacencyListGraph) AsString() string {
	var sb strings.Builder

	// write number of nodes and number of edges
	sb.WriteString(fmt.Sprintf("%v\n", alg.NodeCount()))
	sb.WriteString(fmt.Sprintf("%v\n", alg.ArcCount()))

	sb.WriteString("#Nodes\n")
	// list all nodes structured as "id lat lon"
	for i := 0; i < alg.NodeCount(); i++ {
		node := alg.GetNode(i)
		sb.WriteString(fmt.Sprintf("%v %v %v\n", i, node.Lat(), node.Lon()))
	}

	sb.WriteString("#Edges\n")
	// list all edges structured as "fromId targetId distance"
	for i := 0; i < alg.NodeCount(); i++ {
		for _, arc := range alg.GetArcsFrom(i) {
			sb.WriteString(fmt.Sprintf("%v %v %v\n", i, arc.Destination(), arc.Cost()))
		}
	}
	return sb.String()
}

// Add a node to the graph
func (alg *AdjacencyListGraph) AddNode(n geo.Point) {
	alg.Nodes = append(alg.Nodes, n)
	alg.Edges = append(alg.Edges, make([]Arc, 0))
}

// Add an arc to the graph, going from source to target with the given distance
func (alg *AdjacencyListGraph) AddArc(from, to NodeId, distance int) bool {
	if from >= alg.NodeCount() || to >= alg.NodeCount() {
		panic(fmt.Sprintf("Arc out of range %v -> %v", from, to))
	}
	// Check for duplicates
	arcs := alg.Edges[from]
	for i := range arcs {
		arc := &arcs[i]
		if to == arc.To {
			// Update edge if a better one is found
			// caller is resposible to correctly store the new edge/shortcut (remember which is the connection node)
			if distance < arc.Distance {
				// update distance
				arc.Distance = distance
				return true
			} else {
				return false
			}
		}

	}
	alg.Edges[from] = append(alg.Edges[from], MakeArc(to, distance, true))
	alg.arcCount++
	return true
}

// Set the arc flags for all arcs of the given node
func (alg *AdjacencyListGraph) SetArcFlags(id NodeId, flag bool) {
	// set the arc flags for the outgoing edges
	arcs := alg.GetArcsFrom(id)
	for i := range arcs {
		arcs[i].SetArcFlag(flag)
	}
}
