package main

import (
	"fmt"
	"math"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
)

const graphFile = "graphs/ocean_equi_4.fmi"

func main() {
	aag := graph.NewAdjacencyArrayFromFmiFile(graphFile)
	distanceInformation(aag)
}

func distanceInformation(g graph.Graph) {
	distance := 0
	biggestDistance := 0
	bigDifferenceCounter := 0
	bigDifferences := make([]int, 0)
	totalArcLength := 0
	arcCounter := 0
	for id := 0; id < g.NodeCount(); id++ {
		g.GetNode(id)
		arcs := g.GetArcsFrom(id)
		for _, arc := range arcs {
			totalArcLength = totalArcLength + arc.Cost()
			arcCounter++
			if totalArcLength < 0 {
				fmt.Printf("Arc length overflow")
			}
			if distance == 0 {
				// set initial distance
				distance = arc.Cost()
				fmt.Printf("Init arc distance: %v\n", distance)
			}
			if math.Abs(float64(distance-arc.Cost())) > float64(0.5)*float64(distance) {
				bigDifferenceCounter++
				bigDifferences = append(bigDifferences, int(math.Abs(float64(distance-arc.Cost()))))
				//fmt.Printf("Node %v has arc with bigger distance\n", id)
			}
			if arc.Cost() > biggestDistance {
				biggestDistance = arc.Cost()
			}
		}
	}
	fmt.Printf("Arcs with bigger distance: %v\n", bigDifferenceCounter)
	total := 0
	for _, dif := range bigDifferences {
		total += dif
	}
	if len(bigDifferences) > 0 {
		average := total / len(bigDifferences)
		fmt.Printf("Average Difference: %v", average)
	}

	fmt.Printf("Average arc length: %v", totalArcLength/arcCounter)
}
