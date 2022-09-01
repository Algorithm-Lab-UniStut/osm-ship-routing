package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/geometry"
	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/graph/path"
	"github.com/natevvv/osm-ship-routing/pkg/grid"
)

const density = 710 // parameter for SimpleSphereGrid
const nTarget = 1e6 // parameter for EquiSphereGrid

func main() {
	buildGridGraph := flag.Bool("gridgraph", false, "Build grid graph")
	buildContractedGraph := flag.Bool("contract", false, "Build grid graph")
	flag.Parse()

	if *buildGridGraph {
		createGridGraph()
	}
	if *buildContractedGraph {
		createContractedGraph()
	}
}

func createGridGraph() {
	//arg := loadPolyJsonPolygons("antarctica.poly.json")
	arg := loadPolyJsonPolygons("planet-coastlines.poly.json")

	//grid := grid.NewSimpleSphereGrid(2*density, density, arg)
	grid := grid.NewEquiSphereGrid(nTarget, grid.SIX_NEIGHBORS, arg)

	gridGraph := grid.ToGraph()
	jsonObj, err := json.Marshal(gridGraph)
	if err != nil {
		panic(err)
	}

	wErr := os.WriteFile("graph.json", jsonObj, 0644)
	if wErr != nil {
		panic(err)
	}

	graph.WriteFmi(gridGraph, "graph.fmi")
}

func loadPolyJsonPolygons(file string) []geometry.Polygon {

	start := time.Now()
	bytes, err := os.ReadFile(file)
	if err != nil {
		panic(err)
	}
	elapsed := time.Since(start)
	fmt.Printf("[TIME] Read file: %s\n", elapsed)

	start = time.Now()
	var polygons []geometry.Polygon
	err = json.Unmarshal(bytes, &polygons)
	if err != nil {
		fmt.Println(err)
	}
	elapsed = time.Since(start)
	fmt.Printf("[TIME] Unmarshal: %s\n", elapsed)

	return polygons
}

func createContractedGraph() {
	fmt.Printf("Read graph file\n")
	alg := graph.NewAdjacencyListFromFmiFile("ocean_10k.fmi")
	alg = graph.NewAdjacencyListFromFmiFile("ocean_equi_4.fmi")
	dijkstra := path.NewUniversalDijkstra(alg)
	fmt.Printf("Contract Graph\n")
	ch := path.NewContractionHierarchies(alg, dijkstra)
	ch.SetDebugLevel(0)
	ch.SetPrecomputationMilestones([]float64{0, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 99.99})
	fmt.Printf("Initialized Contraction Hierarchies\n")
	ch.Precompute(nil, path.MakeOrderOptions().SetDynamic(true).SetEdgeDifference(true).SetProcessedNeighbors(true).SetPeriodic(false))
	ch.WriteContractionResult()
}
