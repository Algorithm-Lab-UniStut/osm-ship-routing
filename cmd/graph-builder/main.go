package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"path"
	"runtime"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/geometry"
	"github.com/natevvv/osm-ship-routing/pkg/graph"
	p "github.com/natevvv/osm-ship-routing/pkg/graph/path"
	"github.com/natevvv/osm-ship-routing/pkg/grid"
)

const density = 710 // parameter for SimpleSphereGrid
const nTarget = 1e6 // parameter for EquiSphereGrid

func main() {
	buildGridGraph := flag.Bool("gridgraph", false, "Build grid graph")
	contractGraph := flag.String("contract", "", "Contract the given graph")
	contractionLimit := flag.Float64("contraction-limit", 100, "Limit the level of contractions")
	contractionWorkers := flag.Int("contraction-workers", 6, "Set the number of contraction workers")
	bidirectional := flag.Bool("bidirectional", false, "Compute the contraction bidirectional")
	useHeuristic := flag.Bool("use-heuristic", false, "Use A* search for contraction")
	coldStart := flag.Bool("cold-start", false, "explicitely do a cold start (not hot start) when computing contraction")
	maxNumSettledNodes := flag.Int("max-settled-nodes", math.MaxInt, "Set the number of max allowed settled nodes for each contraction")
	flag.Parse()

	if *buildGridGraph {
		createGridGraph()
	}

	if *contractGraph != "" {
		_, filename, _, ok := runtime.Caller(0)
		if !ok {
			log.Fatal("Error")
		}

		directory := path.Dir(filename)
		graphFile := path.Join(directory, "..", "..", "graphs", *contractGraph)

		options := p.ContractionOptions{Bidirectional: *bidirectional, UseHeuristic: *useHeuristic, HotStart: !*coldStart, MaxNumSettledNodes: *maxNumSettledNodes, ContractionLimit: *contractionLimit, ContractionWorkers: *contractionWorkers}

		createContractedGraph(graphFile, options)
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

func createContractedGraph(graphFile string, options p.ContractionOptions) {
	log.Printf("Read graph file\n")
	alg := graph.NewAdjacencyListFromFmiFile(graphFile)
	dijkstra := p.NewUniversalDijkstra(alg)
	log.Printf("Initialize Contraction Hierarchies\n")
	ch := p.NewContractionHierarchies(alg, dijkstra, options)
	ch.SetDebugLevel(2)
	ch.SetPrecomputationMilestones([]float64{0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 99.99})
	log.Printf("Initialized Contraction Hierarchies, start precomputation\n")
	ch.Precompute(nil, p.MakeOrderOptions().SetLazyUpdate(true).SetEdgeDifference(true).SetProcessedNeighbors(true).SetPeriodic(false).SetUpdateNeighbors(false))
	ch.WriteContractionResult()
	log.Printf("Finished Contraction\n")
}
