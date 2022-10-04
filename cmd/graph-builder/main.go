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

func main() {
	buildGridGraph := flag.String("gridgraph", "", "Build grid graph")
	contractGraph := flag.String("contract", "", "Contract the given graph")

	// Grid graph options
	gridType := flag.String("grid-type", "equi-sphere", "Define the type of the grid")
	nTargets := flag.Int("nTarget", 1e6, "Define the density/number of targets for the grid. Typical values: 1e6 (equi-sphere), 710 (simple-sphere)")
	neighbors := flag.Int("neighbors", 4, "Define the number of neighbors on the grid (only equi-sphere)")

	// CH contraction options
	contractionLimit := flag.Float64("contraction-limit", 100, "Limit the level of contractions")
	contractionWorkers := flag.Int("contraction-workers", 6, "Set the number of contraction workers")
	bidirectional := flag.Bool("bidirectional", false, "Compute the contraction bidirectional")
	useHeuristic := flag.Bool("use-heuristic", false, "Use A* search for contraction")
	coldStart := flag.Bool("cold-start", false, "explicitely do a cold start (not hot start) when computing contraction")
	maxNumSettledNodes := flag.Int("max-settled-nodes", math.MaxInt, "Set the number of max allowed settled nodes for each contraction")
	// CH order options
	noLazyUpdate := flag.Bool("no-lazy-update", false, "Disable lazy update for ch")
	noEdgeDifference := flag.Bool("no-edge-difference", false, "Disable edge difference for ch")
	noProcessedNeighbors := flag.Bool("no-processed-neighbors", false, "Disable processed neighbors for ch")
	periodic := flag.Bool("periodic", false, "recompute contraction priority periodically for ch")
	updateNeighbors := flag.Bool("update-neighbors", false, "update neighbors (priority) of contracted nodes for ch")
	flag.Parse()

	if *buildGridGraph != "" {
		// possible graphFile: "antarctica.poly.json", "planet-coastlines.poly.json"
		graphFile := *buildGridGraph
		createGridGraph(graphFile, *gridType, *nTargets, *neighbors)
	}

	if *contractGraph != "" {
		_, filename, _, ok := runtime.Caller(0)
		if !ok {
			log.Fatal("Error")
		}

		directory := path.Dir(filename)
		graphFile := path.Join(directory, "..", "..", "graphs", *contractGraph)

		oo := p.MakeOrderOptions().SetLazyUpdate(!*noLazyUpdate).SetEdgeDifference(!*noEdgeDifference).SetProcessedNeighbors(!*noProcessedNeighbors).SetPeriodic(*periodic).SetUpdateNeighbors(*updateNeighbors)
		options := p.ContractionOptions{Bidirectional: *bidirectional, UseHeuristic: *useHeuristic, HotStart: !*coldStart, MaxNumSettledNodes: *maxNumSettledNodes, ContractionLimit: *contractionLimit, ContractionWorkers: *contractionWorkers}

		createContractedGraph(graphFile, oo, options)
	}
}

func createGridGraph(graphFile, gridType string, nTargets int, meshType int) {
	arg := loadPolyJsonPolygons(graphFile)

	gridGraph := func() graph.Graph {
		if gridType == "equi-sphere" {
			return grid.NewEquiSphereGrid(nTargets, meshType, arg).ToGraph()
		} else if gridType == "simple-sphere" {
			return grid.NewSimpleSphereGrid(2*nTargets, nTargets, arg).ToGraph()
		}
		panic("grid-type not supported")
	}()

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

func createContractedGraph(graphFile string, oo p.OrderOptions, options p.ContractionOptions) {
	log.Printf("Read graph file\n")
	alg := graph.NewAdjacencyListFromFmiFile(graphFile)
	dijkstra := p.NewUniversalDijkstra(alg)
	log.Printf("Initialize Contraction Hierarchies\n")
	ch := p.NewContractionHierarchies(alg, dijkstra, options)
	ch.SetDebugLevel(1)
	ch.SetPrecomputationMilestones([]float64{0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 99.99})
	log.Printf("Initialized Contraction Hierarchies, start precomputation\n")
	ch.Precompute(nil, oo)
	ch.WriteContractionResult()
	log.Printf("Finished Contraction\n")
}
