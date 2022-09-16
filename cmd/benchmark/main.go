package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"math/rand"
	"os"
	"path"
	"runtime"
	"runtime/pprof"
	"strings"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	p "github.com/natevvv/osm-ship-routing/pkg/graph/path"
)

func main() {
	useRandomTargets := flag.Bool("random", false, "Create (new) random targets")
	amountTargets := flag.Int("n", 100, "How many new targets should get created")
	storeTargets := flag.Bool("store", false, "Store targets (when newly generated)")
	algorithm := flag.String("search", "default", "Select the search algorithm")
	cpuProfile := flag.String("cpu", "", "write cpu profile to file")
	targetGraph := flag.String("graph", "big_lazy", "Select the graph to work with")
	flag.Parse()

	_, filename, _, ok := runtime.Caller(0)
	if !ok {
		log.Fatal("Error")
	}

	directory := path.Dir(filename)
	graphDirectory := path.Join(directory, "..", "..", "graphs", *targetGraph)

	if _, err := os.Stat(graphDirectory); os.IsNotExist(err) {
		log.Fatal("Graph directory does not exist")
	} else {
		fmt.Printf("Using graph directory: %v\n", graphDirectory)
	}

	start := time.Now()

	navigator, referenceDijkstra, aag := getNavigator(*algorithm, graphDirectory)
	if navigator == nil {
		log.Fatal("Navigator not supported")
	}

	elapsed := time.Since(start)
	fmt.Printf("[TIME-Import] = %s\n", elapsed)

	targetFile := path.Join(graphDirectory, "targets.txt")
	var targets [][4]int
	if *useRandomTargets {
		targets = createTargets(*amountTargets, aag, targetFile)
		if *storeTargets {
			writeTargets(targets, targetFile)
		}
	} else {
		targets = readTargets(targetFile)
		if *amountTargets < len(targets) {
			targets = targets[0:*amountTargets]
		}
	}

	if *cpuProfile != "" {
		f, err := os.Create(*cpuProfile)
		if err != nil {
			log.Fatal(err)
		}
		pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}
	benchmark(navigator, targets, referenceDijkstra)
}

func getNavigator(algorithm, graphDirectory string) (p.Navigator, *p.Dijkstra, *graph.AdjacencyArrayGraph) {
	plainGraphFile := path.Join(graphDirectory, "plain_graph.fmi")
	contractedGraphFile := path.Join(graphDirectory, "contracted_graph.fmi")
	shortcutFile := path.Join(graphDirectory, "shortcuts.txt")
	nodeOrderingFile := path.Join(graphDirectory, "node_ordering.txt")

	aag := graph.NewAdjacencyArrayFromFmiFile(plainGraphFile)
	referenceDijkstra := p.NewDijkstra(aag)

	if algorithm == "default" {
		return p.GetNavigator(aag), referenceDijkstra, aag
	} else if algorithm == "dijkstra" {
		return p.NewUniversalDijkstra(aag), referenceDijkstra, aag
	} else if algorithm == "reference_dijkstra" {
		return p.NewDijkstra(aag), referenceDijkstra, aag
	} else if algorithm == "astar" {
		astar := p.NewUniversalDijkstra(aag)
		astar.SetUseHeuristic(true)
		return astar, referenceDijkstra, aag
	} else if algorithm == "bidijkstra" {
		bid := p.NewUniversalDijkstra(aag)
		bid.SetBidirectional(true)
		return bid, referenceDijkstra, aag
	} else if algorithm == "ch" {
		contracted_aag := graph.NewAdjacencyArrayFromFmiFile(contractedGraphFile)
		shortcuts := p.ReadShortcutFile(shortcutFile)
		nodeOrdering := p.ReadNodeOrderingFile(nodeOrderingFile)

		dijkstra := p.NewUniversalDijkstra(contracted_aag)
		dijkstra.SetStallOnDemand(true)

		ch := p.NewContractionHierarchiesInitialized(contracted_aag, dijkstra, shortcuts, nodeOrdering, false)
		return ch, referenceDijkstra, aag
	} else {
		return nil, referenceDijkstra, aag
	}
}

func readTargets(filename string) [][4]int {
	file, err := os.Open(filename)
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	scanner.Split(bufio.ScanLines)

	targets := make([][4]int, 0)

	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 {
			// skip empty lines
			continue
		} else if line[0] == '#' {
			// skip comments
			continue
		}
		var origin, destination, length, hops int
		fmt.Sscanf(line, "%d %d %d %d", &origin, &destination, &length, &hops)
		target := [4]int{origin, destination, length, hops}
		targets = append(targets, target)
	}
	return targets
}

func createTargets(n int, aag *graph.AdjacencyArrayGraph, filename string) [][4]int {
	// targets: origin, destination, length, #hops (nodes from source to target)
	targets := make([][4]int, n)
	seed := rand.NewSource(time.Now().UnixNano())
	rng := rand.New(seed)
	// reference algorithm to compute path
	dijkstra := p.NewDijkstra(aag)
	for i := 0; i < n; i++ {
		origin := rng.Intn(aag.NodeCount())
		destination := rng.Intn(aag.NodeCount())
		length := dijkstra.ComputeShortestPath(origin, destination)
		hops := len(dijkstra.GetPath(origin, destination))
		targets[i] = [4]int{origin, destination, length, hops}
	}
	return targets
}

func writeTargets(targets [][4]int, targetFile string) {
	var sb strings.Builder
	for _, target := range targets {
		sb.WriteString(fmt.Sprintf("%v %v %v %v\n", target[0], target[1], target[2], target[3]))
	}

	//fmt.Printf("Targets:\n%s", sb.String())
	file, cErr := os.Create(targetFile)

	if cErr != nil {
		log.Fatal(cErr)
	}

	writer := bufio.NewWriter(file)
	writer.WriteString(sb.String())
	writer.Flush()
}

// Run benchmarks on the provided graphs and targets
func benchmark(navigator p.Navigator, targets [][4]int, referenceDijkstra *p.Dijkstra) {
	var runtime time.Duration = 0
	var runtimeWithPathExtraction time.Duration = 0

	pqPops := 0
	pqUpdates := 0
	edgeRelaxations := 0
	relaxationAttempts := 0

	invalidLengths := make([][2]int, 0)
	invalidResults := make([]int, 0)
	invalidHops := make([][3]int, 0)

	for i, target := range targets {
		origin := target[0]
		destination := target[1]
		referenceLength := target[2]
		referenceHops := target[3]

		start := time.Now()
		length := navigator.ComputeShortestPath(origin, destination)
		elapsed := time.Since(start)

		pqPops += navigator.GetPqPops()
		pqUpdates += navigator.GetPqUpdates()
		edgeRelaxations += navigator.GetEdgeRelaxations()
		relaxationAttempts += navigator.GetRelaxationAttempts()

		path := navigator.GetPath(origin, destination)
		elapsedPath := time.Since(start)

		fmt.Printf("[%3v TIME-Navigate, TIME-Path, PQ Pops, PQ Updates, relaxed Edges, relax attepmts] = %12s, %12s, %7d, %7d, %7d, %7d\n", i, elapsed, elapsedPath, navigator.GetPqPops(), navigator.GetPqUpdates(), navigator.GetEdgeRelaxations(), navigator.GetRelaxationAttempts())

		if length != referenceLength {
			invalidLengths = append(invalidLengths, [2]int{i, length - referenceLength})
		}
		if length > -1 && (path[0] != origin || path[len(path)-1] != destination) {
			invalidResults = append(invalidResults, i)
		}
		if referenceHops != len(path) {
			invalidHops = append(invalidHops, [3]int{i, len(path), referenceHops})
			fmt.Printf("%v: Hops: %v\n", i, path)
			referenceDijkstra.ComputeShortestPath(origin, destination)
			rp := referenceDijkstra.GetPath(origin, destination)
			fmt.Printf("Reference: %v\n", rp)
		}

		runtime += elapsed
		runtimeWithPathExtraction += elapsedPath
	}

	fmt.Printf("Average runtime: %.3fms, %.3fms\n", float64(int(runtime.Nanoseconds())/len(targets))/1000000, float64(int(runtimeWithPathExtraction.Nanoseconds())/len(targets))/1000000)
	fmt.Printf("Average pq pops: %d\n", pqPops/len(targets))
	fmt.Printf("Average pq updates: %d\n", pqUpdates/len(targets))
	fmt.Printf("Average relaxations attempts: %d\n", relaxationAttempts/len(targets))
	fmt.Printf("Average edge relaxations: %d\n", edgeRelaxations/len(targets))
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
