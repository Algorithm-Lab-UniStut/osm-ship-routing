package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"math/rand"
	"os"
	"os/signal"
	"path"
	"runtime"
	"runtime/pprof"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	p "github.com/natevvv/osm-ship-routing/pkg/graph/path"
	"github.com/natevvv/osm-ship-routing/pkg/slice"
)

func main() {
	useRandomTargets := flag.Bool("random", false, "Create (new) random targets")
	amountTargets := flag.Int("n", 100, "How many new targets should get created")
	storeTargets := flag.Bool("store", false, "Store targets (when newly generated)")
	algorithm := flag.String("search", "default", "Select the search algorithm")
	cpuProfile := flag.String("cpu", "", "write cpu profile to file")
	targetGraph := flag.String("graph", "big_lazy", "Select the graph to work with")
	// CH options
	stallOnDemand := flag.Int("ch-stall-on-demand", 2, "Set the stall on demand level")
	useHeuristic := flag.Bool("ch-heuristic", false, "use astar search in ch")
	manual := flag.Bool("ch-manual", false, "Use manual (not bidirectional) search of dijkstra")
	sortArcs := flag.Bool("ch-sort-arcs", false, "Sort the arcs according if they are active or not for each node")
	flag.Parse()

	if *useHeuristic {
		panic("AStar doesn't work yet for COntraction Hierarchies")
	}

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

	chPathFindingOptions := p.PathFindingOptions{Manual: *manual, UseHeuristic: *useHeuristic, StallOnDemand: *stallOnDemand, SortArcs: *sortArcs}

	start := time.Now()

	navigator, referenceDijkstra := getNavigator(*algorithm, graphDirectory, chPathFindingOptions)
	if navigator == nil {
		log.Fatal("Navigator not supported")
	}

	elapsed := time.Since(start)
	fmt.Printf("[TIME-Import] = %s\n", elapsed)

	targetFile := path.Join(graphDirectory, "targets.txt")
	var targets [][4]int
	if *useRandomTargets {
		targets = createTargets(*amountTargets, referenceDijkstra, targetFile)
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

func getNavigator(algorithm, graphDirectory string, chPathFindingOptions p.PathFindingOptions) (p.Navigator, *p.Dijkstra) {
	plainGraphFile := path.Join(graphDirectory, "plain_graph.fmi")
	contractedGraphFile := path.Join(graphDirectory, "contracted_graph.fmi")
	shortcutFile := path.Join(graphDirectory, "shortcuts.txt")
	nodeOrderingFile := path.Join(graphDirectory, "node_ordering.txt")

	var wg sync.WaitGroup
	wg.Add(1)
	var aag graph.Graph
	var referenceDijkstra *p.Dijkstra
	go func() {
		aag = graph.NewAdjacencyArrayFromFmiFile(plainGraphFile)
		referenceDijkstra = p.NewDijkstra(aag)
		wg.Done()
	}()

	targetNavigator := func() p.Navigator {
		if slice.Contains([]string{"default", "dijkstra"}, algorithm) {
			wg.Wait()
			d := p.NewUniversalDijkstra(aag)
			//d.SetHotStart(true)
			return d
		} else if algorithm == "reference" {
			wg.Wait()
			return p.NewDijkstra(aag)
		} else if algorithm == "astar" {
			wg.Wait()
			astar := p.NewUniversalDijkstra(aag)
			astar.SetUseHeuristic(true)
			return astar
		} else if algorithm == "bidirectional" {
			wg.Wait()
			bid := p.NewUniversalDijkstra(aag)
			//bid.SetHotStart(true)
			bid.SetBidirectional(true)
			return bid
		} else if algorithm == "ch" {
			var contractedGraph graph.Graph
			var shortcuts []p.Shortcut
			var nodeOrdering [][]graph.NodeId

			wg.Add(3)
			go func() {
				contractedGraph = graph.NewAdjacencyArrayFromFmiFile(contractedGraphFile)
				wg.Done()
			}()
			go func() {
				shortcuts = p.ReadShortcutFile(shortcutFile)
				wg.Done()
			}()
			go func() {
				nodeOrdering = p.ReadNodeOrderingFile(nodeOrderingFile)
				wg.Done()
			}()
			wg.Wait()

			dijkstra := p.NewUniversalDijkstra(contractedGraph)
			ch := p.NewContractionHierarchiesInitialized(contractedGraph, dijkstra, shortcuts, nodeOrdering, chPathFindingOptions)
			return ch
		}
		return nil
	}()

	return targetNavigator, referenceDijkstra
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

func createTargets(n int, referenceNavigator *p.Dijkstra, filename string) [][4]int {
	// targets: origin, destination, length, #hops (nodes from source to target)
	targets := make([][4]int, n)
	seed := rand.NewSource(time.Now().UnixNano())
	//seed := rand.NewSource(0)
	rng := rand.New(seed)
	// reference algorithm to compute path
	for i := 0; i < n; i++ {
		origin := rng.Intn(referenceNavigator.GetGraph().NodeCount())
		destination := rng.Intn(referenceNavigator.GetGraph().NodeCount())
		length := referenceNavigator.ComputeShortestPath(origin, destination)
		hops := len(referenceNavigator.GetPath(origin, destination))
		targets[i] = [4]int{origin, destination, length, hops}
	}
	return targets
}

func writeTargets(targets [][4]int, targetFile string) {
	var sb strings.Builder
	for _, target := range targets {
		sb.WriteString(fmt.Sprintf("%v %v %v %v\n", target[0], target[1], target[2], target[3]))
	}

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
	completed := 0

	pqPops := 0
	pqUpdates := 0
	stalledNodes := 0
	unstalledNodes := 0
	edgeRelaxations := 0
	relaxationAttempts := 0

	invalidLengths := make([][3]int, 0)
	invalidResults := make([]int, 0)
	invalidHops := make([][3]int, 0)

	// get the active arcs from the graph
	activeArcsCount := func(g graph.Graph) int {
		counter := 0
		for i := range g.GetNodes() {
			arcs := g.GetArcsFrom(i)
			for j := range arcs {
				arc := &arcs[j]
				if arc.ArcFlag() {
					counter++
				}
			}
		}
		return counter
	}

	activeArcs := activeArcsCount(navigator.GetGraph())
	activeArcsReference := activeArcsCount(referenceDijkstra.GetGraph())

	if activeArcsReference != referenceDijkstra.GetGraph().ArcCount() {
		panic("Active reference arcs are weird")
	}

	showResults := func() {
		fmt.Printf("Average runtime: %.3fms, %.3fms\n", float64(int(runtime.Nanoseconds())/completed)/1000000, float64(int(runtimeWithPathExtraction.Nanoseconds())/completed)/1000000)
		fmt.Printf("Average pq pops: %d\n", pqPops/completed)
		fmt.Printf("Average pq updates: %d\n", pqUpdates/completed)
		fmt.Printf("Average stalled nodes: %d\n", stalledNodes/completed)
		fmt.Printf("Average unstalled nodes: %d\n", unstalledNodes/completed)
		fmt.Printf("Average relaxations attempts: %d\n", relaxationAttempts/completed)
		fmt.Printf("Average edge relaxations: %d\n", edgeRelaxations/completed)
		fmt.Printf("Active arcs: %v, Active arcs (reference): %v\n", activeArcs, activeArcsReference)

		fmt.Printf("%v/%v invalid Result (source/target).\n", len(invalidResults), completed)
		for i, result := range invalidResults {
			fmt.Printf("%v: Case %v (%v -> %v) has invalid result\n", i, result, targets[result][0], targets[result][1])
		}

		fmt.Printf("%v/%v invalid path lengths.\n", len(invalidLengths), completed)
		for i, lengths := range invalidLengths {
			testcase := lengths[0]
			actualLength := lengths[1]
			referenceLength := lengths[2]
			fmt.Printf("%v: Case %v (%v -> %v) has invalid length. Has: %v, Reference: %v, Difference: %v\n", i, testcase, targets[testcase][0], targets[testcase][1], actualLength, referenceLength, actualLength-referenceLength)
		}

		fmt.Printf("%v/%v invalid hops number.\n", len(invalidHops), completed)
		for i, hops := range invalidHops {
			testcase := hops[0]
			actualHops := hops[1]
			referenceHops := hops[2]
			fmt.Printf("%v: Case %v (%v -> %v) has invalid #hops. Has: %v, reference: %v, difference: %v\n", i, testcase, targets[testcase][0], targets[testcase][1], actualHops, referenceHops, actualHops-referenceHops)
		}
	}

	// catch interrupt to still show already calculated results
	c := make(chan os.Signal)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-c
		showResults()
		os.Exit(0)
	}()

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
		stalledNodes += navigator.GetStalledNodesCount()
		unstalledNodes += navigator.GetUnstalledNodesCount()
		edgeRelaxations += navigator.GetEdgeRelaxations()
		relaxationAttempts += navigator.GetRelaxationAttempts()

		path := navigator.GetPath(origin, destination)
		elapsedPath := time.Since(start)

		fmt.Printf("[%3v TIME-Navigate, TIME-Path, PQ Pops, PQ Updates, relaxed Edges, relax attempts] = %12s, %12s, %7d, %7d, %7d, %7d\n", i, elapsed, elapsedPath, navigator.GetPqPops(), navigator.GetPqUpdates(), navigator.GetEdgeRelaxations(), navigator.GetRelaxationAttempts())

		if length != referenceLength {
			invalidLengths = append(invalidLengths, [3]int{i, length, referenceLength})
		}
		if length > -1 && (path[0] != origin || path[len(path)-1] != destination) {
			invalidResults = append(invalidResults, i)
		}
		if referenceHops != len(path) {
			invalidHops = append(invalidHops, [3]int{i, len(path), referenceHops})
			referenceDijkstra.ComputeShortestPath(origin, destination)
		}

		runtime += elapsed
		runtimeWithPathExtraction += elapsedPath
		completed++
	}
	// normal termination, show results
	showResults()
}
