package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"math/rand"
	"os"
	"strings"
	"time"

	"github.com/natevvv/osm-ship-routing/pkg/graph"
	"github.com/natevvv/osm-ship-routing/pkg/graph/path"
)

const graphFile = "graphs/ocean_equi_4.fmi"
const targetFile = "cmd/benchmark/targets.txt"

func main() {
	useRandomTargets := flag.Bool("random", false, "Create (new) random targets")
	amountTargets := flag.Int("n", 100, "How many new targets should get created")
	storeTargets := flag.Bool("store", false, "Store targets (when newly generated)")
	algorithm := flag.String("search", "default", "Select the search algorithm")
	flag.Parse()

	start := time.Now()
	aag := graph.NewAdjacencyArrayFromFmiFile(graphFile)
	elapsed := time.Since(start)
	fmt.Printf("[TIME-Import] = %s\n", elapsed)

	var targets [][2]int
	if *useRandomTargets {
		targets = createTargets(*amountTargets, aag, targetFile)
		if *storeTargets {
			writeTargets(targets, targetFile)
		}
	} else {
		targets = readTargets(targetFile)
	}

	var navigator path.Navigator
	if *algorithm == "default" {
		navigator = path.GetNavigator(aag)
	} else if *algorithm == "dijkstra" {
		navigator = path.NewUniversalDijkstra(aag)
	} else if *algorithm == "astar" {
		astar := path.NewUniversalDijkstra(aag)
		astar.SetUseHeuristic(true)
		navigator = astar
	} else if *algorithm == "bidijkstra" {
		bid := path.NewUniversalDijkstra(aag)
		bid.SetBidirectional(true)
		navigator = bid
	} else {
		panic("Navigator not supported")
	}

	benchmark(navigator, targets)
}

func readTargets(filename string) [][2]int {
	file, err := os.Open(filename)
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	scanner.Split(bufio.ScanLines)

	targets := make([][2]int, 0)

	for scanner.Scan() {
		line := scanner.Text()
		if len(line) < 1 {
			// skip empty lines
			continue
		} else if line[0] == '#' {
			// skip comments
			continue
		}
		var origin, destination int
		fmt.Sscanf(line, "%d %d", &origin, &destination)
		target := [2]int{origin, destination}
		targets = append(targets, target)
	}
	return targets
}

func createTargets(n int, aag *graph.AdjacencyArrayGraph, filename string) [][2]int {
	targets := make([][2]int, n)
	seed := rand.NewSource(time.Now().UnixNano())
	rng := rand.New(seed)
	for i := 0; i < n; i++ {
		origin := rng.Intn(aag.NodeCount())
		destination := rng.Intn(aag.NodeCount())
		targets[i] = [2]int{origin, destination}
	}
	return targets
}

func writeTargets(targets [][2]int, targetFile string) {
	var sb strings.Builder
	for _, target := range targets {
		sb.WriteString(fmt.Sprintf("%v %v\n", target[0], target[1]))
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
func benchmark(navigator path.Navigator, targets [][2]int) {

	runtime := 0
	for i, target := range targets {
		origin := target[0]
		destination := target[1]

		start := time.Now()
		length := navigator.ComputeShortestPath(origin, destination)
		path := navigator.GetPath(origin, destination)
		elapsed := time.Since(start)
		fmt.Printf("[%3v TIME-Navigate] = %s\n", i, elapsed)

		if length > -1 {
			if path[0] != origin || path[len(path)-1] != destination {
				panic("Invalid routing result")
			}
		}

		runtime += int(elapsed)
	}
	fmt.Printf("Average runtime: %.3fms\n", float64(runtime/len(targets))/1000000)
}
