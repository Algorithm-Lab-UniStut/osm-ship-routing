package main

import (
	"bufio"
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
	start := time.Now()
	aag := graph.NewAdjacencyArrayFromFmi(graphFile)
	elapsed := time.Since(start)
	fmt.Printf("[TIME-Import] = %s\n", elapsed)

	benchmark(aag, 100)
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

func createTargets(n int, aag *graph.AdjacencyArrayGraph, filename string) {
	var sb strings.Builder
	for i := 0; i < n; i++ {
		origin := rand.Intn(aag.NodeCount())
		destination := rand.Intn(aag.NodeCount())
		sb.WriteString(fmt.Sprintf("%v %v\n", origin, destination))
	}
	fmt.Printf("Targets:\n%s", sb.String())
	file, cErr := os.Create(targetFile)

	if cErr != nil {
		log.Fatal(cErr)
	}

	targets := sb.String()
	writer := bufio.NewWriter(file)
	writer.WriteString(targets)
	writer.Flush()
}

// Run benchmarks on the provided graphs: Compute n random routes
func benchmark(aag *graph.AdjacencyArrayGraph, n int) {

	createFile := false // set to true to recreate the target file
	if createFile {
		createTargets(n, aag, targetFile)
	}

	targets := readTargets(targetFile)

	runtime := 0
	for i := 0; i < n && i < len(targets); i++ {
		origin := targets[i][0]
		destination := targets[i][1]

		navigator := path.GetNavigator(aag)

		start := time.Now()
		path, length := navigator.GetPath(origin, destination)
		elapsed := time.Since(start)
		fmt.Printf("[TIME-Navigate] = %s\n", elapsed)

		if length > -1 {
			if path[0] != origin || path[len(path)-1] != destination {
				panic("Invalid routing result")
			}
		}

		runtime += int(elapsed)
	}
	fmt.Printf("Average runtime: %.3fms\n", float64(runtime/n)/1000000)
}
