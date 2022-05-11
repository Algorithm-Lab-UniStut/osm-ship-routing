package grid

import (
	"fmt"
	"math"
	"sync"
	"time"

	geo "github.com/dmholtz/osm-ship-routing/pkg/geometry"
	gr "github.com/dmholtz/osm-ship-routing/pkg/graph"
)

type EquiSphereGrid struct {
	nTarget    int // approximate number of points in in the grid
	nCount     int // actual number of points in the grid
	points     [][]geo.Point
	isWater    []bool
	grid2nodes map[IndexTupel]int
	nodes2grid []IndexTupel
	nodes      []gr.Node
	edges      []gr.Edge
}

type IndexTupel struct {
	LatRow int
	lonCol int
}

func NewEquiSphereGrid(n int, coastlines []geo.Polygon) *EquiSphereGrid {
	if n < 2 {
		panic(n)
	}
	esg := EquiSphereGrid{nTarget: n}

	start := time.Now()
	esg.distributePoints()
	elapsed := time.Since(start)
	fmt.Printf("[TIME] Distribute Points on grid: %s\n", elapsed)

	start = time.Now()
	esg.landWaterTest(coastlines)
	elapsed = time.Since(start)
	fmt.Printf("[TIME] Land / Water test: %s\n", elapsed)

	start = time.Now()
	esg.createNodes()
	elapsed = time.Since(start)
	fmt.Printf("[TIME] Create Nodes: %s\n", elapsed)

	start = time.Now()
	esg.createEdges()
	elapsed = time.Since(start)
	fmt.Printf("[TIME] Create Edges: %s\n", elapsed)

	return &esg
}

func (esg *EquiSphereGrid) distributePoints() {
	esg.nCount = 0
	esg.points = make([][]geo.Point, 0)

	a := 4.0 * math.Pi / float64(esg.nTarget)
	d := math.Sqrt(a)
	mTheta := math.Round(math.Pi / d)
	dTheta := math.Pi / mTheta
	dPhi := a / dTheta
	for m := 0; m < int(mTheta); m++ {
		esg.points = append(esg.points, make([]geo.Point, 0))
		theta := math.Pi * (float64(m) + 0.5) / mTheta
		mPhi := math.Round(2.0 * math.Pi * math.Sin(theta) / dPhi)
		for n := 0; n < int(mPhi); n++ {
			phi := 2 * math.Pi * float64(n) / mPhi
			lat := geo.Rad2Deg(-theta + math.Pi/2)
			lon := geo.Rad2Deg(phi - math.Pi)
			esg.points[m] = append(esg.points[m], geo.Point{lat, lon})
			esg.nCount++
		}
	}
	fmt.Printf("Number of points: %d\n", esg.nCount)
}

func (esg *EquiSphereGrid) landWaterTest(polygons []geo.Polygon) {
	esg.isWater = make([]bool, esg.nCount, esg.nCount)

	// pre-compute bounding boxes for every polygon
	bboxes := make([]geo.BoundingBox, len(polygons), len(polygons))
	var wg sync.WaitGroup
	wg.Add(len(polygons))
	for i, polygon := range polygons {
		go func(i int, polygon geo.Polygon) {
			bboxes[i] = polygon.BoundingBox()
			wg.Done()
		}(i, polygon)
	}
	wg.Wait()

	wg.Add(esg.nCount)
	idx := 0
	for _, ring := range esg.points {
		for _, point := range ring {
			go func(idx int, point geo.Point) {
				esg.isWater[idx] = true
				for i, polygon := range polygons {
					// roughly check, whether the point is contained in the bounding box of the polygon
					if bboxes[i].Contains(point) {
						// precisely check, whether the polygon contains the point
						if polygon.Contains(&point) {
							esg.isWater[idx] = false
							break
						}
					}
				}
				wg.Done()
			}(idx, point)
			idx++
		}
	}
	wg.Wait()
}

func (esg *EquiSphereGrid) createNodes() {
	esg.grid2nodes = make(map[IndexTupel]int)
	esg.nodes2grid = make([]IndexTupel, 0)
	esg.nodes = make([]gr.Node, 0)
	cellId := 0
	for latRow, ring := range esg.points {
		for lonCol, point := range ring {
			if esg.isWater[cellId] {
				indexTuple := IndexTupel{LatRow: latRow, lonCol: lonCol}
				esg.grid2nodes[indexTuple] = len(esg.nodes)
				esg.nodes = append(esg.nodes, *gr.NewNode(point.Lon(), point.Lat()))
				esg.nodes2grid = append(esg.nodes2grid, indexTuple)
			}
			cellId++
		}

	}
}

func (esg *EquiSphereGrid) createEdges() {
	for nodeId := range esg.nodes {
		indexTuple := esg.nodes2grid[nodeId]
		neighborIndexTuples := esg.neighborsOf(indexTuple)
		for _, neighborIndexTuple := range neighborIndexTuples {
			if neighborNodeId, ok := esg.grid2nodes[neighborIndexTuple]; ok {
				edge := gr.Edge{From: nodeId, To: neighborNodeId, Distance: 1} // todo: compute distance
				esg.edges = append(esg.edges, edge)
				esg.edges = append(esg.edges, edge.Invert())
			}
		}
	}
}

func (esg *EquiSphereGrid) neighborsOf(cell IndexTupel) []IndexTupel {
	neighbors := make([]IndexTupel, 0)
	// northern neighbor
	if cell.LatRow > 0 {
		lonColNumeric := float64(cell.lonCol*len(esg.points[cell.LatRow-1])) / float64(len(esg.points[cell.LatRow]))
		lonCol := int(math.Round(lonColNumeric)) % len(esg.points[cell.LatRow-1])
		neighbors = append(neighbors, IndexTupel{LatRow: cell.LatRow - 1, lonCol: lonCol})
	}

	// western neighbor
	lonCol := (cell.lonCol + 1) % len(esg.points[cell.LatRow])
	neighbors = append(neighbors, IndexTupel{LatRow: cell.LatRow, lonCol: lonCol})

	return neighbors
}

func (esg *EquiSphereGrid) ToGraph() gr.Graph {
	alg := &gr.AdjacencyListGraph{}
	for _, node := range esg.nodes {
		alg.AddNode(node)
	}
	for _, edge := range esg.edges {
		alg.AddEdge(edge)
	}
	return alg
}
