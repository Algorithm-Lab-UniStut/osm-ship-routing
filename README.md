# OSM-Ship-Routing

Using is Docker is the fastest way to get the backend of the OSM-Ship-Routing service running.
Beside that, an installation from source gives you access to every component of this project including:

-   OSM-Server (backend of the OSM-Ship-Routing service) Coastline Merger
-   Dijkstra Benchmarks
-   Grid Graph Builder
-   Coastline Merger

## Setup Using Docker

**Currently outdated, since only building for Mac was possible. I will hopefully fix this later.**

1. Pull the image from [Dockerhub](https://hub.docker.com/repository/docker/natevvv/osm-ship-routing): `docker pull natevvv/osm-ship-routing:<TAG>`
2. Start a container: `docker run -p 8081:8081 --name osm-server natevvv/osm-ship-routing`

Note that `<TAG>` needs to be replaced by a valid tag. Please find all available tags on [Dockerhub](https://hub.docker.com/repository/docker/natevvv/osm-ship-routing).
Tag `1.0.0` refers to the first submission and tag `latest` referst to the most recent release on Dockerhub.

## Installation from Source

### Prerequisites

The `osm-ship-routing` service is written in [Go](https://go.dev/).
Installing and running requires an installation of the Go Programming Language `v1.18` or later.

It is also assumed that there is a `graphs` folder at the root directory of this repository. In this folder, there are several graph descriptions (each in a sub-folder).

You can download these graphs here: https://drive.google.com/drive/folders/1ISubYd9KAYZ1SSYUvSAoVLiBKKoSrde9?usp=sharing

Extract the graphs and move it to the `graphs` directory.

If you want to contract a graph, you need an plain fmi-graph directly in the `graphs` directory. (more infos later)

### Setup

1. Clone the repository
2. Run `go mod download`
3. Run `go build -o <BINARY> <PATH_TO_MAIN.GO>` to build a binary for a specified go file.

### Merge Coastlines

```bash
go run cmd/merger/main.go
```

Extracts coastline segments from a `.pbf` file and merges them to closed coastlines.
The output (i.e. the list of polygons) is either written to the GeoJSON file or to a normal JSON file, which is less verbose than GeoJSON and which we call PolyJSON.

### Graph Builder

```bash
go run cmd/graph-builder/main.go [-gridgraph] [-contract graphFile] [contraction-limit limit] [-contraction-workers workers]
```

| Option               | Value   | Information                                                                    |
| -------------------- | ------- | ------------------------------------------------------------------------------ |
| -gridgraph           | boolean | Build a gridgraph. See below for more information.                             |
| -contract            | string  | Contract the given graph. The file must be available at the `graphs` directory |
| -contraction-limit   | float   | limit the contraction up to a certain level (this specifies the percentage)    |
| -contraction-workers | int     | specify how many workers can work in parallel on the contraction               |

#### Build the basic grid graph

```bash
go run cmd/graph-builder/main.go -gridgraph
```

Builds a spherical grid graph and implements the point-in-polygon test to check which grid points are in the ocean and will thus become nodes in the graph.
Two types of grids are supported:

##### Simple Grid

Distributes nodes equally along the latidue and longitude axis.

Available Parameters:

-   density: The overall number of grid points will be $2*density^2$.

##### Equidistributed Grid

Distributes nodes equally on the planets surface.

Available Parameters:

-   nTarget: Number of points to distribute on the surface. The actual number of points may vary slightly.
-   meshType: Defines the maximum number of outgoing edges per node. One can choose between four and six neighbors and default value is four neighbors.

#### Output

The output is written to a file in the `fmi` format.

### Run Dijkstra Benchmarks

```bash
go run cmd/benchmark/main.go [-random] [-n amount] [-store] [-search algorithm] [-cpu] [-graph]
```

Runs a benchmark with the given parameters.

| Option  | Value  | Information                                                                                                                                                                                                                                                                                                                                  |
| ------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| -random | bool   | If set, random targets will be created                                                                                                                                                                                                                                                                                                       |
| -n      | int    | specify how many benchmarks are performed                                                                                                                                                                                                                                                                                                    |
| -store  | bool   | If set, the created benchmarks (targets) will be stored                                                                                                                                                                                                                                                                                      |
| -search | string | Select the algorithm which performs the search. Available options are: `dijkstra` (common dijsktra algorithm), `reference_dijkstra` (reference dijkstra with almost no configurability), `astar` (A\* search), `bidijkstra` (Bidirectional Dijkstra), `ch` (Contraction Hierarchies). The default is `dijkstra`                              |
| -cpu    | bool   | if set, a cpu profile is created during the benchmarking                                                                                                                                                                                                                                                                                     |
| -graph  | string | Specify the graph on which the benchmark is performed. This has to be a folder in the `graphs` directory. It should contain 4 files: `plain_graph.fmi` (the plain graph), `contracted_graph.fmi` (the contracted graph), `shortcuts.txt` (the shortcut list for the contraction), `node_ordering.txt` (the node ordering of the contraction) |

Note: The are also other options, which can be controlled in the code.

### OSM-Server

#### Startup

```bash
go run cmd/server/main.go [-graph graph] [-navigator algorithm]
```

Starts a HTTP server at port 8081.

| Option     | Value  | Information                                                                                                                                                                                                                                                                                                 |
| ---------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| -graph     | string | Specify the used graph. This must be a directory in the `graphs` folder. It should contain 4 files: `plain_graph.fmi` (the plain graph), `contracted_graph.fmi` (the contracted graph), `shortcuts.txt` (the shortcut list for the contraction), `node_ordering.txt` (the node ordering of the contraction) |
| -navigator | string | Set the search algorithm. Available options are:Available options are: `dijkstra` (common dijsktra algorithm), `astar` (A\* search), `bidirectional-dijkstra` (Bidirectional Dijkstra), `contraction-hierarchies` (Contraction Hierarchies). The default is `contraction-hierarchies`                       |

#### Change search algorithm

The OSM-Server provides an endpoint at `/navigator` to change the used search algorithm.
The allowed algorithms are the same as in the startup.

A sample query could look like this:

```bash
curl -X POST -d '{"navigator": "dijkstra"}' http://localhost:8081/navigator
```

This sets the search algorithm to `dijkstra`. If the request is sucessful, a string with the updated algorithm is returned (in this case "dijkstra").

## Results

| Algorithm               | Runtime   | Performance | Runtime (with path extraction) | Performance | PQ Pops | Performance |
| ----------------------- | --------- | ----------- | ------------------------------ | ----------- | ------- | ----------- |
| Reference Dijsktra      | 109.037ms | 100%        | 109.385ms                      | 100%        | 351427  | 100%        |
| Dijkstra                | 123.508ms | 113,55%     | 123.603ms                      | 113%        | 351427  | 100%        |
| Bidirectional Dijkstra  | 103.076ms | 94,53%      | 103.167ms                      | 94,31%      | 250515  | 71,29%      |
| A\*                     | 55.665ms  | 51,05%      | 55.729ms                       | 50,95%      | 101073  | 28,76%      |
| Contraction Hierarchies | 49.948ms  | 45,80%      | 51.483ms                       | 47,07%      | 3638    | 1,04%       |

### Plain Dijsktra

No special settings for plain Dijkstra.

Note: This algoirthm is slower than the reference. Most likely, this happens due to several if conditions how this algorithm can get parameterized.

### Bidirectional Dijsktra

No special settigns for bidirectional Dijstra.

### AStar

No special settings for A\*.

### Contraction Hierarchies

**These results were achieved with following settings:**

Graph: Lazy update, parallel processing (with independent set)

Stall on demand: "preemptive"

Search: Bidirectional Dijkstra, stop when best connection is known

_There may be different result with other settings or graphs, which may get reported later. E.g. using reursive stall-on-demand increases drastically the seach runtime._
