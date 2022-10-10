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
3. Run `go build [-o <BINARY>] <PATH_TO_MAIN.GO>` to build a binary for a specified go file.

The main files are stored at `./cmd/<BINARY>`.

### Merge Coastlines

```bash
./merger
```

Extracts coastline segments from a `.pbf` file and merges them to closed coastlines.
The output (i.e. the list of polygons) is either written to the GeoJSON file or to a normal JSON file, which is less verbose than GeoJSON and which we call PolyJSON.

### Graph Builder

```bash
./graph-builder [-gridgraph [-grid-type TYPE] [-nTarget N] [-neighbors N]] [-contract graphFile [contraction-limit LIMIT] [-contraction-workers WORKERS] [-bidirectional] [-use-heuristic] [-use-cache] [-cold-start] [-max-settled-nodes] [-no-lazy-update] [-no-edge-difference] [-no-processed-neighbors] [-periodic] [-update-neighbors] [-dijkstra-debug] [-ch-debug]]
```

| Option                               | Value   | Information                                                                                                                          |
| ------------------------------------ | ------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| <nobr>-gridgraph</nobr>              | boolean | Build a gridgraph. See below for more information.                                                                                   |
| <nobr>-contract</nobr>               | string  | Contract the given graph. The file must be available at the `graphs` directory                                                       |
| <nobr>-grid-type</nobr>              | string  | (-gridgraph only) Define the type of the grid. Either "simple-sphere" or "equi-sphere" (default)                                     |
| <nobr>-nTarget</nobr>                | int     | (-gridgraph only) Define the density/number of targets for the grid. Typical values: 1e6 (equi-sphere, default), 710 (simple-sphere) |
| <nobr>-neighbors</nobr>              | int     | (-gridgraph only) Define the number of neighbors on the grid (only equi-sphere). Either 4 (default) or 6                             |
| <nobr>-contraction-limit</nobr>      | float   | (-contract only) limit the contraction up to a certain level (this specifies the percentage)                                         |
| <nobr>-contraction-workers</nobr>    | int     | (-contract only) specify how many workers can work in parallel on the contraction                                                    |
| <nobr>-bidirectional</nobr>          | boolean | (-contract only) Compute the contraction bidirectional                                                                               |
| <nobr>-use-heuristic</nobr>          | boolean | (-contract only) Use A\* search for contraction                                                                                      |
| <nobr>-use-cache</nobr>              | boolean | (-contract only) Cache the contraction results                                                                                       |
| <nobr>-cold-start</nobr>             | boolean | (-contract only) explicitely do a cold start (not hot start) when computing contraction                                              |
| <nobr>-max-settled-nodes</nobr>      | boolean | (-contract only) Set the number of max allowed settled nodes for each contraction                                                    |
| <nobr>-no-lazy-update</nobr>         | boolean | (-contract only) Disable lazy update for ch                                                                                          |
| <nobr>-no-edge-difference</nobr>     | boolean | (-contract only) Disable edge difference for ch                                                                                      |
| <nobr>-no-processed-neighbors</nobr> | boolean | (-contract only) Disable processed neighbors for ch                                                                                  |
| <nobr>-periodic</nobr>               | boolean | (-contract only) recompute contraction priority periodically for ch                                                                  |
| <nobr>-update-neighbors</nobr>       | boolean | (-contract only) update neighbors (priority) of contracted nodes for ch                                                              |
| <nobr>-dijkstra-debug</nobr>         | int     | (-contract only) Set the debug level for dijkstra                                                                                    |
| <nobr>-ch-debug</nobr>               | int     | (-contract only) Set the debug level for ch                                                                                          |

#### Build the basic grid graph

```bash
./graph-builder -gridgraph
```

Builds a spherical grid graph and implements the point-in-polygon test to check which grid points are in the ocean and will thus become nodes in the graph.
Two types of grids are supported:

##### Simple Grid

Distributes nodes equally along the latidue and longitude axis.

Available Parameters:

-   nTargets: The overall number of grid points will be $2*nTargets^2$. This is a density value

##### Equidistributed Grid

Distributes nodes equally on the planets surface.

Available Parameters:

-   nTarget: Number of points to distribute on the surface. The actual number of points may vary slightly.
-   meshType: Defines the maximum number of outgoing edges per node. One can choose between four and six neighbors and default value is four neighbors.

#### Output

The output is written to a file in the `fmi` format.

### Run Dijkstra Benchmarks

```bash
./benchmark [-random] [-n AMOUNT] [-store] [-search ALGORITHM  [-ch-stall-on-demand LEVEL] [-ch-heuristic] [-ch-manual] [-ch-sort-args]] [-cpu] [-graph FOLDER]
```

Runs a benchmark with the given parameters.

| Option                           | Value  | Information                                                                                                                                                                                                                                                                                                                                  |
| -------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <nobr>-random </nobr>            | bool   | If set, random targets will be created                                                                                                                                                                                                                                                                                                       |
| <nobr>-n </nobr>                 | int    | specify how many benchmarks are performed                                                                                                                                                                                                                                                                                                    |
| <nobr>-store </nobr>             | bool   | If set, the created benchmarks (targets) will be stored                                                                                                                                                                                                                                                                                      |
| <nobr>-search </nobr>            | string | Select the algorithm which performs the search. Available options are: `dijkstra` (common Dijkstra algorithm), `reference` (reference dijkstra with almost no configurability), `astar` (A\* search), `bidijkstra` (Bidirectional Dijkstra), `ch` (Contraction Hierarchies). The default is `dijkstra`                                       |
| <nobr>-graph </nobr>             | string | Specify the graph on which the benchmark is performed. This has to be a folder in the `graphs` directory. It should contain 4 files: `plain_graph.fmi` (the plain graph), `contracted_graph.fmi` (the contracted graph), `shortcuts.txt` (the shortcut list for the contraction), `node_ordering.txt` (the node ordering of the contraction) |
| <nobr>-cpu </nobr>               | bool   | if set, a cpu profile is created during the benchmarking                                                                                                                                                                                                                                                                                     |
| <nobr>-ch-stall-on-demand</nobr> | int    | (only if "-search ch" is used) Set the stall on demand level. 0 = no stalling, 1 = only stall current node, 2 = stall node preemtpive, 3 = stall current node and possible successors, 4 = stall node preemptive and possible successors                                                                                                     |
| <nobr>-ch-heuristic </nobr>      | bool   | (only if "-search ch" is used) use astar search in ch                                                                                                                                                                                                                                                                                        |
| <nobr>-ch-manual </nobr>         | bool   | (only if "-search ch" is used) Use manual (not bidirectional) search of dijkstra                                                                                                                                                                                                                                                             |
| <nobr>-ch-sort-arcs </nobr>      | bool   | (only if "-search ch" is used) Sort the arcs according if they are active or not for each node                                                                                                                                                                                                                                               |

### OSM-Server

#### Startup

```bash
./server [-graph graph] [-navigator algorithm]
```

Starts a HTTP server at port 8081.

| Option                  | Value  | Information                                                                                                                                                                                                                                                                                                 |
| ----------------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <nobr>-graph </nobr>    | string | Specify the used graph. This must be a directory in the `graphs` folder. It should contain 4 files: `plain_graph.fmi` (the plain graph), `contracted_graph.fmi` (the contracted graph), `shortcuts.txt` (the shortcut list for the contraction), `node_ordering.txt` (the node ordering of the contraction) |
| <nobr>-navigator</nobr> | string | Set the search algorithm. Available options are:Available options are: `dijkstra` (common dijkstra algorithm), `astar` (A\* search), `bidirectional-dijkstra` (Bidirectional Dijkstra), `contraction-hierarchies` (Contraction Hierarchies). The default is `contraction-hierarchies`                       |

#### Change search algorithm

The OSM-Server provides an endpoint at `/navigator` to change the used search algorithm.
The allowed algorithms are the same as in the startup.

A sample query could look like this:

```bash
curl -X POST -d '{"navigator": "dijkstra"}' http://localhost:8081/navigator
```

This sets the search algorithm to `dijkstra`. If the request is sucessful, a string with the updated algorithm is returned (in this case "dijkstra").

## Results

| Algorithm                  |  Runtime | Performance | Runtime (with path extraction) | Performance | PQ Pops | Performance |
| -------------------------- | -------: | ----------: | -----------------------------: | ----------: | ------: | ----------: |
| Dijkstra                   | 97.984ms |     100,00% |                       98.071ms |     100,00% |  335123 |     100,00% |
| Bidirectional Dijkstra     | 74.518ms |      76,05% |                       74.598ms |      76,07% |  237588 |      70,90% |
| A\*                        | 44.494ms |      45,41% |                       44.555ms |      45,43% |   96834 |      28,90% |
| Contraction Hierarchies    | 17.274ms |      17,63% |                       18.184ms |      18,54% |    3587 |       1,07% |
| Plain Dijkstra (Reference) | 68.848ms |      70,26% |                       69.040ms |      70,40% |  335123 |     100,00% |

The benchmark was run on a graph with 2806858 edges (and xxx active activated edges in the contraction)

### Plain Dijkstra

No special settings for plain Dijkstra.

Note: This algorithm is slower than the reference. Most likely, this happens due to several if conditions how this algorithm can get parameterized.

```bash
./benchmark -graph big_lazy_parallel -n 1000 -search dijkstra
```

| Details (for 1000 runs)                |          |
| -------------------------------------- | -------: |
| Average runtime                        | 97.984ms |
| Average runtime (with path extraction) | 98.071ms |
| Average PQ Pops                        |   335123 |
| Average PQ Updates                     |   404737 |
| Average Edge relaxations               |  1331144 |
| Average relaxation attempts            |  1331144 |

### Bidirectional Dijkstra

No special settigns for bidirectional Dijstra.

```bash
./benchmark -graph big_lazy_parallel -n 1000 -search bidirectional
```

| Details (for 1000 runs)                |          |
| -------------------------------------- | -------: |
| Average runtime                        | 74.518ms |
| Average runtime (with path extraction) | 74.598ms |
| Average PQ Pops                        |   237588 |
| Average PQ Updates                     |   291170 |
| Average Edge relaxations               |   944132 |
| Average relaxation attempts            |   944132 |

### AStar

No special settings for A\*.

```bash
./benchmark -graph big_lazy_parallel -n 1000 -search astar
```

| Details (for 1000 runs)                |          |
| -------------------------------------- | -------: |
| Average runtime                        | 44.494ms |
| Average runtime (with path extraction) | 44.555ms |
| Average PQ Pops                        |    96834 |
| Average PQ Updates                     |   150640 |
| Average Edge relaxations               |   384865 |
| Average relaxation attempts            |   384865 |

### Contraction Hierarchies

**These results were achieved with following settings:**

Graph Contraction: Lazy update, parallel processing (with independent set) - basically default parameters

Search: "preemptive" stall-on-demand, early termination (when best connection is found) - basically default parameters

```.bash
./benchmark -graph big_lazy_parallel -n 1000 -search ch
```

| Details (for 1000 runs)                |          |
| -------------------------------------- | -------: |
| Average runtime                        | 17.274ms |
| Average runtime (with path extraction) | 18.184ms |
| Average PQ Pops                        |     3587 |
| Average PQ Updates                     |    18928 |
| Average Edge relaxations               |    24415 |
| Average relaxation attempts            |   923624 |
| Average stalled nodes                  |    30011 |
| Average unstalled nodes                |     1974 |

_There may be different result with other settings or graphs, which may get reported later. E.g. using reursive stall-on-demand increases drastically the seach runtime._

### Plain Dijkstra (Reference)

This is just a simple Dijkstra algorithm, which is used as a reference.
It just used a priority queue to maintain its items.
No additional checks or other stuff is added here.

```bash
./benchmark -graph big_lazy_parallel -n 1000 -search reference
```

| Details (for 1000 runs)                |          |
| -------------------------------------- | -------: |
| Average runtime                        | 68.848ms |
| Average runtime (with path extraction) | 69.040ms |
| Average PQ Pops                        |   335123 |
| Average PQ Updates                     |   404737 |
| Average Edge relaxations               |  1331144 |
| Average relaxation attempts            |  1331144 |
