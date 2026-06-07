#ifndef PRIM_H
#define PRIM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PrimGraph PrimGraph;

typedef struct {
    int src;
    int dst;
    int weight;
} PrimEdge;

/*
 * Create graph with vertex count.
 */
PrimGraph*
PrimGraphCreate(int vertices);

/*
 * Destroy graph.
 */
void
PrimGraphDestroy(
	PrimGraph* graph);

/*
 * Add undirected weighted edge.
 *
 * Return:
 *   0  success
 *  -1  error
 */
int
PrimGraphAddEdge(
    PrimGraph* graph,
    int src,
    int dst,
    int weight);

/*
 * Run Prim MST.
 *
 * Parameters:
 *   graph        graph object
 *   startVertex  root vertex
 *   mstEdges     output array
 *   maxEdges     array size
 *   totalCost    output cost
 *
 * Return:
 *   number of MST edges
 *   -1 on error
 */
int
PrimMST(
    PrimGraph* graph,
    int startVertex,
    PrimEdge* mstEdges,
    int maxEdges,
    int* totalCost);

#ifdef __cplusplus
}
#endif

#endif