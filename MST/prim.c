#include "prim.h"

#include <stdlib.h>
#include <limits.h>

/**************************************************************************
 * Graph
 **************************************************************************/

typedef struct Edge {
    int dst;
    int weight;
    struct Edge* next;
} Edge;

struct PrimGraph {
    int V;
    Edge** adj;
};

/**************************************************************************
 * Min Heap
 **************************************************************************/

typedef struct {
    int vertex;
    int key;

} HeapNode;

typedef struct {
    int size;
    int capacity;
    int* pos;
    HeapNode* nodes;
} MinHeap;

/**************************************************************************
 * Heap Internal
 **************************************************************************/

static void
swapNode(
    HeapNode* a,
    HeapNode* b)
{
    HeapNode tmp = *a;
    *a = *b;
    *b = tmp;
}

static void
heapify(
    MinHeap* heap,
    int idx)
{
    int smallest = idx;

    int left  = idx * 2 + 1;
    int right = idx * 2 + 2;

    if (left < heap->size &&
        heap->nodes[left].key <
        heap->nodes[smallest].key)
    {
        smallest = left;
    }

    if (right < heap->size &&
        heap->nodes[right].key <
        heap->nodes[smallest].key)
    {
        smallest = right;
    }

    if (smallest != idx)
    {
        heap->pos[
            heap->nodes[idx].vertex] =
            smallest;

        heap->pos[
            heap->nodes[smallest].vertex] =
            idx;

        swapNode(
            &heap->nodes[idx],
            &heap->nodes[smallest]);

        heapify(
            heap,
            smallest);
    }
}

static int
extractMin(
    MinHeap* heap)
{
    HeapNode root =
        heap->nodes[0];

    HeapNode last =
        heap->nodes[
            heap->size - 1];

    heap->nodes[0] = last;

    heap->pos[last.vertex] = 0;

    heap->size--;

    if (heap->size > 0)
    {
        heapify(heap, 0);
    }

    return root.vertex;
}

static void
decreaseKey(
    MinHeap* heap,
    int vertex,
    int key)
{
    int i =
        heap->pos[vertex];

    heap->nodes[i].key =
        key;

    while (i > 0)
    {
        int parent =
            (i - 1) / 2;

        if (heap->nodes[parent].key <=
            heap->nodes[i].key)
        {
            break;
        }

        heap->pos[
            heap->nodes[parent].vertex] =
            i;

        heap->pos[
            heap->nodes[i].vertex] =
            parent;

        swapNode(
            &heap->nodes[parent],
            &heap->nodes[i]);

        i = parent;
    }
}

/**************************************************************************
 * Public API
 **************************************************************************/

PrimGraph*
PrimGraphCreate(
    int vertices)
{
    if (vertices <= 0)
    {
        return NULL;
    }

    PrimGraph* g =
        calloc(
            1,
            sizeof(*g));

    if (!g)
    {
        return NULL;
    }

    g->V = vertices;

    g->adj =
        calloc(
            vertices,
            sizeof(Edge*));

    if (!g->adj)
    {
        free(g);
        return NULL;
    }

    return g;
}

void
PrimGraphDestroy(
    PrimGraph* graph)
{
    if (!graph) {
        return;
    }

    for (int i = 0; i < graph->V; ++i) {
        Edge* e = graph->adj[i];

        while (e) {
            Edge* next = e->next;
            free(e);
            e = next;
        }
    }

    free(graph->adj);
    free(graph);
}

int
PrimGraphAddEdge(
    PrimGraph* graph,
    int src,
    int dst,
    int weight)
{
    if (!graph) {
        return -1;
    }

    if (src < 0 || dst < 0 || src >= graph->V || dst >= graph->V) {
        return -1;
    }

    Edge* e1 = (Edge*)malloc(sizeof(*e1));

    if (!e1) {
        return -1;
    }

    e1->dst = dst;
    e1->weight = weight;
    e1->next = graph->adj[src];

    graph->adj[src] = e1;

    Edge* e2 = (Edge*)malloc(sizeof(*e2));

    if (!e2) {
        return -1;
    }

    e2->dst = src;
    e2->weight = weight;
    e2->next = graph->adj[dst];

    graph->adj[dst] = e2;

    return 0;
}

int
PrimMST(
    PrimGraph* graph,
    int startVertex,
    PrimEdge* mstEdges,
    int maxEdges,
    int* totalCost)
{
    if (!graph || !mstEdges || !totalCost) {
        return -1;
    }

    int V = graph->V;

    if (maxEdges < (V - 1)) {
        return -1;
    }

    int* parent = (int*)malloc(sizeof(int) * V);

    int* key = (int*)malloc(sizeof(int) * V);

    MinHeap heap;

    heap.size = V;
    heap.capacity = V;

    heap.pos = (int*)malloc(sizeof(int) * V);

    heap.nodes = (HeapNode*)malloc(sizeof(HeapNode) * V);

    if (!parent || !key || !heap.pos || !heap.nodes) {
        free(parent);
        free(key);
        free(heap.pos);
        free(heap.nodes);

        return -1;
    }

    for (int i = 0; i < V; ++i) {
        parent[i] = -1;
        key[i] = INT_MAX;

        heap.nodes[i].vertex = i;
        heap.nodes[i].key = INT_MAX;
        heap.pos[i] = i;
    }

    key[startVertex] = 0;

    heap.nodes[startVertex].key = 0;

    decreaseKey(&heap, startVertex, 0);

    while (heap.size > 0) {
        int u = extractMin(&heap);

        Edge* edge = graph->adj[u];

        while (edge) {
            int v = edge->dst;

            if (heap.pos[v] < heap.size && edge->weight < key[v]) {
                key[v] = edge->weight;
                parent[v] = u;
                decreaseKey(&heap, v, key[v]);
            }

            edge = edge->next;
        }
    }

    *totalCost = 0;

    int edgeCount = 0;

    for (int v = 0; v < V; ++v) {
        if (parent[v] == -1) {
            continue;
        }

        mstEdges[edgeCount].src = parent[v];
        mstEdges[edgeCount].dst = v;
        mstEdges[edgeCount].weight = key[v];
        *totalCost += key[v];
        edgeCount++;
    }

    free(parent);
    free(key);
    free(heap.pos);
    free(heap.nodes);

    return edgeCount;
}