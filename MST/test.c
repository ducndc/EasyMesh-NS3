#include <stdio.h>
#include "prim.h"

int main(void)
{
    PrimGraph* g = PrimGraphCreate(5);

    PrimGraphAddEdge(g,0,1,2);
    PrimGraphAddEdge(g,0,3,6);
    PrimGraphAddEdge(g,1,2,3);
    PrimGraphAddEdge(g,1,3,8);
    PrimGraphAddEdge(g,1,4,5);
    PrimGraphAddEdge(g,2,4,7);
    PrimGraphAddEdge(g,3,4,9);

    PrimEdge mst[4];

    int cost;

    int n = PrimMST(g, 0, mst, 4, &cost);

    for (int i = 0; i < n; ++i) {
        printf("%d -> %d (%d)\n", mst[i].src, mst[i].dst, mst[i].weight);
    }

    printf("Total Cost = %d\n", cost);

    PrimGraphDestroy(g);

    return 0;
}