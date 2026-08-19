#include "solver.h"

#define HUNGARIAN_INF 32767
#define HUNGARIAN_N   4

static int32 cost_mat[HUNGARIAN_N][HUNGARIAN_N];
static int32 u[HUNGARIAN_N + 1];
static int32 v[HUNGARIAN_N + 1];
static int32 p[HUNGARIAN_N + 1];
static int32 way[HUNGARIAN_N + 1];
static int32 minv[HUNGARIAN_N + 1];
static int8 used[HUNGARIAN_N + 1];

int32 hungarian(int32 cost[][HUNGARIAN_N], int n, int8 assignment[])
{
    if(n <= 0) return 0;
    if(n > HUNGARIAN_N) return HUNGARIAN_INF;

    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            cost_mat[i][j] = cost[i][j];
        }
    }

    for(int i = 0; i <= n; i++) {
        u[i] = 0;
        v[i] = 0;
        p[i] = 0;
        way[i] = 0;
    }

    for(int i = 1; i <= n; i++) {
        int j0 = 0;
        p[0] = i;

        for(int j = 0; j <= n; j++) {
            minv[j] = HUNGARIAN_INF;
            used[j] = 0;
        }

        while(1) {
            int i0;
            int32 delta = HUNGARIAN_INF;
            int j1 = 0;

            used[j0] = 1;
            i0 = p[j0];

            for(int j = 1; j <= n; j++) {
                if(!used[j]) {
                    int32 cur = cost_mat[i0 - 1][j - 1] - u[i0] - v[j];
                    if(cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if(minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for(int j = 0; j <= n; j++) {
                if(used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
            if(p[j0] == 0) break;
        }

        while(1) {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if(j0 == 0) break;
        }
    }

    for(int i = 0; i < n; i++) assignment[i] = -1;
    for(int j = 1; j <= n; j++) {
        if(p[j] != 0) assignment[p[j] - 1] = (int8)(j - 1);
    }

    {
        int32 total = 0;
        for(int i = 0; i < n; i++) {
            if(assignment[i] >= 0)
                total += cost[i][assignment[i]];
            else
                return HUNGARIAN_INF;
        }
        return total;
    }
}
