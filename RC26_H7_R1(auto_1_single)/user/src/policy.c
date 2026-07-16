#include "policy.h"

/* ================================================================
   Grid: 3x4, cells numbered 1..12
        col1 col2 col3
   row4  10   11   12   (y=4)
   row3   7    8    9   (y=3)
   row2   4    5    6   (y=2)
   row1   1    2    3   (y=1)   entry=2
   exits = {10, 12}
   ================================================================ */

int kfs1[3];
int kfs2[4];
int kf;
int path[20];                // 存储路径序列（格子编号）
int path_len;                // 路径长度
int picked_k2[4];            // 被拾取的K2位置
int picked_cnt;              // 拾取的K2数量
int removed_k1;              // 被移除的K1位置
int removed_k1_2;            // 移除2个K1时的第二个位置
int removed_cnt;             // 实际移除了几个K1（1或2）
int target_k2;               // 目标拾取K2数量

/* ---- coordinates ---- */
static int cell_x(int n) { return (n - 1) % GRID_W + 1; }
static int cell_y(int n) { return (n - 1) / GRID_W + 1; }
static int cell_num(int x, int y) { return (y - 1) * GRID_W + x; }

/* ---- problem data ---- */
static int g_k1[MAX_K1];   //
static int g_k2[MAX_K2];
static int g_kf;
static int g_blocked[13];

/* ---- BFS queue (static, no heap) ---- */
typedef struct {
    int8_t  x, y;
    uint8_t mask;    /* bit i set = K2[i] picked */
} State;

static State   bfs_queue[MAX_STATES];
static int16_t bfs_parent[MAX_STATES];
static uint8_t bfs_visited[GRID_W + 2][GRID_H + 2][1 << MAX_K2];
static int     bfs_head, bfs_tail;
static const int8_t dx[4] = {-1, 1, 0, 0};
static const int8_t dy[4] = { 0, 0,-1, 1};
static int s_bottom_k2_mask; /* 底部行(1/2/3号格)K2的bitmask，bfs_run内赋值 */

/* ---- output ---- */
static int g_path[20];
static int g_path_len;
static int g_picked_k2[4];
static int g_picked_cnt;

/* ================================================================
   init
   ================================================================ */
void policy_init(const int k1[3], const int k2[4], int kf)   //
{
    memcpy(g_k1, k1, sizeof(g_k1));
    memcpy(g_k2, k2, sizeof(g_k2));
    g_kf = kf;
}

/* ================================================================
   popcount fallback for platforms without __builtin_popcount
   ================================================================ */
static int popcnt(uint8_t x)
{
#ifdef __GNUC__
    return __builtin_popcount(x);
#else
    x = x - ((x >> 1) & 0x55);
    x = (x & 0x33) + ((x >> 2) & 0x33);
    return (x + (x >> 4)) & 0x0F;
#endif
}

/* ================================================================
   bfs: shortest path for a given K1 removal set + target K2 count
   removed_mask: bit i=1 means g_k1[i] is removed
   target_k2: how many K2 to pick (1..4)
   returns true if path found, result in g_path / g_picked_k2
   ================================================================ */
static bool bfs_run(int removed_mask, int target_k2)
{
    memset(bfs_visited, 0, sizeof(bfs_visited));

    /* blocked set: not-removed K1 + Kf */
    memset(g_blocked, 0, sizeof(g_blocked));
    for (int i = 0; i < MAX_K1; i++) {
        if (!(removed_mask & (1 << i)))
            g_blocked[g_k1[i]] = 1;
    }
    g_blocked[g_kf] = 1;

    /* K2 index lookup: 0 = not K2, 1..4 = index+1 */
    int k2_idx[13] = {0};
    for (int i = 0; i < MAX_K2; i++)
        k2_idx[g_k2[i]] = i + 1;

    /* 底部约束：K2在{1,2,3}时必须至少捡一个才能出口 */
    s_bottom_k2_mask = 0;
    for (int i = 0; i < MAX_K2; i++) {
        int cell = g_k2[i];
        if (cell == 1 || cell == 2 || cell == 3)
            s_bottom_k2_mask |= (1 << i);
    }

    /* init BFS */
    bfs_head = bfs_tail = 0;
    bfs_queue[0].x = -1; bfs_queue[0].y = -1; bfs_queue[0].mask = 0;
    bfs_parent[0] = -1;
    bfs_visited[0][0][0] = 1;  /* (-1,-1) -> visited[0][0] */
    bfs_tail = 1;

    int entry_x = cell_x(ENTRY), entry_y = cell_y(ENTRY);

    while (bfs_head < bfs_tail) {
        State s = bfs_queue[bfs_head];
        int8_t x = s.x, y = s.y;
        uint8_t mask = s.mask;
        int pc = popcnt(mask);

        /* ---- outside map: must enter ---- */
        if (x == -1) {
            int nx = entry_x, ny = entry_y;
            int n = cell_num(nx, ny);
            if (g_blocked[n]) { bfs_head++; continue; }

            uint8_t nm = mask;
            int ki = k2_idx[n];
            if (ki && !(mask & (1 << (ki - 1)))) {
                nm |= (1 << (ki - 1));
                if (popcnt(nm) > target_k2) { bfs_head++; continue; }
            }
            if (!bfs_visited[nx][ny][nm]) {
                bfs_visited[nx][ny][nm] = 1;
                bfs_queue[bfs_tail] = (State){nx, ny, nm};
                bfs_parent[bfs_tail] = bfs_head;
                bfs_tail++;
            }
            bfs_head++;
            continue;
        }

        /* ---- check exit ---- */
        int cn = cell_num(x, y);
        if ((cn == EXIT1 || cn == EXIT2) && pc == target_k2) {
            /* 底部约束：K2在{1,2,3}时必须至少捡一个才能出口 */
            if (s_bottom_k2_mask == 0 || (mask & s_bottom_k2_mask)) {
                g_path_len = 0;
                g_picked_cnt = 0;
                uint8_t fmask = mask;
                int idx = bfs_head;
                while (idx >= 0) {
                    if (bfs_queue[idx].x != -1)
                        g_path[g_path_len++] = cell_num(bfs_queue[idx].x, bfs_queue[idx].y);
                    idx = bfs_parent[idx];
                }
                for (int i = 0; i < g_path_len / 2; i++) {
                    int t = g_path[i];
                    g_path[i] = g_path[g_path_len - 1 - i];
                    g_path[g_path_len - 1 - i] = t;
                }
                for (int i = 0; i < MAX_K2; i++)
                    if (fmask & (1 << i)) g_picked_k2[g_picked_cnt++] = g_k2[i];
                return true;
            }
        }

        /* ---- find adjacent unpicked K2 ---- */
        int adj[4], adj_n = 0;
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 1 || nx > GRID_W || ny < 1 || ny > GRID_H) continue;
            int n = cell_num(nx, ny);
            int ki = k2_idx[n];
            if (ki && !(mask & (1 << (ki - 1))))
                adj[adj_n++] = ki - 1;
        }

        /* ---- try all subsets of adjacent K2 ---- */
        for (int sub = 0; sub < (1 << adj_n); sub++) {
            int bits = 0, new_pc = pc;
            for (int i = 0; i < adj_n; i++) {
                if (sub & (1 << i)) { bits |= (1 << adj[i]); new_pc++; }
            }
            if (new_pc > target_k2) continue;
            uint8_t nm = mask | bits;

            if (bits == 0) {
                /* move */
                for (int d = 0; d < 4; d++) {
                    int nx = x + dx[d], ny = y + dy[d];
                    if (nx < 1 || nx > GRID_W || ny < 1 || ny > GRID_H) continue;
                    int n = cell_num(nx, ny);
                    if (g_blocked[n]) continue;
                    int ki = k2_idx[n];
                    if (ki && !(nm & (1 << (ki - 1)))) continue;
                    if (!bfs_visited[nx][ny][nm]) {
                        bfs_visited[nx][ny][nm] = 1;
                        bfs_queue[bfs_tail] = (State){nx, ny, nm};
                        bfs_parent[bfs_tail] = bfs_head;
                        bfs_tail++;
                    }
                }
            } else {
                /* pick K2, stay */
                if (!bfs_visited[x][y][nm]) {
                    bfs_visited[x][y][nm] = 1;
                    bfs_queue[bfs_tail] = (State){x, y, nm};
                    bfs_parent[bfs_tail] = bfs_head;
                    bfs_tail++;
                }
            }
        }
        bfs_head++;
    }
    return false;
}

/* ================================================================
   compact_path: remove consecutive duplicate cells from path
   (caused by picking K2 without moving)
   ================================================================ */
static int compact_path(int *path, int len)
{
    if (len <= 1) return len;
    int w = 1;
    for (int r = 1; r < len; r++) {
        if (path[r] != path[r - 1])
            path[w++] = path[r];
    }
    return w;
}

/* ================================================================
   solve_n: remove remove_n K1s, pick target_k2, find shortest
   ================================================================ */
bool policy_solve_n(int target_k2, int remove_n,
                    int *out_path, int *out_path_len,
                    int *out_picked, int *out_picked_cnt,
                    int *out_removed_k1, int *out_removed_k2)
{
    int best_len = 999;
    bool found = false;

    *out_removed_k2 = 0;

    for (int mask = 1; mask < (1 << MAX_K1); mask++) {
        int pcnt = popcnt((uint8_t)mask);
        if (pcnt != remove_n) continue;

        if (bfs_run(mask, target_k2) && g_path_len < best_len) {
            best_len = g_path_len;
            found = true;
            *out_path_len = g_path_len;
            memcpy(out_path, g_path, g_path_len * sizeof(int));
            *out_picked_cnt = g_picked_cnt;
            memcpy(out_picked, g_picked_k2, g_picked_cnt * sizeof(int));
            *out_path_len = compact_path(out_path, *out_path_len);

            int cnt = 0;
            for (int i = 0; i < MAX_K1; i++) {
                if (mask & (1 << i)) {
                    if (cnt == 0) *out_removed_k1 = g_k1[i];
                    else          *out_removed_k2 = g_k1[i];
                    cnt++;
                }
            }
        }
    }
    return found;
}

/* ================================================================
   solve_best: 4-level priority cascade
     Step 1: 3 K2 + 1 K1, <=5 steps
     Step 2: 3 K2 + 2 K1
     Step 3: 2 K2 + 2 K1
     Step 4: 2 K2 + 1 K1
   ================================================================ */
bool policy_solve_best(int *out_path, int *out_path_len,
                       int *out_picked, int *out_picked_cnt,
                       int *out_removed_k1, int *out_target_k2)
{
    int path[20], pl, pk[4], pc, rk1, rk2;
    const int max_steps = 5;

    /* Step 1: 3 K2 + 1 K1, <=5 steps */
    if (policy_solve_n(3, 1, path, &pl, pk, &pc, &rk1, &rk2) && (pl - 1) <= max_steps) {
        *out_target_k2 = 3; removed_cnt = 1;
        goto output;
    }

    /* Step 2: 3 K2 + 2 K1 */
    if (policy_solve_n(3, 2, path, &pl, pk, &pc, &rk1, &rk2)) {
        *out_target_k2 = 3; removed_cnt = 2;
        goto output;
    }

    /* Step 3: 2 K2 + 2 K1 */
    if (policy_solve_n(2, 2, path, &pl, pk, &pc, &rk1, &rk2)) {
        *out_target_k2 = 2; removed_cnt = 2;
        goto output;
    }

    /* Step 4: 2 K2 + 1 K1 */
    if (policy_solve_n(2, 1, path, &pl, pk, &pc, &rk1, &rk2)) {
        *out_target_k2 = 2; removed_cnt = 1;
        goto output;
    }

    return false;

output:
    *out_path_len = pl;
    memcpy(out_path, path, pl * sizeof(int));
    *out_picked_cnt = pc;
    memcpy(out_picked, pk, pc * sizeof(int));
    *out_removed_k1 = rk1;
    removed_k1   = rk1;
    removed_k1_2 = rk2;
    return true;
}

/* ================================================================
   demo
   compile: gcc -DTEST_MAIN -o policy_test policy.c && ./policy_test
   ================================================================ */
#ifdef TEST_MAIN

static void print_result(const char *label, int tk, int rk1, int rk2, int rcnt,
                         int *path, int pl, int *pk, int pc)
{
    printf("\n-- %s --\n  pick %d K2, remove %d K1", label, tk, rcnt);
    if (rcnt >= 2) printf(" [%d, %d]", rk1, rk2);
    else           printf(" [%d]", rk1);
    printf("\n  path: ");
    for (int i = 0; i < pl; i++) printf("%d ", path[i]);
    printf("\n  picked K2: ");
    for (int i = 0; i < pc; i++) printf("%d ", pk[i]);
    printf("  steps: %d\n", pl - 1);
}

int main(void)
{
    int k1[3] = {3, 6, 10};
    int k2[4] = {2, 4, 8, 11};
    int kf = 9;
    policy_init(k1, k2, kf);

    printf("Map: K1=[%d,%d,%d] K2=[%d,%d,%d,%d] Kf=%d\n",
           k1[0], k1[1], k1[2], k2[0], k2[1], k2[2], k2[3], kf);

    /* Step-by-step fallback trace */
    {
        int path[20], pl, pk[4], pc, rk1, rk2;
        int max_steps = 5;

        printf("\n=== STAGE: 3 K2 + 1 K1 (max %d steps) ===", max_steps);
        if (policy_solve_n(3, 1, path, &pl, pk, &pc, &rk1, &rk2)) {
            printf("  found, steps=%d", pl - 1);
            if (pl - 1 <= max_steps) {
                print_result("OK (<=5 steps)", 3, rk1, rk2, 1, path, pl, pk, pc);
            } else {
                printf("  -- REJECTED (>5 steps)\n");
            }
        } else { printf("  -- no solution\n"); }

        printf("\n=== STAGE: 3 K2 + 2 K1 ===");
        if (policy_solve_n(3, 2, path, &pl, pk, &pc, &rk1, &rk2)) {
            print_result("FALLBACK", 3, rk1, rk2, 2, path, pl, pk, pc);
        } else { printf("  -- no solution\n"); }

        printf("\n=== STAGE: 2 K2 + 2 K1 ===");
        if (policy_solve_n(2, 2, path, &pl, pk, &pc, &rk1, &rk2)) {
            print_result("FALLBACK", 2, rk1, rk2, 2, path, pl, pk, pc);
        } else { printf("  -- no solution\n"); }

        printf("\n=== STAGE: 2 K2 + 1 K1 ===");
        if (policy_solve_n(2, 1, path, &pl, pk, &pc, &rk1, &rk2)) {
            print_result("FALLBACK", 2, rk1, rk2, 1, path, pl, pk, pc);
        } else { printf("  -- no solution\n"); }
    }

    /* best */
    {
        int path[20], pl, pk[4], pc, rk, tk;
        printf("\n=== policy_solve_best() ===");
        if (policy_solve_best(path, &pl, pk, &pc, &rk, &tk)) {
            print_result("BEST", tk, rk, removed_k1_2, removed_cnt, path, pl, pk, pc);
        } else {
            printf("  -- No solution!\n");
        }
    }
    return 0;
}
#endif
