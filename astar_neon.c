#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <arm_neon.h>

#define MAX_PATH_LIST_NUM 1024
#define HEIGHT 20
#define WIDTH 97

typedef struct _path_array_t {
    size_t size;
    size_t y[MAX_PATH_LIST_NUM];
    size_t x[MAX_PATH_LIST_NUM];
} path_array_t;

typedef struct _a_star_node_t {
    int cost;
    int heuristic;
    size_t px;
    size_t py;
    size_t goal_px;
    size_t goal_py;
    struct _a_star_node_t* pre_node;
} a_star_node_t;

a_star_node_t *get_new_node(int px, int py, int goal[2], int cost, a_star_node_t *pre_node) {
    a_star_node_t *new_node = (a_star_node_t *)malloc(sizeof(a_star_node_t));
    new_node->px = px;
    new_node->py = py;
    new_node->goal_px = goal[0];
    new_node->goal_py = goal[1];
    new_node->cost = cost;
    new_node->pre_node = pre_node;
    new_node->heuristic = abs(goal[0] - px) + abs(goal[1] - py);
    return new_node;
}

// Node List
// リストが空 (NULL)
// リストが空でない場合の最終要素 (next: NULL, ptr: 最終要素)
typedef struct _node_list_t {
    struct _node_list* next;
    a_star_node_t* ptr;
} node_list_t;

void node_list_init(node_list_t** listpp) {
    *listpp = NULL;
}

void node_list_free(node_list_t** listpp) {
    node_list_t* list_ptr = *listpp;
    if(list_ptr==NULL) {
        return;
    }

    while(list_ptr!=NULL) {
        node_list_t* pre_ptr = list_ptr;
        list_ptr = list_ptr->next;
        free(pre_ptr);
    }
    *listpp = NULL;
}

void node_list_insert(node_list_t** listpp, a_star_node_t* nodep) {
    node_list_t* list_ptr = *listpp;
    node_list_t* list_pre_ptr = NULL;
    
    int target_cost = nodep->cost + nodep->heuristic;
    while(list_ptr!=NULL) {
        int refer_cost = list_ptr->ptr->cost + list_ptr->ptr->heuristic;
        if(target_cost < refer_cost) {
            break;
        }
        list_pre_ptr = list_ptr;
        list_ptr = list_ptr->next;
    }
    
    // if list_pre_ptr == NULL -> first list replace
    if(list_pre_ptr == NULL) {
        node_list_t* new_listp = (node_list_t*)malloc(sizeof(node_list_t));
        new_listp->next = *listpp;
        new_listp->ptr = nodep;
        *listpp = new_listp;
    } else {
        // list_pre_ptr -> new_listp -> list_ptr
        node_list_t* new_listp = (node_list_t*)malloc(sizeof(node_list_t));
        new_listp->next = list_ptr;
        new_listp->ptr = nodep;
        list_pre_ptr->next = new_listp;
    }
}

a_star_node_t* node_list_pop(node_list_t** listpp) {
    node_list_t* list_ptr = *listpp;
    if(list_ptr->ptr == NULL) {
        return NULL;
    }
    
    node_list_t* nextp = list_ptr->next;
    a_star_node_t* nodep = list_ptr->ptr;
    free(list_ptr);
    
    *listpp = nextp;
    return nodep;
}

uint32x4_t pos_in_list(node_list_t* list, int32x4_t new_px, int32x4_t new_py) {
    uint32x4_t result_mask = vdupq_n_u32(0);

    if (list == NULL) { // 空の場合
        return result_mask;
    }

    // result_x[0:3] = elm->ptr == px[0:3]
    // result_y[0:3] = elm->ptr == py[0:3]
    // result[0:3] |= result_x & result_y
    for (node_list_t *elm = list; elm; elm = elm->next) {
        int32x4_t v_px = vld1q_dup_s32(&(elm->ptr->px) );
        int32x4_t v_py = vld1q_dup_s32(&(elm->ptr->py) );
        uint32x4_t x_eq = vceqq_s32(v_px, new_px);
        uint32x4_t y_eq = vceqq_s32(v_py, new_py);
        uint32x4_t xy_eq = vandq_u32(x_eq, y_eq);
        result_mask = vorrq_u32(result_mask, xy_eq);
    }
    return result_mask;
}

// Algori
path_array_t *a_star_rpg_move(int start[2], int goal[2], int world[HEIGHT][WIDTH]) {
    a_star_node_t *curnode = get_new_node(start[0], start[1], goal, 0, NULL);
    node_list_t *openlist;
    node_list_init(&openlist);
    node_list_insert(&openlist, curnode);
    while (openlist) {
        curnode = node_list_pop(&openlist);
        if (curnode->px == goal[0] && curnode->py == goal[1]) {
            break;
        }

        /*
        int16_t cur_pos[2] = {curnode->px, curnode->py};
        int16_t dir[4][2] = {{0,1},{-1,0},{1,0},{0,-1}};
        int16_t npos[4][2];

        int16x4x2_t v_pos = vld2_dup_s16(cur_pos);
        int16x4x2_t v_dir = vld2_s16(dir);
        int16x8_t v_npos = vaddq_s16((int16x8_t)v_pos, (int16x8_t)v_dir);
        vst2_s16(npos, v_npos);
a       */

        int32_t dir_x[4] = {0, -1, 1,  0};
        int32_t dir_y[4] = {1,  0, 0, -1};
        int32_t npos_x[4];
        int32_t npos_y[4];

        int32x4_t v_pos_x = vld1q_dup_s32(&(curnode->px) );
        int32x4_t v_dir_x = vld1q_s32(dir_x);
        int32x4_t v_npos_x = vaddq_s32(v_pos_x, v_dir_x);
        vst1q_s32(npos_x, v_npos_x);

        int32x4_t v_pos_y = vld1q_dup_s32(&(curnode->py) );
        int32x4_t v_dir_y = vld1q_s32(dir_y);
        int32x4_t v_npos_y = vaddq_s32(v_pos_y, v_dir_y);
        vst1q_s32(npos_y, v_npos_y);

        // int px = curnode->px + vectors[i][0];
        // int py = curnode->py + vectors[i][1];
        uint32x4_t v_insert_mask;
        uint32_t insert_mask[4];
        v_insert_mask = pos_in_list(openlist, v_npos_x, v_npos_y);
        uint32x4_t v_ones = vdupq_n_u32(1);
        v_insert_mask = vsubq_u32(v_ones, v_insert_mask);
        vst1q_u32(insert_mask, v_insert_mask);

        /*
        int64x2_t world_val_0 = vld1q_s64(world);
        int64x2_t world_val_1 = vld1q_s64(world);
        int64x2_t world_offset_0 = vld1q_s64(WIDHT);
        world_offset_0 = 
        world_val_0 = vaddq_s64(world_val_0, 
        */

        for (int i=0; i<4; i++) {
            int px = npos_x[i];
            int py = npos_y[i];

            if(insert_mask[i] && (world[px][py] != 1) ) {
                node_list_insert(&openlist, get_new_node(px, py, goal, curnode->cost+1, curnode));
            }
        }
    }
    path_array_t *path_array = (path_array_t *)malloc(sizeof(path_array_t));
    path_array->size = 0;
    while (curnode) {
        path_array->y[path_array->size] = curnode->py;
        path_array->x[path_array->size] = curnode->px;
        path_array->size++;
        curnode = curnode->pre_node;
    }
    return path_array;
}

int main() {
    int start[2] = {1,1};
    int goal[2] = {1,95};
    
    int world[HEIGHT][WIDTH] = {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},{1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},{1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1},{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};
    
    time_t time_start,time_end;

    time_start = clock();
    path_array_t *result_path = a_star_rpg_move(start, goal, world);
    time_end = clock();
    
    printf("time:%f\n", (float)(time_end-time_start)/CLOCKS_PER_SEC);

    for (int i=0; i<result_path->size; i++) {
        int x = result_path->x[i];
        int y = result_path->y[i];
        world[x][y] = 9;
    }
    
    for (int i=0; i<HEIGHT; i++) {
        for (int j=0; j<WIDTH; j++) {
            printf("%d", world[i][j]);
        }
        puts("");
    }
    return 0;
}
