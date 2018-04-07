#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

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

bool pos_in_list(node_list_t* list, int px, int py) {
    if (list == NULL) { // 空の場合
        return false;
    }
    for (node_list_t *elm = list; elm; elm = elm->next) {
        if (elm->ptr->px == px && elm->ptr->py == py) {
            return true;
        }
    }
    return false;
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
        int vectors[4][2] = {{0,1},{-1,0},{1,0},{0,-1}};
        for (int i=0; i<4; i++) {
            int px = curnode->px + vectors[i][0];
            int py = curnode->py + vectors[i][1];
            if (!pos_in_list(openlist, px, py) && world[px][py] != 1) {
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

    path_array_t *result_path = a_star_rpg_move(start, goal, world);
    
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