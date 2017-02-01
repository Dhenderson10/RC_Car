/*
 * bfs.c
 *
 *  Created on: Nov 15, 2016
 *      Author: henderson/harrison
 */

#include "bfs.h"
#include <stdlib.h>
#include <assert.h>

struct list_element_t_struct;

struct list_element_t_struct
{
    int rank;
    struct list_element_t_struct* next;
};

typedef struct list_element_t_struct list_element_t;
typedef struct
{
    list_element_t *head;
    list_element_t *tail;
} list_t;


void fill_neighbors(int neighbors[], int rank);
void push_front(list_t* list, int rank);
int remove_front(list_t* list);
int isEmpty(list_t* list);
void free_list(list_t* list);
void push_back(list_t* list, int rank);
void print_list(list_t* list);
list_t init_list();


void find_shortest_path(int start_rank, int goal_rank, const int obstacles[], int coordArray[50]) //bfs
{
    //step 1.
    int predecessors[ELEMENT_COUNT];
    predecessors[start_rank] = start_rank;
    for(int i = 0; i < ELEMENT_COUNT; i++)
    {
        if(obstacles[i] == 1)
        {
            predecessors[i] = -2;
        }
        else
        {
            predecessors[i] = -1;
        }

    }

    //step 2.
    list_t list;
    list = init_list();
    push_back(&list, start_rank);

    list_t list2;
    list2 = init_list();

    //step 3.

    int neighbors[4];
    int front = -1;
    int bestCell;
    if(goal_rank < start_rank)
    {
        bestCell = ELEMENT_COUNT - 1;//has to do with ur first comparison, if going from small to large this needs to be EC
    }
    else
    {
        bestCell = ELEMENT_COUNT;//    if going from big to small, this needs to be E
    }

    int nRow;
    int nCol;
    int goalRow = ROW(goal_rank);
    int goalCol = COL(goal_rank);
    int bestCellRow;
    int bestCellCol;
    int flag = 0;
    while(flag == 0)
    {

        front = remove_front(&list);

        //step 4.
        fill_neighbors(neighbors, front);

        //step 5.
        int lastRank = -1;
        int lastNeighbor;


        for(int i = 0; i < 4; i++)
        {
            bestCellRow = ROW(bestCell);
            bestCellCol = COL(bestCell);
            //printf("oldDist: %d\n", oldDist);
            int neighborRank = neighbors[i];
            nRow = ROW(neighborRank);
            nCol = COL(neighborRank);

            if(neighborRank != -1 && predecessors[neighborRank] == -1)// && neighborRank != lastRank)
            {
                printf("neighborRow: %d\n", nRow);

                printf("neighborCol: %d\n", nCol);
                printf("compare\n");
                printf("bestCellRow: %d\n", bestCellRow);

                printf("bestCellCol: %d\n", bestCellCol);

                printf("!\n");
                //works if we go down
                //works if we go down, to the left, and up
                //works if we go to the left
                //works if we go to the left, then down

                //doest work if we have to go up left then down

                if(nCol > goalCol && bestCellCol >= nCol)
                {
                    printf("a\n");
                    bestCell = neighborRank;

                }
                if(nCol == goalCol && bestCellCol > nCol)// && nRow > goalRow )
                {
                    bestCell = neighborRank;
                }

                if(nCol == goalCol && nRow > goalRow && nRow < bestCellRow)
                {
                    printf("b\n");
                    bestCell = neighborRank;

                }


                if(nCol < goalCol && bestCellCol >= nCol )//&& start_rank > goal_rank)
                {
                    printf("q\n");
                    bestCell = neighborRank;

                }



                if(nCol < goalCol && bestCellCol <= nCol)
                {
                    printf("c\n");
                    bestCell = neighborRank;

                }
                if(nCol == goalCol && bestCellCol < nCol) //&& nRow < goalRow) //test more
                {
                    bestCell = neighborRank;
                }

                if(nCol == goalCol && nRow < goalRow && nRow > bestCellRow)
                {
                    printf("d\n");
                    bestCell = neighborRank;

                }
                if(nCol > goalCol && bestCellCol >= nCol )//&& start_rank > goal_rank)
                {
                    printf("p\n");
                    bestCell = neighborRank;

                }



                if(neighborRank == goal_rank)
                {
                    bestCell = neighborRank;
                    flag = 1;
                }

                /*if(neighborRank > goal_rank)// && neighborRank < lastNeighbor)
                {
                    if(bestCell > neighborRank)
                    {
                        bestCell = neighborRank;
                    }
                }

                if(neighborRank < goal_rank)// && neighborRank < lastNeighbor)
                {
                    if(bestCell < neighborRank)
                    {
                        bestCell = neighborRank;
                    }
                }
                if(neighborRank == goal_rank)
                {
                    bestCell = neighborRank;
                }

                lastNeighbor = neighborRank;
                lastRank = neighborRank;

                //printf("newdist: %d\n", newDist);
                push_back(&list, neighborRank);
                predecessors[neighborRank] = neighborRank;
                 *
                 */
            }



        }
        printf("bestCell: Row: %d Col: %d\n", ROW(bestCell), COL(bestCell));
        push_back(&list, bestCell);
        predecessors[bestCell] = bestCell;


        push_back(&list2, bestCell);
    }

    //step 6. //this is messing stuff up

    /*list_t list2;
    list2 = init_list();
    push_back(&list2, start_rank);
    for(int i = (ELEMENT_COUNT - 1); i > -1; i--)
    {
        if(predecessors[i] > 0) // not -1 or -2
        {
            printf("bestCell: Row: %d Col: %d\n", ROW(predecessors[i]), COL(predecessors[i]));
            push_back(&list2, predecessors[i]);
        }
    }*/

    //rintf("list2: ");
    //push_back(&list2, goal_rank);
    //int* elements = (int*)malloc(sizeof(int)*50);
    //coordArray = print_list(&list2);
    /* for(int i = 0; coordArray[i] != -1; i++)
    {

        printf("rank: %d", coordArray[i]);
    }*/
    list_element_t *currentElement = list2.head;
    list_element_t *nextElement = list2.head->next;
    int currRank = currentElement->rank;
    int count = 0;

    while(nextElement != NULL)
    {
        coordArray[count] = currentElement->rank;
        currentElement = nextElement;
        nextElement = currentElement->next;
        count++;

    }

    coordArray[count] = -1;
    for(int i = 0; coordArray[i] != -1; i++)
    {

        printf("rank: %d", coordArray[i]);
    }
    //return coordArray;
    //return elements;
    //printf("goal: %d", goal_rank);

    //printf("list: ");
    //print_list(&list);


}


void fill_neighbors(int neighbors[], int rank)
{

    //neighbors[0] = North
    //neighbors[1] = West
    //neighbors[2] = South
    //neighbors[3] = East

    int north = rank - MAZE_WIDTH;
    int west = rank - 1;
    int south = rank + MAZE_WIDTH;
    int east = rank + 1;
    neighbors[0] = north;
    neighbors[1] = west;
    neighbors[2] = south;
    neighbors[3] = east;


    for(int i = 0; i < 4; i++)
    {
        if(neighbors[i] >= ELEMENT_COUNT || neighbors[i] < 0)
        {
            neighbors[i] = -1;
        }
    }
}

void push_front(list_t* list, int rank)
{
    list_element_t* tmp = (list_element_t*)malloc(sizeof(list_element_t));
    assert(tmp != NULL);
    tmp->rank = rank;
    tmp->next = NULL;
    if(list->head == NULL && list->tail == NULL)
    {
        list->head = tmp;
        list->tail = tmp;
    }
    else
    {
        tmp->next = list->head;
        list->head = tmp;
    }
}
int remove_front(list_t* list)
{
    list_element_t* tmp = (list_element_t*)malloc(sizeof(list_element_t));
    int front_value = list->head->rank;
    if(list->head == NULL && list->tail == NULL) //list is empty
    {
        return -1;
    }
    else if(list->head == list->tail)
    {
        free(list->head);

        *list = init_list();
    }
    else
    {
        tmp = list->head->next;
        list->head->next = NULL;
        free(list->head);
        list->head = tmp;
    }
    return front_value;
}
void free_list(list_t *list)
{
    if(list->head == NULL && list->tail == NULL)
    {
        return;
    }

    list_element_t *currentElement = list->head;
    list_element_t *nextElement = list->head->next;
    free(currentElement);

    while(nextElement != NULL)
    {
        currentElement = nextElement;
        nextElement = currentElement->next;
        free(currentElement);
    }

    *list = init_list();
}

void push_back(list_t* list, int rank)
{
    list_element_t* tmp = (list_element_t*)malloc(sizeof(list_element_t));
    assert(tmp != NULL);
    tmp->rank = rank;
    tmp->next = NULL;

    if(list->head == NULL && list->tail == NULL)
    {
        list->head = tmp;
        list->tail = tmp;
    }
    else{
        list->tail->next = tmp;
        list->tail = tmp;
    }
}
list_t init_list()
{
    list_t emptyList;
    emptyList.head = NULL;
    emptyList.tail = NULL;

    return emptyList;
}

void print_list(list_t* list)
{
    if(list->head == NULL && list->tail == NULL){
        printf("empty list\n");
        return;

    }
    list_element_t *currentElement = list->head;
    list_element_t *nextElement = list->head->next;

    int currRank = currentElement->rank;
    int row = ROW(currRank);
    int col = COL(currRank);

    printf("row %d, col %d \n", row, col );

    while(nextElement != NULL){
        currentElement = nextElement;
        nextElement = currentElement->next;
        currRank = currentElement->rank;
        row = ROW(currRank);
        col = COL(currRank);
        printf("row %d, col %d \n", row, col );
    }
}

int test_main()
{
    list_t l1;
    l1 = init_list();

    assert(l1.head == l1.tail && l1.head == NULL);

    push_back(&l1, 10);

    assert(l1.head != NULL);
    assert(l1.tail == l1.head);
    assert(l1.head->rank == 10);
    assert(l1.head->next == NULL);
    push_back(&l1, 20);

    assert(l1.head != l1.tail);
    assert(l1.head->rank == 10);
    assert(l1.head->next == l1.tail);
    assert(l1.tail->rank == 20);
    assert(l1.tail->next == NULL);
    int front = remove_front(&l1);

    assert(l1.head->rank == 20);
    assert(l1.head->next == NULL);
    assert(front == 10);
    front = remove_front(&l1);

    assert(l1.head == NULL);
    assert(l1.tail == NULL);

    assert(front == 20);
    int i = remove_front(&l1);
    assert(i == -1);
    push_front(&l1, 6);
    assert(l1.head != NULL);
    assert(l1.tail == l1.head);
    assert(l1.head->rank == 6);
    assert(l1.head->next == NULL);

    push_front(&l1, 9);

    assert(l1.head != l1.tail);
    assert(l1.head->rank == 9);
    assert(l1.head->next == l1.tail);
    assert(l1.tail->rank == 6);
    assert(l1.tail->next == NULL);
    free_list(&l1);
    assert(l1.head == l1.tail && l1.head == NULL);

    int neighbors[10];

    fill_neighbors(neighbors, 0);
    assert(neighbors[0] == -1);
    assert(neighbors[1] == -1);
    assert(neighbors[2] == 10);
    assert(neighbors[3] == 1);

    fill_neighbors(neighbors, 11);
    assert(neighbors[0] == 1);
    assert(neighbors[1] == 10);
    assert(neighbors[2] == 21);
    assert(neighbors[3] == 12);

    printf("Pass");

    return 0;
}
