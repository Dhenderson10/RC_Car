/*
 * bfs.h
 *
 *  Created on: Nov 15, 2016
 *      Author: henderson/harrison
 */


#ifndef BFS_H_
#define BFS_H_

#define MAZE_WIDTH (7)
#define ELEMENT_COUNT (MAZE_WIDTH * MAZE_WIDTH)
#define RANK(row, col) ((row)*MAZE_WIDTH+(col))
#define ROW(rank)  ((rank)/MAZE_WIDTH)
#define COL(rank)  ((rank)%MAZE_WIDTH)
void find_shortest_path(int start_rank, int goal_rank, const int obstacles[], int coordArray[50]);
int test_main();

#endif /* BFS_H_ */

