#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/aw_thread.h"

#define MAX_ROBOT_NUM 50

struct robot* robots;
struct message* msgs;
char* docks;

int move[5][5] = {{0, 0}, {-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // 1: 상  2: 하  3: 좌  4: 우  0: 정지
int robot_location[6][7] = {{0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 0, 0},
                        {0, 0, 0, 0, 0, 1, 0},
};



// test code for central control node thread
void test_cnt(void *arg) {
    int *robot_cnt_ptr = (int *)arg;
    int robot_cnt = *robot_cnt_ptr;
    int end_flag = 0;
    int initial_active_robot = -1;
    while(1){
        // 박스로부터 로봇의 정보를 읽어옴
        if(list_size(&blocked_threads) == robot_cnt){
                struct message *robot_msgs = malloc(sizeof(struct message)*robot_cnt);
                int break_cnt = -1;

                for (int i = 0; i < robot_cnt; i++) {
                        //printf("get_msgs : %d %d %d %d %d %d\n", i, boxes_from_robots[i].msg.row, boxes_from_robots[i].msg.col, boxes_from_robots[i].msg.current_payload, boxes_from_robots[i].msg.required_payload, boxes_from_robots[i].msg.cmd);
                        //struct message msg;
                        get_message_box_from_robots(i, &robot_msgs[i]);
                        //printf("message_box_from_robots = %d %d %d %d %d\n", robot_msgs[i].row, robot_msgs[i].col, robot_msgs[i].required_payload, robot_msgs[i].current_payload, robot_msgs[i].cmd);
                        
                        // 로봇의 정보를 조작하여 수정
                        /* 
                        요구사항 4 여기에 구현
                        중앙 관제 노드 (main thread) 에서 로봇을 어떤식으로 이동시킬지 정해봅시다.
                        message 구조체 안에 cmd의 상수값에 따라 상, 하, 좌, 우, 멈춤을 나누어 구현하는 방법을 생각중이다.
                        */

                       // 초기값 설정
                       // 레일에 입장하는 로봇
                       if(initial_active_robot == i && robot_location[5][5] != 0){
                                
                                robot_msgs[i].cmd = 1;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                initial_active_robot++;
                                if(initial_active_robot == robot_cnt){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                }
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }

                      if(robot_msgs[i].row == 4 && robot_msgs[i].col == 5){
                                if(robot_location[robot_msgs[i].row + move[1][0]][robot_msgs[i].col + move[1][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_msgs[i].cmd = 1;
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                      }

                       //move default

                       // - 하역장에 도달하거나, 물건을 집고 다시 레일에 복귀한다.

                       // 1번 도착
                       if(robot_msgs[i].row == 1 && robot_msgs[i].col == 1){
                                if(robot_location[robot_msgs[i].row + move[2][0]][robot_msgs[i].col + move[2][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;   
                                robot_msgs[i].cmd = 2;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       else if(robot_msgs[i].row == 1 && robot_msgs[i].col == 2){
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 1;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] += 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                // A 도착 전
                       }
                       else if(robot_msgs[i].row == 0 && robot_msgs[i].col == 2){
                                //robot_location[robot_msgs[i].row][robot_msgs[i].col] = 1;
                                robot_msgs[i].cmd = 0;
                                //robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] += 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                // A 도착
                       }
                       // 2번 도착
                       else if(robot_msgs[i].row == 1 && robot_msgs[i].col == 3){
                                if(robot_location[robot_msgs[i].row + move[2][0]][robot_msgs[i].col + move[2][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 2;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       // 3번 도착
                       else if(robot_msgs[i].row == 1 && robot_msgs[i].col == 4){
                                if(robot_location[robot_msgs[i].row + move[2][0]][robot_msgs[i].col + move[2][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 2;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       // 4번 도착
                       else if(robot_msgs[i].row == 1 && robot_msgs[i].col == 5){
                                if(robot_location[robot_msgs[i].row + move[2][0]][robot_msgs[i].col + move[2][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 2;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       // 5번 도착
                       else if(robot_msgs[i].row == 4 && robot_msgs[i].col == 1){
                                if(robot_location[robot_msgs[i].row + move[1][0]][robot_msgs[i].col + move[1][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 1;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       else if(robot_msgs[i].row == 4 && robot_msgs[i].col == 2){
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 2;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] += 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                // C 도착 전
                       }
                       else if(robot_msgs[i].row == 4 && robot_msgs[i].col == 2){
                                //robot_location[robot_msgs[i].row][robot_msgs[i].col] = 1;
                                robot_msgs[i].cmd = 0;
                                //robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] += 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                // C 도착
                       }
                       // 6번 도착
                       else if(robot_msgs[i].row == 4 && robot_msgs[i].col == 3){
                                if(robot_location[robot_msgs[i].row + move[1][0]][robot_msgs[i].col + move[1][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 1;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       // 7번 도착
                       else if(robot_msgs[i].row == 4 && robot_msgs[i].col == 4){
                                if(robot_location[robot_msgs[i].row + move[1][0]][robot_msgs[i].col + move[1][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 1;
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                break_cnt = i;
                                break;
                       }
                       else if(robot_msgs[i].row == 2 && robot_msgs[i].col == 0){
                                //robot_location[robot_msgs[i].row][robot_msgs[i].col] = 1;
                                robot_msgs[i].cmd = 0;
                                //robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] += 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                // B 도착
                       }

                       // - 레일의 기본적인 흐름

                       else if(robot_msgs[i].row == 3 && robot_msgs[i].col == 5){
                                if(robot_location[robot_msgs[i].row + move[1][0]][robot_msgs[i].col + move[1][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 1; // up
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                       }
                       else if(robot_msgs[i].row == 2 && robot_msgs[i].col <= 5 && robot_msgs[i].col > 1){
                                // 물건 4
                                if(robot_msgs[i].row == 2 && robot_msgs[i].col == 5 && robot_msgs[i].required_payload == 4 && robot_msgs[i].current_payload == 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 1;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 4;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 물건 3
                                else if(robot_msgs[i].row == 2 && robot_msgs[i].col == 4 && robot_msgs[i].required_payload == 3 && robot_msgs[i].current_payload == 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 1;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 3;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 물건 2
                                else if(robot_msgs[i].row == 2 && robot_msgs[i].col == 3 && robot_msgs[i].required_payload == 2 && robot_msgs[i].current_payload == 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 1;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 2;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 하역장 A
                                else if(robot_msgs[i].row == 2 && robot_msgs[i].col == 2 && docks[i] == 'A' && robot_msgs[i].current_payload != 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 1;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }  
                                else{
                                        if(robot_location[robot_msgs[i].row + move[3][0]][robot_msgs[i].col + move[3][1]] == 1){
                                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                                continue;
                                        }
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 3; // left
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                }
                       }
                       else if(robot_msgs[i].row == 2 && robot_msgs[i].col == 1){
                                // 1번 물건
                                if(robot_msgs[i].required_payload == 1 && robot_msgs[i].current_payload == 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 1;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 1;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 하역장 B
                                if(docks[i] =='B' && robot_msgs[i].current_payload != 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 3;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] += 1;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                if(robot_location[robot_msgs[i].row + move[2][0]][robot_msgs[i].col + move[2][1]] == 1){
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        continue;
                                }        
                                robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                robot_msgs[i].cmd = 2; // down
                                robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                       }
                       else if(robot_msgs[i].row == 3 && robot_msgs[i].col < 5 && robot_msgs[i].col >= 1){
                                // 5번 물건
                                if(robot_msgs[i].row == 3 && robot_msgs[i].col == 1 && robot_msgs[i].required_payload == 5 && robot_msgs[i].current_payload == 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 2;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 5;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 하역장 C
                                else if(robot_msgs[i].row == 3 && robot_msgs[i].col == 2 && docks[i] == 'C' && robot_msgs[i].current_payload != 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 2;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 6번 물건
                                else if(robot_msgs[i].row == 3 && robot_msgs[i].col == 3 && robot_msgs[i].required_payload == 6 && robot_msgs[i].current_payload == 0){

                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 2;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 6;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                // 7번 물건
                                else if(robot_msgs[i].row == 3 && robot_msgs[i].col == 4 && robot_msgs[i].required_payload == 7 && robot_msgs[i].current_payload == 0){
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        robot_msgs[i].cmd = 2;
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_msgs[i].current_payload = 7;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                        break_cnt = i;
                                        break;
                                }
                                else{
                                        if(robot_location[robot_msgs[i].row + move[4][0]][robot_msgs[i].col + move[4][1]] == 1){
                                                set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                                continue;
                                        }
                                        robot_msgs[i].cmd = 4; // right
                                        robot_location[robot_msgs[i].row+move[robot_msgs[i].cmd][0]][robot_msgs[i].col+move[robot_msgs[i].cmd][1]] = 1;
                                        robot_location[robot_msgs[i].row][robot_msgs[i].col] = 0;
                                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                                }
                       }
                        // 수정된 정보를 다시 박스에 저장
                        set_message_box_from_central_control_node(i, &robot_msgs[i]);
                        
                        //printf("get_msgs_central : %d %d %d %d %d %d\n", i, boxes_from_central_control_node[i].msg.row, boxes_from_central_control_node[i].msg.col, boxes_from_central_control_node[i].msg.current_payload, boxes_from_central_control_node[i].msg.required_payload, boxes_from_central_control_node[i].msg.cmd);
                }

                if(initial_active_robot == -1){
                        initial_active_robot++;
                }
                if(break_cnt > -1 && break_cnt < robot_cnt-1){
                        for(int i=break_cnt+1; i<robot_cnt; i++){
                                set_message_box_from_central_control_node(i, &boxes_from_robots[i].msg);
                        }
                }

                print_map(robots, robot_cnt);
                
                if(end_flag){
                        printf("------END OF SIMULATION------\n");
                        break;
                }
                if(robot_location[0][2] + robot_location[2][0] + robot_location[5][2] == robot_cnt){
                        end_flag = 1;
                }
        
                timer_sleep(100);
                unblock_threads(robot_cnt);
        }
    }
}

// test code for robot thread
void test_thread(void* aux){
        int idx = *((int *)aux);
        struct message msg; // 메시지를 담을 구조체 선언

        while(1){
                
        // 로봇 정보를 메시지에 저장
                set_message(&msg, robots[idx].row, robots[idx].col, robots[idx].current_payload, robots[idx].required_payload, 0);
        // 메시지를 해당하는 박스에 저장
                //printf("msgs : %d %d %d %d %d %d\n", idx, msg.row, msg.col, msg.current_payload, msg.required_payload, msg.cmd);
                set_message_box_from_robots(idx, &msg);
                //printf("from_robot(no. %d): %d %d %d %d cmd : %d\n\n", idx, boxes_from_robots[idx].msg.row, boxes_from_robots[idx].msg.col,
                //boxes_from_robots[idx].msg.required_payload, boxes_from_robots[idx].msg.current_payload, boxes_from_robots[idx].msg.cmd);
        // 스레드 block
                //printf("robots : %d %d %d %d %d\n", idx, robots[idx].row, robots[idx].col, robots[idx].current_payload, robots[idx].required_payload);
                //printf("boxes_from_robots : %d %d %d %d %d %d\n\n", idx, boxes_from_robots[idx].msg.row, boxes_from_robots[idx].msg.col, boxes_from_robots[idx].msg.current_payload, boxes_from_robots[idx].msg.required_payload), boxes_from_central_control_node->msg.cmd;
                block_thread();


        // 변경된 값 로봇에 적용 cmd값 따라 적용!!!!!!!!!!!!!!!!!!!여기서부터 해야돼
                robots[idx].row = boxes_from_central_control_node[idx].msg.row + move[boxes_from_central_control_node[idx].msg.cmd][0];
                robots[idx].col = boxes_from_central_control_node[idx].msg.col + move[boxes_from_central_control_node[idx].msg.cmd][1];
                robots[idx].required_payload = boxes_from_central_control_node[idx].msg.required_payload;
                robots[idx].current_payload = boxes_from_central_control_node[idx].msg.current_payload;
                //printf("moved robot(no. %d): %d %d %d %d\n", idx, robots[idx].row, robots[idx].col, robots[idx].required_payload, robots[idx].current_payload);
                //robot_location[robots[idx].row][robots[idx].col] = 1;
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this
        int robot_cnt = atoi(argv[1]); // 로봇의 개수 인자 받기
        char *context = NULL;
        char *tok = strtok_r(argv[2], ":", &context); // 로봇의 목표 좌표 인자 받기
        docks = malloc(sizeof(char)*robot_cnt);
        int docks_cnt = 0;

        for(int i=0; i<2; i++){
                if(!isdigit(tok[i])){
                        docks[docks_cnt++] = tok[i];
                        //printf("%c\n", docks[docks_cnt-1]);
                }
        }
        printf("implement automated warehouse!\n");
        char name[MAX_ROBOT_NUM][10];
        // test case robots
        robots = malloc(sizeof(struct robot) * robot_cnt);
        for(int i=0; i<robot_cnt; i++){
                snprintf(name[i], 11, "R%d", i+1);
                //printf("%s", name[i]);
                if(i == 0){
                        setRobot(&robots[i], name[i], 5, 5, atoi(tok), 0);
                        }
                else{
                        tok = strtok_r(NULL, ":", &context);
                        setRobot(&robots[i], name[i], 5, 5, atoi(tok), 0);
                        for(int i=0; i<2; i++){
                                if(!isdigit(tok[i])){
                                        docks[docks_cnt++] = tok[i];
                                        //printf("%c\n", docks[docks_cnt-1]);
                                }
                        }
                }
        }

        init_boxes_from_robots(robot_cnt);
        init_boxes_from_central_control_node(robot_cnt);

        // example of create thread
        tid_t* threads = malloc(sizeof(tid_t) * (robot_cnt));
        tid_t* main_thread = malloc(sizeof(tid_t));
        int *idxs = (int*)malloc(sizeof(int) * (robot_cnt));
        msgs = malloc(sizeof(struct message)*(robot_cnt));
        idxs[0] = 0;
        list_init(&blocked_threads);

        main_thread = thread_create("CNT", 0, &test_cnt, (void*)&robot_cnt);
        for(int i=0; i<robot_cnt; i++){
                idxs[i] = i;
                threads[i] = thread_create(robots[i].name, 0, &test_thread, &idxs[i]);
        }
        // if you want, you can use main thread as a central control node
}