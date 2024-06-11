#ifndef _PROJECTS_PROJECT1_AW_MESSAGE_H__
#define _PROJECTS_PROJECT1_AW_MESSAGE_H__
#include <stdlib.h>
#include "robot.h"

/**
 * For easy to implement, combine robot and central control node message
 * If you want to modify message structure, don't split it
 */
struct message {
    //
    // To central control node
    //
    /** current row of robot */
    int row;
    /** current column of robot */
    int col;
    /** current payload of robot */
    int current_payload;
    /** required paylod of robot */
    int required_payload;

    //
    // To robots
    //
    /** next command for robot */
    int cmd; // 0 : stop / 1 : UP / 2 : Right / 3 : Down / 4 : Left
};

/** 
 * Simple message box which can receive only one message from sender
*/
struct message_box {
    /** check if the message was written by others */
    int dirtyBit;
    /** stored message */
    struct message msg;
};

/** message boxes from central control node to each robot */
extern struct message_box* boxes_from_central_control_node;
/** message boxes from robots to central control node */
extern struct message_box* boxes_from_robots;

void set_message(struct message* msg, int row, int col, int current_payload, int required_payload, int cmd);

// 전역 변수인 메세지 박스를 로봇 스레드의 개수만큼 동적 할당한다.
void init_boxes_from_robots(int robot_cnt);

void init_boxes_from_central_control_node(int robot_cnt);

void free_boxes_from_robots();

void free_boxes_from_central_control_node();

void set_message_box_from_robots(int index, struct message* msg);

void set_message_box_from_central_control_node(int index, struct message* msg);

void get_message_box_from_robots(int index, struct message* msg);

void get_message_box_from_central_control_node(int index, struct message* msg);

int is_empty_box_from_robots(int i);

int is_empty_box_from_central_control_node();

void set_message(struct message* msg, int row, int col, int current_payload, int required_payload, int cmd);




#endif