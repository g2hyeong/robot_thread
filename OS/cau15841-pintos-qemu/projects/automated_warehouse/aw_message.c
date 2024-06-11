#include "projects/automated_warehouse/aw_message.h"
#include <stdlib.h>
#include <string.h>

struct message_box* boxes_from_central_control_node;
/** message boxes from robots to central control node */
struct message_box* boxes_from_robots;

void init_boxes_from_robots(int robot_cnt){
    boxes_from_robots = malloc(sizeof(struct message_box)*(robot_cnt));
    //memset(boxes_from_robots, 0, (robot_cnt+1)*sizeof(struct message_box));
}
void init_boxes_from_central_control_node(int robot_cnt){
    boxes_from_central_control_node = malloc(sizeof(struct message_box)*(robot_cnt));
    //memset(boxes_from_central_control_node, 0, (robot_cnt+1)*sizeof(struct message_box));
}
void free_boxes_from_robots(){
    free(boxes_from_robots);
}
void free_boxes_from_central_control_node(){
    free(boxes_from_central_control_node);
}
void set_message_box_from_robots(int index, struct message* msg){
    boxes_from_robots[index].msg.row = msg->row;
    boxes_from_robots[index].msg.col = msg->col;
    boxes_from_robots[index].msg.current_payload = msg->current_payload;
    boxes_from_robots[index].msg.required_payload = msg->required_payload;
    boxes_from_robots[index].msg.cmd = msg->cmd;
    //memcpy(&boxes_from_robots[index].msg, msg, sizeof(struct message));
}
void get_message_box_from_robots(int index, struct message* msg){
    memcpy(msg, &boxes_from_robots[index].msg, sizeof(struct message));
}
void set_message_box_from_central_control_node(int index, struct message* msg){
    boxes_from_central_control_node[index].msg.row = msg->row;
    boxes_from_central_control_node[index].msg.col = msg->col;
    boxes_from_central_control_node[index].msg.current_payload = msg->current_payload;
    boxes_from_central_control_node[index].msg.required_payload = msg->required_payload;
    boxes_from_central_control_node[index].msg.cmd = msg->cmd;
    //memcpy(&boxes_from_central_control_node[index], msg, sizeof(struct message));
}
void get_message_box_from_central_control_node(int index, struct message* msg){
    memcpy(msg, &boxes_from_central_control_node[index], sizeof(struct message));
}
int is_empty_box_from_robots(int i){
    if(boxes_from_robots[i].msg.required_payload == 0){
        return 1;
    }
    else
        return 0;
}

int is_empty_box_from_central_control_node(){
    if(boxes_from_central_control_node[1].msg.required_payload == 0){
        return 1;
    }
    else
        return 0;
}

void set_message(struct message* msg, int row, int col, int current_payload, int required_payload, int cmd){
    msg->row = row;
    msg->col = col;
    msg->required_payload = required_payload;
    msg->current_payload = current_payload;
    msg->cmd = cmd;
}
