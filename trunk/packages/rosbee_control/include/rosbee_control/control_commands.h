/*
 * control_commands.h
 *
 */

#ifndef CONTROL_COMMANDS_H_
#define CONTROL_COMMANDS_H_

#define MOVE_CMD 		900
#define ENABLE_MOTION 	901
#define ENABLE_PC 		902
#define CLEAR_ERROR 	908
#define TOGGLE_US 		909
#define READ_ENCODERS 	911
#define READ_US 		912
#define READ_STATUS		913

#define MOVE_COMMAND(cntr,speed,dir) "$900, ## cntr ## , ## speed ## , ## dir ## ,"


#endif /* CONTROL_COMMANDS_H_ */
