#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>

#define XBOX_TYPE_BUTTON    0x01
#define XBOX_TYPE_AXIS      0x02

#define XBOX_BUTTON_A       0x00
#define XBOX_BUTTON_B       0x01
#define XBOX_BUTTON_X       0x02
#define XBOX_BUTTON_Y       0x03
#define XBOX_BUTTON_LB      0x04
#define XBOX_BUTTON_RB      0x05
#define XBOX_BUTTON_START   0x06
#define XBOX_BUTTON_BACK    0x07
#define XBOX_BUTTON_HOME    0x08
#define XBOX_BUTTON_LO      0x09    /* ��ҡ�˰��� */
#define XBOX_BUTTON_RO      0x0a    /* ��ҡ�˰��� */

#define XBOX_BUTTON_ON      0x01
#define XBOX_BUTTON_OFF     0x00

#define XBOX_AXIS_LX        0x00    /* ��ҡ��X�� */
#define XBOX_AXIS_LY        0x01    /* ��ҡ��Y�� */
#define XBOX_AXIS_RX        0x03    /* ��ҡ��X�� */
#define XBOX_AXIS_RY        0x04    /* ��ҡ��Y�� */
#define XBOX_AXIS_LT        0x02
#define XBOX_AXIS_RT        0x05
#define XBOX_AXIS_XX        0x06    /* �����X�� */
#define XBOX_AXIS_YY        0x07    /* �����Y�� */

#define XBOX_AXIS_VAL_UP        -32767
#define XBOX_AXIS_VAL_DOWN      32767
#define XBOX_AXIS_VAL_LEFT      -32767
#define XBOX_AXIS_VAL_RIGHT     32767

#define XBOX_AXIS_VAL_MIN       -32767
#define XBOX_AXIS_VAL_MAX       32767
#define XBOX_AXIS_VAL_MID       0x00

typedef struct xbox_map {
	int     time;
	int     a;
	int     b;
	int     x;
	int     y;
	int     lb;
	int     rb;
	int     start;
	int     back;
	int     home;
	int     lo;
	int     ro;

	int     lx;
	int     ly;
	int     rx;
	int     ry;
	int     lt;
	int     rt;
	int     xx;
	int     yy;
}xbox_map_t;

class XboxJoyStick {
private:
	int xbox_fd_;
	xbox_map_t map_;
	int channel_data[8];
	int xbox_open(char* file_name);
	void xbox_close();
	void directMap();
	int xbox_map_read();
	static void *sub_run(void *args);
public:
	XboxJoyStick() {
		//js_device_path = "/dev/input/js0";
		memset(&map_, 0, sizeof(xbox_map_t));
		char js_device_path[20] = "/dev/input/js0";
		this->xbox_fd_ = xbox_open(js_device_path);
		if (this->xbox_fd_ < 0){
			perror("Error  openning joystick");
		}
		else {
			perror("Xbox joystick on");
		} 
		map_.lt = -32767;
		map_.rt = -32767;
	}
	~ XboxJoyStick() {
		xbox_close(); 
	}
	/*interface to read input channels*/
	int* read();
	int startJoyStickListener();
};
#endif