#include    <pthread.h>
#include	"JoyStick.h"

int XboxJoyStick::xbox_open(char* file_name){
	int fd = open(file_name, O_RDONLY);
	if (fd < 0){
		perror("open");
		return -1;
	}
	return fd;
}

void XboxJoyStick::filterZero(int* cur)
{
	if (*cur < -10000) {
		*cur = (*cur + 10000) * 32767 / (32767 - 10000);
	}
	else if (*cur > 10000) {
		*cur = (*cur - 10000) * 32767 / (32767 - 10000);
	}
	else {
		*cur = 0;
	}
}
int XboxJoyStick::narrow(int cur, int a)
{
	if (cur < -10000 || cur > 10000) {
		return cur / a;
	}
	else {
		return 0;
	}
}
int XboxJoyStick::reverse(int cur)
{
	return 0 - cur;
}
int XboxJoyStick::mapToRCValue(int jsValue) {
	return (jsValue + XBOX_VAL_MAX) * RC_RANGE / (XBOX_VAL_MAX * 2) + RC_OFFSET;
}
void XboxJoyStick::directMap()
{
	channel_data[0] = mapToRCValue(narrow(reverse(map_.ly), 1));
	channel_data[1] = mapToRCValue(narrow(map_.lx, 1));
	channel_data[2] = mapToRCValue(narrow(reverse(map_.ry), 1));
	channel_data[3] = mapToRCValue(narrow(map_.rx, 1));
	channel_data[4] = mapToRCValue(map_.lt);
	channel_data[5] = mapToRCValue(map_.rt);
	channel_data[6] = mapToRCValue(map_.xx);
	channel_data[7] = mapToRCValue(reverse(map_.yy));
}

void XboxJoyStick::thresholdThrottle()
{
	int incremaental_xy = 100;
	if (map_.x == 1) {
		x_down = 1;
		//++
	}
	else {
		if (x_down == 1) {
			if (throttleValue < RC_RANGE) {
				throttleValue = throttleValue + incremaental_xy;
			}
			x_down = 0;
		}
	}
	if (map_.y == 1) {
		y_down = 1;
		//++
	}
	else {
		if (y_down == 1) {
			if (throttleValue > 0) {
				throttleValue = throttleValue - incremaental_xy;
			}
			y_down = 0;
		}
	}
	channel_data[0] = (map_.lt + XBOX_VAL_MAX) * incremaental_xy / (XBOX_VAL_MAX * 2) + throttleValue + RC_OFFSET;
	//channel_data[0] = mapToRCValue(narrow(reverse(map_.ly), 1));
	channel_data[1] = mapToRCValue(narrow(map_.lx, 1));
	channel_data[2] = mapToRCValue(narrow(reverse(map_.ry), 1));
	channel_data[3] = mapToRCValue(narrow(map_.rx, 1));
	//channel_data[4] = mapToRCValue(map_.lt);
	//cut off
	if (map_.b == 1) {
		channel_data[4] = RC_RANGE + RC_OFFSET;
	}
	else {
		channel_data[4] = RC_OFFSET;
	}
	channel_data[5] = mapToRCValue(map_.rt);
	channel_data[6] = mapToRCValue(map_.xx);
	channel_data[7] = mapToRCValue(reverse(map_.yy));
}
int XboxJoyStick::xbox_map_read(){
	int len, type, number, value;
	struct js_event js;
	len = ::read(xbox_fd_, &js, sizeof(struct js_event));
	if (len < 0){
		perror("read ");
		return -1;
	}
	type = js.type;
	number = js.number;
	value = js.value; 
	map_.time = js.time;
	
	if (type == JS_EVENT_BUTTON){
		switch (number)	{
		case XBOX_BUTTON_A:
			map_.a = value;
			break;

		case XBOX_BUTTON_B:
			map_.b = value;
			break;

		case XBOX_BUTTON_X:
			map_.x = value;
			break;

		case XBOX_BUTTON_Y:
			map_.y = value;
			break;

		case XBOX_BUTTON_LB:
			map_.lb = value;
			break;

		case XBOX_BUTTON_RB:
			map_.rb = value;
			break;

		case XBOX_BUTTON_START:
			map_.start = value;
			break;

		case XBOX_BUTTON_BACK:
			map_.back = value;
			break;

		case XBOX_BUTTON_HOME:
			map_.home = value;
			break;

		case XBOX_BUTTON_LO:
			map_.lo = value;
			break;

		case XBOX_BUTTON_RO:
			map_.ro = value;
			break;

		default:
			break;
		}
	}
	else if (type == JS_EVENT_AXIS)
	{
		switch (number)
		{
		case XBOX_AXIS_LX:
			map_.lx = value;
			break;

		case XBOX_AXIS_LY:
			map_.ly = value;
			break;

		case XBOX_AXIS_RX:
			map_.rx = value;
			break;

		case XBOX_AXIS_RY:
			map_.ry = value;
			break;

		case XBOX_AXIS_LT:
			map_.lt = value;
			break;

		case XBOX_AXIS_RT:
			map_.rt = value;
			break;

		case XBOX_AXIS_XX:
			map_.xx = value;
			break;

		case XBOX_AXIS_YY:
			map_.yy = value;
			break;

		default:
			break;
		}
	}
	else
	{
		/* Init do nothing */
	}
	/*printf("\e7");
	printf("\e[H\e[2K");
	printf("sTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
		map_.time, map_.a, map_.b, map_.x, map_.y, map_.lb, map_.rb, map_.start, map_.back, map_.home, map_.lo, map_.ro,
		map_.xx, map_.yy, map_.lx, map_.ly, map_.rx, map_.ry, map_.lt, map_.rt);
	fflush(stdout); 
	printf("\e8");*/

	//directMap();
	//arrowThrottle();
	thresholdThrottle();

	printf("\e7");
	printf("\e[H\e[2K");
	//printf("sTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
	//map_.time, map_.a, map_.b, map_.x, map_.y, map_.lb, map_.rb, map_.start, map_.back, map_.home, map_.lo, map_.ro,
	//map_.xx, map_.yy, map_.lx, map_.ly, map_.rx, map_.ry, map_.lt, map_.rt);
	printf("sTime:%8d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d",
		map_.time, channel_data[0], channel_data[1], channel_data[2], channel_data[3], channel_data[4], channel_data[5], channel_data[6], channel_data[7]);
	fflush(stdout);
	printf("\e8");
	return len;
}



void XboxJoyStick::xbox_close(){
	close(xbox_fd_);
	return;
} 
int * XboxJoyStick::read() {
	return channel_data;
}
int XboxJoyStick::startJoyStickListener() {
	pthread_t tidp;
	int error;
	printf("I am start \n");
	error = ::pthread_create(&tidp, NULL, sub_run, this);
	if (error)
	{
		printf("phread is not created...\n");
		return -1;
	}
	return 0;
}
void *XboxJoyStick::sub_run(void *args){
	while (1) {
		((XboxJoyStick*)args)->xbox_map_read();
		usleep(1 * 1000);//1000hz 123 
		
	}
	return (void *)0;
}
//int main1(void){
//	int xbox_fd;
//	xbox_map_t map;
//	int len, type;
//	int axis_value, button_value;
//	int number_of_axis, number_of_buttons;
//
//	memset(&map, 0, sizeof(xbox_map_t));
//	map.lt = -32767;
//	map.rt = -32767;
//
//	xbox_fd = xbox_open("/dev/input/js0");
//	if (xbox_fd < 0){
//		return -1;
//	}
//
//	while (1){
//		len = xbox_map_read(xbox_fd, &map);
//		if (len < 0)
//		{
//			usleep(10 * 1000);
//			continue;
//		}
//
//		printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
//			map.time, map.a, map.b, map.x, map.y, map.lb, map.rb, map.start, map.back, map.home, map.lo, map.ro,
//			map.xx, map.yy, map.lx, map.ly, map.rx, map.ry, map.lt, map.rt);
//		fflush(stdout);
//	} 
//
//	xbox_close(xbox_fd);
//	return 0;
//}
