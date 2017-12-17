#include     <pthread.h>
#include	"rpi_rc_js_in.h"

int XboxJoyStick::xbox_open(char* file_name){
	int fd = open(file_name, O_RDONLY);
	if (fd < 0){
		perror("open");
		return -1;
	}
	return fd;
}

void XboxJoyStick::directMap()
{
	int max = 32767;
	int offset = 800;
	int range = 350;
	channel_data[0] = (map_.lt + max) * range / max * 2 + offset;
	channel_data[1] = (map_.rt + max) * range / max * 2 + offset;
	int threadhold = 10000;
	if (map_.lx > -10000 && map_.lx < threadhold) {
		map_.lx = 0;
	}
	if (map_.ly > -10000 && map_.ly < 10000) {
		map_.ly = 0;
	}
	if (map_.rx > -10000 && map_.rx < 10000) {
		map_.rx = 0;
	}
	if (map_.ry > -10000 && map_.ry < 10000) {
		map_.ry = 0;
	}
	channel_data[2] = (map_.lx + max) * range / max * 2 + offset;
	channel_data[3] = (map_.ly + max) * range / max * 2 + offset;
	channel_data[4] = (map_.rx + max) * range / max * 2 + offset;
	channel_data[5] = (map_.ry + max) * range / max * 2 + offset;
	channel_data[6] = (map_.xx + max) * range / max * 2 + offset;
	channel_data[7] = (map_.yy + max) * range / max * 2 + offset;

	printf("\e7");
	printf("\e[H\e[2K");
	printf("sTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",
	map_.time, map_.a, map_.b, map_.x, map_.y, map_.lb, map_.rb, map_.start, map_.back, map_.home, map_.lo, map_.ro,
	map_.xx, map_.yy, map_.lx, map_.ly, map_.rx, map_.ry, map_.lt, map_.rt);
	fflush(stdout);
	printf("\e8");
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

	directMap();
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
