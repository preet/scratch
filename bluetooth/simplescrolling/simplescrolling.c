/*
 * simplescrolling.c
 * This file is part of Android Simple Scrolling
 *
 * Copyright (C) 2011 - Manuel Di Cerbo, Nexus-Computing GmbH
 *
 * Android Simple Scrolling is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Android Simple Scrolling is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Android Simple Scrolling; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, 
 * Boston, MA  02110-1301  USA
 */


#include <linux/input.h>
#include <linux/uinput.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>

#include "simplescrolling.h"


// CREDITS to http://thiemonge.org/getting-started-with-uinput
// Great Tutorial on using uinput, thanks a lot!

/*
main (int argc, char *argv[]){
	unsigned char i = 0;
	initInput();
	sleep(2);
	scroll(-1);
	sleep(1);
	scroll(1);
	deinitInput();
	return 0;
}
*/

void deinitInput(){
	if(ioctl(fd, UI_DEV_DESTROY) < 0)
		die("error: ioctl");
	close(fd);
}

void scroll(signed char val){
	memset(&ev, 0, sizeof(struct input_event));
	ev.type = EV_REL;
	ev.code = REL_WHEEL;
	ev.value = val;
	if(write(fd, &ev, sizeof(struct input_event)) < 0)
		die("error: write");
		
	memset(&ev, 0, sizeof(struct input_event));
	ev.type = EV_SYN;
	ev.code = SYN_REPORT;
	ev.value = 0;
	if(write(fd, &ev, sizeof(struct input_event)) < 0)
		die("error: write");
}

void initInput(){
	fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if(fd < 0)
		die("error: open");

		
	ioctl(fd, UI_SET_EVBIT, EV_KEY);
	ioctl(fd, UI_SET_EVBIT, EV_REL);
	ioctl(fd, UI_SET_EVBIT, EV_SYN);

	ioctl(fd, UI_SET_RELBIT, REL_X);
	ioctl(fd, UI_SET_RELBIT, REL_Y);
	ioctl(fd, UI_SET_RELBIT, REL_WHEEL);
	
	ioctl(fd, UI_SET_KEYBIT, BTN_LEFT);
	ioctl(fd, UI_SET_KEYBIT, BTN_RIGHT);
	ioctl(fd, UI_SET_KEYBIT, BTN_MIDDLE);
	

	memset(&uidev, 0, sizeof(uidev));
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "uinput-nexus");
	uidev.id.bustype = BUS_USB;
	uidev.id.vendor  = 0x1;
	uidev.id.product = 0x1; 
	uidev.id.version = 1;

	if(write(fd, &uidev, sizeof(uidev)) < 0)
		die("error: write");

	if(ioctl(fd, UI_DEV_CREATE) < 0)
		die("error: ioctl");
}
