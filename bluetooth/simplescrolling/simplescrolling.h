/*
 * simplescrolling.h
 * This file is part of Android Simple Scrolling
 *
 * Copyright (C) 2011 - Manuel Di Cerbo
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
#define V_PRESSED 1
#define V_RELEASED 0

#define die(str, args...) do { \
        perror(str); \
        exit(EXIT_FAILURE); \
    } while(0)


void initInput();
void scroll(signed char);
void deinitInput();

int fd;
struct uinput_user_dev uidev;
struct input_event     ev;
