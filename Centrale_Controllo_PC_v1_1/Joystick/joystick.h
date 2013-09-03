/* 
    (C) Copyright 2007,2008, Stephen M. Cameron.

    This file is part of wordwarvi.

    wordwarvi is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    wordwarvi is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with wordwarvi; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */
#include <stdio.h>
#include <linux/joystick.h>

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

struct wwvi_js_event {
  int button[11];
  int stick_x;
  int stick_y;
  int stick_z;

};

extern int open_joystick(const char* path);
extern int read_joystick_event(int *joystick_fd, struct js_event *jse);
extern void close_joystick(int *joystick_fd);
extern int get_joystick_status(int *joystick_fd, struct wwvi_js_event *wjse);

#endif
