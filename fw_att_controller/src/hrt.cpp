/*
 * hrt.cpp
 *
 *  Created on: Sep 25, 2014
 *      Author: roman
 */

#include <drivers/drv_hrt.h>
#include <ros/ros.h>

hrt_abstime hrt_absolute_time() {
    struct timeval te;
    te.tv_usec = ros::Time::now().toNSec() / 1e3;
    te.tv_sec = ros::Time::now().toSec();
    hrt_abstime us = (hrt_abstime)te.tv_usec;
    return us;
}

hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then) {
	// not thread safe
    return hrt_absolute_time() - *then;
}
