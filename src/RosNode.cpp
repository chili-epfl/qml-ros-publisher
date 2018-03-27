/*
 * Copyright (C) 2018 EPFL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 */

/**
 * @file QMLRos.cpp
 * @brief QML wrapper source for RosNode
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#include "RosNode.h"

#include <android/log.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdarg.h>

ros::Publisher samplePublisher;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "RosNode", msg, args);
    va_end(args);
}

RosNode::RosNode(QQuickItem* parent):
    QQuickItem(parent)
{
    status = Status::IDLE;
    ipMaster = "192.168.1.100:11311";
    ipNode = "193.168.1.101";
}

RosNode::~RosNode(){
}

void RosNode::startNode() {
    int argc = 3;
    QByteArray nodeName = QString("ros_node_plugin").toUtf8();
    QByteArray master = QString("__master:=http://" + ipMaster + ":11311").toUtf8();
    QByteArray ip = QString("__ip:=" + ipNode).toUtf8();
    char *argv[3] = { nodeName.data(), master.data(), ip.data() };


    log("Initializing ROS");
    for (int i = 0; i < argc; i++) {
        log(argv[i]);
    }

    ros::init(argc, &argv[0], "android_ndk_native_cpp");

    emit RosNode::statusChanged();

    if (ros::master::check()) {
        log("ROS master found!");
    } else {
        log("No ROS master.");
    }

    log(ros::master::getURI().c_str());

    ros::NodeHandle nodeHandle;

    samplePublisher = nodeHandle.advertise<std_msgs::String>("ros_node_sample_topic", 1000);
}

void RosNode::publish(QString msg) {
    if (!ros::ok()) {
        log("ros::ok() returned false!");
        return;
    }

    QByteArray ba = msg.toUtf8();
    std_msgs::String rosMsg;
    rosMsg.data = ba.data();
    samplePublisher.publish(rosMsg);

    ros::spinOnce();
}
