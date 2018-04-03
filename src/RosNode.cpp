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

#ifdef Q_OS_ANDROID
#include <android/log.h>
#else
#include <cstdio>
#endif

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cellulo_msgs/Kidnapped.h>
#include <cellulo_msgs/LongTouch.h>
#include <cellulo_msgs/Pose.h>
#include <cellulo_msgs/String.h>
#include <cellulo_msgs/TouchEnd.h>
#include <cellulo_msgs/TouchStart.h>

#include <QNetworkInterface>

#include <stdarg.h>

ros::Publisher kidnappedPublisher;
ros::Publisher longTouchPublisher;
ros::Publisher posePublisher;
ros::Publisher stringPublisher;
ros::Publisher touchEndPublisher;
ros::Publisher touchStartPublisher;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
#ifdef Q_OS_ANDROID
    __android_log_vprint(ANDROID_LOG_INFO, "RosNode", msg, args);
#else
    vprintf(msg, args);
#endif
    va_end(args);
}

QString getDeviceIpAddress() {
    QList<QHostAddress> list = QNetworkInterface::allAddresses();

    for(int i = 0; i < list.count(); ++i) {
      if(!list[i].isLoopback()) {
          if (list[i].protocol() == QAbstractSocket::IPv4Protocol)
            return list[i].toString();
      }
    }

    return "";
}

RosNode::RosNode(QQuickItem* parent):
    QQuickItem(parent)
{
    status = "Idle";
    masterIp = "192.168.1.100";

    startNode();
}

RosNode::~RosNode(){
    stopNode();
}

void RosNode::startNode() {
    QString nodeIp = getDeviceIpAddress();
    QString sanitizedNodeIp = QString(nodeIp).replace('.', '_');
    QByteArray tmp = nodeIp.toUtf8();
    log("Node IP: %s", tmp.data());

    int argc = 3;
    QByteArray master = QString("__master:=http://" + masterIp + ":11311").toUtf8();
    QByteArray ip = QString("__ip:=" + nodeIp).toUtf8();
    char *argv[argc] = { "ros_qml_plugin", master.data(), ip.data() };

    log("Initializing ROS");
    for (int i = 0; i < argc; ++i) {
        log("Argument %i: %s", i, argv[i]);
    }

    QString nodeName("ros_qml_plugin_" + sanitizedNodeIp);
    const std::string nodeNameStdString = nodeName.toStdString();
    ros::init(argc, &argv[0], nodeNameStdString);

    if (ros::master::check()) {
        log("ROS master found");
    } else {
        log("No ROS master");
    }

    log(ros::master::getURI().c_str());

    ros::NodeHandle nodeHandle;

    kidnappedPublisher = nodeHandle.advertise<cellulo_msgs::Kidnapped>("cellulo_kidnapped", 1000);
    longTouchPublisher = nodeHandle.advertise<cellulo_msgs::LongTouch>("cellulo_long_touch", 1000);
    posePublisher = nodeHandle.advertise<cellulo_msgs::Pose>("cellulo_pose", 1000);
    stringPublisher = nodeHandle.advertise<cellulo_msgs::String>("cellulo_string", 1000);
    touchEndPublisher = nodeHandle.advertise<cellulo_msgs::TouchEnd>("cellulo_touch_end", 1000);
    touchStartPublisher = nodeHandle.advertise<cellulo_msgs::TouchStart>("cellulo_touch_start", 1000);

    status = "Running";
    emit RosNode::statusChanged();

    log("Node started");
}

void RosNode::stopNode() {
    ros::shutdown();

    status = "Idle";
    emit RosNode::statusChanged();
}

void RosNode::publishPose(QString id, float x, float y, float theta) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    cellulo_msgs::Pose pose;
    pose.header.stamp = ros::Time::now().toNSec();
    pose.header.device_id = id.toStdString();
    pose.position.x = x;
    pose.position.y = y;
    pose.angle = theta;

    posePublisher.publish(pose);
}

void RosNode::publishString(QString id, QString msg) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    QByteArray ba = msg.toUtf8();

    cellulo_msgs::String str;
    str.header.stamp = ros::Time::now().toNSec();
    str.header.device_id = id.toStdString();
    str.msg = ba.data();

    stringPublisher.publish(str);
}
