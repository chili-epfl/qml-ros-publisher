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

static void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
#ifdef Q_OS_ANDROID
    __android_log_vprint(ANDROID_LOG_INFO, "RosNode", msg, args);
#else
    vprintf(msg, args);
#endif
    va_end(args);
}

static QString getDeviceIpAddress() {
    QList<QHostAddress> list = QNetworkInterface::allAddresses();

    for(int i = 0; i < list.count(); ++i) {
      if(!list[i].isLoopback()) {
          if (list[i].protocol() == QAbstractSocket::IPv4Protocol)
            return list[i].toString();
      }
    }

    return "";
}

RosNode::RosNode(QQuickItem* parent)
: QQuickItem(parent) {
    status = "Idle";
    masterIp = "192.168.1.100";

    startNode();
}

RosNode::~RosNode() {
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
    char *argv[argc] = { "cellulo_qml_plugin", master.data(), ip.data() };

    log("Initializing ROS");
    for (int i = 0; i < argc; ++i) {
        log("Argument %i: %s", i, argv[i]);
    }

    QString nodeName("cellulo_" + sanitizedNodeIp);
    ros::init(argc, &argv[0], nodeName.toStdString());

    if (ros::master::check()) {
        log("ROS master found");
    } else {
        log("No ROS master");
    }

    log(ros::master::getURI().c_str());

    ros::NodeHandle nodeHandle;

    kidnappedPublisher = nodeHandle.advertise<cellulo_msgs::Kidnapped>("cellulo/kidnapped", 1000);
    longTouchPublisher = nodeHandle.advertise<cellulo_msgs::LongTouch>("cellulo/long_touch", 1000);
    posePublisher = nodeHandle.advertise<cellulo_msgs::Pose>("cellulo/pose", 1000);
    stringPublisher = nodeHandle.advertise<cellulo_msgs::String>("cellulo/string", 1000);
    touchEndPublisher = nodeHandle.advertise<cellulo_msgs::TouchEnd>("cellulo/touch_end", 1000);
    touchStartPublisher = nodeHandle.advertise<cellulo_msgs::TouchStart>("cellulo/touch_start", 1000);

    status = "Running";
    emit RosNode::statusChanged();

    log("Node started");
}

void RosNode::stopNode() {
    ros::shutdown();

    status = "Idle";
    emit RosNode::statusChanged();
}

static void fillHeader(cellulo_msgs::Header &header, const QString &id) {
    header.stamp = ros::Time::now().toNSec();
    header.device_id = id.toStdString();
}

void RosNode::publishKidnapped(const QString &id, bool kidnapped) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    cellulo_msgs::Kidnapped msg;
    fillHeader(msg.header, id);
    msg.kidnapped = kidnapped;

    kidnappedPublisher.publish(msg);
}

void RosNode::publishLongTouch(const QString &id, int key) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    cellulo_msgs::LongTouch msg;
    fillHeader(msg.header, id);
    msg.key = key;

    longTouchPublisher.publish(msg);
}

void RosNode::publishPose(const QString &id, float x, float y, float theta) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    cellulo_msgs::Pose msg;
    fillHeader(msg.header, id);
    msg.position.x = x;
    msg.position.y = y;
    msg.angle = theta;

    posePublisher.publish(msg);
}

void RosNode::publishString(const QString &id, const QString &text) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    QByteArray ba = text.toUtf8();

    cellulo_msgs::String msg;
    fillHeader(msg.header, id);
    msg.text = ba.data();

    stringPublisher.publish(msg);
}

void RosNode::publishTouchEnd(const QString &id, int key) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    cellulo_msgs::TouchEnd msg;
    fillHeader(msg.header, id);
    msg.key = key;

    touchEndPublisher.publish(msg);
}

void RosNode::publishTouchStart(const QString &id, int key) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    cellulo_msgs::TouchStart msg;
    fillHeader(msg.header, id);
    msg.key = key;

    touchStartPublisher.publish(msg);
}
