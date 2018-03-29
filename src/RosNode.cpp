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
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include <QNetworkInterface>

#include <stdarg.h>

ros::Publisher stringPublisher;
ros::Publisher posePublisher;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "RosNode", msg, args);
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
    masterIp = "192.168.1.100:11311";
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

    QString stringTopic("ros_qml_plugin_" + sanitizedNodeIp + "_string");
    // const std::string topicStdString = topic.toStdString();

    QString poseTopic("ros_qml_plugin_" + sanitizedNodeIp + "_pose");
    // const std::string topicGeometryPose = topic.toStdString();

    stringPublisher = nodeHandle.advertise<std_msgs::String>(stringTopic.toStdString(), 1000);
    posePublisher = nodeHandle.advertise<geometry_msgs::Pose>(poseTopic.toStdString(), 1000);

    status = "Running";
    emit RosNode::statusChanged();
}

void RosNode::stopNode() {
    ros::shutdown();

    status = "Idle";
    emit RosNode::statusChanged();
}

void RosNode::publish(QString msg) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    QByteArray ba = msg.toUtf8();
    std_msgs::String rosMsg;
    rosMsg.data = ba.data();
    stringPublisher.publish(rosMsg);
}

void RosNode::publish(QVector3D position, QQuaternion orientation) {
    if (!ros::ok()) {
        log("Cannot publish: ros::ok() returned false");
        return;
    }

    geometry_msgs::Pose rosMsg;
    rosMsg.position.x = position.x();
    rosMsg.position.y = position.y();
    rosMsg.position.z = position.z();
    rosMsg.orientation.x = orientation.x();
    rosMsg.orientation.y = orientation.y();
    rosMsg.orientation.z = orientation.z();
    rosMsg.orientation.w = orientation.scalar();

    posePublisher.publish(rosMsg);
}
