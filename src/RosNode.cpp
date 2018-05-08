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
#include <stdarg.h>

#include <chili_msgs/Bool.h>
#include <chili_msgs/Float32.h>
#include <chili_msgs/Int32.h>
#include <chili_msgs/String.h>
#include <chili_msgs/Vector2Float32.h>
#include <chili_msgs/Vector3Float32.h>
#include <chili_msgs/Vector2Int32.h>

#include <QNetworkInterface>

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

    nodeHandle.reset(new ros::NodeHandle());

    status = "Running";
    emit RosNode::statusChanged();

    log("Node started");
}

void RosNode::stopNode() {
    publishers.clear();
    delete nodeHandle.release();
    ros::shutdown();
    status = "Idle";

    emit statusChanged();
}

static void fillHeader(chili_msgs::Header &header, const QString &id) {
    header.stamp = ros::Time::now().toNSec();
    header.id = id.toStdString();
}

void RosNode::publish(const QString &topic, const QString &id, bool value) {
    auto publisher = obtainPublisher<chili_msgs::Bool>(topic);

    chili_msgs::Bool msg;
    fillHeader(msg.header, id);
    msg.value = value;
    publisher->publish(msg);
}

void RosNode::publish(const QString &topic, const QString &id, int value) {
    auto publisher = obtainPublisher<chili_msgs::Int32>(topic);

    chili_msgs::Int32 msg;
    fillHeader(msg.header, id);
    msg.value = value;
    publisher->publish(msg);
}

void RosNode::publish(const QString &topic, const QString &id, float value) {
    auto publisher = obtainPublisher<chili_msgs::Float32>(topic);

    chili_msgs::Float32 msg;
    fillHeader(msg.header, id);
    msg.value = value;
    publisher->publish(msg);
}

void RosNode::publish(const QString &topic, const QString &id, int x, int y) {
    auto publisher = obtainPublisher<chili_msgs::Vector2Int32>(topic);

    chili_msgs::Vector2Int32 msg;
    fillHeader(msg.header, id);
    msg.x = x;
    msg.y = y;
    publisher->publish(msg);
}

void RosNode::publish(const QString &topic, const QString &id, const QVector2D &value) {
    auto publisher = obtainPublisher<chili_msgs::Vector2Float32>(topic);

    chili_msgs::Vector2Float32 msg;
    fillHeader(msg.header, id);
    msg.x = value.x();
    msg.y = value.y();
    publisher->publish(msg);
}

void RosNode::publish(const QString &topic, const QString &id, const QVector3D &value) {
    auto publisher = obtainPublisher<chili_msgs::Vector3Float32>(topic);

    chili_msgs::Vector3Float32 msg;
    fillHeader(msg.header, id);
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
    publisher->publish(msg);
}

void RosNode::publish(const QString &topic, const QString &id, const QString &value) {
    auto publisher = obtainPublisher<chili_msgs::String>(topic);

    log("Topic: %s, msg: %s", topic.toStdString().c_str(), value.toStdString().c_str());

    chili_msgs::String msg;
    fillHeader(msg.header, id);
    msg.value = value.toStdString();
    publisher->publish(msg);
}
