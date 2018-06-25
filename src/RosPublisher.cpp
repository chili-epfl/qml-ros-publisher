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
 * @file RosPublisher.cpp
 * @brief QML wrapper source for RosPublisher
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#include "RosPublisher.h"

#ifdef Q_OS_ANDROID
#include <android/log.h>
#else
#include <cstdio>
#endif
#include <stdarg.h>

#include <chili_msgs/Bool.h>
#include <chili_msgs/Int.h>
#include <chili_msgs/Double.h>
#include <chili_msgs/String.h>
#include <chili_msgs/IntArray.h>
#include <chili_msgs/DoubleArray.h>

#include <QNetworkInterface>

static void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
#ifdef Q_OS_ANDROID
    __android_log_vprint(ANDROID_LOG_INFO, "RosPublisher", msg, args);
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

RosPublisher::RosPublisher(QQuickItem* parent)
: QQuickItem(parent) {
    status = "Idle";
    masterIp = "192.168.1.100";
}

RosPublisher::~RosPublisher() {
    stopNode();
}

void RosPublisher::startNode() {
    if (status == "Running")
        return;

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
    emit RosPublisher::statusChanged();

    log("Node started");
}

void RosPublisher::stopNode() {
    if (status == "Idle")
        return;

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

void RosPublisher::publishBool(const QString &topic, const QString &id, bool value) {
    auto publisher = obtainPublisher<chili_msgs::Bool>(topic);

    chili_msgs::Bool msg;
    fillHeader(msg.header, id);
    msg.value = value;
    publisher->publish(msg);
}

void RosPublisher::publishInt(const QString &topic, const QString &id, int value) {
    auto publisher = obtainPublisher<chili_msgs::Int>(topic);

    chili_msgs::Int msg;
    fillHeader(msg.header, id);
    msg.value = value;
    publisher->publish(msg);
}

void RosPublisher::publishDouble(const QString &topic, const QString &id, double value) {
    auto publisher = obtainPublisher<chili_msgs::Double>(topic);

    chili_msgs::Double msg;
    fillHeader(msg.header, id);
    msg.value = value;
    publisher->publish(msg);
}

void RosPublisher::publishString(const QString &topic, const QString &id, const QString &value) {
    auto publisher = obtainPublisher<chili_msgs::String>(topic);

    log("Topic: %s, msg: %s", topic.toStdString().c_str(), value.toStdString().c_str());

    chili_msgs::String msg;
    fillHeader(msg.header, id);
    msg.value = value.toStdString();
    publisher->publish(msg);
}

void RosPublisher::publishIntArray(const QString &topic, const QString &id, const QVariantList &data) {
    auto publisher = obtainPublisher<chili_msgs::IntArray>(topic);

    chili_msgs::IntArray msg;
    fillHeader(msg.header, id);

    msg.data.reserve(data.size());
    for (auto &value : data) {
        msg.data.push_back(value.toInt());
    }

    publisher->publish(msg);
}

void RosPublisher::publishDoubleArray(const QString &topic, const QString &id, const QVariantList &data) {
    auto publisher = obtainPublisher<chili_msgs::DoubleArray>(topic);

    chili_msgs::DoubleArray msg;
    fillHeader(msg.header, id);

    msg.data.reserve(data.size());
    for (auto &value : data) {
        msg.data.push_back(value.toDouble());
    }

    publisher->publish(msg);
}
