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
 * @file QMLRos.h
 * @brief QML wrapper header for RosNode
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#ifndef ROSNODE_H
#define ROSNODE_H

#include <QQuaternion>
#include <QQuickItem>

class RosNode : public QQuickItem {
    /* *INDENT-OFF* */
    Q_OBJECT
    /* *INDENT-ON* */

    Q_PROPERTY(QString status READ getStatus NOTIFY statusChanged)
    Q_PROPERTY(QString masterIp READ getMasterIp WRITE setMasterIp NOTIFY masterIpChanged)

public:
    /**
     * @brief Creates a new RosNode with the given QML parent
     *
     * @param parent The QML parent
     */
    RosNode(QQuickItem* parent = 0);

    /**
     * @brief Destroys this RosNode
     */
    ~RosNode();

    /**
     * @brief Gets this ROS node's status
     *
     * @return This ROS node's status
     */
    QString getStatus() const { return status; }

    /**
     * @brief Gets the ROS master's IP address
     *
     * @return The ROS master's IP address
     */
    QString getMasterIp() const { return masterIp; }

    /**
     * @brief Sets the ROS master's IP address
     *
     * @param The ROS master's IP address
     */
    void setMasterIp(QString masterIp) { this->masterIp = masterIp; }

public slots:

    /**
     * @brief Initializes the ROS node
     */
    void startNode();

    /**
     * @brief Kills the ROS node
     */
    void stopNode();

    /**
     * @brief Publishes a string message
     *
     * @param The message to publish
     */
    void publishString(QString id, QString msg);

    /**
     * @brief Publishes a pose
     *
     * @param The message to publish
     */
    void publishPose(QString id, float x, float y, float theta);

signals:
    /**
     * @brief Emitted when this ROS node's status changes
     */
    void statusChanged();

    /**
     * @brief Emitted when the ROS master's IP address changes
     */
    void masterIpChanged();

private:
    QString status;                   ///< Status of this ROS node
    QString masterIp;                 ///< IP address of ROS master
};

#endif /* ROSNODE_H */
