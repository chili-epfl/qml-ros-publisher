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
    const QString &getStatus() const { return status; }

    /**
     * @brief Gets the ROS master's IP address
     *
     * @return The ROS master's IP address
     */
    const QString &getMasterIp() const { return masterIp; }

    /**
     * @brief Sets the ROS master's IP address
     *
     * @param The ROS master's IP address
     */
    void setMasterIp(const QString &masterIp) { this->masterIp = masterIp; }

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
     * @brief Publishes a kidnapped event
     *
     * @param TRUE if kidnapped, FALSE if not
     */
    void publishKidnapped(const QString &id, bool kidnapped);

    /**
     * @brief Publishes a long touch event
     *
     * @param The touch key corresponding to the event
     */
    void publishLongTouch(const QString &id, int key);

    /**
     * @brief Publishes a pose
     *
     * @param The message to publish
     */
    void publishPose(const QString &id, float x, float y, float theta);

    /**
     * @brief Publishes a string message
     *
     * @param The message to publish
     */
    void publishString(const QString &id, const QString &text);

    /**
     * @brief Publishes a touch ended event
     *
     * @param The touch key corresponding to the event
     */
    void publishTouchEnd(const QString &id, int key);

    /**
     * @brief Publishes a touch started event
     *
     * @param The touch key corresponding to the event
     */
    void publishTouchStart(const QString &id, int key);

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
