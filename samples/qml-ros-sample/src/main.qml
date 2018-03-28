import QtQuick 2.2
import QtQuick.Window 2.1
import QtQuick.Controls 1.2

import QMLRos 1.0

ApplicationWindow {
    id: window
    visible: true
    minimumHeight: height
    minimumWidth: width

    RosNode{
        id: rosNode
        masterIp: masterIpField.text

        onStatusChanged: {
            status.text = rosNode.status
            if (rosNode.status == "Idle")
                status.color = "gray";
            else
                status.color = "blue";
        }
    }

    Row{
        GroupBox{
            title: "RosNode"

            Column{
                Label{
                    text: "ROS master IP address"
                }
                TextField{
                    id: masterIpField
                    text: "192.168.1.100"
                }

                Button{
                    text: "Start"
                    onClicked: rosNode.startNode();
                }

                Button{
                    text: "Stop"
                    onClicked: rosNode.stopNode();
                }

                Label{
                    text: "Status:"
                }

                Text{
                    id: status
                    text: "Idle"
                    color: "gray"
                }
            }
        }

        GroupBox{
            title: "Publisher"

            Column{
                Label{
                    text: "Message to publish"
                }

                TextField{
                    id: message
                    text: "Hello from Android"
                }

                Button{
                    text: "Publish"
                    onClicked: rosNode.publish(message.text)
                }
            }
        }
    }
}
