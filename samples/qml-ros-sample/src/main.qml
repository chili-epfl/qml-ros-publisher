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
        ipMaster: masterIpField.text
        ipNode: nodeIpField.text

        onStatusChanged: console.log("Status: " + rosNode.status)
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

                Label{
                    text: "This device's IP address"
                }
                TextField{
                    id: nodeIpField
                    text: "192.168.1.101"
                }

                Button{
                    text: "Run"
                    onClicked: rosNode.startNode()
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
