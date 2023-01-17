/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

Rectangle {
  id: "imageDisplay"
  color: "transparent"
  anchors.fill: parent
  Layout.minimumWidth: 200
  Layout.minimumHeight: 200

  Connections {
    target: DroneHmi
    onNewImage: image.reload();
  }

  ColumnLayout {
    id: imageDisplayColumn
    anchors.fill: parent
    anchors.margins: 10

    Image {
      id: image
      cache: false
      fillMode: Image.PreserveAspectFit
      Layout.fillHeight: true
      Layout.fillWidth: true
      verticalAlignment: Image.AlignTop
      horizontalAlignment: Image.AlignLeft
      function reload() {
        console.log("update")
        // Force image request to C++
        source = "image://attitudedisplay/" + Math.random().toString(36).substr(2, 5);
      }
    }
  }
}
