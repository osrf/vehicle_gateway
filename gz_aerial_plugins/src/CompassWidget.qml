/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import "qrc:/qml"

Rectangle
{
  id: compassWidget
  width: 150
  height: 150

  Canvas
  {
    id: canvas
    anchors.fill: parent

    property double angle_: 0;
    property double margins_: 10.0;
    property variant pointText_:  ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

    Connections {
      target: DroneHmi
      onNewHeading: {
        canvas.angle_ = DroneHmi.heading
        canvas.requestPaint();
      }
    }

    onPaint : {
      var ctx = getContext("2d");
      ctx.reset()
      ctx.translate(compassWidget.width / 2, compassWidget.height / 2);
      var scaleCompass = Math.min((compassWidget.width  - margins_)/120.0,
                                  (compassWidget.height - margins_)/120.0);
      ctx.scale(scale, scale);

      var i = 0;
      var j = 0;
      while(i < 360)
      {
        if(i % 45 == 0)
        {
          ctx.beginPath();
          ctx.fillStyle = "black";
          ctx.fillText(qsTr(pointText_[j]), -5, -52);
          ctx.moveTo(0, -40);
          ctx.lineTo(0, -50);
          ctx.moveTo(0, -40);
          ctx.stroke();
          j++;
        }
        else
        {
          ctx.beginPath();
          ctx.moveTo(0, -45);
          ctx.lineTo(0, -50);
          ctx.stroke();
        }
        ctx.rotate(15 * 3.1416 / 180);
        i += 15;
      }

      ctx.lineWidth = 4
      ctx.strokeStyle = "black"
      ctx.fillStyle = "red"
      ctx.rotate(canvas.angle_);
      ctx.beginPath();
      ctx.moveTo(-10, 0);
      ctx.lineTo(0, -45);
      ctx.lineTo(10, 0);
      ctx.lineTo(0, -15);
      ctx.lineTo(-10, 0);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
  }
}
