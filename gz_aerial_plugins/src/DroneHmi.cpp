// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "DroneHmi.hpp"

#include <QQuickImageProvider>
#include <QPainter>

#include <algorithm>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

namespace gz
{
namespace aerial
{
namespace plugins
{
class ImageProvider : public QQuickImageProvider
{
public:
  ImageProvider()
  : QQuickImageProvider(QQuickImageProvider::Image)
  {
  }

public:
  QImage requestImage(
    const QString &, QSize *,
    const QSize &) override
  {
    if (!this->img.isNull()) {
      // Must return a copy
      QImage copy(this->img);
      return copy;
    }

    // Placeholder in case we have no image yet
    QImage i(200, 200, QImage::Format_RGB888);
    i.fill(QColor(128, 128, 128, 100));
    return i;
  }

public:
  void SetImage(const QImage & _image)
  {
    this->img = _image;
  }

private:
  QImage img;
};

class AttitudeDisplayPrivate
{
  /// \brief To provide images for QML.

public:
  ImageProvider * provider{nullptr};
  double roll{0.0};
  double pitch{0.0};
  double offset{0.0};
  double size{200 - 4};

  QBrush bgGround;

  QPen blackPen;
  QPen pitchPen;
  QPen pitchZero;

  QImage * image;
  double angle{0};
};

DroneHmi::DroneHmi()
: gz::gui::Plugin(), dataPtr(new AttitudeDisplayPrivate)
{
}

DroneHmi::~DroneHmi()
{
  gz::gui::App()->Engine()->removeImageProvider("attitudedisplay");
}

void DroneHmi::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (!_pluginElem) {
    return;
  }

  if (this->title.empty()) {
    this->title = "Drone HMI";
  }

  this->dataPtr->bgGround = QBrush(QColor(0, 147, 57));

  this->dataPtr->blackPen.setWidth(2);
  this->dataPtr->pitchZero.setWidth(3);

  this->dataPtr->pitchPen.setColor(Qt::white);
  this->dataPtr->blackPen.setColor(Qt::black);
  this->dataPtr->pitchZero.setColor(Qt::green);

  this->dataPtr->image = new QImage(200, 200, QImage::Format_RGB888);

  this->dataPtr->provider = new ImageProvider();
  gz::gui::App()->Engine()->addImageProvider("attitudedisplay", this->dataPtr->provider);

  std::string topic{"heading"};
  std::string topicPitch{"pitch"};
  std::string topicRoll{"roll"};

  // Subscribe to new topic
  if (!this->node_.Subscribe(
      topic, &DroneHmi::onHeadingReceived,
      this))
  {
    gzerr << "Unable to subscribe to topic [" << topic << "]" << std::endl;
    return;
  }

  if (!this->node_.Subscribe(
      topicPitch, &DroneHmi::onPitchReceived,
      this))
  {
    gzerr << "Unable to subscribe to topic [" << topic << "]" << std::endl;
    return;
  }

  if (!this->node_.Subscribe(
      topicRoll, &DroneHmi::onRollReceived,
      this))
  {
    gzerr << "Unable to subscribe to topic [" << topic << "]" << std::endl;
    return;
  }
}

void DroneHmi::onPitchReceived(const gz::msgs::Float & _msg)
{
  this->dataPtr->pitch = _msg.data();
}

void DroneHmi::onRollReceived(const gz::msgs::Float & _msg)
{
  this->dataPtr->roll = _msg.data();
}

void DroneHmi::onHeadingReceived(const gz::msgs::Float & _msg)
{
  this->SetHeading(_msg.data());
  this->newHeading();
}

void DroneHmi::SetHeading(const double _angle)
{
  this->dataPtr->angle = _angle;
  this->Update();
}

double DroneHmi::Heading()
{
  return this->dataPtr->angle;
}

void DroneHmi::Update()
{
  QPainter painter(this->dataPtr->image);

  painter.setRenderHint(QPainter::Antialiasing);

  painter.translate(200 / 2, 200 / 2);
  painter.rotate(this->dataPtr->roll);

  DrawBackground(painter);
  DrawPitch(painter);
  DrawRoll(painter);

  this->dataPtr->provider->SetImage(*this->dataPtr->image);
  this->newImage();
}

void DroneHmi::DrawBackground(QPainter & painter)
{
  double y_min, y_max;

  y_min = this->dataPtr->size / 2 * -40.0 / 45.0;
  y_max = this->dataPtr->size / 2 * 40.0 / 45.0;

  double y = this->dataPtr->size / 2 * -this->dataPtr->pitch / 45.;
  if (y < y_min) {
    y = y_min;
  }
  if (y > y_max) {
    y = y_max;
  }

  int x = sqrt(this->dataPtr->size * this->dataPtr->size / 4 - y * y);
  qreal gr = atan(y / x);
  gr = gr * 180. / 3.1415926;

  painter.setPen(QPen(Qt::black));
  painter.setBrush(QColor(48, 172, 220));
  painter.drawChord(
    -this->dataPtr->size / 2, -this->dataPtr->size / 2,
    this->dataPtr->size, this->dataPtr->size,
    gr * 16, (180 - 2 * gr) * 16);

  painter.setBrush(this->dataPtr->bgGround);
  painter.drawChord(
    -this->dataPtr->size / 2, -this->dataPtr->size / 2,
    this->dataPtr->size, this->dataPtr->size,
    gr * 16, -(180 + 2 * gr) * 16);
}

void DroneHmi::DrawPitch(QPainter & painter)
{
  // std::lock_guard<std::mutex> lock(mutex);

  // set mask
  QRegion maskRegion(-this->dataPtr->size / 2,
    -this->dataPtr->size / 2,
    this->dataPtr->size,
    this->dataPtr->size,
    QRegion::Ellipse);
  painter.setClipRegion(maskRegion);

  int x, y, x1, y1;
  int textWidth;
  double p, r;
  int ll = this->dataPtr->size / 8, l;

  int fontSize = 8;
  QString s;

  this->dataPtr->pitchPen.setWidth(2);
  painter.setFont(QFont("", fontSize));

  // draw lines
  for (int i = -9; i <= 9; ++i) {
    p = i * 10;
    s = QString("%1").arg(-p);

    if (i % 3 == 0) {
      l = ll;
    } else {
      l = ll / 2;
    }

    if (i == 0) {
      painter.setPen(this->dataPtr->pitchZero);
      l = l * 1.8;
    } else {
      painter.setPen(this->dataPtr->pitchPen);
    }

    y = this->dataPtr->size / 2 * p / 45.0 -
      this->dataPtr->size / 2 * -this->dataPtr->pitch / 45.;
    x = l;

    r = sqrt(x * x + y * y);
    if (r > this->dataPtr->size / 2) {continue;}

    painter.drawLine(QPointF(-l, 1.0 * y), QPointF(l, 1.0 * y));

    textWidth = 100;

    if (i % 3 == 0 && i != 0) {
      painter.setPen(QPen(Qt::white));

      x1 = -x - 2 - textWidth;
      y1 = y - fontSize / 2 - 1;
      painter.drawText(
        QRectF(x1, y1, textWidth, fontSize + 2),
        Qt::AlignRight | Qt::AlignVCenter, s);
    }
  }

  // draw marker
  float markerSize = this->dataPtr->size / 20.0;
  float fx1, fy1, fx2, fy2, fx3, fy3;

  painter.setBrush(QBrush(Qt::red));
  painter.setPen(Qt::NoPen);

  fx1 = markerSize;
  fy1 = 0;
  fx2 = fx1 + markerSize;
  fy2 = -markerSize / 2;
  fx3 = fx1 + markerSize;
  fy3 = markerSize / 2;

  QPointF points[3] = {
    QPointF(fx1, fy1),
    QPointF(fx2, fy2),
    QPointF(fx3, fy3)
  };
  painter.drawPolygon(points, 3);

  QPointF points2[3] = {
    QPointF(-fx1, fy1),
    QPointF(-fx2, fy2),
    QPointF(-fx3, fy3)
  };
  painter.drawPolygon(points2, 3);
}

void DroneHmi::DrawRoll(QPainter & painter)
{
  // std::lock_guard<std::mutex> lock(mutex);

  int nRollLines = 36;
  float rotAng = 360.0 / nRollLines;
  int rollLineLeng = this->dataPtr->size / 25;
  double fx1, fy1, fx2, fy2, fx3, fy3;
  int fontSize = 8;
  QString s;

  this->dataPtr->blackPen.setWidth(1);
  painter.setPen(this->dataPtr->blackPen);
  painter.setFont(QFont("", fontSize));

  for (int i = 0; i < nRollLines; i++) {
    if (i < nRollLines / 2) {
      s = QString("%1").arg(-i * rotAng);
    } else {
      s = QString("%1").arg(360 - i * rotAng);
    }

    fx1 = 0;
    fy1 = -this->dataPtr->size / 2 + this->dataPtr->offset;
    fx2 = 0;

    if (i % 3 == 0) {
      fy2 = fy1 + rollLineLeng;
      painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));

      fy2 = fy1 + rollLineLeng + 2;
      painter.drawText(
        QRectF(-50, fy2, 100, fontSize + 2),
        Qt::AlignCenter, s);
    } else {
      fy2 = fy1 + rollLineLeng / 2;
      painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2));
    }

    painter.rotate(rotAng);
  }

  // draw roll marker
  int rollMarkerSize = this->dataPtr->size / 25;

  painter.rotate(-this->dataPtr->roll);
  painter.setBrush(QBrush(Qt::black));

  fx1 = 0;
  fy1 = -this->dataPtr->size / 2 + this->dataPtr->offset;
  fx2 = fx1 - rollMarkerSize / 2;
  fy2 = fy1 + rollMarkerSize;
  fx3 = fx1 + rollMarkerSize / 2;
  fy3 = fy1 + rollMarkerSize;

  QPointF points[3] = {
    QPointF(fx1, fy1),
    QPointF(fx2, fy2),
    QPointF(fx3, fy3)
  };
  painter.drawPolygon(points, 3);
}

}  // namespace plugins
}  // namespace aerial
}  // namespace gz

// Register this plugin
GZ_ADD_PLUGIN(gz::aerial::plugins::DroneHmi, gz::gui::Plugin)
