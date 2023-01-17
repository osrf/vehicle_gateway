import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import "qrc:/qml"
import "qrc:/CompassWidget"
import "qrc:/AttitudeWidget"

RowLayout
{
  Layout.minimumWidth: 400
  Layout.minimumHeight: 250

  AttitudeWidget{}

  CompassWidget{}
}
