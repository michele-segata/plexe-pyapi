#!/usr/bin/env python
#
# Copyright (c) 2015-2022 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
import threading
import importlib
from os.path import splitext


TACHOMETER = "img/tachometer.png"
SPEEDOMETER = "img/speedometer.png"
ACCELEROMETER = "img/accelerometer.png"
NEEDLE = "img/needle.png"
CIRCLE = "img/circle.png"
ACCNEEDLE = "img/acc-needle.png"
ACCCIRCLE = "img/acc-circle.png"


class Tachometer:

    def __init__(self, *args):
        self.x = 0
        self.y = 0
        self.w = 100
        self.h = 100
        self.minv = 0.0
        self.maxv = 9000.0
        self.mind = -45.0
        self.maxd = 225.0
        self.v = self.minv
        self.dv = self.minv
        self.upper_limited = False
        self.alpha = 0.01 / (0.1 + 0.01)
        self.tacho = QtGui.QPixmap(TACHOMETER)
        self.needle = QtGui.QPixmap(NEEDLE)
        self.circle = QtGui.QPixmap(CIRCLE)

    def set_pixmaps(self, tacho, needle, circle):
        self.tacho = QtGui.QPixmap(tacho)
        self.needle = QtGui.QPixmap(needle)
        self.circle = QtGui.QPixmap(circle)

    def set_position(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def set_min_max_values(self, minv, maxv):
        self.minv = minv
        self.maxv = maxv

    def set_min_max_degrees(self, mind, maxd):
        self.mind = mind
        self.maxd = maxd

    def set_upper_limited(self, upper_limited):
        self.upper_limited = upper_limited

    def set_value(self, v):
        if v < self.minv:
            v = self.minv
        if v > self.maxv and self.upper_limited:
            v = self.maxv
        self.v = v

    def value_to_degrees(self, v):
        degrees = (v - self.minv) / (self.maxv - self.minv) * \
                  (self.maxd - self.mind) + self.mind
        return degrees

    def paint(self, painter):
        # draw the tachometer
        painter.drawPixmap(self.x, self.y, self.w, self.h, self.tacho)

        # draw the needle, rotated depending on the value
        painter.save()
        painter.translate(self.x + self.w/2, self.y + self.h/2)
        self.dv = self.alpha * self.v + (1-self.alpha)*self.dv
        degrees = self.value_to_degrees(self.dv)
        painter.rotate(degrees)
        painter.translate(-self.x-self.w/2, -self.y-self.h/2)
        painter.drawPixmap(self.x, self.y, self.w, self.h, self.needle)
        painter.restore()

        # draw the top circle
        painter.drawPixmap(self.x, self.y, self.w, self.h, self.circle)


def run_application(application, setter):
    a = importlib.import_module(application)
    a.main(True, True, setter)


class Dashboard(QtWidgets.QWidget):

    def __init__(self, demo_app, parent=None):

        super(Dashboard, self).__init__(parent)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(10)

        self.setWindowTitle(QtCore.QObject.tr(self, "Plexe SUMO Dashboard"))
        self.setStyleSheet("background-color:#222;");
        self.resize(700, 300)

        self.rpm = 0
        self.speed = 0
        self.increasing = +1
        self.sincreasing = +1

        self.rpm_meter = Tachometer()
        self.rpm_meter.set_pixmaps(TACHOMETER, NEEDLE, CIRCLE)
        self.rpm_meter.set_position(0, 0, 300, 300)
        self.rpm_meter.set_min_max_values(0.0, 9000.0)
        self.rpm_meter.set_min_max_degrees(-45.0, 225.0)
        self.rpm_meter.set_value(0.0)

        self.speed_meter = Tachometer()
        self.speed_meter.set_pixmaps(SPEEDOMETER, NEEDLE, CIRCLE)
        self.speed_meter.set_position(400, 0, 300, 300)
        self.speed_meter.set_min_max_values(0.0, 400.0)
        self.speed_meter.set_min_max_degrees(-45.0, 225.0)
        self.speed_meter.set_value(0.0)

        self.acc_meter = Tachometer()
        self.acc_meter.set_pixmaps(ACCELEROMETER, ACCNEEDLE, ACCCIRCLE)
        self.acc_meter.set_position(275, 217, 150, 150)
        self.acc_meter.set_min_max_values(-12.0, 12.0)
        self.acc_meter.set_min_max_degrees(0.0, 180.0)
        self.acc_meter.set_value(0.0)

        self.gear = QtWidgets.QLCDNumber(1, self)
        self.gear.setGeometry(325, 25, 50, 100)
        palette = self.gear.palette()
        palette.setColor(palette.WindowText, QtGui.QColor(180, 0, 0))
        self.gear.setPalette(palette)
        self.gear.setSegmentStyle(QtWidgets.QLCDNumber.Flat)

        self.child = threading.Thread(target=run_application,
                                      args=(demo_app, self.set_values,))
        self.child.setDaemon(True)
        self.child.start()
        self.showNormal()
        self.raise_()
        self.activateWindow()

    def set_values(self, rpm, gear, speed, acceleration):
        self.rpm_meter.set_value(rpm)
        self.speed_meter.set_value(speed * 3.6)
        self.gear.display(gear)
        self.acc_meter.set_value(acceleration)


    def paintEvent(self, event):

        painter = QtGui.QPainter()
        painter.begin(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        self.rpm_meter.paint(painter)
        self.speed_meter.paint(painter)
        self.acc_meter.paint(painter)

        painter.end()

if __name__ == "__main__":
    # show the dashboard and run the demo application
    if len(sys.argv) != 2:
        print("Usage: %s <demo application>" % sys.argv[0])
        sys.exit(1)
    app = QtWidgets.QApplication(sys.argv)
    dashboard = Dashboard(splitext(sys.argv[1])[0])
    dashboard.show()
    sys.exit(app.exec_())

