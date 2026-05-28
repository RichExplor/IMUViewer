# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore, QtGui

class CompassWidget(QtWidgets.QWidget):
    """自定义电子罗盘组件"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.setFixedSize(90, 90)

    def set_yaw(self, yaw):
        self.yaw = yaw
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        
        center = QtCore.QPointF(self.width()/2, self.height()/2)
        radius = self.width()/2 - 4
        painter.setPen(QtGui.QPen(QtGui.QColor("#2d3139"), 2))
        painter.setBrush(QtGui.QColor("#1e222b"))
        painter.drawEllipse(center, radius, radius)
        
        painter.setPen(QtGui.QColor("#abb2bf"))
        painter.setFont(QtGui.QFont("Segoe UI", 8, QtGui.QFont.Bold))
        painter.drawText(int(self.width()/2 - 4), 14, "N")
        painter.drawText(int(self.width()/2 - 4), int(self.height() - 4), "S")
        painter.drawText(4, int(self.height()/2 + 4), "W")
        painter.drawText(int(self.width() - 14), int(self.height()/2 + 4), "E")

        painter.save()
        painter.translate(center)
        painter.rotate(-self.yaw) 
        
        path_north = QtGui.QPainterPath()
        path_north.moveTo(0, -int(radius) + 12)
        path_north.lineTo(-5, 0)
        path_north.lineTo(5, 0)
        painter.fillPath(path_north, QtGui.QColor("#ff5555"))
        
        path_south = QtGui.QPainterPath()
        path_south.moveTo(0, int(radius) - 12)
        path_south.lineTo(-5, 0)
        path_south.lineTo(5, 0)
        painter.fillPath(path_south, QtGui.QColor("#5555ff"))
        
        painter.restore()