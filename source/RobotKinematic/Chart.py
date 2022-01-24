import PyQt5           as Qt
import PyQt5.QtCore    as QtCore
import PyQt5.QtGui     as QtGui
import PyQt5.QtWidgets as QtWidgets

import numpy as np
import collections
import time

from .Widgets import LabelledSlider

class Chart(QtWidgets.QWidget):

    def __init__(self, width, height, timeSpan = 10, sampling = 1/100):

        super(Chart, self).__init__()

        self._width       = width
        self._height      = height
        self._timeSpan    = timeSpan
        self._sampling    = sampling
        self._data        = collections.OrderedDict()
        self._scale       = 1
        self._O           = np.array([10, int(self._height / 2)])
        self._min         = np.array([ 0, -int(self._height / 2) ])
        self._max         = np.array([ self._width - self._O[0],  
                                       int(self._height / 2) ])
        self._gridStep    = 25
        self._bgColor     = QtGui.QColor("white")
        self._gridColor   = QtGui.QColor("black")
        self._drawingArea = QtWidgets.QLabel()
        self._drawingArea.setPixmap(QtGui.QPixmap(width, height))
        self._drawingArea.setFixedSize(self._width, self._height)

        self._scaleSlider  = LabelledSlider(labelSize = 50, labelPfx = "Scale",
                                            sliderMin = 0, sliderMax = 100,
                                            sliderVal = 100)
        self._scaleSlider.setEnabled(True)

        self._scaleSlider.sliderMoved .connect(self._setScale)
        self._scaleSlider.valueChanged.connect(self._setScale)
        self._sliderLayout = QtWidgets.QHBoxLayout()
        self._sliderLayout.addWidget(self._scaleSlider)

        self._layout       = QtWidgets.QVBoxLayout()
        self._contLayout   = QtWidgets.QVBoxLayout()
        self._boxLayout    = QtWidgets.QGridLayout()
        self._contLayout.addLayout(self._boxLayout)
        self._contLayout.addStretch()
        self._contLayout.addLayout(self._sliderLayout)

        self._layout.addWidget(self._drawingArea)
        self._layout.addLayout(self._contLayout)
        self._layout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self._layout.setContentsMargins(0,0,0,0)
        self.setLayout(self._layout)

        self._updater = QtCore.QTimer()
        self._updater.timeout.connect(self.update)
        self._updater.start(1000 / 25)

    ## ------------------------------------------------------------------------
    def _setScale(self, v):
        self._scale = v / 100

    ## ------------------------------------------------------------------------
    def _updateBackground(self, painter):
        painter.save()
        painter.fillRect(0, 0, self._width, self._height, self._bgColor)

        x = self._gridStep
        while x < self._max[0]:
            y = self._gridStep
            while y < self._max[1]:
                self._addPoint(x,  y, painter)
                self._addPoint(x, -y, painter)
                y = y + self._gridStep
            x = x + self._gridStep

        self._addLine(0, self._min[1], 0, self._max[1], painter)
        self._addLine(self._min[0], 0, self._max[0], 0, painter)

        painter.restore()

    ## ------------------------------------------------------------------------
    def update(self):

        painter = QtGui.QPainter()
        painter.begin(self._drawingArea.pixmap())
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        self._updateBackground(painter)

        now = time.time()
        t0  = None
        for k in self._data:
            if not self._data[k]["show"]: continue
            self._cleanData(now, self._data[k]["v"], self._data[k]["t"])
            if len(self._data[k]["t"]) <= 0: continue
            t0 = self._data[k]["t"][0] \
                 if t0 is None else min(t0, self._data[k]["t"][0])

        for k in self._data:
            if not self._data[k]["show"] or \
               len(self._data[k]["t"]) <= 0: continue
            painterPath = QtGui.QPainterPath()
            t = [ (t - t0) / self._timeSpan * self._max[0] for t in \
                  self._data[k]["t"] ]
            v = [ v * self._scale for v in self._data[k]["v"] ]
            self._addPath(t, v, painterPath)
            painter.setPen(self._data[k]["pen"])
            painter.drawPath(painterPath)

        painter.end()

        super(Chart, self).update()

    ## ------------------------------------------------------------------------
    def _addPoint(self, x, y, painter):
        painter.drawPoint(* self._toQtCoord(x, y))

    ## ------------------------------------------------------------------------
    def _addLine(self, x0, y0, x1, y1, painter):
        painter.drawLine(* self._toQtCoord(x0, y0),
                         * self._toQtCoord(x1, y1))

    ## ------------------------------------------------------------------------
    def _addPath(self, x, y, painter):
        painter.moveTo(* self._toQtCoord(x[0], y[0]))

        for idx,(x_,y_) in enumerate(zip(x[1:],y[1:])):
            painter.lineTo(* self._toQtCoord(x_, y_))

    ## ------------------------------------------------------------------------
    def _toQtCoord(self, x, y):
        return self._O[0] + x, self._O[1] - y

    ## ------------------------------------------------------------------------
    def addData(self, label, v, t, color = "black", dotted = False, width = 4):
        if not label in self._data:
            pen = QtGui.QPen(
                        QtGui.QColor(color), width, 
                        QtCore.Qt.DotLine if dotted else QtCore.Qt.SolidLine)
            self._data[label] = { "t"     : collections.deque(),
                                  "v"     : collections.deque(),
                                  "pen"   : pen,
                                  "show"  : True }
            box  = QtWidgets.QCheckBox(label)
            cols = int(self._width * 0.9 / 4)
            row  = int(len(self._data) / cols)
            col  = len(self._data) - (row * cols) - 1
            box.setChecked(True)
            box.toggled.connect(lambda c,k=label: self.showData(k,c) )
            self._boxLayout.addWidget(box, row, col)

        if len(self._data[label]["t"]) > 2 and \
           self._data[label]["t"][-1] - \
           self._data[label]["t"][-2] < self._sampling:
            self._data[label]["t"].pop()
            self._data[label]["v"].pop()

        self._data[label]["t"].append(t)
        self._data[label]["v"].append(v)

    ## ------------------------------------------------------------------------
    def showData(self, label, show):
        self._data[label]["show"] = show

    ## ------------------------------------------------------------------------
    def _cleanData(self, now, v, t):
        while len(t) > 0 and now - t[0] > self._timeSpan:
            t.popleft()
            v.popleft()
