import PyQt5           as Qt
import PyQt5.QtCore    as QtCore
import PyQt5.QtGui     as QtGui
import PyQt5.QtWidgets as QtWidgets

import random
import math
import itertools

from .Components import Joint
from .Components import Link
from .Components import Constraint
from .Components import Obstacle

class WorkingArea(QtWidgets.QLabel):

    @property
    def joints(self)      : return self._joints
    @property
    def constraints(self) : return self._constraints
    @property
    def obstacles(self)   : return self._obstacles
    @property
    def marker(self)      : return None
    @marker.setter
    def marker(self, m):
        if m is None: self._marker = []
        else:         self._marker.append(m)
        self.update()

    MODE_ADD        = 0
    MODE_CONSTRAINT = 1
    MODE_OBSTACLE   = 2
    MODE_GRAB_OPEN  = 3
    MODE_GRAB_CLOSE = 4
    CURSORS         = {}
    COLORS          = [ "#80ff0000" , "#8000ff00" , "#800000ff" ,
                        "#80ffff00" , "#80ff00ff" , "#8000ffff" ]
    ## ------------------------------------------------------------------------
    configChanged   = QtCore.pyqtSignal()
    constraintAdded = QtCore.pyqtSignal()
    obstacleAdded   = QtCore.pyqtSignal()

    ## ------------------------------------------------------------------------
    def __init__(self, app, width, height, mode, lock):
        super(WorkingArea, self).__init__()

        self._app           = app
        self._selected      = None
        self._marker        = []
        self._joints        = []
        self._links         = []
        self._constraints   = []
        self._obstacles     = []
        self._width         = width
        self._height        = height
        self._vStep         = 20
        self._hStep         = 20
        self._bgColor       = QtGui.QColor("white")
        self._gridColor     = QtGui.QColor("black")
        
        self.setFixedSize(self._width, self._height)
        
        self._mousePressCallback = { 
                WorkingArea.MODE_ADD        : self._mousePressAddMode,
                WorkingArea.MODE_CONSTRAINT : self._mousePressConstraintMode,
                WorkingArea.MODE_OBSTACLE   : self._mousePressObstacleMode,
                WorkingArea.MODE_GRAB_OPEN  : self._mousePressGrabMode }
        self._mouseMoveCallback = { 
                WorkingArea.MODE_ADD        : self._mouseMoveDefault,
                WorkingArea.MODE_CONSTRAINT : self._mouseMoveDefault,
                WorkingArea.MODE_OBSTACLE   : self._mouseMoveDefault,
                WorkingArea.MODE_GRAB_OPEN  : self._mouseMoveDefault,
                WorkingArea.MODE_GRAB_CLOSE : self._mouseMoveGrabClose}

        pixmap = QtGui.QPixmap(width, height)
        
        self.setPixmap(pixmap)
        self.setCursor(QtCore.Qt.CrossCursor)
        self.setMouseTracking(True)

        self.setMode(mode)
        self.lock(lock)
        self._updateBackground()
        self.update()

    ## ------------------------------------------------------------------------
    def _configChanged(self):
        self.configChanged.emit()
        self.update()

    ## ------------------------------------------------------------------------
    def setMode(self, mode):
        self._mode = mode
        self.setCursor(WorkingArea.CURSORS[self._mode])

    ## ------------------------------------------------------------------------
    def lock(self, lockStatus):
        self._lock = lockStatus

    ## ------------------------------------------------------------------------
    def _updateBackground(self):
        self._bgImage = QtGui.QImage(self._width, self._height,
                                     QtGui.QImage.Format_RGB32)
        self._bgImage.fill(self._bgColor)
        for x in range(1, self._hStep):
            xPxl = x * self._width / self._hStep
            for y in range(1, self._vStep):
                yPxl = y * self._height / self._vStep
                self._bgImage.setPixelColor(xPxl, yPxl, self._gridColor)

    ## ------------------------------------------------------------------------
    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self.pixmap())
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        ## Draw the background
        painter.drawImage(0, 0, self._bgImage)

        ## Draw the obstacles
        for o in self._obstacles: o.paint(painter)

        ## Draw the constraints
        if len(self._constraints) > 0:
            self._constraints[0].paint(painter)

        ## Draw the links
        for link  in self._links : link .paint(painter)

        ## Draw the joints
        if len(self._joints) > 0:
            self._joints[0].paint(painter)

        ## Draw the marker
        for m in self._marker:
            pen = QtGui.QPen()
            pen.setWidth(2)
            painter.setPen(pen)
            painter.drawLine(m[0] - 5, m[1] - 5, m[0] + 5, m[1] + 5)
            painter.drawLine(m[0] - 5, m[1] + 5, m[0] + 5, m[1] - 5)

        super(WorkingArea, self).paintEvent(event)

    ## ------------------------------------------------------------------------
    def mousePressEvent(self, event): 
        for component in itertools.chain(self._joints, 
                                         self._constraints, 
                                         self._obstacles):
            if component.isHover:
                self._selected = component
                break

        self._mousePressCallback[self._mode](event)
        super(WorkingArea, self).mousePressEvent(event)

    ## ------------------------------------------------------------------------
    def mouseReleaseEvent(self, event):  
        if self._mode == WorkingArea.MODE_GRAB_CLOSE:
            self._mouseReleaseGrabMode(event)

        self._selected = None
        super(WorkingArea, self).mouseReleaseEvent(event)

    ## ------------------------------------------------------------------------
    def mouseMoveEvent(self, event):
        self._mouseMoveCallback[self._mode](event)
        super(WorkingArea, self).mouseMoveEvent(event)

    ## ------------------------------------------------------------------------
    def _mousePressAddMode(self, event):
        if not self._selected is None: return

        x = event.x()
        y = event.y()

        if len(self._joints) == 0:
            x,y = self._withinBound(x, y)
            pj  = None
        else:
            pj = self._joints[-1]
            if self._joints[-1].x == x and self._joints[-1].y == y:
                return

        self._joints.append(Joint(x, y, pj))
        self._app.addSlider(pj)
        self._joints[-1].valueChanged.connect(self._configChanged)

        if len(self._joints) > 1:
            self._links.append(
                Link(self._joints[-2], self._joints[-1],
                     WorkingArea.COLORS[len(self._links) % \
                                        len(WorkingArea.COLORS)]))
        self.update()

    ## ------------------------------------------------------------------------
    def _mousePressConstraintMode(self, event):
        if not self._selected is None: return
        
        x = event.x()
        y = event.y()

        if len(self._constraints) == 0:
            pc  = None
        else:
            pc = self._constraints[-1]
            if self._constraints[-1].x == x and self._constraints[-1].y == y:
                return

        self._constraints.append(Constraint(x, y, pc))
        self.constraintAdded.emit()
        self.update()

    ## ------------------------------------------------------------------------
    def _mousePressObstacleMode(self, event):
        if not self._selected is None: return
        
        x = event.x()
        y = event.y()

        self._obstacles.append(Obstacle(x, y))
        self.obstacleAdded.emit()
        self.update()

    ## ------------------------------------------------------------------------
    def _mousePressGrabMode(self, event):
        self.setMode(WorkingArea.MODE_GRAB_CLOSE) 

    ## ------------------------------------------------------------------------
    def _mouseReleaseGrabMode(self, event):
        self.setMode(WorkingArea.MODE_GRAB_OPEN)

    ## ------------------------------------------------------------------------
    def _mouseMoveDefault(self, event):
        x      = event.x()
        y      = event.y()
        oneSet = False

        for component in itertools.chain(self._joints, 
                                         self._constraints,
                                         self._obstacles):
            d       = (x - component.x) ** 2 + (y - component.y) ** 2
            isHover = d <= max(component.radius ** 2, 100)
            component.hover((not oneSet) and isHover)
            oneSet = oneSet or isHover

        self.update()

    ## ------------------------------------------------------------------------
    def _mouseMoveGrabClose(self, event):
        if not self._selected is None:
            x = event.x()
            y = event.y()

            if   isinstance(self._selected, Joint):
                self._moveJoint(x, y, self._selected)
            elif isinstance(self._selected, Constraint):
                self._moveConstraint(x, y, self._selected)
            elif isinstance(self._selected, Obstacle):
                self._moveObstacle(x, y, self._selected)

    ## ------------------------------------------------------------------------
    def _moveConstraint(self, x, y, constraint):
        constraint.x = x
        constraint.y = y
        self.update()

    ## ------------------------------------------------------------------------
    def _moveObstacle(self, x, y, obstacle):
        obstacle.x = x
        obstacle.y = y
        self.update()

    ## ------------------------------------------------------------------------
    def _moveJoint(self, x, y, joint):
        if joint is self._joints[0]:
            x,y = self._withinBound(x,y)
            joint.update(ref = (0,0), trans = (x - joint.x, y - joint.y),
                         cos = 1, sin = 0)
        else:
            newVec  = (x - joint.pj.x, y - joint.pj.y)
            if newVec[0] == 0 and newVec[1] == 0: return
            cos,sin = joint.pj.computeAngle(newVec)
            theta   = math.copysign(math.acos(cos), sin)
            joint.pj.setJoint(theta)
                
            if not self._lock:
                transNorm = math.sqrt(newVec[0] ** 2 + newVec[1] ** 2) - \
                            joint.linkNorm
                joint.update(ref   = (0,0),
                             trans = (joint.refV[0] * transNorm, 
                                      joint.refV[1] * transNorm),
                             cos   = 1, sin = 0)

        self._configChanged()
            
    ## ------------------------------------------------------------------------
    def _withinBound(self, x, y):
        x = int(max(1, min(self._hStep - 1,
                           round(self._hStep * x / self._width))) * \
                self._width / self._hStep)
        y = int(max(1, min(self._vStep - 1, 
                           round(self._vStep * y / self._height))) * \
                self._height / self._vStep)

        return x,y

WorkingArea.CURSORS[WorkingArea.MODE_ADD       ] = QtCore.Qt.CrossCursor
WorkingArea.CURSORS[WorkingArea.MODE_CONSTRAINT] = QtCore.Qt.CrossCursor
WorkingArea.CURSORS[WorkingArea.MODE_OBSTACLE  ] = QtCore.Qt.CrossCursor
WorkingArea.CURSORS[WorkingArea.MODE_GRAB_OPEN ] = QtCore.Qt.OpenHandCursor
WorkingArea.CURSORS[WorkingArea.MODE_GRAB_CLOSE] = QtCore.Qt.ClosedHandCursor

