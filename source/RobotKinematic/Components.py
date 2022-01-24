import PyQt5        as Qt
import PyQt5.QtCore as QtCore
import PyQt5.QtGui  as QtGui

import math
import time

## ============================================================================
class Joint(QtCore.QObject):

    @property
    def pj(self)         : return self._pj

    @property
    def nj(self)         : return self._nj

    @nj.setter
    def nj(self, j)      : self._nj = j

    @property
    def refV(self)       : return self._refV

    @property
    def linkNorm(self)   : return self._linkNorm

    @property
    def x(self)          : return self._x

    @property
    def y(self)          : return self._y

    @x.setter
    def x(self, x)       : self._x = x

    @y.setter
    def y(self, y)       : self._y = y

    @property
    def radius(self)     : return self._radius

    @property
    def isHover(self)    : return self._isHover

    @property
    def jointValue(self) : return self._jointValue

    @property
    def velocity(self)   : return self._velocity

    @property
    def sin(self)        : return self._sin

    @property
    def cos(self)        : return self._cos

    ## ------------------------------------------------------------------------
    valueChanged    = QtCore.pyqtSignal(float)
    velocityChanged = QtCore.pyqtSignal(float)
    normChanged     = QtCore.pyqtSignal(float)

    ## ------------------------------------------------------------------------
    def __init__(self, x, y, parentJoint = None, scale = 1, color = "black", 
                 hoverColor = "#ff0000"):

        super(Joint, self).__init__()

        self._pj          = parentJoint
        self._nj          = None
        self._linkNorm    = None
        self._jointValue  = None
        self._velocity    = None
        self._radius      = round(3 * scale)
        self._isHover     = False
        self._x           = x
        self._y           = y
        self._dx          = None
        self._d           = self._radius * 2
        self._color       = QtGui.QColor(color)
        self._hoverColor  = QtGui.QColor(hoverColor)
        self._normalBrush = QtGui.QBrush(self._color)
        self._hoverBrush  = QtGui.QBrush(self._hoverColor)
        self._brush       = self._normalBrush 

        self._updateRefV()

        if not self._pj is None:
            cosTheta,\
            sinTheta    = self._pj.computeAngle(self._refV)
            theta       = math.copysign(math.acos(cosTheta), sinTheta)
            self._pj.nj = self
            self._pj.setJoint(theta)

        self._lastTime = None
        self._timer    = QtCore.QTimer()
        self._timer.timeout.connect(self._updateDynamic)
        self._timer.start(0.1)

    ## ------------------------------------------------------------------------
    def _updateDynamic(self):
        if not self._lastTime is None: 
            dt = time.time() - self._lastTime
            if not self._velocity is None and not self._jointValue is None:
                self.setJoint(self._jointValue + self._velocity * dt)

        self._lastTime = time.time()

    ## ------------------------------------------------------------------------
    def computeAngle(self, refV):
        norm     = math.sqrt(refV[0] ** 2 + refV[1] ** 2)
        refV     = (refV[0] / norm, refV[1] / norm)
        cosTheta = self._refV[0] * refV[0] + self._refV[1] * refV[1]
        sinTheta = math.sqrt(1 - cosTheta ** 2)
        rotX     =   refV[0] * cosTheta + refV[1] * sinTheta
        rotY     =  -refV[0] * sinTheta + refV[1] * cosTheta

        if abs(rotX - self.refV[0]) > 1e-8 or  \
           abs(rotY - self.refV[1]) > 1e-8     :
               sinTheta = -sinTheta

        return cosTheta, sinTheta

    ## ------------------------------------------------------------------------
    def setVelocity(self, v):
        self._velocity = v
        self.velocityChanged.emit(v)

    ## ------------------------------------------------------------------------
    def setDX(self, dx):
        self._dx = dx

    ## ------------------------------------------------------------------------
    def setJoint(self, j):
        while j >= 2 * math.pi: j = j - 2 * math.pi
        while j <  0          : j = j + 2 * math.pi
        if j > math.pi: j = - (2 * math.pi - j)

        if j != self._jointValue:
            if not self._jointValue is None:
                rot = j - self._jointValue
                self.update(ref   = (self._x, self._y),
                            trans = (0, 0),
                            cos   = math.cos(rot), 
                            sin   = math.sin(rot))

            self._jointValue = j
            self._sin        = math.sin(j)
            self._cos        = math.cos(j)

            self.valueChanged.emit(self._jointValue)

    ## ------------------------------------------------------------------------
    def update(self, ref, trans, cos, sin):
        x       = self._x - ref[0]
        y       = self._y - ref[1]
        self._x = ref[0] + trans[0] + x * cos - y * sin
        self._y = ref[1] + trans[1] + x * sin + y * cos
        
        self._updateRefV()

        if not self._nj is None:
            self._nj.update(ref, trans, cos, sin)

    ## ------------------------------------------------------------------------
    def _updateRefV(self):
        if not self._pj is None:
            oldNorm        = self._linkNorm
            self._refV     = (self._x - self._pj.x, self._y - self._pj.y)
            self._linkNorm = math.sqrt(self._refV[0] ** 2 + self._refV[1] ** 2)
            self._refV     = (self._refV[0] / self._linkNorm, 
                              self._refV[1] / self._linkNorm)

            if oldNorm != self._linkNorm: self.normChanged.emit(self._linkNorm)
        else:
            self._refV     = (1,0)
            self._linkNorm = None

    ## ------------------------------------------------------------------------
    def paint(self, painter, theta = 0):
        painter.save()
        painter.setBrush(self._brush)
        painter.drawEllipse(round(self._x - self._radius), 
                            round(self._y - self._radius), 
                            self._d, self._d)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.DotLine)
        painter.setPen(pen)
        painter.drawLine(self._x, self._y, 
                         self._x + 75 * self._refV[0],
                         self._y + 75 * self._refV[1])

        if not self._jointValue is None:
            painter.drawArc(self._x - 25, self._y - 25, 50, 50, 
                            -theta * 180 * 16 / math.pi, 
                            -self._jointValue * 180 * 16 / math.pi)

        if self._nj is None and not self._dx is None:
            brush = QtGui.QBrush(QtGui.QColor("#a00000"))
            pen   = QtGui.QPen(brush, 2, QtCore.Qt.DotLine)
            painter.setPen(pen)
            painter.drawLine(self._x, self._y, 
                             self._x + self._dx[0],
                             self._y + self._dx[1])

            
        painter.restore()

        if not self._nj is None: 
            self._nj.paint(painter, theta + self._jointValue)

    ## ------------------------------------------------------------------------
    def hover(self, hover):
        self._isHover = hover

        if self._isHover:
            self._brush = self._hoverBrush
        else:
            self._brush = self._normalBrush

## ============================================================================
class Link:

    def __init__(self, j0, j1, color, scale = 1):
        self._j0     = j0
        self._j1     = j1
        self._height = int(15 * scale)

        self._color  = QtGui.QColor(color)
        self._brush  = QtGui.QBrush(self._color)

    ## ------------------------------------------------------------------------
    def _compute(self):
        self._width  = math.sqrt((self._j0.x - self._j1.x) ** 2 + \
                                 (self._j0.y - self._j1.y) ** 2   )
        self._offset = self._height 
        self._theta  = math.copysign(
                        math.acos((self._j1.x - self._j0.x) / self._width),
                        self._j1.y - self._j0.y)

    ## ------------------------------------------------------------------------
    def paint(self, painter):
        self._compute()
        painter.save()
        painter.setBrush       (self._brush)
        painter.translate      (self._j0.x, self._j0.y)
        painter.rotate         (self._theta * 180. / math.pi)
        painter.translate      (round(-self._offset / 2), 
                                round(-self._height / 2))
        painter.drawRoundedRect(0, 0,
                                self._width + self._offset, 
                                self._height, 
                                self._offset / 2, self._offset / 2)
        painter.restore()

## ============================================================================
class Constraint(QtCore.QObject):

    @property
    def pc(self)         : return self._pc

    @property
    def nc(self)         : return self._nc

    @nc.setter
    def nc(self, c)      : self._nc = c

    @property
    def x(self)          : return self._x

    @property
    def y(self)          : return self._y

    @x.setter
    def x(self, x)       : self._x = x

    @y.setter
    def y(self, y)       : self._y = y

    @property
    def radius(self)     : return self._radius

    @property
    def isHover(self)    : return self._isHover

    ## ------------------------------------------------------------------------
    valueChanged = QtCore.pyqtSignal(float)
    normChanged  = QtCore.pyqtSignal(float)

    ## ------------------------------------------------------------------------
    def __init__(self, x, y, parentConst = None, scale = 1, color = "#00a000",
                 hoverColor = "#ff0000"):

        super(Constraint, self).__init__()

        self._x           = x
        self._y           = y
        self._pc          = parentConst
        self._nc          = None
        self._radius      = round(3 * scale)
        self._isHover     = False
        self._d           = self._radius * 2
        self._linkColor   = QtGui.QColor("black")
        self._color       = QtGui.QColor(color)
        self._hoverColor  = QtGui.QColor(hoverColor)
        self._normalBrush = QtGui.QBrush(self._color)
        self._hoverBrush  = QtGui.QBrush(self._hoverColor)
        self._brush       = self._normalBrush
        self._pen         = QtGui.QPen()
        self._linkBrush   = QtGui.QBrush(self._linkColor)
        self._linkPen     = QtGui.QPen(self._linkBrush, 2, QtCore.Qt.DotLine)

        if not self._pc is None: self._pc.nc = self

    ## ------------------------------------------------------------------------
    def hover(self, hover):
        self._isHover = hover

        if self._isHover:
            self._brush = self._hoverBrush
        else:
            self._brush = self._normalBrush

    ## ------------------------------------------------------------------------
    def paint(self, painter):
        painter.save()
        
        if not self._nc is None:
            painter.setBrush(self._linkBrush)
            painter.setPen(self._linkPen)
            painter.drawLine(self._x, self._y, self._nc.x, self._nc.y)

        painter.setBrush(self._brush)
        painter.setPen  (self._pen)
        painter.drawEllipse(round(self._x - self._radius), 
                            round(self._y - self._radius), 
                            self._d, self._d)

        painter.restore()

        if not self._nc is None: 
            self._nc.paint(painter)

## ============================================================================
class Obstacle(QtCore.QObject):

    @property
    def x(self)          : return self._x

    @property
    def y(self)          : return self._y

    @property
    def dm(self)         : return self._outD / 2

    @x.setter
    def x(self, x)       : self._x = x

    @y.setter
    def y(self, y)       : self._y = y

    @property
    def radius(self)     : return self._radius

    @property
    def isHover(self)    : return self._isHover

    ## ------------------------------------------------------------------------
    def __init__(self, x, y, scale = 1, color = "#0000a0",
                 hoverColor = "#ff0000"):

        super(Obstacle, self).__init__()

        self._x            = x
        self._y            = y
        self._radius       = 25  * scale
        self._d            = 2   * self._radius
        self._outD         = 2.5 * self._d
        self._color        = QtGui.QColor(color)
        self._hoverColor   = QtGui.QColor(hoverColor)
        self._normalBrush  = QtGui.QBrush(self._color)
        self._hoverBrush   = QtGui.QBrush(self._hoverColor)
        self._brush        = self._normalBrush
        self._pen          = QtGui.QPen()
        self._borderBrush  = QtGui.QBrush(QtGui.QColor("black"))
        self._borderPen    = QtGui.QPen(self._borderBrush, 2, 
                                        QtCore.Qt.DotLine)
           
    ## ------------------------------------------------------------------------
    def hover(self, hover):
        self._isHover = hover

        if self._isHover:
            self._brush = self._hoverBrush
        else:
            self._brush = self._normalBrush

    ## ------------------------------------------------------------------------
    def paint(self, painter):
        painter.save()

        painter.setBrush(self._brush)
        painter.setPen  (self._pen)
        painter.drawEllipse(round(self._x - self._radius),
                            round(self._y - self._radius),
                            self._d, self._d)

        painter.setPen(self._borderPen)
        painter.drawArc(round(self._x - self._outD / 2),
                        round(self._y - self._outD / 2),
                        self._outD, self._outD, 0, 360 * 16)
        painter.restore()

