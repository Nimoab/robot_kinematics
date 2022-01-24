import PyQt5           as Qt
import PyQt5.QtCore    as QtCore
import PyQt5.QtGui     as QtGui
import PyQt5.QtWidgets as QtWidgets

import sys
import signal
import time
import collections
import numpy as np
#iimport matplotlib
#import matplotlib.pyplot                  as plt
#import matplotlib.backends.backend_qt5agg as Backend

#matplotlib.rc("text", usetex = True)
#3matplotlib.rc("text.latex", preamble = "\\usepackage{amsmath}")

from .PackageConstant import Const 
from .WorkingArea     import WorkingArea
from .Kinematic       import Kinematic
from .Kinematic       import Controller
from .Chart           import Chart
from .Widgets         import FloatLabel
from .Widgets         import LabelledSlider

class RobotKinematic(QtWidgets.QApplication):
    
    ## ------------------------------------------------------------------------
    def __init__(self):
        super(RobotKinematic, self).__init__(sys.argv)
        self._interrupt      = False
        self._interruptTimer = QtCore.QTimer()
        self._interruptTimer.timeout.connect(self.interruptCheck)
        self._interruptTimer.start(100)

        self._title    = "RobotKinematic"
        self._waWidth  = 500
        self._waHeight = 500
        self._chWidth  = 500
        self._chHeight = 300
        self._left     = 10
        self._top      = 10

        self._initUI()
        self._kinematic = Kinematic(self._workingArea)
        self._workingArea.configChanged.connect(self._kinematic.updateConfig)

        self._lastKinematic    = None
        self._kinematicHistory = { "lastTime" : None}
        self._infoTimer   = QtCore.QTimer()
        self._infoTimer.timeout.connect(self._updateInfo)
        self._infoTimer.start(1000 / 25)

        self._controller = Controller(self._workingArea, self._kinematic)
        self._kinematic.configChanged    .connect(self._controller.update)
        self._workingArea.constraintAdded.connect(self._controller.update)

    ## ------------------------------------------------------------------------
    def _updateInfo(self):

        now = time.time()

        if not self._kinematic.time is None:
            if self._lastKinematic is None or \
               self._lastKinematic < self._kinematic.time:
                kTime               = self._kinematic.time
                self._lastKinematic = kTime
            else:
                kTime = now

            self._chart.addData("FK_x", 
                                self._kinematic.efX, kTime, 
                                "#a00000", dotted = False)
            self._chart.addData("FK_y", 
                                self._kinematic.efY, kTime,
                                "#00a000", dotted = False)
            self._chart.addData("FK_vx", 
                                self._kinematic.efVX, kTime, 
                                "#a000a0", dotted = False)
            self._chart.addData("FK_vy", 
                                self._kinematic.efVY, kTime,
                                "#00a0a0", dotted = False)

            if self._lastKinematic <= self._kinematic.time:
                self._jacobianRank.setValue(
                        np.linalg.matrix_rank(self._kinematic.J))

    ## ------------------------------------------------------------------------
    def _initUI(self):
        ## Working area
        self._workingArea   = WorkingArea(self, self._waWidth, self._waHeight, 
                                          WorkingArea.MODE_ADD, False)
        self._chart         = Chart(self._chWidth, self._chHeight)
        self._workingLayout = QtWidgets.QVBoxLayout()
        self._waLayout      = QtWidgets.QHBoxLayout()
        self._chLayout      = QtWidgets.QHBoxLayout()
        self._waLayout.addWidget(self._workingArea)
        self._waLayout.addStretch()
        self._chLayout.addWidget(self._chart)
        self._chLayout.addStretch()
        self._workingLayout.addLayout(self._waLayout)
        self._workingLayout.addLayout(self._chLayout)
        self._workingLayout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self._waLayout     .setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self._chLayout     .setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self._workingLayout.setSpacing(0)
        self._waLayout     .setSpacing(0)
        self._chLayout     .setSpacing(0)
        self._workingLayout.setContentsMargins(0,0,0,0)
        self._waLayout     .setContentsMargins(0,0,0,5)
        self._chLayout     .setContentsMargins(0,5,0,0)

        ## Buttons
        self._initButtons()

        ## Window
        self._initWindow()

        ## Infos
        self._initInfo()

        ## Layouts
        self._initLayout()

        ## Show
        self._window.setLayout(self._layout)
        self._window.show()

    ## ------------------------------------------------------------------------
    def _initInfo(self):
        self._infoLayout   = QtWidgets.QVBoxLayout()
        self._sliderLayout = QtWidgets.QVBoxLayout()

        self._jacobianRank = FloatLabel(label     = "J Rank: ", 
                                        strFormat = "{:d}",
                                        valueSize = 15)

        self._sliderLayout.setContentsMargins(0,0,0,0)
        self._sliderLayout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self._infoLayout.addLayout(self._sliderLayout)
        self._infoLayout.addStretch()
        self._infoLayout.addWidget(self._jacobianRank)
        self._infoLayout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)

    ## ------------------------------------------------------------------------
    def _initLayout(self):
        self._layout = QtWidgets.QHBoxLayout()
        self._layout.addLayout(self._buttonBarLayout)
        self._layout.addLayout(self._workingLayout)
        self._layout.addLayout(self._infoLayout)

    ## ------------------------------------------------------------------------
    def _initWindow(self):
        self._window = QtWidgets.QWidget()
        self._window.setWindowTitle(self._title)
        self._window.move(10,10)

    ## ------------------------------------------------------------------------
    def _initButtons(self):
        self._grabIcon   = QtGui.QIcon()
        self._lockIcon   = QtGui.QIcon()
        self._unlockIcon = QtGui.QIcon()
        self._playIcon   = QtGui.QIcon()
        self._stopIcon   = QtGui.QIcon()
        self._grabIcon  .addFile(str(Const.ICON_GRAB_MODE_OPEN))
        self._lockIcon  .addFile(str(Const.ICON_LOCK_CLOSE))
        self._unlockIcon.addFile(str(Const.ICON_LOCK_OPEN))
        self._playIcon  .addFile(str(Const.ICON_PLAY))
        self._stopIcon  .addFile(str(Const.ICON_STOP))
        self._addModeBtn        = QtWidgets.QPushButton("+")
        self._addConstraintBtn  = QtWidgets.QPushButton("C")
        self._addObstacleBtn    = QtWidgets.QPushButton("O")
        self._grabModeBtn       = QtWidgets.QPushButton(self._grabIcon  , "")
        self._lockBtn           = QtWidgets.QPushButton(self._unlockIcon, "")
        self._playStopBtn       = QtWidgets.QPushButton(self._stopIcon  , "")
        self._modifBarIcons     = \
            { self._lockBtn     : { True  : self._lockIcon , 
                                    False : self._unlockIcon },
              self._playStopBtn : { True  : self._playIcon, 
                                    False : self._stopIcon } }
        self._buttonCallback   = \
            { self._addModeBtn : lambda: \
                    self._workingArea.setMode(WorkingArea.MODE_ADD),
              self._addConstraintBtn : lambda: \
                    self._workingArea.setMode(WorkingArea.MODE_CONSTRAINT),
              self._addObstacleBtn : lambda: \
                    self._workingArea.setMode(WorkingArea.MODE_OBSTACLE),
              self._grabModeBtn: lambda: \
                    self._workingArea.setMode(WorkingArea.MODE_GRAB_OPEN),
              self._lockBtn    : lambda: \
                    self._workingArea.lock(self._lockBtn.isChecked()),
              self._playStopBtn: self._playStop }

        self._toolBar         = [ self._addModeBtn    , self._addConstraintBtn,
                                  self._addObstacleBtn, self._grabModeBtn ]
        self._modifBar        = [ self._lockBtn, self._playStopBtn ]
        self._buttonBarLayout = QtWidgets.QVBoxLayout()

        for bar,handler in zip([ self._toolBar,self._modifBar],
                               [ self._toolBarHandler,
                                 self._modifBarHandler ]):
            for btn in bar:
                self._setupButton(btn)
                btn.clicked.connect(lambda chk, btn=btn, hdl=handler: hdl(btn))
                self._buttonBarLayout.addWidget(btn)
            self._buttonBarLayout.addSpacing(10)

        self._addModeBtn .setChecked(True)
        self._playStopBtn.setChecked(False)
        self._buttonBarLayout.insertStretch(-1)

    ## ------------------------------------------------------------------------
    def _setupButton(self, btn):
        btn.setMinimumSize(30,30)
        btn.setMaximumSize(30,30)
        btn.setCheckable(True)

    ## ------------------------------------------------------------------------
    def _toolBarHandler(self, button):
        for candidate in self._toolBar:
            candidate.setChecked(button is candidate)
        self._buttonCallback[button]()

    ## ------------------------------------------------------------------------
    def _modifBarHandler(self, button):
        button.setIcon(self._modifBarIcons[button][button.isChecked()])
        self._buttonCallback[button]()

    ## ------------------------------------------------------------------------
    def _playStop(self):
        self._controller.pause(self._playStopBtn.isChecked())

        if self._playStopBtn.isChecked():
            self._workingArea.marker = None
            if not self._workingArea.joints is None and \
               len(self._workingArea.joints) > 0:
                j = self._workingArea.joints[0]
                while not j is None:
                    j.setVelocity(0)
                    j.setDX(None)
                    j = j.nj
        else:
            self._controller.update()

    ## ------------------------------------------------------------------------
    def addSlider(self, joint):
        if joint is None: return
        for idx in range(len(self._workingArea.joints)):
            if joint is self._workingArea.joints[idx]: break
        slider = Slider(joint, idx)
        self._sliderLayout.addWidget(slider)

    ## ------------------------------------------------------------------------
    def interrupt(self):
        self._interrupt = True

    ## ------------------------------------------------------------------------
    def interruptCheck(self):
        if self._interrupt: super(RobotKinematic, self).quit()

## ============================================================================
class Slider(QtWidgets.QWidget):

    ## ------------------------------------------------------------------------
    def __init__(self, joint, jIdx):
        super(Slider, self).__init__()

        self._joint       = joint
        self._nameLabel   = QtWidgets.QLabel("J{}".format(jIdx))
        self._normLabel   = QtWidgets.QLabel()
        self._sliderPos   = LabelledSlider(labelScale = 100, labelSfx = " d",
                                sliderMin = -18000, sliderMax = 18000)
        self._sliderVel   = LabelledSlider(labelScale = 1, labelSfx = " d/s",
                                sliderMin = -180, sliderMax = 180)

        self._nameLabel.setFixedWidth(20)
        self._normLabel.setFixedWidth(45)
        self._normLabel.setText("{: >6.2f}".format(self._joint.nj.linkNorm))
        self._joint.nj.normChanged.connect(
                lambda v: self._normLabel.setText("{: >6.2f}".format(v)))

        self._joint.valueChanged   .connect(self._updatePosValue)
        self._joint.velocityChanged.connect(self._updateVelValue)

        self._layout = QtWidgets.QHBoxLayout()
        self._layout.addWidget(self._nameLabel)
        self._layout.addWidget(self._normLabel)

        self._sliderLayout =QtWidgets.QVBoxLayout()
        self._sliderLayout.addWidget(self._sliderPos)
        self._sliderLayout.addWidget(self._sliderVel)
        self._sliderLayout.setContentsMargins(0,0,0,0)
        self._layout.addLayout(self._sliderLayout)

        if not self._joint.jointValue is None:
            self._updatePosValue(self._joint.jointValue)
        
        self._sliderPos.sliderMoved .connect(self._sliderPosMoved)
        self._sliderPos.valueChanged.connect(self._sliderPosMoved)
        self._sliderVel.sliderMoved .connect(self._sliderVelMoved)
        self._sliderVel.valueChanged.connect(self._sliderVelMoved)

        self.setLayout(self._layout)

    def _sliderPosMoved(self, value):
        self._joint.setJoint(value * np.pi / 18000)

    def _sliderVelMoved(self, value):
        self._joint.setVelocity(value * np.pi / 180)

    def _updatePosValue(self, value):
        self._sliderPos.setValue(value * 18000 / np.pi)
        self._sliderPos.setEnabled(True)
        self._sliderVel.setEnabled(True)

    def _updateVelValue(self, value):
        self._sliderVel.setValue(value * 180 / np.pi)

## ============================================================================

if __name__ == "__main__":
    app = RobotKinematic()

    signal.signal(signal.SIGINT, lambda *args,**kwargs: app.interrupt())

    app.exec()

