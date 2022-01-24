import PyQt5           as Qt
import PyQt5.QtCore    as QtCore
import PyQt5.QtGui     as QtGui
import PyQt5.QtWidgets as QtWidgets

## ============================================================================
class LabelledSlider(QtWidgets.QWidget):

    sliderMoved  = QtCore.pyqtSignal(int)
    valueChanged = QtCore.pyqtSignal(int)

    def __init__(self,
                 labelSize = 0, labelScale = 1, labelPfx = "", labelSfx = "",
                 sfxSize = 0, valueSize = 50,
                 sliderWidth = 200, sliderMin = None, sliderMax = None,
                 sliderVal   = 0):

        super(LabelledSlider, self).__init__()
        self._slider     = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._label      = FloatLabel(label = labelPfx, unit = labelSfx,
                                      labelSize = labelSize,
                                      valueSize = valueSize,
                                      unitSize  = sfxSize)
        self._labelScale = labelScale

        self._slider.valueChanged.connect(
                lambda v: self._label.setValue(v/self._labelScale))

        self._slider.sliderMoved .connect(lambda v: self.sliderMoved .emit(v))
        self._slider.valueChanged.connect(lambda v: self.valueChanged.emit(v))
        self._slider.setContentsMargins(0,0,0,0)
        self.setContentsMargins(0,0,0,0)

        if not sliderWidth is None: self._slider.setFixedWidth(sliderWidth)
        if not sliderMin   is None: self._slider.setMinimum(sliderMin)
        if not sliderMax   is None: self._slider.setMaximum(sliderMax)
        if not sliderVal   is None: self        .setValue  (sliderVal)
        self._slider.setEnabled(False)

        self._layout = QtWidgets.QHBoxLayout()
        self._layout.setContentsMargins(0,0,0,0)
        self._layout.addWidget(self._label)
        self._layout.addWidget(self._slider)
        self._layout.addStretch()
        self.setLayout(self._layout)

    def setValue(self, v):
        self._slider.blockSignals(True)
        self._slider.setValue(v)
        self._label .setValue(v / self._labelScale)
        self._slider.blockSignals(False)

    def setEnabled(self, b):
        self._slider.setEnabled(b)

## ============================================================================
class FloatLabel(QtWidgets.QWidget):
    def __init__(self, label = "", unit = "", strFormat = "{: > 7.2f}",
                 labelSize = None, valueSize = None, unitSize = None):
        super(FloatLabel, self).__init__()
        self._label     = QtWidgets.QLabel()
        self._value     = QtWidgets.QLabel()
        self._unit      = QtWidgets.QLabel()
        self._strFormat = strFormat
        self._labelText = label
        self._unitText  = unit
        self._labelSize = labelSize
        self._valueSize = valueSize
        self._unitSize  = unitSize
        self._first     = True

        self._label.setContentsMargins(0,0,0,0)
        self._value.setContentsMargins(0,0,0,0)
        self._unit .setContentsMargins(0,0,0,0)
        self       .setContentsMargins(0,0,0,0)

        self._layout = QtWidgets.QHBoxLayout()
        self._layout.setSpacing(0)
        self._layout.setContentsMargins(0,0,0,0)
        self._layout.addWidget(self._label)
        self._layout.addWidget(self._value)
        self._layout.addWidget(self._unit)
        self.setLayout(self._layout)
    
    def setValue(self, v):
        if self._first:
            self._label.setText(self._labelText)
            self._unit .setText(self._unitText)
            self._label.setFixedWidth(7 * len(self._labelText))
            self._unit .setFixedWidth(7 * len(self._unitText))
            if not self._labelSize is None: 
                self._label.setFixedWidth(self._labelSize)
            if not self._valueSize is None: 
                self._value.setFixedWidth(self._valueSize)
            if not self._unitSize is None: 
                self._unit.setFixedWidth(self._unitSize)
            self._first = False

        self._value.setText(self._strFormat.format(v))


