import PyQt5           as Qt
import PyQt5.QtCore    as QtCore

import time
import numpy as np

## ============================================================================
class Kinematic(QtCore.QObject):
    
    @property
    def efX(self)   : return self._x[0]
    @property
    def efY(self)   : return self._x[1]
    @property
    def efVX(self)  : return self._dx[0]
    @property
    def efVY(self)  : return self._dx[1]
    @property
    def J(self)     : return self._J
    @property
    def time(self)  : return self._time
    @property
    def joints(self): return self._joints
    @property
    def CS(self)    : return self._CS
    @property
    def L(self)     : return self._L
    @property
    def flip(self)  : return self._flip

    ## ------------------------------------------------------------------------
    configChanged = QtCore.pyqtSignal()

    ## ------------------------------------------------------------------------
    def __init__(self, workingArea):

        super(Kinematic, self).__init__()

        self._wa     = workingArea
        self._q      = None
        self._dq     = None
        self._l      = None
        self._x      = None
        self._dx     = None
        self._J      = None
        self._joints = None
        self._CS     = None
        self._time   = None
        self._flip   = np.array([[0,-1],[1,0]])

        self.updateConfig()

    ## ------------------------------------------------------------------------
    def updateConfig(self):

        if len(self._wa.joints) <= 0: return
        if self._joints is None : self._joints = [ self._wa.joints[0] ]
        
        while not self._joints[-1].nj is None: 
            self._joints.append(self._joints[-1].nj)

        self._cosSum = [1]
        self._sinSum = [0]

        for joint in self._joints[:-1]:
            cosSum_ = self._cosSum[-1] * joint.cos - \
                      self._sinSum[-1] * joint.sin
            sinSum_ = self._sinSum[-1] * joint.cos + \
                      self._cosSum[-1] * joint.sin
            self._cosSum.append(cosSum_)
            self._sinSum.append(sinSum_)

        self._cosSum = self._cosSum[1:]
        self._sinSum = self._sinSum[1:]
        self._CS     = np.array([self._cosSum, self._sinSum])

        self._time = time.time()
        self._q    = np.array([j.jointValue for j in self._joints[:-1]])
        self._dq   = np.array([j.velocity if not j.velocity is None else 0.  \
                              for j in self._joints[:-1]])
        self._l    = np.array([j.linkNorm   for j in self._joints[1:]])
        self._L    = np.tri(len(self._joints) - 1) * self._l

        self._x    = np.matmul(self._CS, self._l)
        self._J    = np.matmul(np.matmul(self._flip, self._CS), self._L)
        self._dx   = np.matmul(self._J, self._dq)

        self.configChanged.emit()

## ============================================================================
class Controller(QtCore.QObject):

    ## ------------------------------------------------------------------------
    def __init__(self, workingArea, kinematic):
        super(Controller, self).__init__()

        self._wa          = workingArea
        self._kinematic   = kinematic
        self._joints      = None
        self._constraints = None
        self._constId     = 0
        self._direction   = 1
        self._maxSpeed    = (30 / 180) * np.pi
        self._pause       = False

    ## ------------------------------------------------------------------------
    def update(self):
        
        if self._pause                       or \
           self._wa.joints      is None      or \
           self._wa.constraints is None      or \
           self._kinematic.J    is None      or \
           self._kinematic.CS   is None      or \
           len(self._wa.joints)      == 0    or \
           len(self._wa.constraints) <= 1: return

        if self._joints is None: 
            self._joints = [self._wa.joints[0]]

        if self._constraints is None: 
            self._constraints = [self._wa.constraints[0]]

        while not self._joints[-1].nj is None:
            self._joints.append(self._joints[-1].nj)

        while not self._constraints[-1].nc is None:
            self._constraints.append(self._constraints[-1].nc)

        try:
            self._Ji = np.linalg.pinv(self._kinematic.J)
            JiJ      = np.matmul(self._Ji, self._kinematic.J)
            I        = np.eye(* JiJ.shape)
            self._N  = I - JiJ
            print(self._kinematic.J)
            print(self._Ji)
            print(self._N)
            ## Send a success signal
        except np.linalg.LinAlgError as lae:
            ## Send a failure signal
            return

        self._computeMainConstraint()
        self._computeSecondaryConstraint()

        self._dq    = self._Jidx + self._NdPhi
        self._dqMax = np.abs(self._dq).max()
        #if self._dqMax > self._maxSpeed:
        #    self._dq = self._dq * (self._maxSpeed / self._dqMax)
        self._dq = self._dq * (self._maxSpeed / self._dqMax)

        self._dx = np.matmul(self._kinematic.J, self._dq)
        self._joints[-1].setDX(self._dx)
        for j,dq in zip(self._joints, self._dq): j.setVelocity(dq)

    ## ------------------------------------------------------------------------
    def _computeMainConstraint(self):
        jx = self._joints[-1].x 
        jy = self._joints[-1].y
        cx = self._constraints[self._constId].x
        cy = self._constraints[self._constId].y

        if ((jx - cx) ** 2 + (jy - cy) ** 2) < \
           self._constraints[self._constId].radius ** 2:
            self._constId = self._constId + self._direction
            if self._constId >= len(self._constraints) or self._constId < 0:
                self._direction = self._direction * (-1)
                self._constId   = self._constId + 2 * self._direction

            cx = self._constraints[self._constId].x
            cy = self._constraints[self._constId].y

        c0       = self._constraints[self._constId - self._direction]
        c0c1     = np.array([cx - c0.x, cy - c0.y])
        c0c1     = c0c1 / np.sqrt((c0c1 ** 2).sum())
        c0j      = np.array([jx - c0.x, jy - c0.y])
        c0jDc0c1 = np.dot(c0j, c0c1)
        c0e      = c0jDc0c1 * c0c1
        e        = [ c0.x + c0e[0], c0.y + c0e[1] ]
        je       = np.array([ e[0] - jx, e[1] - jy ])

        self._dx = np.array([cx - jx, cy - jy])
        if (self._dx ** 2).sum() > 100: self._dx = 0.75 * self._dx + 0.25 * je
        dxNorm   = np.sqrt((self._dx ** 2).sum())
        self._dx = 100 * self._dx / dxNorm

        self._Jidx = np.matmul(self._Ji, self._dx)

    ## ------------------------------------------------------------------------
    def _computeSecondaryConstraint(self):

        self._NdPhi     = np.zeros([len(self._joints) - 1], np.float32)
        self._wa.marker = None

        if self._wa.obstacles is None or len(self._wa.obstacles) == 0: return
        
        self._obstacle = self._wa.obstacles[-1]
        ox             = self._obstacle.x
        oy             = self._obstacle.y
        obsCount       = 0

        for jIdx,j in enumerate(self._joints[1:]):
            obsV    = [ ox - j.pj.x, oy - j.pj.y ]
            dotProd = j.refV[0] * obsV[0] + j.refV[1] * obsV[1]

            if dotProd < 0: continue

            l0 = min(j.linkNorm, dotProd)
            x0 = np.array([j.pj.x + j.refV[0] * l0, j.pj.y + j.refV[1] * l0])
            d0 = np.array([x0[0] - ox, x0[1] - oy])
            d  = (d0 ** 2).sum()
            
            if d <= self._obstacle.dm ** 2:
                try: 
                    idx      = jIdx + 1
                    d0Norm   = np.sqrt(d)
                    n0       = d0 / d0Norm
                    L0       = np.copy(self._kinematic.L[:idx,:idx])
                    L0[-1,:] = l0
                    CS0      = self._kinematic.CS[:,:L0.shape[0]] 
                    J0       = np.matmul(
                                np.matmul(self._kinematic.flip, CS0), L0)

                    while J0.shape[1] < len(self._joints) - 1:
                        J0 = np.concatenate(
                                [ J0, np.zeros([J0.shape[0],1], np.float32) ],
                                axis = 1)

                    Jd0    = np.matmul(n0.T, J0).reshape([1,-1])
                    Jd0N   = np.matmul(Jd0, self._N)
                    Jd0N_i = np.linalg.pinv(Jd0N)
                    Jd0Ji  = np.matmul(Jd0, self._Ji)
                    v0     = 75 * ((self._obstacle.dm / d0Norm) ** 2 - 1)

                    self._NdPhi     = self._NdPhi + np.matmul(Jd0N_i, 
                                            (v0 - np.matmul(Jd0Ji, self._dx)))
                    self._wa.marker = x0
                    obsCount        = obsCount + 1

                except np.linalg.LinAlgError as lae:
                    pass
            
        if obsCount > 0:
            self._NdPhi = self._NdPhi / obsCount

    ## ------------------------------------------------------------------------
    def pause(self, setPause):
        self._pause = setPause
