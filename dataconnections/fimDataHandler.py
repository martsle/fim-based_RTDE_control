from abc import abstractmethod
from time import time
from typing import Tuple, Union, List, TYPE_CHECKING
import ifcopenshell as ifc
import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from neo4j import GraphDatabase, Driver, Session
from neo4j.graph import Node
from multiprocessing import Queue
import uuid
import string
import logging

from fimkin import Waypoint

if TYPE_CHECKING:
    from fimkin import Screwkincalc

chars = string.digits + string.ascii_uppercase + string.ascii_lowercase + "_$"

class Layer():
    def __init__(self, lId, ifcData, pd:'PathData', registerSelf: bool = True) -> None:
        #region protected
        self._pathData: PathData = pd
        
        self._id: int = lId
        self._ifcData = ifcData
        self._hasTransition: bool = False
        #self._layerTimeActual: float = 0 # time to print layer in seconds
        #self._transitionCurve: int = 0 # index of transition
        self._topLayer: bool = True
        
        self._path: List[Curve] = []
        self._surface: List[Surface] = []

        self._pathDiscretized: List[Waypoint] = []
        
        if registerSelf:
            pd.addLayer(self)
        #endregion
    
    #region Properties
    @property
    def scalingFactor(self) -> float:
        return self._pathData.scalingFactor

    @property
    def maxAccel(self) -> float:
        return self._pathData.maxAcceleration # maximum normal Acceleration (due to curvature) in m/s^2
    
    @property
    def maxRobotVelocity(self) -> float:
        return self._pathData.maxRobotVelocity # maximum robot velocity in m/s

    @property
    def transitionAcceleration(self) -> float:
        return self._pathData.transitionAcceleration

    @property
    def robotAcceleration(self) -> float:
        return self._pathData.maxRobotAcceleration # maximum robot velocity in m/s
    
    @property
    def _layerTimeTarget(self) -> float:
        return self._pathData.layerTime # time to print layer in seconds

    @property
    def transitionPosition(self) -> float:
        return self._pathData.transitionPosition
    
    @property
    def transitionWidth(self) -> float:
        return self._pathData.transitionWidth

    @property
    def _transitionIndex(self) -> int:
        return self._pathData._transitionIDX

    @property
    def isClosed(self) -> bool:
        return len(self._path) > 0 and np.all(self._path[-1]._endPt - self._path[0]._startPt < 10E-4)

    @property
    def length(self) -> float:
        l = 0
        for c in self._path:
            l += c.length
        return l
    
    # @property
    # def accelerationLimit(self) -> float:
    #     return self.maxAccel
    
    @property
    def targetMeanVelocity(self) -> float:
        return self.length / self._layerTimeTarget
    
    @property
    def isPlanar(self) -> bool:
        for surf in self._surface:
            if not surf.isPlanar:
                return False
        return True
    
    @property
    def isHorizontal(self) -> bool:
        for surf in self._surface:
            if not surf.isHorizontal:
                return False
        return True
    
    @property
    def layerHeight(self) -> Union[None, float]:
        if self.isHorizontal and self.isPlanar:
            if self._id == 0:
                return self._surface[0]._origin[2,0]
            else:
                Q = self._surface[0]._origin[2]
                return Q / (self._id + 1)
                # TODO: implement this
                # Q = self._surface[0]._origin.reshape((3,))
                # otherSurf = self._registeredLayers[self._id - 1]._surface[0]
                # P = otherSurf._origin.reshape((3,))
                # PQ = Q - P
                # d = np.dot(PQ, otherSurf._normal.reshape((3,)))
                # return d
        return None
    
    @property
    def layerTimeActual(self) -> float:
        return sum([c.curveTime for c in self._path])
    
    @property
    def firstWP(self) -> Waypoint:
        return self._pathDiscretized[0]
    
    @property
    def lastWP(self) -> Waypoint:
        return self._pathDiscretized[-1]
  
    @property
    def poses(self):
        return self._pathDiscretized
    #endregion

    #region methods
    # @classmethod
    # def setLayerTime(cls, value: float) -> None:
    #     cls._layerTimeTarget = value
    
    # @staticmethod
    # def setAccelerationLimit(value: float) -> None:
    #     Curve._maxAccel = value

    def scale(self, scalingFactor) -> None:
        """
        scales path segments and layer surface
        """
        for item in self._path:
            item.scale(scalingFactor)
        
        for item in self._surface:
            item.scale(scalingFactor)
    
    def offset(self, offsetValue) -> None:
        # TODO: implement
        for curve in self._path:
            curve.offset(offsetValue)
        for surf in self._surface:
            surf.offset(offsetValue)

    def setRot(self, rot):
        for wp in self._pathDiscretized:
            wp.setRot(rot)

    def draw2D(self, ax: plt.Axes) -> None:
        """
        Draws the layer path as a 2D plot. Returns
        """
        for curve in self._path:
            pts = curve.draw()
            ax.plot(pts[0,:], pts[1,:], color = 'black', label = None)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect('equal', adjustable='box')
    
    def drawVelocityProfile(self, ax: plt.Axes) -> None:
        transitionCurve = self._path[0]
        idx = np.argwhere(transitionCurve._velocity[0] == 0.0)[1,0]
        vStart = transitionCurve._velocity[:, 0:idx]
        vEnd   = transitionCurve._velocity[:, idx:]

        ax.plot(vStart[0], vStart[1], color = 'black')
        t = vStart[0, -1]
        for curve in self._path[1:]:
            v = curve._velocity
            v[0] += t
            t = v[0, -1]

            ax.plot(v[0], v[1], color = 'black')

        tValues = vEnd[0] + t
        ax.plot(tValues, vEnd[1], color = 'black')

        ax.set_xlabel("time t [s]")
        ax.set_ylabel("Velocity v [m/s]")

    def setInitialCurveVelocities(self, start: float) -> float:
        for idx, curve in enumerate(self._path):
            newT, start = curve.setInitialVelocity(idx, start)
            curve.curveTime = newT

        newT, start = self._path[0].setInitialVelocity(len(self._path), start)
        self._path[0].curveTime += newT
        
        return start
            
    def discretize(self, dt:float, initialT:float) -> float:
        for idx, curve in enumerate(self._path):
            initialT = curve.discretize(self._pathDiscretized, idx, dt, initialT)

        initialT = self._path[0].discretize(self._pathDiscretized, idx+1, dt, initialT)
        return initialT

    def rearrangePath(self) -> None:
        self._path = self._path[self._transitionIndex:] + self._path[:self._transitionIndex]
        self._path[0].hasTransition = True

    def __str__(self) -> str:
        if isinstance(self._ifcData, Node):
            return self._ifcData['Name']
        else:
            return self._ifcData.Name
    #endregion

class TransitionLayer(Layer):
    def __init__(self, pd: 'PathData') -> None:
        super().__init__(None, None, pd)
        self._targetMeanVelocity = 0.01
        # self._registeredLayers.remove(self)
    
    @property
    def targetMeanVelocity(self) -> float:
        return self._targetMeanVelocity

    def _registerSelf(self) -> None:
        pass
    
class Geometry():
    _scalingFactor: float = 1
    _offset: NDArray[np.float64] = np.zeros((3,1))
    significantDigits: int = 6

    def __init__(self, ifcEntity, layer) -> None:
        self._ifcEntity = ifcEntity
        self._parentLayer: Layer = layer
    
    #region methods
    def scale(self, scalingFactor) -> None:
        pass

    def offset(self, offsetValue) -> None:
        pass
    #endregion

class Curve(Geometry):
    #_maxRobotVelocity: float = 1
    # _robotAcceleration: float = 1
    # transitionPosition: float = 0.5 # Parameter position on Curve
    # transitionWidth: float = 0.010 # m
    
    def __init__(self, ifcEntity, layer: 'Layer') -> None:
        super().__init__(ifcEntity, layer)
        #region public
        self.hasTransition: bool = False
        #endregion
        
        #region protected
        self._startPt: NDArray[np.float64] = np.array([0, 0, 0])
        self._endPt: NDArray[np.float64] = np.array([0, 0, 0])
        if layer and layer._surface:
            self._relatedSurface: Surface = layer._surface[0]
        else:
            self._relatedSurface: Surface = None
        self._curveTime: float = 1 # time in seconds
        self._velocity: NDArray[np.float64] = np.zeros((2,0))
        self._peakVelocity: float = 1.
        self._evalPars: NDArray[np.float64] = np.linspace(0.,1.,2)
        #endregion

    #region properties
    @property
    def length(self) -> float:
        return 0
    
    @property
    def curvature(self) -> List[float]:
        return 0
    
    @property
    def maxRobotVelocity(self) -> float:
        return self._parentLayer.maxRobotVelocity

    @property
    def robotAcceleration(self) -> float:
        return self._parentLayer.transitionAcceleration

    @property
    def maxAccel(self) -> float:
        return self._parentLayer.maxAccel
    
    @property
    def transitionPosition(self) -> float:
        return self._parentLayer.transitionPosition
    
    @property
    def transitionWidth(self) -> float:
        return self._parentLayer.transitionWidth

    @property
    def curveTime(self) -> float:
        return self._curveTime
    
    @curveTime.setter
    def curveTime(self, value: float) -> None:
        if value > 0:
            self._curveTime = np.round(value, self.significantDigits)
        else:
            raise ValueError("Time travel is not allowed!")
    
    @property
    def criticalVelocity(self) -> float:
        return self.maxRobotVelocity

    @property
    def maxVelocity(self) -> float:
        return min(self.criticalVelocity, self._parentLayer.targetMeanVelocity)
    
    @property
    def maxXYZ(self) -> NDArray[np.float64]:
        return np.max(self.evaluate(self._evalPars), axis=1)
    
    @property
    def minXYZ(self) -> NDArray[np.float64]:
        return np.min(self.evaluate(self._evalPars), axis=1)
    #endregion

    #region methods
    @classmethod
    def byFile(cls, ifcEntity, layer) -> 'Curve':
        if ifcEntity.is_a("IfcPolyline"):
                cObj = Line(ifcEntity, layer)
        elif ifcEntity.is_a("IfcTrimmedCurve"):
            if ifcEntity.BasisCurve.is_a("IfcCircle"):
                cObj = Arc(ifcEntity, layer)
            elif ifcEntity.BasisCurve.is_a("IfcLine"):
                cObj = Line(ifcEntity, layer)
            # else:
            #     cObj = ElipseArc(curve, surf, layer)
        # elif ifcEntity.is_a("IfcBSplineCurveWithKnots"):
        #     cObj = NurbsCurve(ifcEntity, layer)
        else:
            logging.error(f"Curve type {ifcEntity} not available")

        cObj.fileExtraction()
        return cObj
    
    @classmethod
    def byGraph(cls, ifcEntity, layer) -> 'Curve':
        if ifcEntity['type'] == "Line":
                cObj = Line(ifcEntity, layer)
        elif ifcEntity['type'] == "Arc":
            cObj = Arc(ifcEntity, layer)
        # elif ifcEntity['type'] == "EllipseArc":
        #     cObj = ElipseArc(curve, surf, layer)
        # elif ifcEntity['type'] == "Nurbs":
        #     cObj = NurbsCurve(ifcEntity, layer)
        else:
            logging.error(f"Curve type {ifcEntity['type']} not available")

        cObj.graphExtraction()
        return cObj

    def fileExtraction(self) -> None:
        pass

    def graphExtraction(self) -> None:
        pass

    def scale(self, scalingFactor) -> None:
        super().scale(scalingFactor)
        self._startPt *= scalingFactor
        self._endPt *= scalingFactor
    
    def evaluate(self, t: Union[NDArray[np.float64], float]) -> NDArray[np.float64]:
        if isinstance(t, np.ndarray):
            return t
        else:
            return np.array(t)

    def setInitialVelocity(self, crvIdx: int = 1, start: float = 0, end: float = -1) -> Tuple[float, float]:
        """Set up the initial velocity profile for this curve and insert the required
        velocity breakpoints into the curve's `_velocity` table.

        The method computes acceleration/deceleration phases so the curve reaches
        or leaves the peak velocity (self.maxVelocity) subject to the robot's
        acceleration limits. It then inserts discrete time/velocity points using
        `instertVelocity` and returns the total time used on this curve and the
        end velocity reached.

        Assumption: constant acceleration/deceleration!

        Parameters
        ----------
        crvIdx : int
            Index of this curve in the parent layer's path. Special values:
            - 0: transition curve at the start
            - len(parent._path): used to indicate the last curve when called
              from the layer aggregator (see `Layer.setInitialCurveVelocities`).
        start : float
            Starting velocity at the beginning of this curve (m/s).
        end : float
            Desired ending velocity for this curve (m/s). If set to -1 the
            function chooses a reasonable default based on neighboring curves
            and whether this layer is the top layer.

        Returns
        -------
        Tuple[float, float]
            (duration, end_velocity) where `duration` is the time consumed by
            the velocity profile on this curve and `end_velocity` is the
            computed exit velocity for this curve.
        """

        # If end == -1 choose an appropriate end velocity based on context
        if end == -1:
            postIdx = crvIdx + 1
            # If this points past the last curve, select either 0 (top layer)
            # or the layer default max velocity.
            if postIdx == len(self._parentLayer._path) + 1:
                if self._parentLayer._topLayer:
                    end = 0
                else:
                    end = self.maxVelocity  # TODO: adapt for NURBS if required
            else:
                # Otherwise, limit end to the minimum of this curve's max and
                # the following curve's max velocity to avoid abrupt jumps.
                end = min(
                    self.maxVelocity,
                    self._parentLayer._path[(crvIdx + 1) % len(self._parentLayer._path)].maxVelocity,
                )

        # dv1/dv2: available delta to reach peak velocity from start/end
        dv1 = self.maxVelocity - start
        dv2 = self.maxVelocity - end

        # t1/t2: time to accelerate/decelerate with constant robot acceleration
        t1 = dv1 / self.robotAcceleration
        t2 = dv2 / self.robotAcceleration

        # dx1/dx2: distances covered during the acceleration/deceleration phases
        dx1 = (start + dv1 / 2) * t1
        dx2 = (end + dv2 / 2) * t2

        # Determine the available curve length `l` for acceleration/deceleration
        if crvIdx == 0:  # transition curve at start: only part of length available
            l = self.length * (1 - (self.transitionPosition))
        elif crvIdx == len(self._parentLayer._path):
            # last curve called from Layer.setInitialCurveVelocities: only
            # transitionPosition part of the curve length applies
            l = self.length * self.transitionPosition
        else:
            # full curve length available
            l = self.length

        # If the deceleration distance alone exceeds the available length plus
        # the initial acceleration distance, we need to propagate velocity
        # changes backwards into the previous curves (recursive adjustment).
        if dx2 >= l + dx1:
            # compute new start velocity which allows decelerating over length l
            newStart = np.sqrt(end**2 + self.robotAcceleration * l * 2)
            dv2 = newStart - end
            t2 = dv2 / self.robotAcceleration

            # insert a two-point profile: immediately at t=0 jump to newStart,
            # then decelerate to `end` over t2 seconds
            self.instertVelocity([0, t2], [newStart, end])

            # update previous curve to connect smoothly
            prevIdx = crvIdx - 1
            if prevIdx >= 0:
                prev = self._parentLayer._path[prevIdx]
                prevStart = prev._velocity[1][0]
                # clear previous profile and re-compute its profile so it
                # ends at `newStart`
                prev._velocity = np.zeros((2, 0))
                newT, _ = prev.setInitialVelocity(prevIdx, prevStart, newStart)
                prev.curveTime = newT
            else:
                # If there is no previous curve, adapting earlier layers would
                # be required (not implemented - should not normally happen).
                pass

            return t2, end

        # remaining distance after accounting for pure accel/decel
        delta = dx1 + dx2 - l
        newT = 0

        # Case A: sufficient spare distance to accelerate beyond decel need
        if delta >= 2 * dx2:
            # compute an `end` velocity that uses the whole length l
            end = np.sqrt(start**2 + self.robotAcceleration * l * 2)
            dv1 = end - start
            t1 = dv1 / self.robotAcceleration

            # simple two-point acceleration (start -> end)
            self.instertVelocity([0, t1], [start, end])
            return t1, end
        # Case B: no spare distance => plateau at maxVelocity or triangular profile
        elif delta <= 0:
            maxV = self.maxVelocity
            # extra time spent at plateau velocity to cover -delta distance
            t = -delta / maxV

            # build the profile by inserting breakpoints at times when the
            # velocity changes: start, optionally maxV plateau, and end
            self.instertVelocity([0], [start])
            if dx1:
                self.instertVelocity([t1], [maxV])
            t1 += t
            # small guard: only insert plateau points if there is meaningful time
            if t > 10E-3 and dx2:
                self.instertVelocity([t1], [maxV])
            t1 += t2
            self.instertVelocity([t1], [end])

            return t1, end
        else:
            # Case C: intermediate case -> symmetric reduction of accel/decel
            dx1 -= delta / 2
            dx2 -= delta / 2

            # compute achievable peak velocity given reduced distances
            maxV = np.sqrt(start**2 + self.robotAcceleration * dx1 * 2)
            dv1 = maxV - start
            dv2 = maxV - end

            t1 = dv1 / self.robotAcceleration
            t2 = dv2 / self.robotAcceleration

            # insert a three-point profile: start -> maxV -> end
            self.instertVelocity([0, t1, t1 + t2], [start, maxV, end])

            return t1 + t2, end

    def instertVelocity(self, t: float, v: float) -> None:
        # self._velocity: 
        # - row 0: time [s]
        # - row 1: velocity [m/s]
        if not isinstance(t, list):
            t = [t]
        
        if not isinstance(v, list):
            v = [v]

        idx = self._velocity.shape[1]
        self._velocity = np.insert(self._velocity, [idx], [t, v], axis=1)

    def discretize(self, wpList: List[Waypoint], idx: int = 1, dt: float = 1, initialT: float = 0) -> Tuple[float, NDArray[np.float64]]:
        if idx == len(self._parentLayer._path):
            a = np.argwhere(self._velocity[0] == 0.0)[1,0] + 1
        else:
            a = 1
        
        if idx == 0:
            b = np.argwhere(self._velocity[0] == 0.0)[1,0]
            dx = self.transitionPosition# - self.transitionWidth / self.length
        else:
            b = self._velocity.shape[1]
            dx = 0

        #pointCoords = np.zeros((3,0))
        for i in range(a, b):
            deltaT = self._velocity[0, i] - self._velocity[0, i-1]
            deltaV = self._velocity[1, i] - self._velocity[1, i-1]
            m = deltaV / deltaT
            v1 = m * initialT + self._velocity[1, i-1]
            
            x0 = ((self._velocity[1, i-1] + v1) / 2 * initialT) / self.length + dx

            n, rest = np.divmod(deltaT - initialT, dt)
            factor = m * dt**2 / self.length
            summand = v1 * dt / self.length
            
            if abs(factor) > 10E-10:
                x = np.arange(0.5, n, 1) * factor + summand
                #v = np.arange(1, n+1, 1) * m * dt + v1
                v = np.array([max(self._velocity[1, i], self._velocity[1, i-1])]*int(n))
            else:
                x = np.array([summand]*int(n))
                v = np.array([v1]*int(n))

            if x0 < 1:
                x = np.insert(x, 0, x0)
                v = np.insert(v, 0, v1)
            par = np.cumsum(x)
            pts = self.evaluate(par)

            if idx == 0 and par[0] < (self.transitionPosition + self.transitionWidth / self.length): # TODO: works only for transition at idx 0
                transitionIndices = np.nonzero(par<(self.transitionPosition + self.transitionWidth / self.length))
                deltaZ = par[transitionIndices] - self.transitionPosition
                deltaZ /= (self.transitionWidth / self.length)
                deltaZ *= self._parentLayer.layerHeight
                deltaZ -= self._parentLayer.layerHeight

                pts[2,transitionIndices] += deltaZ
            
            if par.size > 0:
                dx = par[-1] + (m * (deltaT - rest/2) + self._velocity[1, i-1]) * rest / self.length
            initialT = dt - rest
            
            wpList.extend(Waypoint.newList(pts, v, self))
        
        return initialT

    def draw(self, N: int) -> NDArray[np.float64]:
        return self.evaluate(np.linspace(0, 1, N))

    def offset(self, offsetValue) -> None:
        self._startPt += offsetValue
        self._endPt   += offsetValue
    #endregion

class Line(Curve):
    #region properties
    @property
    def length(self) -> float:
        return np.round(np.linalg.norm(self._endPt - self._startPt), self.significantDigits)
    
    @property
    def maxXYZ(self) -> NDArray[np.float64]:
        return np.max(np.concatenate((self._startPt, self._endPt), axis=1), axis=1)

    @property
    def minXYZ(self) -> NDArray[np.float64]:
        return np.min(np.concatenate((self._startPt, self._endPt), axis=1), axis=1)
    #endregion

    #region methods
    def fileExtraction(self) -> None:
        if self._ifcEntity.is_a("IfcPolyLine"):
            self._startPt: NDArray[np.float64] = np.array(self._ifcEntity.Points[0].Coordinates, ndmin=2).transpose()
            self._endPt: NDArray[np.float64] = np.array(self._ifcEntity.Points[1].Coordinates, ndmin=2).transpose()
        elif self._ifcEntity.is_a("IfcTrimmedCurve"):
            if self._ifcEntity.MasterRepresentation == 'CARTESIAN':
                self._startPt: NDArray[np.float64] = np.array(self._ifcEntity.Trim1[0].Coordinates, ndmin=2).transpose()
                self._endPt: NDArray[np.float64] = np.array(self._ifcEntity.Trim2[0].Coordinates, ndmin=2).transpose()
            else:
                raise Exception("Parameter trimmed line not implemented yet!")
    
    def graphExtraction(self) -> None:
        self._startPt: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['startPt'], ndmin=2).transpose()
        self._endPt: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['endPt'], ndmin=2).transpose()

    def evaluate(self, t: Union[NDArray[np.float64], float]) -> NDArray[np.float64]:
        t = super().evaluate(t)
        return self._startPt + t * (self._endPt - self._startPt)
    
    def draw(self, N: int = 2) -> NDArray[np.float64]:
        return super().draw(N)
    #endregion

class Line3D(Line):
    def __init__(self, A, B, pd) -> None:
        Curve.__init__(self, None, pd)
        self._startPt: NDArray[np.float64] = A
        self._endPt: NDArray[np.float64]   = B
    
    @property
    def maxVelocity(self) -> float:
        return self.criticalVelocity

    @property
    def robotAcceleration(self) -> float:
        return self._parentLayer.transitionAcceleration

class Arc(Curve):
    #region properties
    @property
    def refDir2(self) -> NDArray[np.float64]:
        return np.cross(self._normal, self._refDir, 0, 0).transpose()

    @property
    def length(self) -> float:
        return np.round(((self._trim2-self._trim1)%(2*np.pi))*self._radius, self.significantDigits)
    
    @property
    def curvature(self):
        return np.round(1/self._radius, self.significantDigits)
    
    @property
    def criticalVelocity(self) -> float:
        return min(np.round(np.sqrt(self.maxAccel * self._radius), self.significantDigits), self.maxRobotVelocity)
    #endregion

    #region methods
    def fileExtraction(self) -> None:
        self._centerPt: NDArray[np.float64] = np.array(self._ifcEntity.BasisCurve.Position.Location.Coordinates, ndmin=2).transpose()
        self._normal: NDArray[np.float64] = np.array(self._ifcEntity.BasisCurve.Position.Axis.DirectionRatios, ndmin=2).transpose()
        self._refDir: NDArray[np.float64] = np.array(self._ifcEntity.BasisCurve.Position.RefDirection.DirectionRatios, ndmin=2).transpose()
        self._radius: float = self._ifcEntity.BasisCurve.Radius
        self._startPt: NDArray[np.float64] = np.array(self._ifcEntity.Trim1[0].Coordinates, ndmin=2).transpose()
        self._endPt: NDArray[np.float64] = np.array(self._ifcEntity.Trim2[0].Coordinates, ndmin=2).transpose()
        self._trim1: float = self._getAngle(self._startPt)
        self._trim2: float = self._getAngle(self._endPt)
    
    def graphExtraction(self) -> None:
        self._centerPt: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['origin'], ndmin=2).transpose()
        self._normal: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['zDir'], ndmin=2).transpose()
        self._refDir: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['xDir'], ndmin=2).transpose()
        self._radius: float = self._ifcEntity['crvParameter']['radius']
        self._startPt: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['startPt'], ndmin=2).transpose()
        self._endPt: NDArray[np.float64] = np.array(self._ifcEntity['crvParameter']['endPt'], ndmin=2).transpose()
        self._trim1: float = self._getAngle(self._startPt)
        self._trim2: float = self._getAngle(self._endPt)

    def _getAngle(self, point: NDArray[np.float64]) -> float:
        """
        calculates parametric (angular) position of point on circle.
        """
        vec = point - self._centerPt
        phi = np.arccos(np.vdot(vec, self._refDir) / np.linalg.norm(vec))
        localAxis = np.cross(self._refDir, vec, 0, 0)

        if np.dot(localAxis, self._normal) < 0:
            phi = 2*np.pi - phi
                
        return phi
    
    def scale(self, scalingFactor) -> None:
        super().scale(scalingFactor)
        self._centerPt *= scalingFactor
        self._radius *= scalingFactor
      
    def evaluate(self, t: Union[NDArray[np.float64], float]) -> NDArray[np.float64]:
        t = super().evaluate(t)
        phi = ((self._trim2 - self._trim1) % (2*np.pi)) * t + self._trim1
        return self._centerPt + self._radius * (np.cos(phi) * self._refDir + np.sin(phi) * self.refDir2)

    def draw(self, N: int = 20) -> NDArray[np.float64]:
        return super().draw(N)
    
    def offset(self, offsetValue) -> None:
        super().offset(offsetValue)
        self._centerPt += offsetValue
    #endregion

"""
# TODO: finish Ellipse implementation!
# class ElipseArc(Curve):
#     def __init__(self, ifcEntity, surface, layer) -> None:
#         super().__init__(ifcEntity, surface, layer)
#         self._centerPt: NDArray[np.float64] = np.array(ifcEntity.BasisCurve.Position.Location.Coordinates, ndmin=2).transpose()
#         self._normal: NDArray[np.float64] = np.array(ifcEntity.BasisCurve.Position.Axis.DirectionRatios, ndmin=2).transpose()
#         self._refDir: NDArray[np.float64] = np.array(ifcEntity.BasisCurve.Position.RefDirection.DirectionRatios, ndmin=2).transpose()
#         self._radiusA: float = ifcEntity.BasisCurve.MajorRadius
#         self._radiusB: float = ifcEntity.BasisCurve.MinorRadius
#         self._startPt: NDArray[np.float64] = np.array(ifcEntity.Trim1[0].Coordinates, ndmin=2).transpose()
#         self._endPt: NDArray[np.float64] = np.array(ifcEntity.Trim2[0].Coordinates, ndmin=2).transpose()
#         self._trim1: float = self._getAngle(self._startPt)
#         self._trim2: float = self._getAngle(self._endPt)

#     #region properties
#     @property
#     def refDir2(self) -> NDArray[np.float64]:
#         return np.cross(self._normal, self._refDir, 0, 0).transpose()
    
#     @property
#     def length(self) -> float:
#         super().length
#         m = 1 - (self._radiusB / self._radiusA)**2
#         t1 = ellipeinc(self._trim2 - 0.5*np.pi, m)
#         t2 = ellipeinc(self._trim1 - 0.5*np.pi, m)
#         return self._radiusA * (t1 - t2)
#     #endregion

#     #region methods
#     def _getAngle(self, point: NDArray[np.float64]) -> float:
#         # TODO: implement
#         return 0

#     def offset(self, offset: NDArray[np.float64]) -> None:
#         super().offset(offset)
#         self._centerPt += offset
#     #endregion

# class NurbsCurve(Curve, Geom.Geom_BSplineCurve):
#     def __init__(self, ifcEntity, layer) -> None:
#         super().__init__(ifcEntity, layer)
#         #region protected
#         self._startPt = np.asarray(self._ifcEntity.ControlPointsList[0].Coordinates) #controlPt[:,0]
#         self._endPt = np.asarray(self._ifcEntity.ControlPointsList[-1].Coordinates) #controlPt[:,-1]
#         #endregion


#     #region properties
#     @property
#     def length(self) -> float:
#         adaptor = GeomAdaptor.GeomAdaptor_Curve(self)
#         return np.round(GCPnts_AbscissaPoint.Length(adaptor), self.significantDigits)
    
#     @property
#     def criticalVelocity(self):
#         # TODO: remove?
#         t = np.linspace(0,1,50)
#         k = max(self.curvatureAtParameter(t))
#         if k > 10E-6:
#             return min(np.round(np.sqrt(self._maxAccel / k), self.significantDigits), self._maxRobotVelocity)
#         else:
#             return self._maxRobotVelocity
    
#     # @property
#     # def maxVelocity(self):
#     #     # TODO: needs update
#     #     return self.criticalVelocity
#     #endregion

#     #region methods
#     def fileExtraction(self) -> None:
#         count = len(self._ifcEntity.ControlPointsList)
#         ctPts = TColgp_Array1OfPnt(1,count)
#         wgts  = TColStd_Array1OfReal(1,count)
#         for n, pt in enumerate(self._ifcEntity.ControlPointsList):
#             ctPts.SetValue(n+1, gp_Pnt(pt.Coordinates[0], pt.Coordinates[1], pt.Coordinates[2]))
#             wgts.SetValue(n+1, 1)

#         degree = self._ifcEntity.Degree
#         self._evalPars = np.linspace(0.,1.,int(degree*10))

#         knts  = TColStd_Array1OfReal(1,len(self._ifcEntity.Knots))
#         mlt  = TColStd_Array1OfInteger(1,len(self._ifcEntity.Knots))
#         for n in range(0, len(self._ifcEntity.Knots)):
#             knts.SetValue(n+1, self._ifcEntity.Knots[n])
#             mlt.SetValue(n+1, self._ifcEntity.KnotMultiplicities[n])
#         Geom.Geom_BSplineCurve.__init__(self, ctPts, wgts, knts, mlt, degree, False, True)

#     def graphExtraction(self) -> None:
#         pass

#     def denormalizeT(self, t: Union[NDArray[np.float64], float]) -> float:
#         return (self.LastParameter() - self.FirstParameter()) * t + self.FirstParameter()

#     def scale(self) -> None:
#         super().scale()
#         self.Scale(gp_Pnt(0, 0, 0), Geometry._scalingFactor)

#     def curvatureAtParameter(self, t: Union[NDArray[np.float64], float]) -> Union[NDArray[np.float64], float]:
#         # TODO: update
#         t = self.denormalizeT(t)
#         lprop = GeomLProp.GeomLProp_CLProps(self, 2, 10E-5)
#         if isinstance(t, float):
#             lprop.SetParameter(t)
#             return lprop.Curvature()
#         else:
#             curvatures = np.zeros(len(t))
#             for idx, ti in enumerate(t):
#                 lprop.SetParameter(ti)
#                 curvatures[idx] = lprop.Curvature()
#             return curvatures
    
#     def maxVelocityAtParameter(self, t) -> float: #, maxVal = None):
#         # TODO: update
#         # if not maxVal:
#         #     maxVal = self.averageVelocity
#         return np.round(np.sqrt(self._maxAccel / self.curvatureAtParameter(t)), self.significantDigits)

#     def evaluate(self, t: Union[NDArray[np.float64], float]) -> NDArray[np.float64]:
#         t = super().evaluate(t)
#         evalPnts = np.zeros((3, len(t)))
#         for idx, ti in enumerate(t):
#             pnt = self.Value(self.denormalizeT(ti))
#             evalPnts[:,idx] = [pnt.X(), pnt.Y(), pnt.Z()]
#         return evalPnts

#     def draw(self, N: int = 30) -> NDArray[np.float64]:
#         return super().draw(N)
    
#     def offset(self, offset: NDArray[np.float64]) -> None:
#         off = gp_Vec(*offset.reshape((3,)).tolist())
#         self.Translate(off)
#     #endregion
"""

class Surface(Geometry):
    def __init__(self, ifcEntity, layer: 'Layer') -> None:
        super().__init__(ifcEntity, layer)
        self._origin = np.zeros((3,1))
        self._normal = np.array([0,0,1], ndmin=2).transpose()
        self._refDir = np.array([1,0,0], ndmin=2).transpose()
    
    @property
    def isHorizontal(self) -> bool:
        return False
    
    @property
    def isPlanar(self) -> bool:
        return False
    
    @classmethod
    def byFile(cls, ifcEntity, layer: 'Layer') -> 'Surface':
        if ifcEntity.is_a("IfcCurveBoundedPlane"):
            sObj = CurveBoundedPlane(ifcEntity, layer)
        elif ifcEntity.is_a("IfcBSplineSurfaceWithKnots"):
            logging.warning(f"Surface type {ifcEntity} not implemented yet.")
        else:
            logging.warning(f"Surface type {ifcEntity} not supported")

        sObj.fileExtraction()
        # if scale:
        #     sObj.scale()
        # if offset:
        #     sObj.offset()
        return sObj
    
    @classmethod
    def byGraph(cls, ifcEntity, layer: 'Layer') -> 'Surface':
        if "IfcCurveBoundedPlane" in ifcEntity['type']:
            sObj = CurveBoundedPlane(ifcEntity, layer)
        elif "IfcPlane" in ifcEntity['type']:
            sObj = Plane(ifcEntity, layer)
        elif "IfcBSplineSurfaceWithKnots" in ifcEntity['type']:
            logging.warning(f"Surface type {ifcEntity['type']} not implemented yet.")
        else:
            logging.warning(f"Surface type {ifcEntity['type']} not supported")

        sObj.graphExtraction()
        # if scale:
        #     sObj.scale()
        return sObj
    
    def fileExtraction(self):
        pass

    def graphExtraction(self):
        pass

    def getNormalAtUV(self, _):
        return self._normal
    
    def getNormalAtCP(self, _):
        return self._normal
    
class Plane(Surface):
    @property
    def refDir2(self) -> NDArray[np.float64]:
        return np.cross(self._normal, self._refDir, 0, 0).transpose()
    
    @property
    def isHorizontal(self) -> bool:
        z = np.array((0., 0., 1.), ndmin=2).transpose()
        return np.allclose(z, self._normal)
    
    @property
    def isPlanar(self) -> bool:
        return True
    
    def fileExtraction(self) -> None:
        position = self.filePosition()
        self._origin: NDArray[np.float64] = np.array(position.Location.Coordinates, ndmin=2).transpose()
        if position.Axis:
            self._normal: NDArray[np.float64] = np.array(position.Axis.DirectionRatios, ndmin=2).transpose()
        if position.RefDirection:
            self._refDir: NDArray[np.float64] = np.array(position.RefDirection.DirectionRatios, ndmin=2).transpose()

    def graphExtraction(self):
        self._origin: NDArray[np.float64] = np.array(self._ifcEntity['surfParameter']['origin'], ndmin=2).transpose() 
        self._normal: NDArray[np.float64] = np.array(self._ifcEntity['surfParameter']['zDir'], ndmin=2).transpose()
        self._refDir: NDArray[np.float64] = np.array(self._ifcEntity['surfParameter']['xDir'], ndmin=2).transpose()

    def filePosition(self):
        return self._ifcEntity.Position
    
    def scale(self, scalingFactor) -> None:
        super().scale(scalingFactor)
        self._origin *= scalingFactor

class CurveBoundedPlane(Plane):
    def filePosition(self):
        return self._ifcEntity.BasisSurface.Position

class NurbsSurface(Surface):
    def __init__(self, ifcEntity, layer) -> None:
        super().__init__(ifcEntity, layer)

class PathData():
    """
    Container for FIM data. Provides methods for reading and interpreting FIM-IFC files.
    """
    def __init__(self) -> None:
        """
        ifcFilename: filename of the .ifc
        """
        #region public
        self.sigDig: int = 6 # public
        self.segmentLimit: float = 0.015 # limiter for startEnd selection
        self.layerLimit = 1
        self.offsetValue = np.zeros((3,1))
        self.scalingFactor = 1.0
        self.layerTime: float = 1.0 # time for one layer in [s]
        self.maxAcceleration: float = 1.0 # maximum acceleration of the robot in a curve in [m/s^2]
        self.maxRobotVelocity: float = 0.05 # maximum velocity of the robot in [m/s]
        self.maxRobotAcceleration: float = 1.0 # maximum acceleration of the robot in [m/s^2]
        self.transitionAcceleration: float = 0.1
        self.targetMeanVelocity: float = 0.01
        self.transitionPosition: float = 0.5
        self.transitionWidth: float = 0.01
        #endregion

        #region protected
        self._importer: GraphImporter = None
        
        self._layers: List[Layer] = [] # public
        self._startTransition: Line3D = None
        self._startCoordinate: NDArray[np.float64] = np.zeros((3,1))
        
        self._frequency = 50  # [Hz]
        self._discretizedTransition: List[Waypoint] = []
        
        #self._kinematicSolver = kin
        #self._transitionIDX = 0
        self._rootEvent = None
        self._initialWaypoint: Waypoint = None
        self._currentWaypoint: Waypoint = None
        #endregion
        
        #self.verbose: bool = False # public
        #self._ifcFilename: str = ''
        #self._ifcFile = None
        #self._ifcComponent = None

    #region PathData properties
    @property
    def hasTransitions(self) -> bool:
        # TODO: may be removed? 
        res = False
        for layer in self._layers:
            if not layer.isClosed:
                res = True
                break
        return res
    
    @property
    def isPlanar(self) -> bool:
        for layer in self._layers:
            for surf in layer._surface:
                if not isinstance(surf, Plane):
                    return False
        return True # returns also True if no surfaces are loaded
    
    @property
    def isHorizontal(self) -> bool:
        # TODO: implement
        return True

    @property
    def pathLength(self) -> float:
        return sum([l.length for l in self._layers])
    
    @property
    def dt(self):
        return 1/self._frequency
    
    # @property
    # def frequency(self) -> float:
    #     return self._frequency
    
    # @frequency.setter
    # def frequency(self, freq: float) -> None:
    #     self._frequency = freq
    #endregion

    #region methods
    # def initiateImporter(self, ifcFilename):
    #     self._importer.initiateImport(ifcFilename)

    # def getStartingElement(self) -> Tuple[dict, Node]:
    #     return self._importer.getStartingElement()

    # def readFirstLayer(self) -> None:
    #     return self._importer.readFirstLayer()

    # def readLayers(self, lim:int = None) -> None:
    #     self._importer.readLayers(lim)

    # def readNextTask(self) -> 'Process':
    #     return self._importer.readNextTask()
    
    # def disconnectImporter(self) -> None:
    #     self._importer.closeSession()
    
    # def connectImporter(self) -> None:
    #     self._importer.startSession()
    
    @property
    def layers(self) -> List[Layer]:
        return self._layers
    
    def addLayer(self, layer: Layer) -> None:
        self._layers.append(layer)
    
    def removeLayer(self, layer: Layer) -> None:
        self._layers.remove(layer)

    def setConditions(self, layerTime: float = 1.0, maxAcceleration: float = 1.0, maxRobotAcceleration: float = 1.0, maxRobotVelocity: float = 0.1, transitionAcceleration: float = 0.1, scale: float = 1.0, limitLayers: int = 1, frequency: int = 50, **kvargs) -> None:
        self.layerTime = layerTime
        self.maxAcceleration = maxAcceleration
        self.maxRobotAcceleration = maxRobotAcceleration
        self.transitionAcceleration = transitionAcceleration
        self.maxRobotVelocity = maxRobotVelocity
        #Layer.setLayerTime(layerTime)
        #Layer.setAccelerationLimit(maxAcceleration)
        #Curve._robotAcceleration = maxRobotAcceleration
        assert scale > 0, "Scale must be greater than 0!"
        self.scalingFactor = scale
        self.layerLimit = limitLayers
        self._frequency = frequency
    
    def setOffset(self, offset: NDArray[np.float64]) -> None:
        self.offsetValue = offset
        #Geometry._offset = offset

    def setInitialWaypointByJC(self, jc: list[float], solver: 'Screwkincalc') -> None:
        wp = Waypoint.byJC(jc, solver)
        self._initialWaypoint = wp
        self.setCurrentWaypoint(wp)

    def setCurrentWaypoint(self, wp: Waypoint) -> None:
        self._currentWaypoint = wp

    def scale(self) -> None:
        """
        Scale up/down segments of layers. 
        """
        for layer in self._layers:
            logging.info(f"Scaling segments of {layer} by factor = {self.scalingFactor}.")
            layer.scale(self.scalingFactor)

    def offset(self) -> None:
        """
        Offset the layers using the layer.offset method iteratively 
        """
        if all([off == 0 for off in self.offsetValue]):
            return
        for layer in self._layers:
            layer.offset(self.offsetValue)
            logging.info(
                f"Offsetting segments of layer {layer} by offset = {self.offsetValue}.")

    def selectStartEndPt(self, importer: 'FIMimporter', layer: Layer, segmentLim: float = 0.005) -> None:
        logging.info(" >>> Select the point out of the  midpoints of segments")
        logging.info(
            f"     Only segments with length > {segmentLim} [m] are available.")

        frame, _ = importer.getStartingElement()
        self._startCoordinate = np.array(frame['origin']).reshape((3,1)) * self.scalingFactor + self.offsetValue
        # TODO: implement line matching

        index = 0
        lines = []
        #layer = self._layers[0]
        noc = len(layer._path)
        f, a = plt.subplots()
        layer.draw2D(a)

        selectable:List[Curve] = []
        for curve in layer._path:
            if curve.length > segmentLim:
                selectable.append(curve)
        
        maxlen = round(max([s.length for s in selectable]),3)

        # f.subplots_adjust(left=0.25, bottom=0.25)
        axPos = f.add_axes([0.15, 0.2, 0.7, 0.03])
        posSlider = Slider(
            ax=axPos,
            label="Position [-]", 
            valmin=0.0, 
            valmax=1.0, 
            valinit=0.5, 
            valstep=0.01)

        axWidth = f.add_axes([0.15, 0.1, 0.7, 0.03])
        wdtSlider = Slider(
            ax=axWidth,
            label="Width [m]",
            valmin=0.005,
            valmax=maxlen,
            valinit=0.005,
            valstep=0.001
            )

        for idx, curve in enumerate(selectable):
            if idx == index:
                color = "r"
            else:
                color = "y"
            points = curve.evaluate([posSlider.val, posSlider.val + wdtSlider.val/curve.length])
            lines.append(a.plot(points[0,:], points[1,:], marker="x", c=color, picker=True)[0])

        def onpick(event) -> None:
            nonlocal index
            nonlocal noc

            a.lines[index+noc].set(color='y')
            index = a.lines.index(event.artist)
            a.lines[index].set(color='r')
            index -= noc
            logging.info(
                f"You have selected the curve marked in red [{index}]. Close the figure to confirm.")
            # a.scatter(points[index][0], points[index][1], c='r')
            a.set_title("Close to confirm selection of red point.")
            f.canvas.draw()

        def updateTransition(_) -> None:
            #nonlocal lines
            #nonlocal selectable
            for l, s in zip(lines, selectable):
                p1 = posSlider.val + wdtSlider.val/s.length
                points = s.evaluate([posSlider.val, p1])
                if p1 <= 1:
                    l.set(xdata=points[0, :], ydata=points[1, :], picker=True, visible=True)
                else:
                    l.set(picker=False, visible=False)
            
        posSlider.on_changed(updateTransition)
        wdtSlider.on_changed(updateTransition)

        f.canvas.mpl_connect("pick_event", onpick)
        a.set_title("Select point to set as a start-end point, or close for default.")
        plt.show(block=True)

        if not lines[index]._visible:
            logging.warning(
                f"The selected line is invalid. Settings were resetted to default!")
            index = 0
        else:
            self.transitionPosition = posSlider.val
            self.transitionWidth = wdtSlider.val

        # TODO: transition shift needs better solution for components with different layer paths
        #self._transitionIDX = layer._path.index(selectable[index])
        self._startCoordinate = selectable[index].evaluate(posSlider.val).reshape((3,1))
        self._startCoordinate[2] = self.offsetValue[2]
        importer.moveStartingElement(self._startCoordinate)

        # for layer in self._layers:
        #     layer._path = layer._path[idx:] + layer._path[:idx]
        #     layer._path[0].hasTransition = True

    def generateVelocityProfile(self) -> None:
        transitionSpeed = 0

        for layer in self._layers:
            # TODO: add iteration step to improve velocity profile (time is not accurate)
            transitionSpeed = layer.setInitialCurveVelocities(transitionSpeed)

        self.drawVelocity()
        plt.show(block=True)

    def drawVelocity(self, layerID: int = 0) -> None:
        fig, (ax1, ax2) = plt.subplots(nrows=2)
        layer = self._layers[layerID]

        layer.draw2D(ax1)
        layer.drawVelocityProfile(ax2)

    def discretizePath(self) -> None:
        #fig = plt.figure()
        #ax = fig.add_subplot(projection='3d')
        initialT = 0
        for layer in self._layers:
            initialT = layer.discretize(self.dt, initialT)
        # for layer in self._layers:
        #     ax.plot(layer._discretizedPath[0, :], layer._discretizedPath[1, :])#, layer._discretizedPath[2, :], ".")
        #plt.show(block=True)

    def getRot(self):
        firstWP = self._discretizedTransition[-1]

        for layer in self._layers:
            if layer.isPlanar:
                for wp in layer._pathDiscretized:
                    wp.setRot(firstWP._rot)
            else:
                pass # TODO: implement for nonplanar


    def getJointValues(self, jcoord):
        """
        Uses the discretized path and an inverse kinematics object to get the joint values for each time step.
        """
        for wp in self._discretizedTransition:
            jcoord = wp.inverse(jcoord)
        
        for layer in self._layers:
            for wp in layer._pathDiscretized:
                jcoord = wp.inverse(jcoord)

    def generateLIN(self, startWP: Waypoint, endWP: Waypoint) -> Tuple[List[Waypoint], NDArray[np.float64]]:
        """
        Generates a linear path between two waypoints.
        """
        sPt = startWP._cartesian.reshape((3,1))
        ePt = endWP._cartesian.reshape((3,1))
        path = Line3D(sPt,ePt, TransitionLayer(self))
        path.setInitialVelocity(start=0, end=0)
        # path._velocity[1][1] = path.maxRobotVelocity
        # path._velocity[1][2] = path.maxRobotVelocity
        discretizedPath: List[Waypoint] = []
        rest = path.discretize(discretizedPath, dt=self.dt)

        if rest > self.dt/20:
            discretizedPath.append(endWP)

        A = startWP._rot[0:3,2]
        B = -1*endWP._rot[0:3,2]
        
        # if np.all(np.abs(A - B) < 1E-4):
        #     return
        
        par = len(discretizedPath) - 1
        
        refRot = startWP._rot
        for n, wp in enumerate(discretizedPath):
            normal = A * (1 - n/par) + B * n/par
            normal /= np.linalg.norm(normal)
            wp.rotByNormalAndReference(normal, refRot)
            refRot = wp._rot
        
        return discretizedPath, discretizedPath[-1]._rot
        

    def addStartTransition(self, initialPose: Waypoint) -> None:
        XYZrobot = initialPose._cartesian.reshape((3,1))
        XYZpath = self._layers[0].firstWP._cartesian[0:3].reshape((3,1))
        st = self._startTransition = Line3D(XYZrobot, XYZpath, TransitionLayer())
        st.setInitialVelocity(start=0, end=0)
        rest = st.discretize(self._discretizedTransition, dt=self.dt)

        if rest > self.dt/20:
            self._discretizedTransition.append(Waypoint(st.evaluate(1).reshape((3,)), st))
        
        A = initialPose._rot[0:3,2]
        B = -self._layers[0].firstWP.getNormal().reshape(3,)
        par = len(self._discretizedTransition) - 1
        
        refRot = initialPose._rot
        for n, wp in enumerate(self._discretizedTransition):
            normal = A * (1 - n/par) + B * n/par
            normal /= np.linalg.norm(normal)
            wp.rotByNormalAndReference(normal, refRot)
            refRot = wp._rot
    
    def drawLayerFrames(self):
        fig, ax = plt.subplots(1,1, subplot_kw=dict(projection='3d'))
        
        xlim = [100,0]
        ylim = [100,0]
        zlim = [100,0]
        for i in range(0, len(self._layers[0]._pathDiscretized), 10):
            wp = self._layers[0]._pathDiscretized[i]
            origin = wp._cartesian
            xlim = [min(origin[0], xlim[0]), max(origin[0], xlim[1])]
            ylim = [min(origin[1], ylim[0]), max(origin[1], ylim[1])]
            zlim = [min(origin[2], zlim[0]), max(origin[2], zlim[1])]

            ax.scatter(*origin.tolist(), c='k')
            x = origin + 0.02 * wp._rot[0:3,0]
            y = origin + 0.02 * wp._rot[0:3,1]
            z = origin + 0.02 * wp._rot[0:3,2]

            ax.plot([origin[0], x[0]], [origin[1], x[1]], [origin[2], x[2]], c='r')
            ax.plot([origin[0], y[0]], [origin[1], y[1]], [origin[2], y[2]], c='g')
            ax.plot([origin[0], z[0]], [origin[1], z[1]], [origin[2], z[2]], c='b')

        mid = [sum(xlim)/2, sum(ylim)/2, sum(zlim)/2]
        max_range = np.array([abs(xlim[1]-xlim[0]), abs(ylim[1]-ylim[0]), abs(zlim[1]-zlim[0])]).max()
        Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + mid[0]
        Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + mid[1]
        Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + mid[2]
        # Comment or uncomment following both lines to test the fake bounding box:
        for xb, yb, zb in zip(Xb, Yb, Zb):
            ax.plot([xb], [yb], [zb], 'w')

        plt.show()

    def drawTransitionFrames(self):
        fig, ax = plt.subplots(1,1, subplot_kw=dict(projection='3d'))
        
        for i in range(0, len(self._discretizedTransition), int(np.floor(len(self._discretizedTransition)/10))):
            wp = self._discretizedTransition[i]
            origin = wp._cartesian
            ax.scatter(*origin.tolist(), c='k')
            x = origin + 0.02 * wp._rot[0:3,0]
            y = origin + 0.02 * wp._rot[0:3,1]
            z = origin + 0.02 * wp._rot[0:3,2]

            ax.plot([origin[0], x[0]], [origin[1], x[1]], [origin[2], x[2]], c='r')
            ax.plot([origin[0], y[0]], [origin[1], y[1]], [origin[2], y[2]], c='g')
            ax.plot([origin[0], z[0]], [origin[1], z[1]], [origin[2], z[2]], c='b')

        first = self._discretizedTransition[0]._cartesian
        last  = self._discretizedTransition[-1]._cartesian
        mid = (first + last) / 2
        max_range = np.array([abs(first[0]-last[0]), abs(first[1]-last[1]), abs(first[2]-last[2])]).max()
        Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + mid[0]
        Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + mid[1]
        Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + mid[2]
        # Comment or uncomment following both lines to test the fake bounding box:
        for xb, yb, zb in zip(Xb, Yb, Zb):
            ax.plot([xb], [yb], [zb], 'w')

        plt.show(block=True)

    def plotJointPath(self):
        """
        Plot variation in joint values
        """
        if len(self._discretizedTransition) == 0:
            return
        
        fig, axes = plt.subplots(6, 1, sharex = True)
        t = np.ones((len(self._discretizedTransition), 1))
        t[0] = 0
        t *= self.dt
        t = np.cumsum(t)
        
        transition = np.array([wp._jointangles for wp in self._discretizedTransition])
        
        for i, ax in enumerate(axes[:]):
            ax.plot(t, transition[:,i])

        for layer in self._layers:
            path = np.array([wp._jointangles for wp in layer._pathDiscretized])
            lastT = t[-1]
            t = np.ones((len(layer._pathDiscretized), 1))
            t *= self.dt
            t[0] += lastT
            t = np.cumsum(t)

            for i,ax in enumerate(axes[:]):
                ax.plot(t, path[:,i])    

        plt.show(block=True)
        
        return
        # suffix = ["Position", "Velocity", "Acceleration"]
        # for j, values in enumerate(availableVals):
        #     for i,ax in enumerate(axes[:,j]):
        #         ax.plot(self.t, values[:,i])
        #         if i == 0:
        #             ax.set_title(f"{suffix[j]}")
        #         if j == 0:
        #             ax.set_ylabel(f"{i}: {self.jointLabels[i]}",size = 'large')# ax.set_xticks(self.t, minor=True)
        # fig.subplots_adjust(left=0.15)
        # plt.xlabel("Time")
        # plt.show()

    def drawPath(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        PathData.wpPlot(self._discretizedTransition, ax)
        for layer in self._layers:
            PathData.wpPlot(layer._pathDiscretized, ax)
        plt.show(block=True)

    def getXYZBaseBoundary(self) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        maxXYZ = np.max([curve.maxXYZ for curve in self._layers[0]._path], axis=0)
        minXYZ = np.min([curve.minXYZ for curve in self._layers[0]._path], axis=0)
        return minXYZ, maxXYZ

    def __len__(self) -> int:
        return len(self._layers)
    
    @abstractmethod
    def wpPlot(wpList, ax: plt.Axes):
        x, y, z = [], [], []
        for wp in wpList:
            x.append(wp._cartesian[0])
            y.append(wp._cartesian[1])
            z.append(wp._cartesian[2])
        ax.plot(x, y, z, marker='o')

class Process():
    def __init__(self, ifcEntity: Node, associatedEntity: Node, hasSubprocess:bool) -> None:
        self._ifcData = ifcEntity
        self.Name = ifcEntity.get('Name')
        self._associatedEntity = associatedEntity
        self.hasSubprocess = hasSubprocess

    @property
    def elementId(self):
        return self._ifcData.element_id
    
    def getProperty(self, prop: str):
        return self._ifcData.get(prop)
    
    def addAction(self, action: 'Action'):
        pass

    def unpackAssociatedEntity(self, importer: 'FIMimporter'):
        pass

class Task(Process):
    def __init__(self, ifcEntity: Node, associatedEntity: Node, hasSubprocess:bool = False) -> None:
        super().__init__(ifcEntity, associatedEntity, hasSubprocess)
        self._actions = []

    @property
    def workMethod(self):
        return self._ifcData['WorkMethod']
    
    def unpackAssociatedEntity(self, importer: 'FIMimporter'):
        if not self._associatedEntity:
            return
        
        if self.Name == "MoveTo":
            p0 = importer.currentWaypoint
            if "IfcFabricationLayer" in self._associatedEntity.labels:
                frame = importer.getLayerLocation(self._associatedEntity)
            elif "IfcVirtualElement" in self._associatedEntity.labels:
                frame = importer.readVE(self._associatedEntity) #importer.getStartingElement()
            else:
                raise Exception('associatedEntity not recognized!')
            frame['origin'] = (np.array(frame['origin']).reshape((3,1)) * importer.scalingFactor + importer.offsetValue).reshape((3,)).tolist()
            p1 = Waypoint.byFrame(frame, importer._kinematicSolver)
            path, importer._baseRot = importer._pathData.generateLIN(p0, p1)
            payload = []
            previous = p0._jointangles
            for wp in path:
                previous = wp.inverse(previous, importer._kinematicSolver)
                payload.append(wp)
            importer.currentWaypoint = path[-1]
            self._actions.append(Move(self, payload))
        elif self.Name == "Extrude":
            layer: Layer = importer.readLayer(self._associatedEntity, self.getProperty('Identification'))
            layer.setInitialCurveVelocities(0)
            layer.discretize(importer.dt, 0)
            # layer._path.remove(layer._path[0])
            previous = importer.currentWaypoint._jointangles
            payload = []
            for wp in layer._pathDiscretized:
                wp.setRot(importer._baseRot)
                previous = wp.inverse(previous, importer._kinematicSolver)
                payload.append(wp)
            importer.currentWaypoint = layer._pathDiscretized[-1]
            self._actions.append(Move(self, payload))
            self._actions.append(Extrude(self, layer.layerHeight))
        elif self.Name == "TakeScan":
            name = self._associatedEntity.get('Name', None)
            if not name:
                raise("Scan name not defined.")
            self._actions.append(UseSensor(self, name))
        else:
            pass
            # raise Exception("Action not supported")
    
    def readLayerPosition(self, importer):
        
        
        pass
    
class Event(Process):
    def __init__(self, ifcEntity) -> None:
        super().__init__(ifcEntity, None, False)

class Action():
    def __init__(self, parent: 'Process', job) -> None:
        self.target = []
        self.parentProcess = parent
        self.job = job

class Move(Action):
    def __init__(self, parent: 'Process', job) -> None:
        super().__init__(parent, job)
    pass

class Extrude(Action):
    pass

class KeepAlive(Action):
    pass

class UseSensor(Action):
    pass

class FIMimporter():
    def __init__(self, fimAccess: dict) -> None:
        self._connection = fimAccess
        
        self._component = None

        self._pathData = PathData()
        self._kinematicSolver = None
        self._currentTask = None
        self._baseRot = np.eye(3)
    
    @property
    def dt(self):
        return self._pathData.dt
        
    @property
    def scalingFactor(self):
        return self._pathData.scalingFactor
    
    @property
    def offsetValue(self):
        return self._pathData.offsetValue
    
    @property
    def currentWaypoint(self):
        return self._pathData._currentWaypoint
    
    @currentWaypoint.setter
    def currentWaypoint(self, wp: Waypoint):
        self._pathData.setCurrentWaypoint(wp)
    
    @property
    def kinematicSolver(self):
        return self._kinematicSolver
    
    @kinematicSolver.setter
    def kinematicSolver(self, kin: 'Screwkincalc'):
        self._kinematicSolver = kin
    
    def getLayerLocation(self, layer):
        pass

    def setPathData(self, pd: PathData) -> None:
        self._pathData = pd
    
    def setOffset(self, offset: NDArray[np.float64]) -> None:
        self._pathData.setOffset(offset)
    
    def initiateImport(self):
        pass

    def setConditions(self, **kwargs) -> None:
        self._pathData.setConditions(**kwargs)

    def getBottomEdgeLoop(self) -> List[Curve]:
        pass

    def getFootprintBounds(self)-> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        bottomEdgeLoop = self.getBottomEdgeLoop()
        maxXYZ = np.max([curve.maxXYZ for curve in bottomEdgeLoop], axis=0)
        minXYZ = np.min([curve.minXYZ for curve in bottomEdgeLoop], axis=0)
        return minXYZ, maxXYZ

    def calculateOffset(self, buildPlate: List[NDArray[np.float64]]) -> None:
        minP, maxP = self.getFootprintBounds()
        P0 = buildPlate[0]
        P1 = buildPlate[1]
        centerPlattform = np.mean([P0, P1], axis=0)
        centerComponent = np.mean([minP, maxP], axis=0)
        offset = (centerPlattform - centerComponent).reshape((3,1))

        #assert np.linalg.norm(P1-P0) >= np.linalg.norm(maxP-minP), "Object too large for build plattform!"
        #assert self._pathData is not None, "PathData not set!"
        self.setOffset(offset)
    
    def setInitialWaypointByJC(self, rc: list[float]) -> None:
        self._pathData.setInitialWaypointByJC(rc, self._kinematicSolver)

    def readFirstLayer(self):
        pass

    def readLayer(self, node: 'Node') -> 'Layer':
        return

    def readLayers(self, lim:int = None):
        pass

    def getStartingElement(self) -> Tuple[NDArray[np.float64], Node]:
        pass

    def readVE(self, entity: Node) -> NDArray[np.float64]:
        pass

    def moveStartingElement(self, newOrigin: NDArray[np.float64]):
        pass
        
    def extractTasks(self, tasks: 'Queue'):
        self.startSession()
        nol = 0
        while True:
            task = self.readNextTask()
            if not task:
                print("Done!")
                self.closeSession()
                return
            if task._associatedEntity and task.Name == 'Print' and 'IfcFabricationLayer' in task._associatedEntity.labels:
                nol += 1

            if nol > self._pathData.layerLimit and not task.Name == 'End':
                continue
            task.unpackAssociatedEntity(self)
            tasks.put(task)


    def readNextTask(self) -> 'Task':
        pass
    
    def startSession(self):
        pass

    def closeSession(self):
        pass
    
    #region static methods
    @staticmethod
    def newGUID():
        g = uuid.uuid4().hex
        bs = [int(g[i : i + 2], 16) for i in range(0, len(g), 2)]

        def b64(v, l=4):
            return "".join([chars[(v // (64**i)) % 64] for i in range(l)][::-1])

        return "".join([b64(bs[0], 2)] + [b64((bs[i] << 16) + (bs[i + 1] << 8) + bs[i + 2]) for i in range(1, 16, 3)])
    #endregion

class FileImporter(FIMimporter):
    def __init__(self, fimAccess: dict) -> None:
        super().__init__(fimAccess)
        self._ifcFilename = ""
        self._ifcFile = None

    
    def initiateImport(self) -> None:
        # self._ifcFilename = data["uri"]
        self._ifcFile = ifc.open(self._connection["uri"])
        # [DEPRECATED] FIM-Model contains only one component "aggregated" in IfcProject
        # TODO: not have Component in all examples an IfcWall but can be something different.
        project = self._ifcFile.by_type("IfcProject")[0] # There is only one IfcProject!
        
        if len(project.IsDecomposedBy) > 1:
            print("Warning: too many aggregations in IfcProject, defaulting to first Aggregation")

        component = [o for o in project.IsDecomposedBy[0].RelatedObjects if o.is_a("IfcBuildingElement")] # Carefull: IfcOpenShell issue -> should be IfcBuiltElement!
        if len(component) == 0:
            sites = [o for o in project.IsDecomposedBy[0].RelatedObjects if o.is_a("IfcSite")]
            if sites:
                if sites[0].ContainsElements:
                    self._ifcComponent = sites[0].ContainsElements[0].RelatedElements[0]
                else:
                    self._ifcComponent = sites[0].IsDecomposedBy[0].RelatedObjects[0].ContainsElements[0].RelatedElements[0]
        elif len(component) == 1:
            self._ifcComponent = self._ifcFile.by_type("IfcProject")[0].IsDecomposedBy[0].RelatedObjects[0]
        else:
            raise Exception("Data structure not supported")
    
    def readLayers(self, lim:int = np.iinfo(np.int32).max) -> None:
        # Component always aggregates layers
        #Layer._registeredLayers = self._pathData._layers
        for n,l in enumerate(self._ifcComponent.IsDecomposedBy[0].RelatedObjects):
            layer = Layer(n, l, self._pathData)
            self._readSurface(layer)
            self._readPath(layer)
            layer.scale(self.scalingFactor)
            layer.offset(self.offsetValue)
            if lim and n >= lim-1:
                break
        layer._topLayer = True
        logging.info(f"{n+1} layers read from ifc file {self._ifcFilename}:")

    def _readPath(self, layer: Layer) -> None:
        # get Axis Representation and respective ParentCurves
        axisRep = [r for r in layer._ifcData.Representation.Representations if r.RepresentationIdentifier == 'Axis'][0]
        pc = [seg.ParentCurve for seg in axisRep.Items[0].Segments]
        # TODO: data model needs to be updated to support IfcSurfaceCurve
        curveList = [Curve.byFile(curve, layer) for curve in pc]
        layer._path = curveList

    def _readSurface(self, layer: 'Layer') -> None:
        rep = [r for r in layer._ifcData.Representation.Representations if r.RepresentationIdentifier == 'Surface'][0]
        surfList = [Surface.byFile(surf, layer) for surf in rep.Items]
        layer._surface = surfList

class GraphImporter(FIMimporter):
    def __init__(self, fimAccess: dict) -> None:
        super().__init__(fimAccess)
        self._driver: Driver = None
        self._session: Session = None
    
    def startSession(self):
        self._driver = GraphDatabase.driver(**self._connection)
        self._session = self._driver.session(database='neo4j')

    def closeSession(self):
        self._session.close()
        self._driver.close()
        self._driver = None
        self._session = None

    def initiateImport(self) -> None:
        #Layer._registeredLayers = self._pathData._layers
        # self._connection = data
        # self.driver = GraphDatabase.driver(**self._connection)
        # self.session = self.driver.session(database='neo4j')
        self.startSession()
        tx = self._session.begin_transaction()
    	
        # Find start event
        query = """
            MATCH (se:IfcTask)<-[:RelatingProcess]-(:IfcRelAssignsToProcess)-[:RelatedObjects]->(:IfcBuiltElement) 
            OPTIONAL MATCH (se)<-[:RelatingProcess]-(n:IfcRelNests)
            RETURN se, n;
        """
        record = tx.run(query).single()
        self._currentTask = self._pathData._rootEvent = Task(record['se'], None, record['n']!=None)
        tx.commit()
    
    def getLayerLocation(self, layer: Node):
        tx = self._session.begin_transaction()
        query = """
            MATCH (l:IfcFabricationLayer) WHERE elementID(l) = $lID
            MATCH (l)-->(:IfcProductDefinitionShape)-->(:IfcShapeRepresentation)-->(:IfcSurface)-->(:IfcSurface)-[:Position]->(laxis:IfcAxis2Placement3D)
            MATCH (lo)<-[:Location]-(laxis)-[:Axis]->(lz), (laxis)-[:RefDirection]->(lx)
            MATCH (ve:IfcVirtualElement {Tag: "Init"})-->(:IfcLocalPlacement)-[:RelativePlacement]->(vaxis:IfcAxis2Placement3D)
            MATCH (vo)<-[:Location]-(vaxis)
            RETURN {origin: [vo.Coordinates[0], vo.Coordinates[1], vo.Coordinates[2]+lo.Coordinates[2]], zDir: lz.DirectionRatios, xDir: lx.DirectionRatios} as axisParameter
        """
        result = tx.run(query, lID=layer.element_id).single()
        tx.commit()
        return result['axisParameter']
    
    def getBottomEdgeLoop(self) -> List[Curve]:
        #super().getBottomEdgeLoop()
        tx = self._session.begin_transaction()
        query = """
            MATCH (:IfcGeometricRepresentationContext {ContextType: 'Model'})<--(:IfcTopologyRepresentation {RepresentationIdentifier: 'BottomSurface'})-[*3..3]->(:IfcEdgeLoop)-->(:IfcOrientedEdge)-[:EdgeElement]->(ec:IfcEdgeCurve)-[:EdgeGeometry]-(crv)
            MATCH (ePt)<-[:VertexGeometry]-(:IfcVertexPoint)<-[:EdgeEnd]-(ec)-[:EdgeStart]->(:IfcVertexPoint)-[:VertexGeometry]->(sPt)
            CALL apoc.do.when(\"IfcCircle\" in labels(crv), \"MATCH (crv)-[:Position]->(ax) MATCH (ref)<-[:RefDirection]-(ax)-[:Axis]->(nrm), (ax)-[:Location]->(o) RETURN 'Arc' as type, {radius:crv.Radius, xDir:ref.DirectionRatios, zDir:nrm.DirectionRatios, origin:o.Coordinates, startPt:sPt.Coordinates, endPt:ePt.Coordinates} as params\", 
            \"RETURN 'Line' as type, {startPt:sPt.Coordinates, endPt:ePt.Coordinates} as params\", 
            {crv:crv, sPt:sPt, ePt:ePt}) YIELD value 
            RETURN value.type as type, value.params as crvParameter;
        """
        result = tx.run(query).data()
        curveList = []
        for curve in result:
            c = Curve.byGraph(curve, None)
            if self.scalingFactor != 1.0:
                c.scale(self.scalingFactor)
            curveList.append(c)

        tx.commit()
        return curveList

    def getStartingElement(self) -> Tuple[dict, Node]:
        #super().getStartingElement()
        tx = self._session.begin_transaction()
        query = """
            MATCH (ve:IfcVirtualElement {Tag: 'Init'})-[:ObjectPlacement]->(:IfcLocalPlacement)-[:RelativePlacement]->(axis:IfcAxis2Placement3D)-[:Location]->(o:IfcCartesianPoint)
            MATCH (axis)-[:Axis]->(zDir), (axis)-[:RefDirection]->(xDir)
            RETURN {origin: o.Coordinates, zDir: zDir.DirectionRatios, xDir: xDir.DirectionRatios} as axisParameter, ve;
        """
        result = tx.run(query).single()
        tx.commit()
        return result['axisParameter'], result['ve'] 
            

        # if result[0][0] and 'IfcTask' in result[0][0].labels:
        #     self._currentTask = Task(result[0][0], result[0][1])
        # else:
        #     self._currentTask = Event(result[0][0])

    def readVE(self, entity: Node) -> NDArray[np.float64]:
        tx = self._session.begin_transaction()
        query = """
            MATCH (ve:IfcVirtualElement)-[:ObjectPlacement]->(:IfcLocalPlacement)-[:RelativePlacement]->(axis:IfcAxis2Placement3D)-[:Location]->(o:IfcCartesianPoint) WHERE elementId(ve) = $ve
            MATCH (axis)-[:Axis]->(zDir), (axis)-[:RefDirection]->(xDir)
            RETURN {origin: o.Coordinates, zDir: zDir.DirectionRatios, xDir: xDir.DirectionRatios} as axisParameter;
        """
        result = tx.run(query, ve=entity.element_id).single()
        tx.commit()
        return result['axisParameter']

    def moveStartingElement(self, newOrigin: NDArray[np.float64]) -> None:
        origin = ((newOrigin - self.offsetValue) / self.scalingFactor).reshape((3,))

        tx = self._session.begin_transaction()
        query = """
            MATCH (ve:IfcVirtualElement {Tag: 'Init'})-[:ObjectPlacement]->(:IfcLocalPlacement)-[:RelativePlacement]->(axis:IfcAxis2Placement3D)-[:Location]->(o:IfcCartesianPoint)
            SET o.Coordinates = $newOrigin
            RETURN ve;
        """
        result = tx.run(query, newOrigin=origin.tolist()).single()
        tx.commit()

    def selectLayerTransitionPosition(self):
        layer1 = self.readFirstLayer()
        self._pathData.selectStartEndPt(self, layer1, 0.0045)

    def readFirstLayer(self):
        tx = self._session.begin_transaction()
        query = (
            "MATCH (:IfcGeometricRepresentationContext {ContextType: 'Fabrication'})<--(rep:IfcShapeRepresentation {RepresentationIdentifier: 'Toolpath'})<--(:IfcProductDefinitionShape)<--(fl:IfcFabricationLayer) "
            "RETURN fl ORDER BY fl.Name LIMIT 1;"
        )
        result = tx.run(query).single()
        tx.commit()

        layer = Layer(0, result['fl'], self._pathData, False)
        self._readSurface(layer)
        self._readPath(layer)
        layer.scale(self.scalingFactor)
        layer.offset(self.offsetValue)
        return layer

    def readNextTask(self) -> 'Process':
        #super().readNextTask()
        tx = self._session.begin_transaction()

        if self._currentTask.hasSubprocess:
            query = """
                MATCH (se:IfcProcess)<-[:RelatingProcess]-(:IfcRelNests)-[:RelatedProcesses]->(nt:IfcProcess {Name: "Start"}) WHERE elementId(se)=$rtID 
                OPTIONAL MATCH (nt)<-[:RelatingProcess]-(rt:IfcRelAssignsToProcess)-[:RelatedObjects]->(be)
                OPTIONAL MATCH (nt)<-[:RelatingProcess]-(n:IfcRelNests)
                RETURN nt, be, n;
            """
        elif self._currentTask.Name == 'End':
            query = """
                MATCH (se:IfcProcess)<-[:RelatedProcesses]-(:IfcRelNests)-[:RelatingProcess]->(:IfcProcess)<-[:RelatingProcess]-(:IfcRelSequence)-[:RelatedProcess]-(nt) WHERE elementId(se)=$rtID 
                OPTIONAL MATCH (nt)<-[:RelatingProcess]-(rt:IfcRelAssignsToProcess)-[:RelatedObjects]->(be)
                RETURN nt, be, Null as n;
            """
            pass
        else:
            query = """
                MATCH (se:IfcProcess)<-[:RelatingProcess]-(:IfcRelSequence)-[:RelatedProcess]->(nt) WHERE elementId(se)=$rtID 
                OPTIONAL MATCH (nt)<-[:RelatingProcess]-(:IfcRelAssignsToProcess)-[:RelatedObjects]->(be)
                OPTIONAL MATCH (nt)<-[:RelatingProcess]-(n:IfcRelNests)
                RETURN nt, be, n;
            """

        record = tx.run(query, rtID=self._currentTask.elementId).single() # TODO: make it work for multiple tasks
        tx.commit()
        if record and record['nt'] and 'IfcTask' in record['nt'].labels:
            self._currentTask = Task(record['nt'], record['be'], record['n']!=None)
        elif record and record['nt'] and 'IfcEvent' in record['nt'].labels:
            self._currentTask = Event(record['nt'])
        else:
            return None
        
        return self._currentTask

    def readLayer(self, node: 'Node', identification: int) -> 'Layer':
        layer = Layer(identification, node, self._pathData)
        self._readSurface(layer)
        self._readPath(layer)
        layer.scale(self.scalingFactor)
        layer.offset(self.offsetValue)
        layer.rearrangePath()
        layer._topLayer = True
        print(f"{identification+1} layers read from ifc graph:")
        logging.info(f"{identification+1} layers read from ifc graph:")
        return layer

    def readLayers(self, lim:int = None) -> None:
        # Component always aggregates layers
        #Layer._registeredLayers = self._pathData._layers
        query = (
            "MATCH (:IfcGeometricRepresentationContext {ContextType: 'Fabrication'})<--(rep:IfcShapeRepresentation {RepresentationIdentifier: 'Axis'})<--(:IfcProductDefinitionShape)<--(fl:IfcFabricationLayer) "
            "RETURN fl ORDER BY fl.Name;"
        )
        with GraphDatabase.driver(**self._connection) as driver:
            result = driver.execute_query(query, database_="neo4j")
        for n,l in enumerate(result[0]):
            layer = Layer(n,l['fl'])
            self._readSurface(layer)
            self._readPath(layer)
            #obj.setData(l, n)
            if lim and n >= lim-1:
                break
        layer._topLayer = True
        logging.info(f"{n+1} layers read from ifc graph:")

    def insert_camera_task(self):
        self._insert_SenseTask()
        self._merge_virtual_elements()
        self._create_moveto_and_takescan_tasks()
        self._build_task_chain()

    def _insert_SenseTask(self):
        sequenceParams = [self.sequence_params("Task transition"), self.sequence_params("Layer transition")]
        senseTaskParams = self.task_params("SenseTask", "optical scan")

        tx = self._session.begin_transaction()
        query = """
        MATCH (layer:IfcFabricationLayer)<--()-->(process1:IfcProcess)<-[:RelatingProcess]-(seqNode:IfcRelSequence)-[:RelatedProcess]->(process2:IfcProcess)
        CREATE (layer)<-[:RelatedObjects {ListIdx: 0}]-(:IfcRelAssignsToProcess)-[:RelatingProcess]->(st:IfcTask:IfcProcess)
        SET st += $stparams
        SET st.Description = "Optical scan of the layer"

        CREATE (process1)<-[:RelatingProcess]-(newSeq1:IfcRelSequence)-[:RelatedProcess]->(st)
        CREATE (process2)<-[:RelatedProcess]-(newSeq2:IfcRelSequence)-[:RelatingProcess]->(st)
        SET newSeq1 += $seqparams[0]
        SET newSeq1.Description = "Transition from print task to sense task"
        SET newSeq2 += $seqparams[1]
        SET newSeq2.Description = "Transition from sense task to print task"

        // Remove the old sequence node.
        DETACH DELETE seqNode

        RETURN layer
        """
        tx.run(query, stparams=senseTaskParams, seqparams=sequenceParams)
        tx.commit()

    def sequence_params(self, name: str) -> dict:
        return {
            'GlobalId': self.newGUID(),
            'OwnerHistory': '$',
            'Name': name,
            'Description': "$",
            'TimeLag': '$',
            'SequenceType': 'FINISH_START',
            'UserDefinedSequenceType': '$'
        }
    
    def task_params(self, name:str = 'MoveTo', wm:str = 'LINEAR'):
        return {
            'GlobalId': self.newGUID(),
            'OwnerHistory': '$',
            'Name': name,
            'Description': 'Move to position',
            'OjectType': '$',
            'Identification': '0',
            'LongDescription': '$',
            'Status': 'NOTSTARTED',
            'WorkMethod': wm,
            'IsMilestone': False,
            'Priority': 0,
            'PredefinedType': 'MOVE'
        }

    def aggregate_params(self):
        return {
            'GlobalId': self.newGUID(),
            'OwnerHistory': '$',
            'Name': '$',
            'Description': '$'
        }

    def _merge_virtual_elements(self):
        tx = self._session.begin_transaction()
        query = """
        // Execution Query: Traverse from IfcFabricationLayer to IfcCompositeCurve,
        // then via the Segments relationship (which carries ListIdx) to IfcCompositeCurveSegment,
        // then via a variable-length path to an IfcCircle with Radius = 0.01, and finally to its IfcAxis2Placement3D.
        
        MATCH (layer:IfcFabricationLayer)-[*3..3]->(curve:IfcCompositeCurve)
        MATCH (layer)-->(:IfcProductDefinitionShape)-->(:IfcShapeRepresentation {RepresentationType: 'Surface3D'})-[:Items]->()-[:BasisSurface]->()-[:Position]->(ax:IfcAxis2Placement3D)
        
        CREATE (layer)-[:Aggregates]->(agg:IfcRelAggregates)
        SET agg.GlobalId = randomUUID(), agg.Name = "$"
        
        WITH agg, ax, curve, layer
        MATCH (z)-[:Axis]-(ax)-[:RefDirection]->(x)
        MATCH (curve)-[segRel:Segments]->(seg:IfcCompositeCurveSegment)
        MATCH (seg)-[*]->(trim:IfcTrimmedCurve)
        MATCH (trim)-[*]->(circle:IfcCircle)
        WHERE circle.Radius = 0.01
        MATCH (circle)-[*]->(placement:IfcAxis2Placement3D)-[:Location]->(o)
        WITH layer, segRel.ListIdx AS lidx, o, x, z, agg ORDER BY lidx
        WITH collect(o) AS origins, layer, x, z, agg
        UNWIND range(2, size(origins)-1) AS idx
            WITH layer, idx, [origins[idx].Coordinates[0] + 0.077*5, origins[idx].Coordinates[1]+0.032*5, origins[idx].Coordinates[2] + 0.01*5] AS coords, x, z, agg
            // Create a new VirtualElement with its local placement linked to the existing placement.
            CREATE (ve:IfcVirtualElement {GlobalId: randomUUID(), Name: layer.Name + "_Node_" + toString(idx)})
            // Link the aggregator to the VirtualElement with the ListIdx property.
            CREATE (agg)-[:RelatedElement {ListIdx: idx}]->(ve)
            // Link the FabricationLayer to the aggregator.
            CREATE (ve)-[:ObjectPlacement]->(lp:IfcLocalPlacement)-[:RelativePlacement]->(placement:IfcAxis2Placement3D)-[:Location]->(:IfcCartesianPoint:IfcPoint:IfcGeometricRepresentationItem {Coordinates: coords})
            CREATE (z)<-[:Axis]-(placement)-[:RefDirection]->(x)
        RETURN layer, agg, ve, lp, placement, idx
        """ # origin coordinates needed to be scaled! 
        tx.run(query)
        tx.commit()

    def _create_moveto_and_takescan_tasks(self):
        tx = self._session.begin_transaction()
        query = """
        // 1. For each layer, match the Print task and its connected SenseTask, then collect the VirtualElements (with their ListIdx) from that layer.
        MATCH (layer:IfcFabricationLayer)<-[:RelatedObjects]-(:IfcRelAssignsToProcess)-[:RelatingProcess]->(sense:IfcTask {Name:"SenseTask"})
        MATCH (layer)-[:Aggregates]->(agg:IfcRelAggregates)-[r:RelatedElement]->(ve:IfcVirtualElement)
        WITH sense, r.ListIdx AS idx, ve
        ORDER BY idx
        WITH sense, collect({ve:ve, idx: idx}) AS sortedEntries
        
        // 2. Create one nest node for this SenseTask and attach it.
        CREATE (sense)<-[:RelatingProcess]-(nest:IfcRelNests {GlobalId: randomUUID()})
        
        // 3. Create a single Start and End node for the overall chain.
        CREATE (nest)-[:RelatedProcesses]->(start:IfcProcess:IfcEvent {GlobalId: randomUUID(), Name:"Start", SequenceType:"FINISH_START", OwnerHistory:"$"})
        CREATE (nest)-[:RelatedProcesses]->(end:IfcProcess:IfcEvent {GlobalId: randomUUID(), Name:"End", SequenceType:"FINISH_START", OwnerHistory:"$"})
        WITH nest, start, end, sortedEntries
        
        // 4. For each VirtualElement, create a MoveTo and a TakeScan task.

        UNWIND sortedEntries AS entry
        WITH entry.ve AS ve, entry.idx AS idx, nest, start, end
        // Calculate the relationship order for MoveTo and TakeScan.
        WITH ve, nest, idx, (2 * idx) AS listIdxMove, ((2 * idx) + 1) AS listIdxTake
        CREATE (move:IfcTask:IfcProcess {GlobalId: randomUUID(), Name:"MoveTo", Status:"NOTSTARTED", PredefinedType:"MOVE", OwnerHistory:"$"})<-[:RelatingProcess]-(:IfcRelAssignsToProcess)-[:RelatedObjects]->(ve)
        CREATE (take:IfcTask:IfcProcess {GlobalId: randomUUID(), Name:"TakeScan", Status:"NOTSTARTED", OwnerHistory:"$"})<-[:RelatingProcess]-(:IfcRelAssignsToProcess)-[:RelatedObjects]->(ve)
        // Attach each task to the nest with the computed ListIdx.
        CREATE (nest)-[:RelatedProcesses {ListIdx: listIdxMove}]->(move)
        CREATE (nest)-[:RelatedProcesses {ListIdx: listIdxTake}]->(take)
        """
        tx.run(query)
        tx.commit()

    def _build_task_chain(self):
        tx = self._session.begin_transaction()
        query = """
        // 1. Match the nest for the SenseTask on a layer and get all tasks attached to it.
        MATCH (layer:IfcFabricationLayer)<-[:RelatedObjects]-(:IfcRelAssignsToProcess)-[:RelatingProcess]->(sense:IfcTask {Name:"SenseTask"})<--(nest:IfcRelNests)
        MATCH (nest)-[rel:RelatedProcesses]->(task:IfcTask)
        MATCH (nest)-[]->(start:IfcEvent {Name:"Start"})
        MATCH (nest)-[]->(end:IfcEvent {Name:"End"})
        WITH layer, sense, nest, task, rel.ListIdx AS listIdx, start, end
        ORDER BY listIdx  
        WITH layer, sense, collect(task) AS sortedTasks, start, end
        // Pre-bind the first and last tasks.
        WITH layer, sense, sortedTasks, start, end, head(sortedTasks) AS firstTask, last(sortedTasks) AS lastTask

        // 2. Link Start to the first task.
        CREATE (start)<-[:RelatingProcess]-(seqStart:IfcRelSequence {GlobalId: randomUUID(), Name:"ChainSeqStart"})-[:RelatedProcess]->(firstTask)
        
        WITH layer, sense, sortedTasks, start, end, lastTask
        OPTIONAL MATCH (nextLayer:IfcFabricationLayer)<-[:RelatedObjects]-(:IfcRelAssignsToProcess)-[:RelatingProcess]->(:IfcTask {Name:"Print"})-[:RelatedProcess]-(:IfcRelSequence)-[:RelatingProcess]-(sense)
        CALL apoc.do.when(nextLayer IS NOT NULL, 
            'CREATE (lastTask)<-[:RelatingProcess]-(seqToMove:IfcRelSequence {GlobalId: randomUUID(), Name:"ChainToFinalMove"})-[:RelatedProcess]->(moveToLayer:IfcTask:IfcProcess {Name:"MoveTo"})<-[:RelatingProcess]-(:IfcRelAssignsToProcess)-[:RelatedObjects {ListIdx: 0}]->(currentLayer) RETURN moveToLayer as n', 
            "RETURN lastTask as n", {nextLayer:nextLayer, lastTask:lastTask, currentLayer:layer}) YIELD value

        // 3. Link the last task to End.
        WITH value.n as lastTask, start, end, sortedTasks
        CREATE (lastTask)<-[:RelatingProcess]-(seqEnd:IfcRelSequence {GlobalId: randomUUID(), Name:"ChainSeqEnd"})-[:RelatedProcess]->(end)
        WITH start, end, sortedTasks, lastTask

        // 4. Chain consecutive tasks.
        UNWIND range(0, size(sortedTasks)-2) AS i
            WITH start, end, i, sortedTasks[i] AS currentTask, sortedTasks[i+1] AS nextTask
            CREATE (seqPair:IfcRelSequence {GlobalId: randomUUID(), Name:"ChainSeq"})
            CREATE (currentTask)<-[:RelatingProcess]-(seqPair)
            CREATE (seqPair)-[:RelatedProcess]->(nextTask)
    
        """
        tx.run(query)
        tx.commit()

    def _readSurface(self, layer: 'Layer', applyScaling: bool = True, applyOffset: bool = True) -> None:
        tx = self._session.begin_transaction()
        query = (
            "MATCH (n:IfcFabricationLayer {Name: $Name})-[*]->(:IfcShapeRepresentation {RepresentationIdentifier: \"Slice\"})-[:Items]->(surf)-[*1..2]->(spl:IfcAxis2Placement3D) "
            "MATCH (zDir)<-[:Axis]-(spl)-[:RefDirection]->(refDir), (spl)-[:Location]->(o) "
            "RETURN labels(surf) AS type, {zDir: zDir.DirectionRatios, xDir:refDir.DirectionRatios, origin:o.Coordinates} AS surfParameter;"
        )
        # with GraphDatabase.driver(**self._connection) as driver:
        result = tx.run(query, Name=layer._ifcData['Name']).data()
        surfList = [Surface.byGraph(surf, layer) for surf in result]
        layer._surface = surfList
        tx.commit()

    def _readPath(self, layer: 'Layer') -> None:
        tx = self._session.begin_transaction()
        query = (
            "MATCH (n:IfcFabricationLayer {Name: $Name})-->*(:IfcShapeRepresentation {RepresentationIdentifier: \"Toolpath\"})-->(cc:IfcCompositeCurve)-[ccs:Segments]->()-->(trim:IfcTrimmedCurve)-[:BasisCurve]->(crv) "
            "MATCH (pt1)<-[:Trim1]-(trim)-[:Trim2]->(pt2) "
            "CALL apoc.do.when(\"IfcCircle\" in labels(crv), \"MATCH (crv)-[:Position]->(ax) MATCH (ref)<-[:RefDirection]-(ax)-[:Axis]->(nrm), (ax)-[:Location]->(o) RETURN 'Arc' as type, {radius:crv.Radius, xDir:ref.DirectionRatios, zDir:nrm.DirectionRatios, origin:o.Coordinates, startPt:pt1.Coordinates, endPt:pt2.Coordinates} as params\", "
            "\"RETURN 'Line' as type, {startPt:pt1.Coordinates, endPt:pt2.Coordinates} as params\", "
            "{crv:crv, pt1:pt1, pt2:pt2}) YIELD value "
            "WITH value ORDER BY ccs.ListIdx "
            "RETURN value.type as type, value.params as crvParameter;"
        )
        # with GraphDatabase.driver(**self._connection) as driver:
        result = tx.run(query, Name=layer._ifcData['Name']).data()
        curveList = [Curve.byGraph(curve, layer) for curve in result]
        layer._path = curveList
        tx.commit()

    # @abstractmethod
    # def getTimeDerivative(timeSeries, dt = 1):
    #     d = []
    #     if len(timeSeries.shape) == 1:
    #         timeSeries = np.expand_dims(timeSeries, axis = 1)
    #     dims = timeSeries.shape[1]
    #     for i in range(dims):
    #         d.append(np.gradient(timeSeries[:,i]))
    #     d = np.transpose(np.array(d))/dt
    #     assert d.shape == timeSeries.shape 
    #     return d
    #endregion

# not fixed yet:
'''
    def _readNonPlanarSurfaces(self):
        """
            This function reads in the NonplanarSurfaces of the IFC File. And link the NonPlanarSurface 
            to the Layers. The NonPlanarSurface is saved as ControlPoints in each Layer.
        """
        shapereps = self.ifcFile.by_type('IFCSHAPEREPRESENTATION')
        for shaperep in shapereps:
            #filter for NonPlanarSurfaces
            if shaperep.get_info()["RepresentationIdentifier"]=="NonPlanarSurfaces":
                surfaces=shaperep.get_info()["Items"]
                #get the ControlPoints and the degrees in u and v
                for surface, layer in zip(surfaces,self.objLayers):
                    degreeu=surface.get_info()["UDegree"]
                    degreev=surface.get_info()["VDegree"]
                    cp=surface.get_info()["ControlPointsList"]
                    #setup controlpoint matrix
                    n_dim, n, m = 3, len(cp), len(cp[0])
                    P = np.zeros((n_dim, n, m))
                    #sort in the controlpoints
                    for i,row in enumerate(cp):
                        for j,entri in enumerate(row):
                            P[:, i, j]=np.asarray(entri.get_info()["Coordinates"])
                    layer.NonPlanarControlPoints=P  
    
'''

