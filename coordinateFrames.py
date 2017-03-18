from numpy import *
from numpy.linalg import lstsq, svd, inv
from math498 import *
from math import *

class CoordinateFrames:
	def __init__(self):
		self.T = None
		self.T_inverse = None
		self.realWaypoint = None
		self.waypoint = None
		self.waypointYaw = None

	def calculateRealToCameraTransformation(self, cameraPts, realPts):
		# T turns cameraPts into realPts
		self.T = fitHomography(cameraPts, realPts)
		self.T_inverse = inv(self.T)

	def convertCameraToReal(self, arbitraryPt):
		pt = array([arbitraryPt[0], arbitraryPt[1], 1])
		newPt = applyHomography(self.T, pt)
		return array([newPt[0], newPt[1]])

	def convertRealToCamera(self, realPt):
		pt = array([realPt[0], realPt[1], 1])
		newPt = applyHomography(self.T_inverse, pt)
		return array([newPt[0], newPt[1]])

	def calculateRealToWaypointTransformation(self, waypoint1, waypoint2):
		v1 = array(waypoint2) - array(waypoint1)
		v1 = v1 / linalg.norm(v1)

		v2 = array([0., 0.])

		if floatClose(v1[0], 0):
			v2[0] = 1
			v2[1] = 0
		elif floatClose(v1[1], 0):
			v2[0] = 0
			v2[1] = 1
		else:
			alpha = v1[1] / v1[0]
			v2[1] = sqrt(1 / (1 + alpha**2))
			v2[0] = v2[1] * -alpha

		if cross(v1, v2) < 0:
			v2 = -v2

		self.realWaypoint = array([v1, v2])
		self.waypoint = waypoint1
		self.waypointYaw = atan2(self.realWaypoint[0, 1], self.realWaypoint[0, 0])

	def convertRealToWaypoint(self, pt):
		result = dot(self.realWaypoint, array(pt) - array(self.waypoint))
		return result

	def convertWaypointToReal(self, pt):
		result = dot(self.realWaypoint.T, array(pt)) + self.waypoint
		return result

	def getRealToWaypointYaw(self):
		return self.waypointYaw

	@staticmethod
	def rotateCCW(pt, yaw):
		"""
		Rotates a point by yaw number of radians counter clockwise
		"""
		cosTheta = cos(yaw)
		sinTheta = sin(yaw)
		x = cosTheta * pt[0] - sinTheta * pt[1]
		y = sinTheta * pt[0] + cosTheta * pt[1]
		return array([x, y])
