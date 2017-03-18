from common import *
from numpy.random import randn, uniform
from numpy import *
from numpy.linalg import *
from coordinateFrames import *
from joy import *
import pdb

class Particle:
  def __init__(self, d=0, yaw=0, prob=0):
    self.d = d
    self.yaw = yaw
    self.prob = prob

class ParticleFilter:
  """
  Particle Filter that estimates
  (r, d, theta) with respect to the current waypoint line
  Particles consist of (d, theta)
  """
  def __init__(self, core):
    # noise constants
    self.xNoise = 0.1 # cm
    self.yNoise = 0.1 # cm
    self.yawNoise = 0.002 # radians


    self.alpha = 0.4
	
    self.coordinateFrames = core.coordinateFrames

    # particles
    self.numChosenParticles = 100
    self.numRandomParticles = 40 # must be multiple of 4!
    self.numTotalParticles = self.numChosenParticles + self.numRandomParticles
    self.particles = []


    self.equalProbability = 1.0 / self.numTotalParticles
    for i in range(self.numTotalParticles):
      initialPos = 0
      initialYaw = 0
      self.particles.append(Particle(initialPos, initialYaw, self.equalProbability))

    self.mostProbable = self.particles[0]
    self.sensor = []
    self.waypoints = []

    # state
    self.r = 0

  def setState(self, pos, yaw):
    """
    input is position and yaw in real coordinates
    """
    [r, d] = self.coordinateFrames.convertRealToWaypoint(pos)
    self.r = r
    for particle in self.particles:
      randPos = randn() * 5
      randYaw = randn() * 0.05

      particle.prob = self.equalProbability
      particle.d = d + randPos
      particle.yaw = -self.coordinateFrames.getRealToWaypointYaw() + yaw + randYaw

    self.particles[0].d = d
    self.particles[0].yaw = -self.coordinateFrames.getRealToWaypointYaw() + yaw
    self.mostProbable = self.particles[0]

  def printParticles(self):
    for particle in self.particles:
      print("pos: " + str(particle.d) + "\tyaw: " + str(particle.yaw) + "\tprob: " + str(particle.prob))


  def setSensorAndWaypoints(self, sensor, waypoints):
    self.sensor = sensor
    if (len(waypoints) != len(self.waypoints)):
      self.coordinateFrames.calculateRealToWaypointTransformation(waypoints[0], waypoints[1])
      self.setState(waypoints[0], 0)
    self.waypoints = waypoints

  def correct(self):
    totalProb = 0
    sensorReal = array(ParticleFilter.convertSensor(self.sensor))

    # try to localize R
    self.correctR(sensorReal, self.sensor)

    for particle in self.particles:
      rotatedLength = CoordinateFrames.rotateCCW([Constants.tagLength, 0], particle.yaw)
      frontDist = abs(particle.d + rotatedLength[1])
      backDist = abs(particle.d - rotatedLength[1])

      self.sensorModel(particle, sensorReal, array([frontDist, backDist]))
      totalProb += particle.prob


    self.particles.sort(key=lambda x: x.prob, reverse=True)
    # self.mostProbable.d = self.mostProbable.d + self.alpha * (self.particles[0].d - self.mostProbable.d)
    # self.mostProbable.yaw = self.mostProbable.yaw + self.alpha * (self.particles[0].yaw - self.mostProbable.yaw)
    self.mostProbable = self.particles[0]


    # normalize probabilities so they add to one
    scalar = 1.0 / totalProb
    for particle in self.particles:
      particle.prob *= scalar


    # draw new samples
    newParticles = []
    for i in range(self.numChosenParticles):
      randNum = uniform()

      cumulativeProb = 0
      drewParticle = False
      for particle in self.particles:
        cumulativeProb += particle.prob
        if (cumulativeProb > randNum):
          newParticle = Particle(particle.d, particle.yaw, self.equalProbability)
          newParticles.append(newParticle)
          drewParticle = True
          break
      if (not drewParticle):
        newParticles.append(self.mostProbable)

    # draw scattering of particles around each solution
    solutions = ParticleFilter.generateSolutions(sensorReal)
    numParticlesPerSolution = self.numRandomParticles / len(solutions)
    for i in range(self.numRandomParticles):
      randPos = randn() * 1.5
      randYaw = randn() * 0.03

      solutionNum = int(floor(i / numParticlesPerSolution))
      solution = solutions[solutionNum]

      newParticles.append(Particle(solution[0] + randPos, solution[1] + randYaw, self.equalProbability))


    self.particles = newParticles


  def correctR(self, sensorReal, sensor):
    # modify self.r if necessary
    rotatedLength = CoordinateFrames.rotateCCW([Constants.tagLength, 0], -self.coordinateFrames.getRealToWaypointYaw())

    # locations of the front and back sensor of most probable particle
    frontR = self.r + rotatedLength[0]
    backR = self.r - rotatedLength[0]
    waypointDist = norm(array(self.waypoints[1]) - array(self.waypoints[0]))

    print("Sensor: " + str(sensor) + " estimated: " + str([frontR, backR]))



    if (sensorReal[1] == -1 and sensorReal[0] == -1):
      # we are completely not between waypoints
      if ((backR > 0 or backR < waypointDist) or
        (frontR > 0 or frontR < waypointDist)):
        # our estimate is that at least one of our sensors is still
        # in range of waypoints

        # determine which end we are off of by looking at our current estimate
        if (self.r < waypointDist / 2):
          # closest to start
          # decrease r
          print("Case 1 start")
          self.r = 0
        else:
          # closest to end
          # increase r
          print("Case 1 end")
          self.r = waypointDist
    elif (sensorReal[1] == -1 or sensorReal[0] == -1):
      # one sensor is no longer in range of waypoints
      if (backR > 0 and frontR > 0 and backR < waypointDist and frontR < waypointDist):
        # if we think we are in range of waypoints

        # find which end we are closest to
        if (self.r < waypointDist / 2):
          # we are close to start
          # move r to within range [0, rotatedLength[1]]
          print("Case 2 a start")
          print("actual f and b: " + str(sensorReal) + "\t" + str(sensor))
          print("estimated f and b: " + str([frontR, backR]))
          print("before r: " + str(self.r))
          self.r = self.r - abs(rotatedLength[0]) * 0.2
          print("after r: " + str(self.r))
        else:
          # we are close to end
          # move r to within range
          print("Case 2 a end")
          self.r = self.r + abs(rotatedLength[0]) * 0.2

      if ((backR < 0 or backR > waypointDist) and 
        (frontR < 0 or frontR > waypointDist)):
        # our estimate is completely off


        # find which end we are closest to
        if (self.r < waypointDist / 2):
          # we are close to start
          # move r to within range [0, rotatedLength[1]]
          print("Case 2 b start")
          self.r = uniform() * abs(rotatedLength[0])
        else:
          # we are close to end
          # move r to within range
          print("Case 2 b end")
          self.r = waypointDist - (uniform() * abs(rotatedLength[0]))
    elif (backR < 0 or backR > waypointDist or 
      frontR < 0 or frontR > waypointDist):
      # we estimate that our sensors are not between waypoints
      # however we are getting measurements

      # find which end we are closest to
      if (self.r < waypointDist / 2):
        # we are close to start
        # move r to within range [0, rotatedLength[1]]
        print("Case 3 start")
        print("actual f and b: " + str(sensorReal) + "\t" + str(sensor))
        print("estimated f and b: " + str([frontR, backR]))
        print("before r: " + str(self.r))
        self.r = abs(rotatedLength[0])
        print("after r: " + str(self.r))
      else:
        # we are close to end
        # move r to within range
        print("Case 3 end")
        print("actual f and b: " + str(sensorReal) + "\t" + str(sensor))
        print("estimated f and b: " + str([frontR, backR]))
        print("before r: " + str(self.r))
        self.r = waypointDist - abs(rotatedLength[0])
        print("after r: " + str(self.r))


  @staticmethod
  def generateSolutions(sensor):
    solutions = []

    f = sensor[0]
    b = sensor[1]

    d1 = (f+b)/2
    theta1 = atan2((f-b), 2 * Constants.tagLength)

    d2 = (f-b)/2
    theta2 = atan2((f+b), 2 * Constants.tagLength)

    solutions.append([d1, theta1])
    solutions.append([-d1, -theta1])

    solutions.append([d2, theta2])
    solutions.append([-d2, -theta2])

    return solutions

  def sensorModel(self, particle, sensorDists, particleDists):
    # distsDiff measures how close the sensor values are
    # to what the distance of this particle to the line
    if (sensorDists[0] == -1):
      distsDiff = sqrt(2) * abs(sensorDists[1] - particleDists[1])
    elif (sensorDists[1] == -1):
      distsDiff = sqrt(2) * abs(sensorDists[0] - particleDists[0])
    elif (sensorDists[0] == -1 and sensorDists[1] == -1):
      distsDiff = 0
    else:
      distsDiff = norm(sensorDists - particleDists)

    # Yawdiff measures how close the yaw of the particle is to our 
    # estimated yaw
    yawScalar = 50
    # yawDiff = yawScalar * abs(particle.yaw - self.mostProbable.yaw)
    yawDiff = yawScalar * abs(particle.yaw + self.coordinateFrames.getRealToWaypointYaw())

    # scalar = 0.2231
    scalar = 0.11

    # the probability we multiply our particle prob by,
    # p = P(sensor value | particle location), will be
    # e^(-(distsDiff + yawDiff) * scalar)
    # so that when distsDiff+yawDiff is 0, p=1
    # and for distsDiff+yawDiff > 0, p < 1 and drops off rapidly
    # how rapid the drop depends on the scalar
    sensorProb = exp(-(distsDiff + yawDiff) * scalar)
    particle.prob *= sensorProb

  def actionModel(self, direction):
    travelDist = array([0., 0.])
    noiseVar = 0
    if direction == (Directions.PosX):
      travelDist[0] += Constants.wheelSideLength
      noiseVar = self.xNoise
    elif direction == (Directions.NegX):
      travelDist[0] -= Constants.wheelSideLength
      noiseVar = self.xNoise
    elif direction == (Directions.PosY):
      travelDist[1] += Constants.wheelSideLength
      noiseVar = self.yNoise
    elif direction == (Directions.NegY):
      travelDist[1] -= Constants.wheelSideLength
      noiseVar = self.yNoise
    travelDir = travelDist / Constants.wheelSideLength
 

    rotatedTravelDist = CoordinateFrames.rotateCCW(travelDist, -self.coordinateFrames.getRealToWaypointYaw())

    self.r += rotatedTravelDist[0]

    # move each particle with noise
    for particle in self.particles:
      particleTravelDist = travelDist + (travelDir * noiseVar * randn())
      rotatedTravelDist = CoordinateFrames.rotateCCW(particleTravelDist, particle.yaw)
      particle.d += rotatedTravelDist[1]
      particle.yaw += randn() * self.yawNoise

  def getState(self):
    realPos = self.coordinateFrames.convertWaypointToReal([self.r, self.mostProbable.d])
    return RobotState(realPos, self.mostProbable.yaw)


  def getWaypointState(self):
    return RobotState([self.r, self.mostProbable.d], self.mostProbable.yaw)

  @staticmethod
  def convertSensor(sensor):
    # converts sensor values into real distances
    # model for sensor is
    # y: real dist
    # x: sensor dist
    # y = t * sqrt((255 / x) - 1)


    t = 6.5 # cm

    ret = [-1, -1]
    if (sensor[0] >= 10):
      ret[0] = t * sqrt((255. / sensor[0]) - 1)
    if (sensor[1] >= 10):
      ret[1] = t * sqrt((255. / sensor[1]) - 1)

    return ret



