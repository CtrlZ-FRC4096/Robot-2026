#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2024
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

DEBUG = True

import logging

# Import our files


from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    CommandScheduler,
)
import wpilib
from wpilib import Timer, DataLogManager, DriverStation, Field2d
import wpilib.sysid
import wpimath.geometry
import const
import oi
import math
import random
import ntcore
import subsystems.drivetrain
from wpimath.units import inchesToMeters

from wpimath.estimator import SwerveDrive4PoseEstimator


from phoenix6 import controls

# import subsystems.limelight
import subsystems.leds

import subsystems.limelight
import subsystems.poseEstimator

from wpilibextra.coroutine.coroutine_robot import CoroutineRobot
from wpilibextra.remote_shell import RemoteShell
from coroutines import Coroutines

import inspect
import autoroutines

from pathplannerlib.path import PathPlannerPath, Waypoint, IdealStartingState, GoalEndState, PathPoint
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, NamedCommands, FollowPathCommand, PathConstraints
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Rotation2d, Pose2d, Translation2d, Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import degreesToRadians

from field_const import FieldConstants

from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)
import time
from wpilibextra.coroutine import CoroutineCommand
from wpilib import SmartDashboard

class FuelSim:
    period = 0.02
    subticks = 5
    gravity = Translation3d(0, 0, -9.81) # m/s^2
    
    #coefficients of restitution
    field_cor = math.sqrt(22 / 51.5)
    fuel_cor = 0.5
    net_cor = 0.2
    robot_cor = 0.1

    fuel_radius = 0.075
    field_length = 16.51
    field_width = 8.04
    friction = 0.1

    instance = None

    field_xz_line_starts = [
        Translation3d(0,0,0),
        Translation3d(3.96, 1.57, 0),
        Translation3d(3.96, field_width / 2 + 0.60, 0),
        Translation3d(4.61, 1.57, 0.165),
        Translation3d(4.61, field_width / 2 + 0.60, 0.165),
        Translation3d(field_length -5.18, 1.57, 0),
        Translation3d(field_length - 5.18, field_width / 2 + 0.60, 0),
        Translation3d(field_length - 4.61, 1.57, 0.165),
        Translation3d(field_length - 4.61, field_width / 2 + 0.60, 0.165)
    ]

    field_xz_line_ends = [
        Translation3d(field_length, field_width, 0),
        Translation3d(4.61, field_width / 2 - 0.60, 0.165),
        Translation3d(4.61, field_width - 1.57, 0.165),
        Translation3d(5.18, field_width / 2 - 0.60, 0),
        Translation3d(5.18, field_width - 1.57, 0),
        Translation3d(field_length - 4.61, field_width / 2 - 0.60, 0.165),
        Translation3d(field_length - 4.61, field_width - 1.57, 0.165),
        Translation3d(field_length - 3.96, field_width / 2 - 0.60, 0),
        Translation3d(field_length - 3.96, field_width - 1.57, 0)
    ]

    class Fuel():
        def __init__(self, sim : "FuelSim", pos : Translation3d, vel : Translation3d):
            self.sim = sim
            self.pos = pos
            self.vel = vel if vel is not None else Translation3d()
        
        def update(self):
            self.pos += self.vel * (self.sim.period / self.sim.subticks)
            if self.pos.Z() > self.sim.fuel_radius:
                self.vel += self.sim.gravity * (self.sim.period / self.sim.subticks)
            
            if abs(self.vel.Z()) < 0.05 and self.pos.Z() <= self.sim.fuel_radius + 0.03:
                self.vel = Translation3d(self.vel.X(), self.vel.Y(), 0)
                self.vel *= 1 - (self.sim.friction * self.sim.period / self.sim.subticks)
            self.handleFieldCollisions()
        
        def handleXZLineCollisions(self, lineStart : Translation3d, lineEnd : Translation3d):
            if self.pos.Y() < lineStart.Y() or self.pos.Y() > lineEnd.Y():
                return
            start2d = Translation2d(lineStart.X(), lineStart.Z())
            end2d = Translation2d(lineEnd.X(), lineEnd.Z())
            pos2d = Translation2d(self.pos.X(), self.pos.Z())
            lineVec = end2d - start2d

            projected = start2d + (lineVec * (pos2d - start2d).dot(lineVec) / lineVec.squaredNorm())

            if projected.distance(start2d) + projected.distance(end2d) > lineVec.norm():
                return #projected point not on line 
            dist = pos2d.distance(projected)
            if dist > self.sim.fuel_radius:
                return # not intersecting line
            
            normal = Translation3d(-lineVec.Y(), 0, lineVec.X()) / (lineVec.norm())

            self.pos += normal * (self.sim.fuel_radius - dist)
            if self.vel.dot(normal) > 0:
                return #already moving away from line
            self.vel -= normal * ((1 + self.sim.field_cor) * self.vel.dot(normal))

        def handleFieldCollisions(self):
            # floor and bumps
            for i in range(len(self.sim.field_xz_line_starts)):
                self.handleXZLineCollisions(self.sim.field_xz_line_starts[i], self.sim.field_xz_line_ends[i])
            
            #edges
            if self.pos.X() < self.sim.fuel_radius and self.vel.X() < 0:
                self.pos += Translation3d(self.sim.fuel_radius - self.pos.X(), 0, 0)
                self.vel += Translation3d(-(1 + self.sim.field_cor) * self.vel.X(), 0, 0)
            elif self.pos.X() > self.sim.field_length - self.sim.fuel_radius and self.vel.X() > 0:
                self.pos += Translation3d(self.sim.field_length - self.sim.fuel_radius - self.pos.X(), 0, 0)
                self.vel += Translation3d(-(1 + self.sim.field_cor) * self.vel.X(), 0, 0)
            
            if self.pos.Y() < self.sim.fuel_radius and self.vel.Y() < 0:
                self.pos += Translation3d(0, self.sim.fuel_radius - self.pos.Y(), 0)
                self.vel += Translation3d(0, -(1 + self.sim.field_cor) * self.vel.Y(), 0)
            elif self.pos.Y() > self.sim.field_width - self.sim.fuel_radius and self.vel.Y() > 0:
                self.pos += Translation3d(0, self.sim.field_width - self.sim.fuel_radius - self.pos.Y(), 0)
                self.vel += Translation3d(0, -(1 + self.sim.field_cor) * self.vel.Y(), 0)

            # hubs
           #2 handleHubCollisions
    
    class Hub():
        def __init__(self, sim : "FuelSim", center : Translation2d, exit : Translation3d, exitVelXMult : int):
            # constants
            self.sim = sim
            
            self.entry_height = 1.83
            self.entry_radius = 0.56

            self.side = 1.2

            self.net_height_max = 3.057
            self.net_height_min = 1.5
            self.net_offset = self.side / 2 + 0.261
            self.net_width = 1.484

            self.center = center
            self.exit = exit
            self.exitVelXMult = exitVelXMult
            self.score = 0
        
        def handleHubInteraction(self, fuel : "FuelSim.Fuel"):
            if self.didFuelScore(fuel):
                fuel.pos = self.exit
                fuel.vel = self.getDispersalVelocity()
                self.score += 1

        def didFuelScore(self, fuel : "FuelSim.Fuel"):
            return (fuel.pos.toTranslation2d().distance(self.center) <= self.entry_radius 
                and fuel.pos.Z() <= self.entry_height
                and (fuel.pos - (fuel.vel * (self.sim.period / self.sim.subticks))).Z() > self.entry_height
            )
        
        def getDispersalVelocity(self):
            return Translation3d(self.exitVelXMult * (random.random() + 0.1) *1.5,
                                 random.random() * 2 - 1,
                                 0)
        
        def resetScore(self):
            self.score = 0
        
        def getScore(self):
            return self.score
        
        def fuelCollideSide(self, fuel : "FuelSim.Fuel"):
            if fuel.pos.Z() > self.entry_height - 0.1:
                return Translation2d()
            
            distance_to_left = self.center.X() - self.side / 2 - self.sim.fuel_radius - fuel.pos.X()
            distance_to_right = fuel.pos.X() - self.center.X() - self.side / 2 - self.sim.fuel_radius
            distance_to_top = self.center.Y() - self.side / 2 - self.sim.fuel_radius - fuel.pos.Y()
            distance_to_bottom = fuel.pos.Y() - self.center.Y() - self.side / 2 - self.sim.fuel_radius

            if distance_to_left > 0 or distance_to_right > 0 or distance_to_top > 0 or distance_to_bottom > 0:
                return Translation2d() # not inside hub
            
            
            
