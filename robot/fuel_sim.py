#! python3
"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2024
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""


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
from wpimath.kinematics import ChassisSpeeds

from field_const import FieldConstants

from commands2 import (
    Command,
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
)
from robot import Robot
import time
from wpilibextra.coroutine import CoroutineCommand
from wpilib import SmartDashboard

def dot_trans_2d(a: Translation2d, b : Translation2d):
    return a.X() * b.X() + a.Y() * b.Y()

def dot_trans_3d(a: Translation3d, b: Translation3d):
    return a.X() * b.X() + a.Y() * b.Y()+ a.Z() * b.Z()

class FuelSim:
    def __init__(self, robot : "Robot"):
        self.robot = robot

        self.period = 0.02
        self.subticks = 1
        self.gravity = Translation3d(0, 0, -9.81) # m/s^2
        
        #coefficients of restitution
        self.field_cor = math.sqrt(22 / 51.5)
        self.fuel_cor = 0.5
        self.net_cor = 0.2
        self.robot_cor = 0.1

        self.fuel_radius = 0.075
        self.field_length = 16.51
        self.field_width = 8.04
        self.friction = 0.1

        self.field_xz_line_starts = [
            Translation3d(0,0,0),
            Translation3d(3.96, 1.57, 0),
            Translation3d(3.96, self.field_width / 2 + 0.60, 0),
            Translation3d(4.61, 1.57, 0.165),
            Translation3d(4.61, self.field_width / 2 + 0.60, 0.165),
            Translation3d(self.field_length -5.18, 1.57, 0),
            Translation3d(self.field_length - 5.18, self.field_width / 2 + 0.60, 0),
            Translation3d(self.field_length - 4.61, 1.57, 0.165),
            Translation3d(self.field_length - 4.61, self.field_width / 2 + 0.60, 0.165)
        ]

        self.field_xz_line_ends = [
            Translation3d(self.field_length, self.field_width, 0),
            Translation3d(4.61, self.field_width / 2 - 0.60, 0.165),
            Translation3d(4.61, self.field_width - 1.57, 0.165),
            Translation3d(5.18, self.field_width / 2 - 0.60, 0),
            Translation3d(5.18, self.field_width - 1.57, 0),
            Translation3d(self.field_length - 4.61, self.field_width / 2 - 0.60, 0.165),
            Translation3d(self.field_length - 4.61, self.field_width - 1.57, 0.165),
            Translation3d(self.field_length - 3.96, self.field_width / 2 - 0.60, 0),
            Translation3d(self.field_length - 3.96, self.field_width - 1.57, 0)
        ]

        self.fuels : list[FuelSim.Fuel] = []
        self.fuels_to_update = set()
        self.running = False
        self.robot_supplier = self.robot.drivetrain.get_pose
        self.robot_speeds_supplier = self.robot.drivetrain.get_field_relative_speeds
        self.robot_width = inchesToMeters(35.87)
        self.robot_length = inchesToMeters(33.37)
        self.bumper_height = inchesToMeters(7.0)
        self.intake = FuelSim.SimIntake(self, inchesToMeters(16.69), inchesToMeters(26.18), inchesToMeters(-18.98), inchesToMeters(13.01), self.robot.intake.can_intake_sim, self.robot.intake.intake_sim_callback)

        self.blue_hub = self.Hub(self, Translation2d(4.61, self.field_width / 2), Translation3d(5.3, self.field_width / 2, 0.89), 1)
        self.red_hub = self.Hub(self, Translation2d(self.field_length - 4.61, self.field_width / 2), Translation3d(self.field_length - 5.3, self.field_width / 2, 0.89), -1)
        
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

            projected = start2d + lineVec * (dot_trans_2d(pos2d - start2d, lineVec) / (lineVec.norm() ** 2))

            if projected.distance(start2d) + projected.distance(end2d) > lineVec.norm():
                return #projected point not on line 
            dist = pos2d.distance(projected)
            if dist > self.sim.fuel_radius:
                return # not intersecting line
            
            normal = Translation3d(-lineVec.Y(), 0, lineVec.X()) / (lineVec.norm())

            self.pos += normal * (self.sim.fuel_radius - dist)
            if dot_trans_3d(self.vel, normal) > 0:
                return #already moving away from line
            self.vel -= normal * ((1 + self.sim.field_cor) * dot_trans_3d(self.vel, normal))

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
            self.handleHubCollisions(self.sim.blue_hub)
            self.handleHubCollisions(self.sim.red_hub)
        
        def handleHubCollisions(self, hub : "FuelSim.Hub"):
            hub.handleHubInteraction(self)
            collision = hub.fuelCollideSide(self)
            if collision.X() != 0:
                self.pos += Translation3d(collision)
                self.vel += Translation3d(-(1 + self.sim.field_cor) * self.vel.X(), 0, 0)
            elif collision.Y() != 0:
                self.pos += Translation3d(collision)
                self.vel += Translation3d(0, -(1 + self.sim.field_cor) * self.vel.Y(), 0)

            netCollision = hub.fuelHitNet(self)
            if netCollision != 0:
                self.pos += Translation3d(netCollision, 0, 0)
                self.vel = Translation3d(-self.vel.X() * self.sim.net_cor, self.vel.Y() * self.sim.net_cor, self.vel.Z())

        def addImpulse(self, impulse : Translation3d):
            self.vel += impulse

    def handleFuelCollisions(self):
        for i in range(len(self.fuels) - 1):
            for j in range(i + 1, len(self.fuels)):
                if self.fuels[i].pos.distance(self.fuels[j].pos) < self.fuel_radius * 2:
                    normal = self.fuels[i].pos - self.fuels[j].pos
                    distance = normal.norm()
                    if distance == 0:
                        normal = Translation3d(1, 0, 0)
                        distance = 1
                    normal = normal / distance
                    impulse = 0.5 * (1 + self.fuel_cor) * dot_trans_3d(self.fuels[j].vel - self.fuels[i].vel, normal)
                    intersection = self.fuel_radius * 2 - distance
                    self.fuels[i].pos += normal * (intersection / 2)
                    self.fuels[j].pos -= normal * (intersection / 2)

                    self.fuels[i].addImpulse(normal * impulse)
                    self.fuels[j].addImpulse(normal * -impulse)

                    self.fuels_to_update.add(self.fuels[i])
                    self.fuels_to_update.add(self.fuels[j])
    
    def clearFuel(self):
        self.fuels.clear()
        
    def spawnStartingFuel(self):
        # center fuel
        center = Translation3d(self.field_length / 2, self.field_width / 2, self.fuel_radius)
        for i in range(15):
            for j in range(6):
                self.fuels.append(FuelSim.Fuel(self, center + Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0), Translation3d()))
                self.fuels.append(FuelSim.Fuel(self, center + Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0), Translation3d()))
                self.fuels.append(FuelSim.Fuel(self, center + Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0), Translation3d()))
                self.fuels.append(FuelSim.Fuel(self, center + Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0), Translation3d()))

        # depots
        for i in range(3):
            for j in range(4):
                self.fuels.append(FuelSim.Fuel(self, Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, self.fuel_radius), Translation3d()))
                self.fuels.append(FuelSim.Fuel(self, Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, self.fuel_radius), Translation3d()))
                self.fuels.append(FuelSim.Fuel(self, \
                        Translation3d(self.field_length - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, self.fuel_radius), Translation3d()))
                self.fuels.append(FuelSim.Fuel(self, \
                        Translation3d(self.field_length - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, self.fuel_radius), Translation3d()))

        for index in range(len(self.fuels)):
            SmartDashboard.putNumberArray(f"Fuels/Fuel {index + 1}", [self.fuels[index].pos.X(), self.fuels[index].pos.Y(), self.fuels[index].pos.Z(), 0.0, 0.0, 0.0, 0.0])
        
    def start(self):
        self.running = True
        self.clearFuel()
        self.spawnStartingFuel()
    
    def stop(self):
        self.running = False
    
    def updateSim(self):
        if not self.running:
            return
        self.stepSim()

    def stepSim(self):
        for i in range(self.subticks):
            for i in range(len(self.fuels)):
                self.fuels[i].update()
            
            self.handleFuelCollisions()
            self.handleRobotCollisions()
            self.handleIntakes()
            self.logFuels()

    def logFuels(self):
        num_loops = 0
        for index in range(len(self.fuels)):
            if self.fuels[index] in self.fuels_to_update:
                num_loops += 1
                SmartDashboard.putNumberArray(f"Fuels/Fuel {index + 1}", [self.fuels[index].pos.X(), self.fuels[index].pos.Y(), self.fuels[index].pos.Z(), 0.0, 0.0, 0.0, 0.0])
        print(num_loops)
        self.fuels_to_update = set()
    def spawnFuel(self, pos : Translation3d, vel : Translation3d):
        self.fuels.append(FuelSim.Fuel(self, pos, vel))
    
    def handleRobotCollisions(self):
        robot = self.robot_supplier()
        speeds = self.robot_speeds_supplier()
        robot_vel = Translation2d(speeds.vx, speeds.vy)
        
        for i in range(len(self.fuels)):
            
            relative_pos = Pose2d(self.fuels[i].pos.toTranslation2d(), Rotation2d()).relativeTo(robot).translation()

            if self.fuels[i].pos.Z() > self.bumper_height:
                return
            distance_to_bottom = -self.fuel_radius - self.robot_length / 2 - relative_pos.X()
            distance_to_top = -self.fuel_radius - self.robot_length / 2 + relative_pos.X()
            distance_to_right = -self.fuel_radius - self.robot_length / 2 - relative_pos.Y()
            distance_to_left = -self.fuel_radius - self.robot_length / 2 + relative_pos.Y()

            if distance_to_bottom > 0 or distance_to_top > 0 or distance_to_right > 0 or distance_to_left > 0:
                return

            if distance_to_bottom >= distance_to_top \
                            and distance_to_bottom >= distance_to_right \
                            and distance_to_bottom >= distance_to_left: 
                posOffset = Translation2d(distance_to_bottom, 0)
            elif distance_to_top >= distance_to_bottom \
                            and distance_to_top >= distance_to_right \
                            and distance_to_top >= distance_to_left:
                posOffset = Translation2d(-distance_to_top, 0)
            elif distance_to_right >= distance_to_bottom \
                            and distance_to_right >= distance_to_top \
                            and distance_to_right >= distance_to_left:
                posOffset = Translation2d(0, distance_to_right)
            else:
                posOffset = Translation2d(0, -distance_to_left)

            posOffset = posOffset.rotateBy(robot.rotation())
            self.fuels[i].pos += Translation3d(posOffset)
            normal = posOffset / posOffset.norm()
            if dot_trans_2d(self.fuels[i].vel.toTranslation2d(), normal) < 0:
                self.fuels[i].addImpulse(
                        Translation3d(normal * -1 * dot_trans_2d(self.fuels[i].vel.toTranslation2d(), normal) * (1 + self.robot_cor)))
            if  dot_trans_2d(robot_vel, normal) > 0:
                self.fuels[i].addImpulse(Translation3d(normal * (dot_trans_2d(robot_vel, normal))))

            self.fuels_to_update.add(self.fuels[i])

    def handleIntakes(self):
        robot = self.robot_supplier()
        for fuel in self.fuels[:]:
            if self.intake.shouldIntake(fuel, robot):
                self.robot.fuel_in_hopper.append(fuel)
                self.fuels_to_update.add(fuel)
                self.fuels.remove(fuel)

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
            
            if (fuel.pos.X() < self.center.X() - self.side / 2
                or (
                    distance_to_left >= distance_to_right
                and distance_to_left >= distance_to_top
                and distance_to_left >= distance_to_bottom)):
                return Translation2d(distance_to_left, 0)
            elif (fuel.pos.X() >= self.center.X() + self.side / 2
                or (
                    distance_to_right >= distance_to_left
                and distance_to_right >= distance_to_top
                and distance_to_right >= distance_to_bottom)):
                return Translation2d(-distance_to_right, 0)
            elif (fuel.pos.Y() > self.center.Y() + self.side / 2
                or (
                    distance_to_top >= distance_to_left
                and distance_to_top >= distance_to_right
                and distance_to_top >= distance_to_bottom)):
                return Translation2d(0, -distance_to_top)
            else:
                return Translation2d(0, distance_to_bottom)
            
        def fuelHitNet(self, fuel : "FuelSim.Fuel"):
            if fuel.pos.Z() > self.net_height_max or fuel.pos.Z() < self.net_height_min:
                return 0
            
            if fuel.pos.Y() > self.center.Y() + self.net_width / 2 or fuel.pos.Y() < self.center.Y() - self.net_width / 2:
                return 0
            
            if fuel.pos.X() > self.center.X() + self.net_offset * self.exitVelXMult:
                return max(0, self.center.X() + self.net_offset * self.exitVelXMult - (fuel.pos.X() - self.sim.fuel_radius))
            else:
                return min(0, self.center.X() + self.net_offset * self.exitVelXMult - (fuel.pos.X() + self.sim.fuel_radius))
            
    class SimIntake():
        def __init__(self, sim : "FuelSim", x_min, x_max, y_min, y_max, able_to_intake, intake_callback):
            self.sim = sim
            
            self.x_min = x_min
            self.x_max = x_max
            self.y_min = y_min
            self.y_max = y_max

            self.able_to_intake = able_to_intake
            self.callback = intake_callback
            
        def shouldIntake(self, fuel : "FuelSim.Fuel", robot_pose : Pose2d):
            if (not self.able_to_intake) or (fuel.pos.Z() > self.sim.bumper_height):
                return False
            
            fuel_relative_pos = Pose2d(fuel.pos.toTranslation2d(), Rotation2d()).relativeTo(robot_pose).translation()
            result = fuel_relative_pos.X() >= self.x_min \
                    and fuel_relative_pos.X() <= self.x_max \
                    and fuel_relative_pos.Y() >= self.y_min \
                    and fuel_relative_pos.Y() <= self.y_max
            if result:
                self.callback()

            return result