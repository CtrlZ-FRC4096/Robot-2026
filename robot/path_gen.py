from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import (
    Pose2d,
    Translation2d,
    Rotation2d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)
import heapq

from wpilib import DriverStation
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpimath.units import inchesToMeters, degreesToRadians, radiansToDegrees
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from field_const import FieldConstants
import math
import numpy as np
from wpilib import SmartDashboard
# from shapely.geometry import Polygon, Point

class QueueNode():
    def __init__(self, data, cost):
        self.data = data
        self.cost = cost

class PriorityQueue():
    def __init__(self):
        self.nodes = []

    def add(self, data, cost):
        self.nodes.append(QueueNode(data, cost))

    def remove(self):
        lowest = self.nodes[0].cost
        lowestIndex = 0

        for i in range(len(self.nodes)):
            if self.nodes[i].cost < lowest:
                lowest = self.nodes[i].cost
                lowestIndex = i

        return self.nodes.pop(lowestIndex).data
    def isEmpty(self):
        return len(self.nodes) == 0

# class Obstacle():
#     def __init__(self, lowerLeftCorner : Translation2d, upperRightCorner : Translation2d):
#         buffer = 0.5
#         x = [lowerLeftCorner.X(), upperRightCorner.X()]
#         y = [lowerLeftCorner.Y(), upperRightCorner.Y()]

#         self.lowerLeft = Translation2d(min(x) - buffer, min(y) - buffer)
#         self.upperRight = Translation2d(max(x) + buffer, max(y) + buffer)

class ObstacleRotation():
    def __init__(self, center : Translation2d, width : float, height : float, rotation : Rotation2d):
        buffer = 0.5
        self.center = center
        self.width = width + 2 * buffer
        self.height = height + 2 * buffer
        self.rotation = rotation

def get_path_to_reef(use_calibrated_field, face: int, right_branch: bool, margin_dist_offset=1.0, do_side_offset=True, do_manip_offset=True):
        manip_offset = 3.25

        side_offset = inchesToMeters(19)#(inchesToMeters(6.47) if not do_manip_offset else (inchesToMeters(6.47 + manip_offset) if right_branch else inchesToMeters(6.47 - manip_offset)))  # distance b/w center of face to branch
        dist_offset = (
            (inchesToMeters(29.5) / 2) + (inchesToMeters(7.25) / 2) + inchesToMeters(margin_dist_offset)
        )  # robot size + bumper addition + error protection

        center_face_pose = FieldConstants.flip_Pose2d(FieldConstants.Reef.centerFaces[face - 1])
        angle_face = center_face_pose.rotation()

        # manip_distance = 38
        center_face_x = (
            center_face_pose.X() #+ inchesToMeters(manip_distance)*math.sin(angle_face.radians())
        )  # pose of center face (this is directly on the side of the reef)
        center_face_y = center_face_pose.Y() #+ inchesToMeters(manip_distance)*math.cos(angle_face.radians())
        x_offset = (
            math.cos(angle_face.radians()) * dist_offset
        )  # offsetting that pose by a set offset that extends the pose as if there's a vector from the center face with angle: angle_face
        y_offset = math.sin(angle_face.radians()) * (dist_offset)
        target_pose_face = Pose2d(
            center_face_x + x_offset, center_face_y + y_offset, angle_face - Rotation2d.fromDegrees(90)
        )

        if do_side_offset:
            angle_to_branch = (
                (angle_face.degrees() + 90) % 360
                if right_branch
                else (angle_face.degrees() - 90) % 360
            )  # angle change needed to do math to get to the branch, right branch needs + 90 degrees (CCW), left_branch needs -90 (CW)

            x_offset_branch = (
                math.cos(degreesToRadians(angle_to_branch)) * side_offset
            )  # same as above, extending the pose from the point outside of the reef in the direction of the desired branch
            y_offset_branch = math.sin(degreesToRadians(angle_to_branch)) * side_offset

            target_pose_3 = Pose2d(
                target_pose_face.X() + x_offset_branch,
                target_pose_face.Y() + y_offset_branch,
                Rotation2d.fromDegrees(
                    angle_face.degrees() - 90
                ),  # don't know if this + 90 is needed, because our battery is facing forward and we want the camera side (scoring side) to face reef
            )

            if use_calibrated_field:
                alliance_color = "red" if DriverStation.getAlliance() == DriverStation.Alliance.kRed else "blue"
                branch = "right" if right_branch else "left"
                target_pose_3 = FieldConstants.ReefCalibratedToField.calibrated_data[alliance_color][branch][face]

            return target_pose_3
        else:
            return target_pose_face

class ObstacleConstants():
    buffer = 0.8
    buffer_2 = 0.45
    reefVertices = [
            get_path_to_reef(False, 1, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
            get_path_to_reef(False, 2, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
            get_path_to_reef(False, 3, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
            get_path_to_reef(False, 4, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
            get_path_to_reef(False, 5, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
            get_path_to_reef(False, 6, True, margin_dist_offset=-18.375, do_side_offset=True, do_manip_offset=False),
        ]#face's right branch point
    reefAngles = []
    for idx in range(6):
        # face: idx + 1
        angle_face = FieldConstants.Reef.centerFaces[idx].rotation().degrees()
        angle_face_minus_1 = FieldConstants.Reef.centerFaces[(idx - 1) % 6].rotation().degrees()
        delta_angle = (angle_face_minus_1 - angle_face) % 360
        if delta_angle > 180:
            delta_angle -= 360
        angle_mid = (angle_face + delta_angle / 2) % 360
        reefAngles.append(angle_mid)
    # reefAngles = [(FieldConstants.Reef.centerFaces[idx].rotation().degrees() -  FieldConstants.Reef.centerFaces[(idx - 1) % 6].rotation().degrees()) // 2 for idx in range(6)]
    reefForInReef = []
    reefForNormal = []
    for idx in range(6):
        reefForNormal.append(Translation2d(reefVertices[idx].X() + (buffer * math.cos(degreesToRadians(reefAngles[idx]))), reefVertices[idx].Y() + (buffer * math.sin(degreesToRadians(reefAngles[idx])))))
        reefForInReef.append(Translation2d(reefVertices[idx].X() + (buffer_2 * math.cos(degreesToRadians(reefAngles[idx]))), reefVertices[idx].Y() + (buffer_2 * math.sin(degreesToRadians(reefAngles[idx])))))
    
    #obstacleList = [Obstacle(Translation2d(3.0, 3.0), Translation2d(4.5, 4.5))]


class PathGenerator():
    def __init__(self, initialPosition : Pose2d, finalPosition : Pose2d, useObstacles=True):
        self.initialPosition = FieldConstants.flip_Pose2d(initialPosition)
        self.finalPosition = FieldConstants.flip_Pose2d(finalPosition)
        self.lastSlope = 1
        self.currentSlope = 1
        self.useObstacles = useObstacles
        self.smoothity = 6
        self.tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWeldedWelded)
        self.controlPoints = self.buildPath(self.astar(initialPosition.translation(), finalPosition.translation()))
        #self.removeDuplicateSlopes()
        self.controlPoints.reverse()
        self.allPoints = self.getPointList()
        self.smooth_path = self.smooth_points(self.allPoints, 0.001, 0.999, 0.07)
        #self.prunePath()

    def generate_circle_poses(self, center: Translation2d, radius: float, num_points: int) -> list[Pose2d]:
        poses = []
        for i in range(num_points):
            theta = (i / num_points) * 2 * math.pi  # Evenly distribute points around the circle
            x = center.X() + radius * math.cos(theta)
            y = center.Y() + radius * math.sin(theta)
            
            # Set rotation to be tangent to the circle (theta + 90 degrees)
            rotation = Rotation2d(theta + math.pi / 2)
            
            poses.append(Pose2d(Translation2d(x, y), rotation))
        
        return poses
    def calculate_closest_reef_tag(self, curPose : Pose2d):
        min_distance_to_tag = math.inf
        closest_reef_tag = None
        for tagID in FieldConstants.reef_tags:
            tag_pose = self.tag_layout.getTagPose(tagID).toPose2d()
            distance = (curPose - tag_pose).translation().norm()
            if distance < min_distance_to_tag:
                min_distance_to_tag = distance
                closest_reef_tag = tagID
        return [closest_reef_tag, FieldConstants.tag_to_face[closest_reef_tag]]
    def find_closest_reef_exit(self, initialPose : bool):
        if initialPose:
            circle_points = self.generate_circle_poses(self.initialPosition.translation(), 0.6, 30)
            min_dist_to_target = math.inf
            min_dist_point_idx = 0
            angle_pm_90 = FieldConstants.Reef.centerFaces[self.calculate_closest_reef_tag(self.initialPosition)[1] - 1].rotation().degrees()
            for idx, point in enumerate(circle_points):
                theta = radiansToDegrees((idx / 30) * 2 * math.pi)
                print("theta ", theta, "for idx ", idx)
                diff = abs(theta - angle_pm_90)
                if diff > 180:
                    diff = 360 - diff
                if diff <= 72.5:
                    if (self.finalPosition.translation() - point.translation()).norm() < min_dist_to_target:
                        min_dist_to_target = (self.finalPosition.translation() - point.translation()).norm()
                        min_dist_point_idx = idx
            return circle_points[min_dist_point_idx]
        else:
            circle_points = self.generate_circle_poses(self.finalPosition.translation(), 0.6, 30)
            min_dist_to_target = math.inf
            min_dist_point_idx = 0
            angle_pm_90 = FieldConstants.Reef.centerFaces[self.calculate_closest_reef_tag(self.finalPosition)[1] - 1].rotation().degrees()
            print("plus minus angle ", angle_pm_90)
            for idx, point in enumerate(circle_points):
                theta = radiansToDegrees((idx / 30) * 2 * math.pi)
                print("theta ", theta, "for idx ", idx)
                diff = abs(theta - angle_pm_90)
                if diff > 180:
                    diff = 360 - diff
                if diff <= 72.5:
                    print(idx, " no obstacle")
                    if (self.initialPosition.translation() - point.translation()).norm() < min_dist_to_target:
                        print(idx, " lower dist")
                        min_dist_to_target = (self.initialPosition.translation() - point.translation()).norm()
                        min_dist_point_idx = idx
            return circle_points[min_dist_point_idx]
    
    class PathNode():
        def __init__(self, position, finalPosition, parent=None):
            self.position = position
            self.finalPosition = finalPosition
            self.parent = parent

    # def getSmoothPath(self):
    #     return self.smooth_path

    def inReef(self, pose : Translation2d, useForExit=False):
        if useForExit:
            hexagon_points = [(ObstacleConstants.reefForInReef[idx].X(), ObstacleConstants.reefForInReef[idx].Y()) for idx in range(6)]
            # hexagon = Polygon(hexagon_points)
            # point = Point(pose.X(), pose.Y())
            # return hexagon.contains(point)
            return True
        else:
            # normal pathfind
            hexagon_points = ((ObstacleConstants.reefForNormal[idx].X(), ObstacleConstants.reefForNormal[idx].Y()) for idx in range(6))
            # hexagon = Polygon(hexagon_points)
            #print("pose in reef test ", pose)
            # point = Point(pose.X(), pose.Y())
            # return hexagon.contains(point)
            return True
        
    def inObstacle(self, pose : Translation2d) -> bool:
        if self.inReef(pose):
                return True
        return False

    def obstacleBetween(self, initialPose : Translation2d, finalPose : Translation2d, useForExit=False, doLessSteps = False):
        steps = 8 if doLessSteps else 50
        step = Translation2d((finalPose.X() - initialPose.X()) / steps,  (finalPose.Y() - initialPose.Y()) / steps)
        # print("init pose between", initialPose)
        # print("final pose between", finalPose)
        for _ in range(steps):
            if self.inReef(initialPose, useForExit):
                return True
            initialPose = initialPose.__add__(step)
        return False

    def getNeighbors(self, node : PathNode, finalPosition : Translation2d) -> list[PathNode]:
        neighbors = []
        step_size = 1 / self.smoothity
        angle_step = 360 / 12
        for angle in range(0, 360, int(angle_step)):
            rad = degreesToRadians(angle)
            dx = math.cos(rad) * step_size
            dy = math.sin(rad) * step_size
            neighbor = Translation2d(round(node.position.X() + dx, 3), round(node.position.Y() + dy, 3))
            if self.useObstacles:
                if not(self.inObstacle(neighbor)):
                    element = self.PathNode(neighbor, finalPosition)
                    neighbors.append(element)
            else:
                element = self.PathNode(neighbor, finalPosition)
                neighbors.append(element)
        return neighbors

    def astar(self, initialPosition : Translation2d, finalPosition : Translation2d):
        frontier = PriorityQueue()
        frontier.add(self.PathNode(initialPosition, finalPosition), 0)
        visited = []
        node_idx = 1
        while not(frontier.isEmpty()):
            # print(node_idx)
            currentNode = frontier.remove()
            # print(currentNode.position)
            if (currentNode.position - finalPosition).norm() < 2 / self.smoothity:
                return currentNode
            for child in self.getNeighbors(currentNode, finalPosition):
                if not self.pointInVisited(visited, child.position):
                    visited.append(child.position)
                    child.parent = currentNode
                    frontier.add(child, (child.position - finalPosition).norm())# + ((self.find_nearest_point_on_reef(child.position) - child.position).norm() * 10))# + (child.position - initialPosition).norm())# + (((1 / self.find_nearest_point_on_reef(child.position) - child.position).norm()) * 3) +\# + (child.position - initialPosition).norm())
            node_idx += 1
        return None
    def pointInVisited(self, visitedList : list[Translation2d], pose : Translation2d):
        for point in visitedList:
            if (pose - point).norm() <= 0.02:
                return True
        return False

    def buildPath(self, finalNode : PathNode | None):
        path = []
        currentNode = finalNode
        while currentNode.parent != None:
            path.append(currentNode.position)
            currentNode = currentNode.parent
        return path

    def getSlope(self, first : Translation2d, second : Translation2d):
        if (first.X() - second.X()) == 0:
            return self.lastSlope
        slope = abs(first.Y() - second.Y()) / abs(first.X() - second.X())
        return slope

    def removeDuplicateSlopes(self):
        if len(self.controlPoints) < 2:
            return
        newPath = []
        self.lastSlope = self.getSlope(self.initialPosition.translation(), self.controlPoints[1])
        for idx in range(len(self.controlPoints) - 1):
            self.currentSlope = self.getSlope(self.controlPoints[idx], self.controlPoints[idx + 1])
            if self.currentSlope != self.lastSlope:
                newPath.append(self.controlPoints[idx])
            self.lastSlope = self.currentSlope
        self.controlPoints = newPath

    def prunePath(self):
        for i in reversed(range(len(self.smooth_path))):
            if not(self.obstacleBetween(self.initialPosition.translation(), self.smooth_path[i])):
                del self.smooth_path[0:i]
                break
        for i in range(len(self.smooth_path)):
            if not(self.obstacleBetween(self.finalPosition.translation(), self.smooth_path[i])):
                del self.smooth_path[i+1:]
                break

    def getPointList(self):
        points = self.controlPoints# + [self.finalPosition.translation()]#[self.initialPosition.translation()] + self.controlPoints + [self.finalPosition.translation()]
        return points

    def closest_point_on_segment(self, point: Translation2d, seg_start: Translation2d, seg_end: Translation2d):
        """ Returns the closest point on the segment [seg_start, seg_end] to the given point. """
        seg_vector = seg_end - seg_start
        point_vector = point - seg_start
        seg_length_squared = seg_vector.norm() ** 2  # Squared length of the segment

        if seg_length_squared == 0:  # Segment is just a point
            return seg_start

        # Project point onto the line (but not necessarily the segment)
        t = ((point_vector.X() * seg_vector.X()) + (point_vector.Y() * seg_vector.Y())) / seg_length_squared

        # Clamp t to the segment [0,1]
        t = max(0, min(1, t))

        # Compute closest point
        return seg_start + seg_vector * t

    def find_nearest_point_on_reef(self, position: Translation2d):
        min_distance = math.inf
        closest_point = None

        reef_vertices = ObstacleConstants.reefForNormal

        for i in range(len(reef_vertices)):
            p1 = reef_vertices[i]
            p2 = reef_vertices[(i + 1) % len(reef_vertices)]  # Loop around to form closed shape

            # Find nearest point on segment p1 -> p2
            nearest_point = self.closest_point_on_segment(position, p1, p2)
            distance = (position - nearest_point).norm()

            if distance < min_distance:
                min_distance = distance
                closest_point = nearest_point

        return closest_point

    def smooth_points(self, path: list, weight_smoothing, weight_data, tolerance):
        newPath = [p for p in path]  # Copy path to avoid modifying the original
        change = tolerance
        iteration = 0
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):  # Exclude first and last point
                # Cache values to reduce redundant calls
                old_x, old_y = newPath[i].X(), newPath[i].Y()
                path_x, path_y = path[i].X(), path[i].Y()
                prev_x, prev_y = newPath[i-1].X(), newPath[i-1].Y()
                next_x, next_y = newPath[i+1].X(), newPath[i+1].Y()
                # dynamic_smoothing = weight_smoothing *  (0.5 ** iteration)
                # dynamic_data = weight_data * (0.5 ** iteration)
                dynamic_smoothing = weight_smoothing / (1 + iteration * 0.2)
                dynamic_data = weight_data / (1 + iteration * 0.2)

                # Smoothing formula
                new_x = old_x + dynamic_smoothing * (path_x - old_x) + dynamic_data * (prev_x + next_x - 2 * old_x)
                new_y = old_y + dynamic_smoothing * (path_y - old_y) + dynamic_data * (prev_y + next_y - 2 * old_y)
                new_point = Translation2d(new_x, new_y)

                # Obstacle check only if significant change
                if abs(old_x - new_x) + abs(old_y - new_y) < 0.01:
                    continue

                # Skip if obstacle detected
                if self.obstacleBetween(Translation2d(old_x, old_y), new_point):
                    continue

                # Track change
                change += abs(old_x - new_x) + abs(old_y - new_y)
                newPath[i] = new_point
            iteration += 1
            print(f"Total change after iteration: {change} and iteration {iteration}")  # Print only once per iteration
        return newPath


class PurePursuitController():
    def __init__(self, lookahead_dist, smooth_path):
        self.last_closest_point_idx = 0
        self.lookahead_dist = lookahead_dist
        self.last_lookahead_point_idx = 0
        self.last_lookahead_point = None
        self.path = smooth_path

    def getClosestPoint(self, curPose : Pose2d, start_idx : int):
        min_dist = math.inf
        # print("len path: ", len(self.path))
        for i in range(start_idx, len(self.path) - 2):
            dist = (self.path[i] - curPose.translation()).norm()
            if dist < min_dist:
                min_dist = dist
                start_idx = i
        # print("closest idx: ", start_idx) # Ensure progress
        self.last_closest_point_idx = start_idx
        return [self.path[start_idx], start_idx]



        # path = self.path
        # closest_point = path[0]
        # closest_point_idx = 0 # default
        # for idx in range(len(path)):
        #     if (curPose.translation() - path[idx]).norm() <= (curPose.translation() - closest_point).norm():
        #         closest_point = path[idx]
        #         closest_point_idx = idx
        # self.last_closest_point_idx = closest_point_idx
        # return [closest_point, closest_point_idx]

    def getLookaheadIntersectionAllPath(self, curPose: Pose2d):
        best_lookahead = curPose.translation()
        best_alignment = -1
        robot_heading = curPose.rotation()
        robot_direction = Translation2d(robot_heading.cos(), robot_heading.sin())

        for idx in range(self.getClosestPoint(curPose, 0)[1] + 1, len(self.path) - 2):
            intersections = self.getLookaheadIntersection(curPose, idx)
            #print(intersections)
            if intersections:
                for lookahead in intersections:
                    path_segment = (self.path[idx  + 1] - self.path[idx])
                    alignment = robot_direction.X() * path_segment.X() + robot_direction.Y() * path_segment.Y()

                    if alignment > best_alignment:
                        best_alignment = alignment
                        best_lookahead = lookahead
            if best_lookahead == curPose.translation():
                continue
            else:
                self.last_lookahead_point = best_lookahead
                self.last_lookahead_point_idx = idx
                break
        else:
            # print("no intersections")
            best_lookahead = self.path[self.getClosestPoint(curPose, 0)[1] + 1] # SHOULDN'T NEED THIS

        # print("best lookahead: ", best_lookahead)

        return best_lookahead



    def getLookaheadIntersection(self, curPose : Pose2d, start_point_idx : int):
        path = self.path
        start_point = path[start_point_idx]
        end_point = path[start_point_idx + 1]

        direction_vector = end_point - start_point
        pose_to_start_point = start_point - curPose.translation()

        a = (direction_vector.X() ** 2) + (direction_vector.Y() ** 2)  # Squared magnitude of d
        b = 2 * (pose_to_start_point.X() * direction_vector.X() + pose_to_start_point.Y() * direction_vector.Y())  # Interaction between d and f
        c = ((pose_to_start_point.X() ** 2) + (pose_to_start_point.Y() ** 2)) - (self.lookahead_dist ** 2)  # Determines circle intersection condition

        discriminant = (b ** 2) - (4 * a * c)
        if discriminant < 0:
            return False
        discriminant = math.sqrt(discriminant)

        intersections = []
        candidate_intersection_1 = (-b - discriminant) / (2 * a)
        candidate_intersection_2 = (-b + discriminant) / (2 * a) # because quadratic equation is plus-minus

        for candidate in [candidate_intersection_1, candidate_intersection_2]:
            if 0 <= candidate <= 1:
                intersection = Translation2d(
                    start_point.X() + candidate * direction_vector.X(),
                    start_point.Y() + candidate * direction_vector.Y()
                )

                # Ensure forward progress (dot product check)
                lookahead_direction = intersection - curPose.translation()
                if (lookahead_direction.X() * direction_vector.X() + lookahead_direction.Y() * direction_vector.Y()) > 0:
                    intersections.append(intersection)


        if intersections == []:
            return False
        else:
            # print("have intersections: ", intersections)
            return intersections
    def isAtEnd(self, curPose : Pose2d):
        if (curPose.translation() - self.path[-1]).norm() < 0.3: #if we are closer than 0.3 meters
            return True
        return False

    def getVelocities(self, curPose : Pose2d):# last_tick_velocities : Translation2d):
        # lookahead_point = self.getLookaheadIntersectionAllPath(curPose)

        if self.isAtEnd(curPose):
            return False
        # try:
        #     lookahead_point = self.path[self.getClosestPoint(curPose, 0)[1] + 10]
        # except:
        #     lookahead_point = self.path[self.getClosestPoint(curPose, 0)[1] + 1]
        lookahead_point = self.getLookaheadIntersectionAllPath(curPose)
        vx = 1 * (lookahead_point.X() - curPose.X())
        vy = 1 * (lookahead_point.Y() - curPose.Y())
        # print("in velocities")
        SmartDashboard.putString("cur pose", str(curPose))
        SmartDashboard.putString("lookahead point", str(lookahead_point))
        SmartDashboard.putNumber("vx raw", vx)
        SmartDashboard.putNumber("vy raw", vy)

        good_vx = min(vx, 0.2) if vx > 0 else max(vx, -0.2)
        good_vy = min(vy, 0.2) if vy > 0 else max(vy, -0.2)


        # if (vx - last_tick_velocities.X()) / 0.05 > 4.0 or (vy - last_tick_velocities.Y()) / 0.05 > 4.0:
        #     vx = last_tick_velocities.X()
        return Translation2d(good_vx, good_vy)