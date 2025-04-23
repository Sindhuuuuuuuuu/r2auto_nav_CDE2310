# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import time
import heapq
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# constants
WALL_THRESHOLD = 10
wall_penalty = 1
cluster_distance = 8
localization_tolerance = 6
rotatechange = 1
speedchange = 0.065
dr = [-1,  0, 0, 1]
dc = [ 0, -1, 1, 0]
MOVE_COST = [1, 1, 1, 1]

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


class AutoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.can_update = True
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.visited_points = set()
        self.initial_angle = 0

        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def isValid(self, row, col):
        rowsize, colsize = self.cur_map.shape
        return 0 <= row < rowsize and 0 <= col < colsize and self.cur_map[row][col] < WALL_THRESHOLD and self.cur_map[row][col] != -1

    # def heuristic(self, curpoint, targetpoint):
    #     return (targetpoint[0] - curpoint[0]) ** 2 + (targetpoint[1] - curpoint[1]) ** 2

    def reconstruct_path(self, parent_map, start, target):
        path = []
        node = target
        while node != start:
            path.append(node)
            node = parent_map[node]
        path.append(start)
        path.reverse()
        return path
    
    def astar(self, target_row, target_col):
        start_row = self.currow
        start_col = self.curcol
        # Priority queue: (cost + heuristic, row, col)
        astar = []
        heapq.heappush(astar, (0, start_row, start_col))
        
        cost_map = { (start_row, start_col): 0 }
        parent_map = { (start_row, start_col): None } 

        find_path = False
        while astar:
            cost, currow, curcol = heapq.heappop(astar)

            if (currow, curcol) == (target_row, target_col):
                find_path = True
                break

            for idx in range(4):
                nextrow = currow + dr[idx]
                nextcol = curcol + dc[idx]
                if not self.isValid(nextrow, nextcol):
                    continue
                
                too_close_to_wall = False
                for penaltyx in range(-wall_penalty, wall_penalty + 1):
                    if too_close_to_wall:
                        break
                    for penaltyy in range(-wall_penalty, wall_penalty + 1):
                        next_row = nextrow + penaltyx
                        next_col = nextcol + penaltyy
                        if not self.isValid(next_row, next_col):
                            too_close_to_wall = True
                            break
                if too_close_to_wall:
                    continue

                nextcost = cost + MOVE_COST[idx]
                if (nextrow, nextcol) not in cost_map or nextcost < cost_map[(nextrow, nextcol)]:
                    cost_map[(nextrow, nextcol)] = nextcost
                    parent_map[(nextrow, nextcol)] = (currow, curcol)
                    # priority = nextcost + self.heuristic((nextrow, nextcol), (target_row, target_col))
                    heapq.heappush(astar, (nextcost, nextrow, nextcol))

        if find_path:
            print("A* Search found a path")
            path = self.reconstruct_path(parent_map, (start_row, start_col), (target_row, target_col))
            return path
        else:
            return []

    def point_to_point_distance(self, a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info("Received map data!")

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            return
            
        self.cur_pos = trans.transform.translation
        
        # convert quaternion to Euler angles
        # cur_rot = trans.transform.rotation
        # roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.roll,self.pitch,self.yaw = roll,pitch,yaw
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        if self.can_update:
            # get map resolution and map origin 
            self.map_res = msg.info.resolution
            self.map_origin = msg.info.origin.position
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            received_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            adjusted_map = np.fliplr(np.rot90(received_map))
            self.cur_map = adjusted_map
            self.get_logger().info(f'Updating Map!')

        # get map grid positions for x, y position
        grid_x = round((self.cur_pos.x - self.map_origin.x) / self.map_res) # column in numpy 
        grid_y = round(((self.cur_pos.y - self.map_origin.y) / self.map_res)) # row in numpy
        
        self.currow = self.map_width - 1 - grid_x
        self.curcol = self.map_height - 1 - grid_y
        self.visited_points.add((self.cur_pos.x, self.cur_pos.y))

        # self.get_logger().info("Robot position is at " + str(self.currow) + " " + str(self.curcol))
        # self.get_logger().info("The Grid value is " + str(self.cur_map[self.currow][self.curcol]))


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        # self.get_logger().info('Scan: ' + str(float(self.laser_range[-1])))


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # Ensure odom_callback updates yaw before computing target_yaw
        for _ in range(5): # Try multiple times to get an updated yaw
            rclpy.spin_once(self)
            time.sleep(0.1) # Allow time for odometry updates

        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def stopbot(self):
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
        self.get_logger().info('Stop the robot')


    def find_target(self):
        dis = -1
        targetrow, targetcol = -1, -1
        num_rows, num_cols = self.cur_map.shape
        self.get_logger().info('Finding Target')
        for idxrow in range(num_rows):
            for idxcol in range(num_cols):
                too_close_to_wall = False
                for penaltyx in range(-wall_penalty, wall_penalty + 1):
                    if too_close_to_wall:
                        break
                    for penaltyy in range(-wall_penalty, wall_penalty + 1):
                        next_row = idxrow + penaltyx
                        next_col = idxcol + penaltyy
                        if not self.isValid(next_row, next_col):
                            too_close_to_wall = True
                            break
                if too_close_to_wall:
                    continue
                if self.cur_map[idxrow][idxcol] == 0:
                    min_dis = math.inf
                    for x, y in self.visited_points:
                        grid_x = round((x - self.map_origin.x) / self.map_res)
                        grid_y = round(((y - self.map_origin.y) / self.map_res))
                        convertx = self.map_width - 1 - grid_x
                        converty = self.map_height - 1 - grid_y
                        min_dis = min(min_dis, (idxrow - convertx) ** 2 + (idxcol - converty) ** 2)
                    if dis < min_dis:
                        dis = min_dis
                        targetrow = idxrow
                        targetcol = idxcol
        return (targetrow, targetcol)

    def cluster_path(self, find_path):
        if find_path == []:
            self.get_logger().info('No path is found!')
            return
        clustered_path = []
        clustered_path.append(find_path[0])
        for idx in range(1, len(find_path)):
            point = find_path[idx]
            if self.point_to_point_distance(point, clustered_path[-1]) <= cluster_distance ** 2:
                continue
            clustered_path.append(point)
        return clustered_path
    

    def calculate_cw_rotation_angle(self, F, T):
        currow, curcol = F
        targetrow, targetcol = T
        coldiff = abs(curcol - targetcol)
        rowdiff = abs(currow - targetrow)
        if currow == targetrow and curcol == targetcol:
            return 0
        if currow > targetrow and curcol == targetcol:
            return 0
        if currow > targetrow and curcol < targetcol:
            return np.degrees(np.arctan(coldiff / rowdiff))
        if currow == targetrow and curcol < targetcol:
            return 90
        if currow < targetrow and curcol < targetcol:
            return 90 + np.degrees(np.arctan(rowdiff / coldiff))
        if currow < targetrow and curcol == targetcol:
            return 180
        if currow < targetrow and curcol > targetcol :
            return 180 + np.degrees(np.arctan(coldiff / rowdiff))
        if currow == targetrow and curcol > targetcol:
            return 270
        if currow > targetrow and curcol > targetcol:
            return 270 + np.degrees(np.arctan(rowdiff / coldiff))


    def move_through_path(self, points):
        if points == None:
            return
        twist = Twist()
        self.can_update = False
        self.get_logger().info(f'Path produced by A* Search')
        for row, col in points[1:]:
            self.get_logger().info(f'Row Col: {row}, {col}')
        for row, col in points[1:]:
            self.get_logger().info(f'Moving to {row}, {col}')
            distance = self.point_to_point_distance((self.currow, self.curcol), (row, col))
            first_time = True
            while distance > localization_tolerance ** 2:
                for _ in range(5):
                    rclpy.spin_once(self)
                    time.sleep(0.1)

                self.get_logger().info(f'Current row col: {self.currow}, {self.curcol}')
                # self.get_logger().info(f'Target row col: {row}, {col}')
                # self.get_logger().info(f'Current Angle: {self.initial_angle}')
                # self.get_logger().info(f'Distance to target: {distance}')

                distance = self.point_to_point_distance((self.currow, self.curcol), (row, col))

                self.visited_points.add((self.cur_pos.x, self.cur_pos.y))

                if first_time:
                    # Calculate the angle between (self.currow, self.curcol) and (row, col)
                    map_angle = self.calculate_cw_rotation_angle((self.currow, self.curcol), (row, col))
                    map_angle = 360 - map_angle
                    self.get_logger().info(f'Head up to the angle of {map_angle}')

                    # Rotate the robot to that angle
                    self.get_logger().info(f'Current Angle: {self.initial_angle}')
                    angle_to_rotate = (map_angle - self.initial_angle + 360) % 360
                    self.rotatebot(angle_to_rotate)
                    self.initial_angle = (self.initial_angle + angle_to_rotate) % 360
                    self.get_logger().info(f'Current Angle: {self.initial_angle}')

                    # Publish the twist
                    twist.linear.x = speedchange
                    self.publisher_.publish(twist)
                    first_time = False
            self.stopbot()
        self.can_update = True
        self.get_logger().info(f'Path traversed successfully')

    def mover(self):
        # Trigger all callbacks with a loop and timeout
        self.get_logger().info('Triggering all callbacks for 5 seconds')
        start_time = time.time()
        timeout = 5  # Set a short timeout to allow callbacks to be triggered
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)  # Short timeout for spinning once
        
        self.initial_angle = 0

        while True:
            print(f"Current position is at {self.currow}, {self.curcol}")
            targetrow, targetcol = self.find_target()
            print(f"Searching path to (x,y)=({targetrow},{targetcol})")

            if targetrow == -1 and targetcol == -1:
                start_time = time.time()
                timeout = 5  # Set a short timeout to allow callbacks to be triggered
                while time.time() - start_time < timeout:
                    rclpy.spin_once(self, timeout_sec=0.1)  # Short timeout for spinning once

                edited_map = self.cur_map
                np.savetxt("letsgo.txt", edited_map, fmt="%2d")
                continue

            find_path = self.astar(targetrow, targetcol)
            clustered_path = self.cluster_path(find_path)
            
            # Edit map and save to a text file
            edited_map = self.cur_map
            for x,y in find_path:
                edited_map[x][y] = -8 # path generated by the A* search
            if clustered_path != None:
                for x,y in clustered_path:
                    edited_map[x][y] = -9 # clustered path
            edited_map[self.currow][self.curcol] = -6
            for x, y in self.visited_points:
                grid_x = round((x - self.map_origin.x) / self.map_res)
                grid_y = round(((y - self.map_origin.y) / self.map_res))
                convertx = self.map_width - 1 - grid_x
                converty = self.map_height - 1 - grid_y
                edited_map[convertx][converty] = -2
            edited_map[targetrow][targetcol] = -5
            np.savetxt("letsgo.txt", edited_map, fmt="%2d")

            self.move_through_path(clustered_path)
            self.stopbot()
            self.get_logger().info('Triggering all callbacks for 5 seconds')
            start_time = time.time()
            timeout = 5  # Set a short timeout to allow callbacks to be triggered
            while time.time() - start_time < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)  # Short timeout for spinning once


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()