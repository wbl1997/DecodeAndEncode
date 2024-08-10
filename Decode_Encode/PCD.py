#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import os
import math
import json
import random
from collections import defaultdict, OrderedDict


class PointCloudSaver:
    def __init__(self, topic, save_dir, frame_count):
        self.topic = topic
        self.save_dir = save_dir + '/3D'
        self.frame_count = frame_count
        self.current_frame = 0

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.save_dir_2d = os.path.join(save_dir, '2D')
        if not os.path.exists(self.save_dir_2d):
            os.makedirs(self.save_dir_2d)
        self.save_dir_json = os.path.join(save_dir, 'JSON')
        if not os.path.exists(self.save_dir_json):
            os.makedirs(self.save_dir_json)

        self.gt_poses = self.read_gt_poses('data1_gt.txt')

        rospy.init_node('pointcloud_saver', anonymous=True)
        rospy.Subscriber(self.topic, PointCloud2, self.callback)
        rospy.spin()

    def callback(self, data):
        if self.current_frame < self.frame_count:
            self.save_pointcloud(data)
            self.current_frame += 1
        else:
            rospy.signal_shutdown('Finished saving point clouds')

    def save_pointcloud(self, data):
        points = []
        for point in pc2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points.append(point)

        pcd_filename = os.path.join(self.save_dir, "frame_{}.pcd".format(self.current_frame))
        self.write_pcd_ascii(pcd_filename, points)

        # Convert to 2D point cloud and save
        points_2d = self.convert_to_2d(points)
        points_2d_sampled = self.sample_points(points_2d)
        pcd_2d_filename = os.path.join(self.save_dir_2d, "frame_{}.pcd".format(self.current_frame))
        self.write_pcd_ascii(pcd_2d_filename, points_2d_sampled)
        rospy.loginfo("Saved {}".format(pcd_2d_filename))

        # Generate JSON metadata file
        self.generate_json(data, points, points_2d_sampled)

    def write_pcd_ascii(self, filename, points, fields=["x", "y", "z", "intensity"]):
        header = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS {fields}
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {width}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points}
DATA ascii
""".format(
            fields=" ".join(fields),
            width=len(points),
            points=len(points)
        )

        with open(filename, 'w') as f:
            f.write(header)
            for point in points:
                f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(point[0], point[1], point[2], point[3]))

    def convert_to_2d(self, points):
        points_2d = []
        for point in points:
            x, y, _, intensity = point
            points_2d.append((x, y, 0.0, intensity))
        return points_2d

    def sample_points(self, points, distance_threshold=0.05):
        grid_size = distance_threshold
        grid = defaultdict(list)

        for point in points:
            grid_key = (int(point[0] // grid_size), int(point[1] // grid_size))
            grid[grid_key].append(point)

        sampled_points = []
        for cell_points in grid.values():
            averaged_point = self.average_points(cell_points)
            sampled_points.append(averaged_point)

        return sampled_points

    def average_points(self, points):
        x = sum(point[0] for point in points) / len(points)
        y = sum(point[1] for point in points) / len(points)
        z = 0.0
        intensity = sum(point[3] for point in points) / len(points)
        return (x, y, z, intensity)

    def read_gt_poses(self, filename):
        gt_poses = []
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                timestamp = float(parts[0])
                pos = list(map(float, parts[1:4]))
                quat = list(map(float, parts[4:]))
                gt_poses.append((timestamp, pos, quat))
        return gt_poses

    def find_closest_gt_pose(self, timestamp):
        closest_pose = min(self.gt_poses, key=lambda pose: abs(pose[0] - timestamp))
        return closest_pose

    def quaternion_to_yaw(self, quat):
        # Assuming the quaternion is [x, y, z, w]
        _, _, z, w = quat
        yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z * z))
        return yaw

    def generate_json(self, data, points_3d, points_2d):
        timestamp = data.header.stamp.secs + data.header.stamp.nsecs / 1e9
        closest_gt_pose = self.find_closest_gt_pose(timestamp)
        pos_yaw = closest_gt_pose[1] + [round(self.quaternion_to_yaw(closest_gt_pose[2]), 6)]

        json_data = OrderedDict([
            ("device_id", 100002),
            ("device_time", timestamp),
            ("pos_yaw", pos_yaw),
            ("floor", 7),
            ("is_exit", random.random() < 0.1),
            ("fire_point", None if random.random() < 0.9 else closest_gt_pose[1]),
            ("3dpcd_num", len(points_3d)),
            ("2dpcd_num", len(points_2d))
        ])

        json_filename = os.path.join(self.save_dir_json, "frame_{}.json".format(self.current_frame))
        with open(json_filename, 'w') as f:
            json.dump(json_data, f, indent=4)
        rospy.loginfo("Saved {}".format(json_filename))


if __name__ == '__main__':
    topic = '/livox/lidar'
    save_dir = './PCD'
    frame_count = 100

    PointCloudSaver(topic, save_dir, frame_count)
