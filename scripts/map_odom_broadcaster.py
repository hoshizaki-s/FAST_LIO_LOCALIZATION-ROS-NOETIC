#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
import tf.transformations as tft
import numpy as np

class MapOdomBroadcaster:
    def __init__(self):
        rospy.init_node('map_odom_broadcaster')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf.TransformBroadcaster()
        
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_transform)

    def transform_to_matrix(self, transform):
        """TransformStampedからHomogeneous transformation matrixを作成"""
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # クォータニオンから回転行列を作成
        matrix = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        # 並進を設定
        matrix[0:3, 3] = [trans.x, trans.y, trans.z]
        return matrix

    def matrix_to_transform_msgs(self, matrix):
        """Homogeneous transformation matrixから並進とクォータニオンを取得"""
        # 回転行列からクォータニオンを取得
        quaternion = tft.quaternion_from_matrix(matrix)
        # 並進を取得
        translation = matrix[0:3, 3].tolist()
        return translation, quaternion

    def publish_transform(self, event=None):
        try:
            # 各変換を取得
            map_to_body = self.tf_buffer.lookup_transform(
                "camera_init",
                "body",
                rospy.Time(0)
            )
            odom_to_footprint = self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                rospy.Time(0)
            )

            # 各変換を行列に変換
            T_map_body = self.transform_to_matrix(map_to_body)
            T_odom_footprint = self.transform_to_matrix(odom_to_footprint)

            # map->odomの変換を計算
            # T_map_odom = T_map_body * T_body_footprint * T_footprint_odom
            # ここでT_body_footprintは単位行列と仮定（bodyとfootprintは同じと考える）
            T_footprint_odom = np.linalg.inv(T_odom_footprint)
            T_map_odom = np.dot(T_map_body, T_footprint_odom)

            # 行列から並進とクォータニオンを取得
            translation, quaternion = self.matrix_to_transform_msgs(T_map_odom)

            # 変換を発行
            self.br.sendTransform(
                translation,
                quaternion,
                rospy.Time.now(),
                "odom",
                "map"
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1, f"TF lookup failed: {e}")

if __name__ == '__main__':
    try:
        node = MapOdomBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass