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
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # バッファ時間を10秒に増加
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf.TransformBroadcaster()
        
        # 前回の有効な変換を保存
        self.last_valid_translation = None
        self.last_valid_quaternion = None
        
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_transform)

    def transform_to_matrix(self, transform):
        """TransformStampedからHomogeneous transformation matrixを作成"""
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        matrix = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        matrix[0:3, 3] = [trans.x, trans.y, trans.z]
        return matrix

    def matrix_to_transform_msgs(self, matrix):
        """Homogeneous transformation matrixから並進とクォータニオンを取得"""
        quaternion = tft.quaternion_from_matrix(matrix)
        translation = matrix[0:3, 3].tolist()
        return translation, quaternion

    def publish_transform(self, event=None):
        try:
            # 現在時刻を取得
            now = rospy.Time.now()
            
            # 少し前の時刻でTF取得を試みる（センサーの遅延を考慮）
            transform_time = now - rospy.Duration(0.1)
            
            # 各変換を取得（タイムアウト付き）
            map_to_body = self.tf_buffer.lookup_transform(
                "map",
                "body",
                transform_time,
                rospy.Duration(0.5)  # タイムアウトを0.5秒に設定
            )
            
            odom_to_footprint = self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                transform_time,
                rospy.Duration(0.5)
            )

            # 各変換を行列に変換
            T_map_body = self.transform_to_matrix(map_to_body)
            T_odom_footprint = self.transform_to_matrix(odom_to_footprint)

            # map->odomの変換を計算
            T_footprint_odom = np.linalg.inv(T_odom_footprint)
            T_map_odom = np.dot(T_map_body, T_footprint_odom)

            # 行列から並進とクォータニオンを取得
            translation, quaternion = self.matrix_to_transform_msgs(T_map_odom)

            # 変換を発行
            self.br.sendTransform(
                translation,
                quaternion,
                now,  # 現在時刻を使用
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