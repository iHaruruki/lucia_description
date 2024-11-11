import rospy
import tf2_ros
import geometry_msgs.msg

def static_tf_publisher():
    rospy.init_node('static_tf_publisher')

    static_transformBroadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    # odomからbase_linkへの変換行列を定義
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "odom"
    static_transformStamped.child_frame_id = "base_link"
    static_transformStamped.transform.translation.x = 0.0  # x方向の距離
    static_transformStamped.transform.translation.y = 0.0  # y方向の距離
    static_transformStamped.transform.translation.z = 0.0  # z方向の距離
    # (クォータニオンを定義)

    static_transformBroadcaster.sendTransform(static_transformStamped)
    rospy.spin()

if __name__ == '__main__':
    static_tf_publisher()