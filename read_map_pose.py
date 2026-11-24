"""Read map -> base_link through the system ROS Melodic TF binding."""
import json
import math
import sys

import rospy
import tf


def main():
    rospy.init_node('tomato_search_tf', anonymous=True)
    listener = tf.TransformListener()
    listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(2.))
    stamp = listener.getLatestCommonTime('map', 'base_link')
    age = (rospy.Time.now() - stamp).to_sec()
    if age < -.1 or age > float(sys.argv[1]):
        raise RuntimeError('MAP_TF_STALE: age={:.3f}s'.format(age))
    trans, rot = listener.lookupTransform('map', 'base_link', stamp)
    yaw = tf.transformations.euler_from_quaternion(rot)[2]
    pose = [trans[0], trans[1], yaw]
    if any(math.isnan(x) or math.isinf(x) for x in pose):
        raise RuntimeError('MAP_TF_INVALID')
    print(json.dumps([pose, stamp.to_sec()], allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except tf.Exception as exc:
        sys.stderr.write('MAP_TF_UNAVAILABLE: {}\n'.format(exc))
        sys.exit(1)
    except RuntimeError as exc:
        sys.stderr.write('{}\n'.format(exc))
        sys.exit(1)
