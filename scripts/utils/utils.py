import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

# Utilities functions
def get_pose(tf_buffer, parent_frame, child_frame):
    pose = None
    try:
        pose = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("No transform found. \""+parent_frame+"\" and \""+child_frame+"\"")
        pass

    return pose

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix  

def fromTransfMatrixToTransformStamped(cvMat):
    
    transform = TransformStamped()
        
    transform.transform.translation.x = cvMat[0,3]
    transform.transform.translation.y = cvMat[1,3]
    transform.transform.translation.z = cvMat[2,3]

    r = R.from_matrix([[cvMat[0,0], cvMat[0,1], cvMat[0,2]], 
                       [cvMat[1,0], cvMat[1,1], cvMat[1,2]], 
                       [cvMat[2,0], cvMat[2,1], cvMat[2,2]]])
        
    q = r.as_quat()
    
    transform.transform.rotation.x = q[0];
    transform.transform.rotation.y = q[1];
    transform.transform.rotation.z = q[2];
    transform.transform.rotation.w = q[3];  
    
    return transform


def fromTransformStampedToTransfMatrix(tf):
    Tmatrix = np.identity(4)
    Tmatrix[0:3, 3] = [tf.transform.translation.x,
                       tf.transform.translation.y,
                       tf.transform.translation.z]

    Tmatrix[0:3, 0:3] = quaternion_rotation_matrix([tf.transform.rotation.w,
                                                    tf.transform.rotation.x,
                                                    tf.transform.rotation.y,
                                                    tf.transform.rotation.z])
    return Tmatrix
