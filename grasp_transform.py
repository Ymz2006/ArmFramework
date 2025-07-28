import numpy as np
from scipy.spatial.transform import Rotation as R
import src.ec_robot as ec

def rotation_matrix_to_euler(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])
    singular = sy < 1e-6  # 判断是否为奇异情况

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])

    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    print(x,y,z)
    return x, y, z

def rotation_matrix_to_euler2(mat):
    r = R.from_matrix(mat)
    euler_angles = r.as_euler('xyz', degrees=False)
    return euler_angles



# 将欧拉角转换为旋转矩阵
def euler_to_rotation_matrix(roll, pitch, yaw):
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return R_z @ R_y @ R_x

def euler_to_rotation_matrix2(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    rotation_matrix = r.as_matrix()
    return rotation_matrix
def pose_to_transform_matrix(pose):
    """
    将 6D 位姿转换为 4x4 齐次变换矩阵
    :param pose: 6D 位姿 [x, y, z, rx, ry, rz]
    :return: 4x4 齐次变换矩阵
    """
    # 提取平移分量
    x, y, z = pose[:3]

    # 提取欧拉角（假设顺序为 XYZ）
    rx, ry, rz = pose[3:]

    # 将欧拉角转换为旋转矩阵
    rotation = euler_to_rotation_matrix2(rx, ry, rz)

    # 构建 4x4 齐次变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation
    transform_matrix[:3, 3] = [x, y, z]

    return transform_matrix

def transform_matrix_to_pose(transform_matrix):
    """
    将 4x4 齐次变换矩阵转换为 6D 位姿
    :param transform_matrix: 4x4 齐次变换矩阵
    :return: 6D 位姿 [x, y, z, rx, ry, rz]
    """
    # 提取平移分量
    x, y, z = transform_matrix[:3, 3]

    # 提取旋转矩阵并转换为欧拉角（假设顺序为 XYZ）
    rotation = transform_matrix[:3,:3]
    # print('rotation:', rotation)
    rx, ry, rz = rotation_matrix_to_euler2(rotation)

    return np.array([x, y, z, rx, ry, rz])


def adjust_for_end_effector_offset(transform_matrix, offset):
    """
    调整变换矩阵以考虑末端执行器的偏移
    :param transform_matrix: 4x4 齐次变换矩阵
    :param offset: 沿 Z 轴的偏移量（单位：米）
    :return: 调整后的 4x4 齐次变换矩阵
    """
    # 创建偏移变换矩阵
    offset_matrix = np.eye(4)
    offset_matrix[2, 3] = offset  # 沿 Z 轴移动

    # 将偏移应用到原始变换矩阵
    adjusted_matrix = transform_matrix @ offset_matrix

    return adjusted_matrix


def camera_to_robot_pose(camera_pose, transformation_matrix, offset, is_matrix=False, return_matrix=False):
    """
    将相机坐标系下的 6D 位姿转换为机械臂坐标系下的 6D 位姿，并考虑末端执行器的偏移
    :param camera_pose: 相机坐标系下的 6D 位姿 [x, y, z, rx, ry, rz]
    :param transformation_matrix: 相机到机械臂的 4x4 齐次变换矩阵
    :param offset: 末端执行器的偏移量（单位：米）
    :param is_matrix: 是否是齐次矩阵
    :return: 机械臂坐标系下的 6D 位姿
    """
    # 将相机坐标系下的 6D 位姿转换为齐次变换矩阵
    if not is_matrix:
        camera_pose = pose_to_transform_matrix(camera_pose)

    # 将相机坐标系下的位姿变换到机械臂坐标系
    robot_transform = transformation_matrix @ camera_pose

    # 考虑末端执行器的偏移
    adjusted_robot_transform = adjust_for_end_effector_offset(robot_transform, offset)

    # print('camera_pose:', camera_pose)
    # print('transformation_matrix:', transformation_matrix)
    # print('robot_transform:', robot_transform)
    # print('adjusted_robot_transform:', adjusted_robot_transform)
    # print('----------------------------------------------------')

    # 将最终的齐次变换矩阵转换回 6D 位姿
    if not return_matrix:
        robot_pose = transform_matrix_to_pose(adjusted_robot_transform)
    else:
        robot_pose = adjusted_robot_transform

    return robot_pose

def camera_to_robot_pose_eye_in_hand(base2end, cam2obj, cam2cam , end2cam, offset, is_matrix=False, return_matrix=False):
    """
    将眼在手上相机坐标系下的 6D 位姿转换为机械臂基坐标系下的 6D 位姿，并考虑末端执行器的偏移
    :param base2end: 机械臂基坐标系下的机械臂末端位姿 [x, y, z, rx, ry, rz]
    :param cam2obj: 相机坐标系下的 6D 位姿 [x, y, z, rx, ry, rz]
    :param end2cam: 相机到机械臂末端的 4x4 齐次变换矩阵
    :param offset: 末端执行器的偏移量(单位:米),即obj2grasp
    :param is_matrix: 是否是齐次矩阵
    :return: 机械臂坐标系下的 6D 位姿
    """
    # 将相机坐标系下的 6D 位姿转换为齐次变换矩阵
    if not is_matrix:
        base2end = pose_to_transform_matrix(base2end)
        cam2obj = pose_to_transform_matrix(cam2obj)
        cam2cam = pose_to_transform_matrix(cam2cam)
        grasp_offset = pose_to_transform_matrix(offset)

    # 将相机坐标系下的位姿变换到基座坐标系
    base2obj = base2end @ end2cam @ cam2cam @ cam2obj

    # 考虑末端执行器的偏移
    base2grasp = base2obj @ grasp_offset 

    # 将最终的齐次变换矩阵转换回 6D 位姿
    if not return_matrix:
        robot_pose = transform_matrix_to_pose(base2grasp)
    else:
        robot_pose = base2grasp

    return robot_pose


if __name__ == "__main__":

    robot = ec.ECRobot("ec_robot")
    base2end = robot.read_current_pose()
    print(f"base2end:{base2end}")

    # 输入数据
    cam2obj = np.array([0,0,0,0,0,0]) # Grasp Net获得

    end2cam = np.array([[0.26267131 ,0.96379226 ,0.04591586,0.11402119],
                [0.9648848 ,-0.26232153,-0.01359218,-0.08165324],
                [-0.00105532 ,0.04787379,-0.99885284,0.11600166],
                [0,0,0,1]])
    # end_effector_offset = -0.19  # 10 厘米（可变量）
    end_effector_offset = np.array([0,0,-0.19,0,0,0])
    # 计算机械臂坐标系下的抓取点 6D 位姿
    robot_pose = camera_to_robot_pose_eye_in_hand(base2end, cam2obj, end2cam, end_effector_offset)

    # 输出结果
    print("机械臂坐标系下的抓取点 6D 位姿：")
    print(robot_pose)

    result = robot.set_pose(robot_pose.tolist())
    print(result)
