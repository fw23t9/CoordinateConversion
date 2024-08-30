from scipy.spatial.transform import Rotation as R  
import numpy as np  
  
# 计算从B->A的变换矩阵
# return
# 整体变换矩阵(4*4), 旋转矩阵(3*3), 平移矩阵(3*1), 欧拉角(3*1, 与计算顺序有关)
def compute_transform_matrix(A, B, rotate_order = 'xyz'):  
    """  
    计算两个三维点集A和B之间的转换矩阵  
    A和B是Nx3的numpy数组，其中N是点的数量  
    """  
    # 计算质心  
    centroid_A = np.mean(A, axis=0)  
    centroid_B = np.mean(B, axis=0)  
  
    # 中心化点集  
    A_centered = A - centroid_A  
    B_centered = B - centroid_B  
  
    # 使用SVD找到最佳旋转矩阵  
    H = np.dot(B_centered.T, A_centered)  
    U, S, Vt = np.linalg.svd(H)  
  
    # 构建旋转矩阵  
    Rotate_Mat = np.dot(Vt.T, U.T)  
  
    # 如果R是反射而不是旋转，则调整最后一个列向量  
    if np.linalg.det(Rotate_Mat) < 0:  
       Vt[-1,:] *= -1  
       Rotate_Mat = np.dot(Vt.T, U.T)  
  
    # 计算平移向量  
    t = centroid_A.T - np.dot(Rotate_Mat, centroid_B.T)  
  
    # 构建完整的变换矩阵  
    T = np.identity(4)  
    T[:3, :3] = Rotate_Mat  
    T[:3, 3] = t.flatten()  

    # 使用scipy的Rotation类  
    rot = R.from_matrix(Rotate_Mat)  
    
    # 转换为欧拉角，'zyx'表示Z-Y-X旋转顺序  
    euler_angles = rot.as_euler(rotate_order, degrees=True)
  
    return T, Rotate_Mat, t.flatten(), euler_angles