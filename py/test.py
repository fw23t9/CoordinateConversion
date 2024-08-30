import numpy as np

from CoordinateConversion import compute_transform_matrix

if __name__ == '__main__':
    # 示例点集  
    # A coordinate
    A = np.array([
        [-4637.10, 6521.83, 1191.98], 
        [-7090.79, -4915.28, 445.28], 
        [-4418.70, 1921.64, 904.06],
        [-5145.94, -3165.80, 1213.44]
    ])  

    # B coordinate
    B = np.array([
        [1344.15, 342.17, 6.42],
        [13041.84, -399.23, 2.12],
        [5795.92, 56.90, -1173.72],
        [10922.47, 368.79, -1531.35]
    ])
    
    # 计算转换矩阵  A to B
    T, mat, t, angles = compute_transform_matrix(B, A)  

    print("转换矩阵:\n", T)
    print("旋转矩阵:\n", mat)
    print("平移矩阵:\n", t)
    print("角度:\n", angles)

    print("计算误差:")
    ones_column = np.ones((A.shape[0], 1))

    # 沿列方向连接  
    pa = np.concatenate((A, ones_column), axis=1) 

    print(np.dot(T, pa.T).T[:, :3] - B)  