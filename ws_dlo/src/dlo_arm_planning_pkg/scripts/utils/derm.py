import numpy as np
from sklearn.preprocessing import normalize
from scipy.spatial.transform import Rotation as sciR
import matplotlib.pyplot as plt


# -------------------------------------------------------
# 用于绘制axis比例相同的三维图
def set_aspect_equal_3d(ax):
    """Fix equal aspect bug for 3D plots."""

    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    from numpy import mean
    xmean = mean(xlim)
    ymean = mean(ylim)
    zmean = mean(zlim)

    plot_radius = max([abs(lim - mean_)
                       for lims, mean_ in ((xlim, xmean),
                                           (ylim, ymean),
                                           (zlim, zmean))
                       for lim in lims])

    ax.set_xlim3d([xmean - plot_radius, xmean + plot_radius])
    ax.set_ylim3d([ymean - plot_radius, ymean + plot_radius])
    ax.set_zlim3d([zmean - plot_radius, zmean + plot_radius])


# -------------------------------------------------------
def getBishopFrame(tangents, u0):
        cross_products = np.cross(tangents[:-1, :], tangents[1:, :], axis=1)
        dot_products = np.sum(tangents[:-1, :] * tangents[1:, :], axis=1) # 批量向量点乘
        dot_products[dot_products > 1.0] = 1.0
        angles = np.arccos(dot_products)
        # print("angles: ", angles)
        rot_vec = normalize(cross_products) * angles.reshape(-1, 1)
        rot_mat = sciR.from_rotvec(rot_vec).as_matrix()
        u = np.zeros(tangents.shape)

        u[0, :] = u0.reshape(-1,)
        for i in range(1, tangents.shape[0]):
            if angles[i-1] < 1e-6: # 两个edges完全平行
                rot_mat[i-1, :, :] = np.eye(3, 3)
            u[i, :] = (rot_mat[i-1, :, :] @ u[i-1, :].reshape(-1, 1)).reshape(-1,)
        
        v = np.cross(tangents, u)
        return tangents, u, v


# -------------------------------------------------------
def edges2FpsPos(edges, x0):
    return np.cumsum(np.concatenate([x0.reshape(1,-1), edges], axis=0), axis=0)


# -------------------------------------------------------
def fpsPos2Edges(fps_pos):
    return fps_pos[1:, :] - fps_pos[:-1, :]


# -------------------------------------------------------
def edges2L(edges):
    L = np.linalg.norm(edges[:-1, :], axis=1) + np.linalg.norm(edges[1:, :], axis=1)
    L = np.concatenate([[0], L])
    return L


# -------------------------------------------------------
def edges2Tangents(edges):
    return normalize(edges, axis=1)


# -------------------------------------------------------
def getTheta(m1, u, v):
    m1 /= np.linalg.norm(m1)
    u /= np.linalg.norm(u)
    v /= np.linalg.norm(v)

    cos_theta = np.clip(np.dot(m1, u), -1.0, 1.0)
    sin_theta = np.clip(np.dot(m1, v), -1.0, 1.0)

    if sin_theta >= 0:
        return np.arccos(cos_theta)
    if sin_theta < 0:
        return -np.arccos(cos_theta)


# --------------------------------------------
def plotRod(ax, fps_pos, left_quat=None, right_quat=None, theta_n=None, frame='material', 
            label=None, plot_edge_frame=False, vertexsize=20, linewidth=3, axessize=0.07, color='b'):

    fps_pos = np.array(fps_pos).reshape(-1, 3)
    num_fps = fps_pos.shape[0]

    if not (left_quat is None or right_quat is None):

        # ------------- 计算每个edge对应的bishop frame和material frame -------------
        left_rot_mat = sciR.from_quat(left_quat).as_matrix()
        right_rot_mat = sciR.from_quat(right_quat).as_matrix()

        # 根据末端朝向，在两端加上两个vertex
        t_0 = left_rot_mat[:, 1] # 取旋转矩阵的y列作为t的轴
        t_n = right_rot_mat[:, 1]
        fp_left = fps_pos[0, :] - 0.05 * t_0.reshape(1, -1)
        fp_right = fps_pos[-1, :] + 0.05 * t_n.reshape(1, -1)
        fps_pos = np.concatenate([fp_left, fps_pos, fp_right], axis=0)

        m1_0 = left_rot_mat[:, 2] # 取旋转矩阵的z列作为m1的轴
        m1_n = right_rot_mat[:, 2]

        edges = fpsPos2Edges(fps_pos)
        tangents = edges2Tangents(edges)

        u0 = m1_0
        tangents, u, v = getBishopFrame(tangents, u0)

        theta_0 = getTheta(m1_0, u[0, :], v[0, :])
        if theta_n is None:
            theta_n = getTheta(m1_n, u[-1, :], v[-1, :]) # 改为直接从数据中读取
        thetas = theta_0 + np.arange(0, num_fps + 1) / num_fps * (theta_n - theta_0)
        thetas = thetas.reshape(-1, 1)

        # print("thetas: ", thetas)

        print("label: ", label, ", theta_n: ", theta_n)

        m1 = np.cos(thetas) * u + np.sin(thetas) * v
        m2 = -np.sin(thetas) * u + np.cos(thetas) * v

        edges_pos = (fps_pos[:-1, :] + fps_pos[1:, :]) / 2

        if frame == 'bishop':
            a = u
            b = v
        elif frame == 'material':
            a = m1
            b = m2

        # 画坐标系
        if plot_edge_frame:
            # ax.quiver(edges_pos[1:-1, 0], edges_pos[1:-1, 1], edges_pos[1:-1, 2], \
            #     tangents[1:-1, 0], tangents[1:-1, 1], tangents[1:-1, 2], length=axessize, normalize=True, color='r')
            ax.quiver(edges_pos[1:-1, 0], edges_pos[1:-1, 1], edges_pos[1:-1, 2], \
                a[1:-1, 0], a[1:-1, 1], a[1:-1, 2], length=axessize, normalize=True, color='b')
            ax.quiver(edges_pos[1:-1, 0], edges_pos[1:-1, 1], edges_pos[1:-1, 2], \
                b[1:-1, 0], b[1:-1, 1], b[1:-1, 2], length=axessize, normalize=True, color='g')
            # ax.quiver(edges_pos[[0, -1], 0], edges_pos[[0, -1], 1], edges_pos[[0, -1], 2], \
            #     tangents[[0, -1], 0], tangents[[0, -1], 1], tangents[[0, -1], 2], length=axessize*1, normalize=True, color='r')
            ax.quiver(edges_pos[[0, -1], 0], edges_pos[[0, -1], 1], edges_pos[[0, -1], 2], \
                a[[0, -1], 0], a[[0, -1], 1], a[[0, -1], 2], length=axessize*1, normalize=True, color='b')
            ax.quiver(edges_pos[[0, -1], 0], edges_pos[[0, -1], 1], edges_pos[[0, -1], 2], \
                b[[0, -1], 0], b[[0, -1], 1], b[[0, -1], 2], length=axessize*1, normalize=True, color='g')
        else:
            # ax.quiver(edges_pos[[0, -1], 0], edges_pos[[0, -1], 1], edges_pos[[0, -1], 2], \
            #     tangents[[0, -1], 0], tangents[[0, -1], 1], tangents[[0, -1], 2], length=axessize, normalize=True, color='r')
            ax.quiver(edges_pos[[0, -1], 0], edges_pos[[0, -1], 1], edges_pos[[0, -1], 2], \
                a[[0, -1], 0], a[[0, -1], 1], a[[0, -1], 2], length=axessize, normalize=True, color='k')
            # ax.quiver(edges_pos[[0, -1], 0], edges_pos[[0, -1], 1], edges_pos[[0, -1], 2], \
            #     b[[0, -1], 0], b[[0, -1], 1], b[[0, -1], 2], length=axessize, normalize=True, color='b')


    # 画顶点
    ax.plot(fps_pos[:, 0], fps_pos[:, 1], fps_pos[:, 2], label=label, linewidth=linewidth, color=color)
    ax.scatter(fps_pos[:, 0], fps_pos[:, 1], fps_pos[:, 2], s=vertexsize, marker='o', color=color)


