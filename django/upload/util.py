import numpy as np

def align(vs1, vs2):
    vecs1 = np.array(vs1)
    vecs2 = np.array(vs2)

    norm1 = np.cross(vecs1[1] - vecs1[0], vecs1[2] - vecs1[0])
    norm1 = norm1 / np.linalg.norm(norm1)
    norm2 = np.cross(vecs2[1] - vecs2[0], vecs2[2] - vecs2[0])
    norm2 = norm2 / np.linalg.norm(norm2)

    v = np.cross(norm2, norm1)
    vx = np.array(
        [[0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]]
    )
    c = np.dot(norm1, norm2)
    norm_rot = np.identity(3) + vx + (np.matmul(vx, vx) * (1 / (1 + c)))
    after_rot2 = np.zeros((3, 3))
    for i in range(3):
        after_rot2[i] = norm_rot.dot(vecs2[i])

    norm1 = np.cross(norm1, vecs1[1] - vecs1[0])
    norm1 = norm1 / np.linalg.norm(norm1)
    norm2 = np.cross(after_rot2[1] - after_rot2[0], after_rot2[2] - after_rot2[0])
    norm2 = np.cross(norm2, vecs2[1] - vecs2[0])
    norm2 = norm2 / np.linalg.norm(norm2)

    print(norm1)
    print(norm2)

    v = np.cross(norm2, norm1)
    vx = np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]]
    )
    c = np.dot(norm1, norm2)
    h_rot = np.identity(3) + vx + (np.matmul(vx, vx) * (1 / (1 + c)))
    for i in range(3):
        after_rot2[i] = h_rot.dot(after_rot2[i])

    print(h_rot)
    full_rot = np.matmul(h_rot, norm_rot)

    translation = vecs1[0] - after_rot2[0]
    rot_mat = np.identity(4)
    trans_mat = np.identity(4)

    hvecs1 = np.ones((3, 4))
    hvecs2 = np.ones((3, 4))
    for i in range(3):
        trans_mat[i][3] = translation[i]
        for j in range(3):
            rot_mat[i][j] = full_rot[i][j]
            hvecs1[i][j] = vecs1[i][j]
            hvecs2[i][j] = vecs2[i][j]

    print(hvecs1)

    for i in range(3):
        hvecs2[i] = np.matmul(trans_mat, rot_mat).dot(hvecs2[i])

    print(hvecs2)


if __name__ == "__main__":
    print(align([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 0], [1, 0, 1], [1, 1, 0]]))