import numpy as np
import subprocess
import json


CPP_SIC_EXECUTABLE = "../registration/build/sic_test"


def make_homogeneous(vec):
    res = np.ones(4)
    res[0] = vec[0]
    res[1] = vec[1]
    res[2] = vec[2]
    return res


def make_un_homogeneous(vec):
    res = np.zeros(3)
    res[0] = vec[0]
    res[1] = vec[1]
    res[2] = vec[2]
    return res


def align(vs1, vs2):
    vecs1 = np.array(vs1)
    vecs2 = np.array(vs2)

    norm1 = np.cross(vecs1[1] - vecs1[0], vecs1[2] - vecs1[0])
    norm1 = norm1 / np.linalg.norm(norm1)
    norm2 = np.cross(vecs2[1] - vecs2[0], vecs2[2] - vecs2[0])
    norm2 = norm2 / np.linalg.norm(norm2)

    scale1 = np.linalg.norm(vecs1[1] - vecs1[0]) / np.linalg.norm(vecs2[1] - vecs2[0])
    scale2 = np.linalg.norm(vecs1[2] - vecs1[1]) / np.linalg.norm(vecs2[2] - vecs2[1])
    scale3 = np.linalg.norm(vecs1[2] - vecs1[0]) / np.linalg.norm(vecs2[2] - vecs2[0])
    scale = (scale1 + scale2 + scale3) / 3

    v = np.cross(norm2, norm1)
    vx = np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]]
    )
    c = np.dot(norm1, norm2)
    norm_rot_scale_mat = np.identity(3) + vx + (np.matmul(vx, vx) * (1 / (1 + c)))
    norm_rot_scale_mat = norm_rot_scale_mat * scale
    after_rot2 = np.zeros((3, 3))
    for i in range(3):
        after_rot2[i] = norm_rot_scale_mat.dot(vecs2[i])

    rot_norm1 = np.cross(norm1, vecs1[1] - vecs1[0])
    rot_norm1 = rot_norm1 / np.linalg.norm(rot_norm1)
    after_rot_norm2 = np.cross(after_rot2[1] - after_rot2[0], after_rot2[2] - after_rot2[0])
    after_rot_norm2 = after_rot_norm2 / np.linalg.norm(after_rot_norm2)
    rot_norm2 = np.cross(after_rot_norm2, after_rot2[1] - after_rot2[0])
    rot_norm2 = rot_norm2 / np.linalg.norm(rot_norm2)

    v = np.cross(rot_norm2, rot_norm1)
    vx = np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]]
    )
    c = np.dot(rot_norm1, rot_norm2)
    h_rot_mat = np.identity(3) + vx + (np.matmul(vx, vx) * (1 / (1 + c)))
    for i in range(3):
        after_rot2[i] = h_rot_mat.dot(after_rot2[i])

    full_rot = np.matmul(h_rot_mat, norm_rot_scale_mat)

    translation1 = vecs1[0] - after_rot2[0]
    translation2 = vecs1[1] - after_rot2[1]
    translation3 = vecs1[2] - after_rot2[2]
    translation = (translation1 + translation2 + translation3) / 3

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

    result = np.matmul(trans_mat, rot_mat)

    return result.tolist()


def run_icp_alignment(img_set1_file, img_set2_file, starting_mat, iterations):
    ply_file_1 = img_set1_file
    ply_file_2 = img_set2_file

    args = [
        CPP_SIC_EXECUTABLE,
        ply_file_1,
        ply_file_2,
        str(iterations)
    ]

    for vec in starting_mat:
        for x in vec:
            args.append(str(x))

    output = ""
    try:
        output = subprocess.check_output(args, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as err:
        print('Error running sic testing executable:', err.returncode)
        print(bytes.decode(err.output))
        raise err

    result = output.decode("ascii").strip().split("\n")

    result_mat = []
    mat_row = 0

    for row in range(len(result) - 4, len(result)):
        result_mat.append([])
        row_str = list(filter(None, result[row].split(' ')))
        for col in range(len(row_str)):
            result_mat[mat_row].append(float(row_str[col]))
        mat_row += 1

    return result_mat


def calc_mean_error_with_mat(np_p1, np_p2, matrix):
    matrix = np.array(matrix)
    trans_points = np.zeros((len(np_p2), 4))
    for i in range(len(np_p2)):
        trans_points[i] = matrix.dot(np_p2[i])

    error = 0
    for i in range(len(np_p2)):
        error_vec = np_p1[i] - trans_points[i]
        error += np.linalg.norm(error_vec)

    return error / len(np_p2)


def run_sic_test(img_set1_file, img_set2_file, points1, points2):
    starting_matrix = align(points1, points2)

    correction_matrix = run_icp_alignment(img_set1_file, img_set2_file, starting_matrix, 1)

    np_starting_matrix = np.array(starting_matrix)
    np_correction_matrix = np.array(correction_matrix)
    np_ideal_matrix = np.matmul(np_correction_matrix, np_starting_matrix)
    np_ideal_matrix_inv = np.linalg.inv(np_ideal_matrix)

    np_p1 = np.zeros((3, 4))
    np_p2_ideal = np.zeros((3, 4))
    points2_ideal = [0, 0, 0]
    for i in range(3):
        np_p1[i] = make_homogeneous(points1[i])
        np_p2_ideal[i] = np_ideal_matrix_inv.dot(np_p1[i])
        points2_ideal[i] = make_un_homogeneous(np_p2_ideal[i]).tolist()

    rand_vecs = np.random.rand(3, 3)
    for i in range(3):
        rand_vecs[i] = rand_vecs[i] / np.linalg.norm(rand_vecs[i])

    print("offset\tstarting_matrix\ticp_matrix\tinitial_error\titeration_error")
    for i in range(10):
        offset_value = i * 0.1
        print(str(offset_value) + '\t', end="")

        np_mutated2 = np.array(points2_ideal) + (rand_vecs * offset_value)

        np_mutated2_h = np.ones((3, 4))
        for j in range(3):
            np_mutated2_h[j] = make_homogeneous(np_mutated2[j])

        mutated_starting_matrix = align(points1, np_mutated2.tolist())
        print(json.dumps(mutated_starting_matrix) + '\t', end="")
        initial_error = calc_mean_error_with_mat(np_p1, np_mutated2_h, mutated_starting_matrix)

        matrix_attempt = run_icp_alignment(img_set1_file, img_set2_file, mutated_starting_matrix, 1)
        print(json.dumps(matrix_attempt) + '\t', end="")
        print(json.dumps(initial_error) + '\t', end="")

        error = calc_mean_error_with_mat(np_p1, np_mutated2_h, matrix_attempt)
        print(json.dumps(error) + '\t', end="")
        print()


if __name__ == "__main__":
    # print(json.dumps(align([[1.83,-0.69,5.18], [-3.26,-0.54,-1.10], [0.99,-1.58,-1.74]],
    #            [[-0.65,-1.01,6.82], [-2.05,-0.82,-2.48], [2.7,-1.6,-0.67]])))

    run_sic_test("sfm_files/nano_building-20180507-021448/dense/0/fused.ply",
                      "sfm_files/kirbydslr-20180605-072254/dense/0/fused.ply",
                      [[-0.6403428688645363, -1.020506888628006, 6.8297353237867355],
                       [-2.0502807423472404, -0.8205224201083183, -2.4808142259716988],
                       [2.699486270546913, -1.600491352379322, -0.6708918809890747]],
                      [[-2.5500783771276474, -1.5407080054283142, -3.5204269886016846],
                       [3.569728910923004, -0.5307079553604126, 1.4693803489208221],
                       [-0.45007848739624023, -0.9208309650421143, 3.0799418091773987]])