import numpy as np
import subprocess
import json
import math
import random

CPP_EXECUTABLES = ["../registration/build_const/sic_test_general",
                   #"../registration/build_const/sic_test",
                   #"../registration/build_const/sic_test_normal",
                   #"../registration/build_const/sic_test_crop_box"
                   ]

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


def run_icp_alignment(img_set1_file, img_set2_file, starting_mat, iterations, executable, include_points, points1,
                      points2):
    ply_file_1 = img_set1_file
    ply_file_2 = img_set2_file

    args = [
        executable,
        ply_file_1,
        ply_file_2,
        str(iterations)
    ]

    for vec in starting_mat:
        for x in vec:
            args.append(str(x))

    if include_points:
        for vec in points1:
            for x in vec:
                args.append(str(x))
        for vec in points2:
            for x in vec:
                args.append(str(x))

    args.append(";")
    args.append("exit")
    args.append("0")
    output = ""
    try:
        output = subprocess.check_output(" ".join(args), shell=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as err:
        me = ":("
    # this is expected to error because of an unavoidable segfault

    result = output.decode("ascii").strip().split("\n")

    result_mat = []
    mat_row = 0

    if 'egmentation' in result[-1]:
        result.pop()

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


def calc_mat_mse(mat1, mat2):
    mat1 = np.array(mat1)
    mat2 = np.array(mat2)
    diff = mat1 - mat2
    return np.linalg.norm(diff)

def check_error_from_ident(arr):
    return calc_mat_mse(arr, np.identity(4))


def get_rotation_matrix(tx, ty, tz):
    tx, ty, tz

    Rx = np.array([[1, 0, 0], [0, math.cos(tx), -math.sin(tx)], [0, math.sin(tx), math.cos(tx)]])
    Ry = np.array([[math.cos(ty), 0, math.sin(ty)], [0, 1, 0], [-math.sin(ty), 0, math.cos(ty)]])
    Rz = np.array([[math.cos(tz), -math.sin(tz), 0], [math.sin(tz), math.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rx, np.dot(Ry, Rz))


def run_sic_test(img_set1_file, img_set2_file, points1, points2,
                 executable, include_points,
                 xtheta, ytheta, ztheta, rand_trans):
    starting_matrix = align(points1, points2)

    iterations = 40

    correction_matrix = [[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1], ]

    np_starting_matrix = np.array(starting_matrix)
    np_correction_matrix = np.array(correction_matrix)
    np_ideal_matrix = np.matmul(np_correction_matrix, np_starting_matrix)

    """""
    Unused
    np_ideal_matrix_inv = np.linalg.inv(np_ideal_matrix)

    np_p1 = np.zeros((3, 4))
    np_p2_ideal = np.zeros((3, 4))
    points2_ideal = [0, 0, 0]
    for i in range(3):
        np_p1[i] = make_homogeneous(points1[i])
        np_p2_ideal[i] = np_ideal_matrix_inv.dot(np_p1[i])
        points2_ideal[i] = make_un_homogeneous(np_p2_ideal[i]).tolist()
    """""

    print(
        "executable\titerations\tfile_name_1\tfile_name_2\tmultiplier\trandom_matrix\toutput_matrix\trm_full\tom_full\tinitial_error\tfinal_error")
    for i in range(11):
        multiplier = i * 0.1
        rand_rot_mat = get_rotation_matrix(multiplier * xtheta,
                                           multiplier * ytheta,
                                           multiplier * ztheta)
        rand_mat = np.zeros((4, 4))
        rand_mat[3][3] = 1
        for i in range(3):
            for j in range(3):
                rand_mat[i][j] = rand_rot_mat[i][j]
            rand_mat[i][3] = rand_trans[i] * multiplier

        print(executable + '\t', end="")
        print(str(iterations) + '\t', end="")
        print(img_set1_file + '\t', end="")
        print(img_set2_file + '\t', end="")
        print(str(multiplier) + '\t', end="")
        print(json.dumps(rand_mat.tolist()) + '\t', end="")

        rand_mat_inv = np.linalg.inv(rand_mat)
        new_starting_matrix = np.matmul(rand_mat, np_ideal_matrix)

        initial_error = calc_mat_mse(rand_mat_inv, np.identity(4))
        matrix_attempt = run_icp_alignment(img_set1_file, img_set2_file, new_starting_matrix.tolist(),
                                           iterations, executable, include_points, points1, points2)
        np_matrix_attempt = np.array(matrix_attempt)

        print(json.dumps(matrix_attempt) + '\t', end="")
        print(json.dumps(new_starting_matrix.tolist()) + '\t', end="")
        print(json.dumps(np.matmul(np_matrix_attempt, new_starting_matrix).tolist()) + '\t', end="")

        print(str(initial_error) + '\t', end="")
        final_error = calc_mat_mse(rand_mat_inv, matrix_attempt)
        print(str(final_error) + '\t', end="")
        print()


def run_sic_test_rot_only(img_set1_file, img_set2_file, points1, points2,
                 executable, include_points,
                 xtheta, ytheta, ztheta, rand_trans):
    starting_matrix = align(points1, points2)

    iterations = 40

    correction_matrix = [[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1], ]

    np_starting_matrix = np.array(starting_matrix)
    np_correction_matrix = np.array(correction_matrix)
    np_ideal_matrix = np.matmul(np_correction_matrix, np_starting_matrix)

    """""
    Unused
    np_ideal_matrix_inv = np.linalg.inv(np_ideal_matrix)

    np_p1 = np.zeros((3, 4))
    np_p2_ideal = np.zeros((3, 4))
    points2_ideal = [0, 0, 0]
    for i in range(3):
        np_p1[i] = make_homogeneous(points1[i])
        np_p2_ideal[i] = np_ideal_matrix_inv.dot(np_p1[i])
        points2_ideal[i] = make_un_homogeneous(np_p2_ideal[i]).tolist()
    """""

    print(
        "executable\titerations\tfile_name_1\tfile_name_2\tmultiplier\trandom_matrix\toutput_matrix\trm_full\tom_full\tinitial_error\tfinal_error")
    for i in range(11):
        multiplier = i * 0.1
        rand_rot_mat = get_rotation_matrix(multiplier * xtheta,
                                           multiplier * ytheta,
                                           multiplier * ztheta)
        rand_mat = np.zeros((4, 4))
        rand_mat[3][3] = 1
        for i in range(3):
            for j in range(3):
                rand_mat[i][j] = rand_rot_mat[i][j]
            rand_mat[i][3] = rand_trans[i] * multiplier

        print(executable + '\t', end="")
        print(str(iterations) + '\t', end="")
        print(img_set1_file + '\t', end="")
        print(img_set2_file + '\t', end="")
        print(str(multiplier) + '\t', end="")
        print(json.dumps(rand_mat.tolist()) + '\t', end="")

        rand_mat_inv = np.linalg.inv(rand_mat)
        new_starting_matrix = np.matmul(rand_mat, np_ideal_matrix)

        initial_error = calc_mat_mse(rand_mat_inv, np.identity(4))
        matrix_attempt = run_icp_alignment(img_set1_file, img_set2_file, new_starting_matrix.tolist(),
                                           iterations, executable, include_points, points1, points2)
        np_matrix_attempt = np.array(matrix_attempt)

        print(json.dumps(matrix_attempt) + '\t', end="")
        print(json.dumps(new_starting_matrix.tolist()) + '\t', end="")
        print(json.dumps(np.matmul(np_matrix_attempt, new_starting_matrix).tolist()) + '\t', end="")

        print(str(initial_error) + '\t', end="")
        final_error = calc_mat_mse(rand_mat_inv, matrix_attempt)
        print(str(final_error) + '\t', end="")
        print()


def run_full_test(img_set1_file, img_set2_file, points1, points2):
    xtheta = (math.pi * random.random() - math.pi / 2) / 2
    ytheta = (math.pi * random.random() - math.pi / 2) / 2
    ztheta = (math.pi * random.random() - math.pi / 2) / 2

    #xtheta = 0
    #ytheta = 0
    #ztheta = 0

    rand_trans = np.random.rand(3)
    rand_trans = rand_trans / np.linalg.norm(rand_trans)
    rand_trans = rand_trans * 5

    for ex in CPP_EXECUTABLES:
        include_points = "crop_box" in ex
        run_sic_test(img_set1_file, img_set2_file, points1, points2,
                     ex, include_points,
                     xtheta, ytheta, ztheta, rand_trans)


if __name__ == "__main__":
    # print(json.dumps(align([[1.83,-0.69,5.18], [-3.26,-0.54,-1.10], [0.99,-1.58,-1.74]],
    #            [[-0.65,-1.01,6.82], [-2.05,-0.82,-2.48], [2.7,-1.6,-0.67]])))

    """""
    run_sic_test("sfm_files/kirbydslr-20180605-072254/dense/0/fused.ply",
                 "sfm_files/nano_building-20180507-021448/dense/0/fused.ply",
                 [[-2.790201187133789, -1.250707983970642, -0.21030402183532715],
                  [-1.790297657251358, -1.7404887080192566, -4.030646208673716],
                  [3.519728906452656, -0.11014655232429504, 1.4693803489208221]],
                 [[2.1895484030246735, -1.3604447543621063, 4.099859583191574],
                  [-1.7302962839603424, -1.1705068796873093, 6.719735324382782],
                  [-1.960280753672123, -0.3205534815788269, -2.5108142271637917]])
    """""
    """""
    run_sic_test("sfm_files/nano_building-20180507-021448/dense/0/fused.ply",
                 "sfm_files/kirbydslr-20180605-072254/dense/0/fused.ply",
                 [[-0.6403428688645363, -1.020506888628006, 6.8297353237867355],
                  [-2.0502807423472404, -0.8205224201083183, -2.4808142259716988],
                  [2.699486270546913, -1.600491352379322, -0.6708918809890747]],
                 [[-2.5500783771276474, -1.5407080054283142, -3.5204269886016846],
                  [3.569728910923004, -0.5307079553604126, 1.4693803489208221],
                  [-0.45007848739624023, -0.9208309650421143, 3.0799418091773987]])
    """""

    """""
    run_full_test("sfm_files/ssms-20180603-222504/dense/0/fused.ply",
                  "sfm_files/arts-20180514-031331/dense/0/fused.ply",
                  [[4.779156073927879, -1.75099765509367, 0.5790739953517914],
                   [-1.32035294175148, 0.8092750310897827, -3.8508168645203114],
                   [-2.360316574573517, -0.5209612846374512, 2.069183349609375]],
                  [[0.41922062635421753, -1.3604117631912231, 4.25939767062664],
                   [7.029585599899292, -2.390230119228363, -2.150967299938202],
                   [0.6292206645011902, -0.030775129795074463, -4.300422310829163]])
    """""

    """""
    run_full_test("sfm_files/ssms-20180603-222504/dense/0/fused.ply",
                  "sfm_files/ssms-20180603-222504/dense/0/fused.ply",
                  [[4.779156073927879, -1.75099765509367, 0.5790739953517914],
                   [-1.32035294175148, 0.8092750310897827, -3.8508168645203114],
                   [-2.360316574573517, -0.5209612846374512, 2.069183349609375]],
                  [[4.779156073927879, -1.75099765509367, 0.5790739953517914],
                   [-1.32035294175148, 0.8092750310897827, -3.8508168645203114],
                   [-2.360316574573517, -0.5209612846374512, 2.069183349609375]])
    """""
    """""
    run_full_test("../registration/build/all_split1.ply",
                  "../registration/build/all_split2.ply",
                  [[1.0593387186527252, -1.3008719086647034, -2.960729107260704],
                   [1.1797945499420166, -1.340871900320053, -3.1207291185855865],
                   [1.1997945457696915, -0.7009602338075638, -3.3701849579811096]],
                  [[1.0593387186527252, -1.3008719086647034, -2.960729107260704],
                   [1.1797945499420166, -1.340871900320053, -3.1207291185855865],
                   [1.1997945457696915, -0.7009602338075638, -3.3701849579811096]])

    """""
    """""
    run_full_test("sfm_files/ssms-20180603-222504/dense/0/fused.ply",
                  "sfm_files/ssms-20180603-222504/dense/0/fused.ply",
                  [[4.779156073927879, -1.75099765509367, 0.5790739953517914],
                   [-1.32035294175148, 0.8092750310897827, -3.8508168645203114],
                   [-2.360316574573517, -0.5209612846374512, 2.069183349609375]],
                  [[4.779156073927879, -1.75099765509367, 0.5790739953517914],
                   [-1.32035294175148, 0.8092750310897827, -3.8508168645203114],
                   [-2.360316574573517, -0.5209612846374512, 2.069183349609375]])
                   """""

    run_full_test("sfm_files/ssms-20180603-222504/dense/0/fused.ply",
                  "sfm_files/arts-20180514-031331/dense/0/fused.ply",
                  [[4.779156073927879, -1.75099765509367, 0.5790739953517914],
                   [-1.32035294175148, 0.8092750310897827, -3.8508168645203114],
                   [-2.360316574573517, -0.5209612846374512, 2.069183349609375]],
                  [[0.41922062635421753, -1.3604117631912231, 4.25939767062664],
                   [7.029585599899292, -2.390230119228363, -2.150967299938202],
                   [0.6292206645011902, -0.030775129795074463, -4.300422310829163]])


"""""
    run_full_test("sfm_files/kirbydslr-20180605-072254/dense/0/fused.ply",
                  "sfm_files/kohn_corner_hololens/dense/0/fused.ply",
                  [[1.9399218559265137, -0.23083090782165527, 0.009696006774902344],
                   [0.7090445160865784, 0.07985347509384155, -0.19074255228042603],
                   [-1.420516923069954, -0.18014654517173767, -3.1308654844760895]],
                  [[-29.520148277282715, 2.709713578224182, -23.18048520386219],
                   [-26.19014835357666, -0.160286545753479, -13.07048511505127],
                   [2.479081630706787, -1.0002865195274353, 0.09974503517150879]])
    """""
