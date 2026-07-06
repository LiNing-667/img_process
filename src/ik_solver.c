/**
 ******************************************************************************
 * @file    ik_solver.c
 * @brief   6 轴机械臂逆运动学数值求解器实现
 ******************************************************************************
 */

#include "ik_solver.h"
#include <math.h>
#include <string.h>

/* ========== 三维向量工具 ========== */
static float vec_norm(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static void vec_normalize(float v[3]) {
    float n = vec_norm(v);
    if (n > 1e-8f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
static float vec_dot(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static void vec_cross(const float a[3], const float b[3], float res[3]) {
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
}

/* ========== 6x6 线性方程组求解（高斯消元） ========== */
static int linear_solve_6(float A[6][6], float b[6], float x[6]) {
    const int n = 6;
    float aug[6][7];
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) aug[i][j] = A[i][j];
        aug[i][n] = b[i];
    }

    for (int col = 0; col < n; ++col) {
        /* 选主元 */
        int pivot = col;
        float maxv = fabsf(aug[col][col]);
        for (int row = col+1; row < n; ++row) {
            if (fabsf(aug[row][col]) > maxv) {
                maxv = fabsf(aug[row][col]);
                pivot = row;
            }
        }
        if (maxv < 1e-8f) return 0;  /* 奇异 */

        if (pivot != col) {
            for (int j = 0; j <= n; ++j) {
                float tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }

        for (int row = col+1; row < n; ++row) {
            float factor = aug[row][col] / aug[col][col];
            aug[row][col] = 0.0f;
            for (int j = col+1; j <= n; ++j)
                aug[row][j] -= factor * aug[col][j];
        }
    }

    /* 回代 */
    for (int i = n-1; i >= 0; --i) {
        float sum = aug[i][n];
        for (int j = i+1; j < n; ++j) sum -= aug[i][j] * x[j];
        x[i] = sum / aug[i][i];
    }
    return 1;
}

/* ========== 4x4 矩阵乘法 ========== */
static void mat4_mult(const float A[4][4], const float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
        }
}

/* ========== DH 变换矩阵生成（角度输入为度） ========== */
static void DH_transform(float alpha_deg, float a, float theta_deg, float d, float T[4][4]) {
    float alpha = DEG2RAD(alpha_deg);
    float theta = DEG2RAD(theta_deg);
    float ca = cosf(alpha), sa = sinf(alpha);
    float ct = cosf(theta), st = sinf(theta);

    float Rx[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f,   ca,  -sa, 0.0f},
        {0.0f,   sa,   ca, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    float Dx[4][4] = {
        {1.0f, 0.0f, 0.0f,    a},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    float Rz[4][4] = {
        {  ct,  -st, 0.0f, 0.0f},
        {  st,   ct, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    float Dz[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f,    d},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };

    float tmp1[4][4], tmp2[4][4];
    mat4_mult(Rx, Dx, tmp1);
    mat4_mult(tmp1, Rz, tmp2);
    mat4_mult(tmp2, Dz, T);
}

/* ========== 6 轴正运动学（度输入） ========== */
static void forward_kinematics(const float th_deg[6], float pos[3], float R[3][3]) {
    float T[4][4];
    float T_next[4][4];
    float T_acc[4][4] = {
        {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}
    };

    /* T1 */
    DH_transform(0.0f, 0.0f, th_deg[0], X_01, T);
    mat4_mult(T_acc, T, T_next);
    memcpy(T_acc, T_next, sizeof(T_acc));
    /* T2 */
    DH_transform(90.0f, 0.0f, th_deg[1], 0.0f, T);
    mat4_mult(T_acc, T, T_next);
    memcpy(T_acc, T_next, sizeof(T_acc));
    /* T3 */
    DH_transform(-90.0f, 0.0f, th_deg[2] + 180.0f, X_23, T);
    mat4_mult(T_acc, T, T_next);
    memcpy(T_acc, T_next, sizeof(T_acc));
    /* T4 */
    DH_transform(-90.0f, 0.0f, th_deg[3], 0.0f, T);
    mat4_mult(T_acc, T, T_next);
    memcpy(T_acc, T_next, sizeof(T_acc));
    /* T5 */
    DH_transform(0.0f, Z_45, -90.0f + th_deg[4], 0.0f, T);
    mat4_mult(T_acc, T, T_next);
    memcpy(T_acc, T_next, sizeof(T_acc));
    /* T6 */
    DH_transform(-90.0f, Z_56, th_deg[5], X_56, T);
    mat4_mult(T_acc, T, T_next);
    memcpy(T_acc, T_next, sizeof(T_acc));

    /* 工具点 (0,0,TOOL_Z,1) */
    float tool[4] = {0.0f, 0.0f, TOOL_Z, 1.0f};
    float P[4] = {0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P[i] += T_acc[i][j] * tool[j];

    pos[0] = P[0]; pos[1] = P[1]; pos[2] = P[2];

    /* 提取旋转矩阵 */
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = T_acc[i][j];
}

/* ========== 误差向量（6x1） ========== */
static void compute_error(const float th_deg[6],
                          const float P_target[3],
                          const float z_target[3],
                          const float x_target[3],
                          float err_vec[6]) {
    float pos[3];
    float R[3][3];
    forward_kinematics(th_deg, pos, R);

    /* 位置误差 */
    err_vec[0] = pos[0] - P_target[0];
    err_vec[1] = pos[1] - P_target[1];
    err_vec[2] = pos[2] - P_target[2];

    /* 姿态误差： cross(z_c, z_t) + cross(x_c, x_t) */
    float z_c[3] = {R[0][2], R[1][2], R[2][2]};
    float x_c[3] = {R[0][0], R[1][0], R[2][0]};
    float ez[3], ex[3];
    vec_cross(z_c, z_target, ez);
    vec_cross(x_c, x_target, ex);
    err_vec[3] = ez[0] + ex[0];
    err_vec[4] = ez[1] + ex[1];
    err_vec[5] = ez[2] + ex[2];
}

/* ========== 数值雅可比 ========== */
static void compute_jacobian(const float th_deg[6],
                             const float P_target[3],
                             const float z_target[3],
                             const float x_target[3],
                             float J[6][6]) {
    float F0[6];
    compute_error(th_deg, P_target, z_target, x_target, F0);
    const float delta = 5e-4f;
    for (int j = 0; j < 6; ++j) {
        float th_pert[6];
        memcpy(th_pert, th_deg, 6 * sizeof(float));
        th_pert[j] += delta;
        float F1[6];
        compute_error(th_pert, P_target, z_target, x_target, F1);
        for (int i = 0; i < 6; ++i)
            J[i][j] = (F1[i] - F0[i]) / delta;
    }
}

/* ========== Levenberg-Marquardt 迭代 ========== */
static int lm_solve(const float init_deg[6],
                    const float P_target[3],
                    const float z_target[3],
                    const float x_target[3],
                    float result_deg[6]) {
    float theta[6];
    memcpy(theta, init_deg, 6 * sizeof(float));
    float lambda = LM_LAMBDA_INIT;
    float F[6], Fnew[6];

    compute_error(theta, P_target, z_target, x_target, F);
    float err = vec_norm(F);  /* 用6维向量长度作为总误差 */

    for (int iter = 0; iter < LM_MAX_ITER; ++iter) {
        if (err < LM_TOL_F) {
            memcpy(result_deg, theta, 6 * sizeof(float));
            return 0;
        }

        float J[6][6];
        compute_jacobian(theta, P_target, z_target, x_target, J);

        /* JTJ = J^T * J,  JTF = J^T * F */
        float JTJ[6][6] = {0};
        float JTF[6] = {0};
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                for (int k = 0; k < 6; ++k)
                    JTJ[i][j] += J[k][i] * J[k][j];
            }
            JTJ[i][i] += lambda;   /* 加阻尼 */
            for (int k = 0; k < 6; ++k)
                JTF[i] += J[k][i] * F[k];
        }

        float delta[6], neg_JTF[6];
        for (int i = 0; i < 6; ++i) neg_JTF[i] = -JTF[i];
        if (!linear_solve_6(JTJ, neg_JTF, delta)) {
            lambda *= LM_LAMBDA_FACTOR;
            continue;
        }

        float step_norm = sqrtf(delta[0]*delta[0] + delta[1]*delta[1] +
                                delta[2]*delta[2] + delta[3]*delta[3] +
                                delta[4]*delta[4] + delta[5]*delta[5]);

        float theta_new[6];
        for (int i = 0; i < 6; ++i) theta_new[i] = theta[i] + delta[i];

        compute_error(theta_new, P_target, z_target, x_target, Fnew);
        float err_new = vec_norm(Fnew);

        if (err_new < err) {
            memcpy(theta, theta_new, 6 * sizeof(float));
            memcpy(F, Fnew, 6 * sizeof(float));
            err = err_new;
            lambda /= LM_LAMBDA_FACTOR;
            if (step_norm < LM_TOL_X) {
                memcpy(result_deg, theta, 6 * sizeof(float));
                return 0;
            }
        } else {
            lambda *= LM_LAMBDA_FACTOR;
        }
    }

    /* 最后检查一次 */
    compute_error(theta, P_target, z_target, x_target, F);
    if (vec_norm(F) < LM_TOL_F) {
        memcpy(result_deg, theta, 6 * sizeof(float));
        return 0;
    }
    return -1;
}

/* ========== 主逆解接口 ========== */
int ik6_pose(float x, float y, float z,
             const float z_axis[3], const float x_axis[3],
             float theta_out[6]) {
    float P_target[3] = {x, y, z};

    /* 目标方向向量归一化 + 正交化 */
    float z_t[3], x_t[3];
    memcpy(z_t, z_axis, sizeof(z_t));
    memcpy(x_t, x_axis, sizeof(x_t));
    vec_normalize(z_t);
    vec_normalize(x_t);
    float d = vec_dot(z_t, x_t);
    if (fabsf(d) > 1e-6f) {
        x_t[0] -= d * z_t[0];
        x_t[1] -= d * z_t[1];
        x_t[2] -= d * z_t[2];
        vec_normalize(x_t);
    }

    /* 初始猜测（度） */
    const float init_guess[][6] = {
        {  0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f},
        { 30.0f,  30.0f,  30.0f,  30.0f,  30.0f,  30.0f},
        {-30.0f,  30.0f, -30.0f,  30.0f, -30.0f,  30.0f},
        { 45.0f, -45.0f,  45.0f, -45.0f,  45.0f, -45.0f},
        {-45.0f,  45.0f, -45.0f,  45.0f, -45.0f,  45.0f},
        { 10.0f,  20.0f, -10.0f,  50.0f,  10.0f, -10.0f},
        {-10.0f,  40.0f,  20.0f, -20.0f,  15.0f,   5.0f},
        {  0.0f,  90.0f,   0.0f, -90.0f,   0.0f,   0.0f}
    };
    const int num_init = sizeof(init_guess) / sizeof(init_guess[0]);

    float candidates[MAX_CANDIDATES][6];
    int cand_cnt = 0;

    for (int i = 0; i < num_init && cand_cnt < MAX_CANDIDATES; ++i) {
        float sol[6];
        if (lm_solve(init_guess[i], P_target, z_t, x_t, sol) == 0) {
            float err_vec[6];
            compute_error(sol, P_target, z_t, x_t, err_vec);
            if (vec_norm(err_vec) < 1e-3f) {   /* 确认精度 */
                /* 去重 */
                int dup = 0;
                for (int k = 0; k < cand_cnt; ++k) {
                    float diff = 0.0f;
                    for (int m = 0; m < 6; ++m) {
                        float dlt = candidates[k][m] - sol[m];
                        diff += dlt * dlt;
                    }
                    if (sqrtf(diff) < 0.1f) { dup = 1; break; }
                }
                if (!dup) {
                    memcpy(candidates[cand_cnt], sol, 6 * sizeof(float));
                    cand_cnt++;
                }
            }
        }
    }

    if (cand_cnt == 0) return 1;  /* 无解 */

    /* 关节限位筛选，返回第一个满足的 */
    for (int i = 0; i < cand_cnt; ++i) {
        float *ang = candidates[i];
        if (ang[0] < J1_MIN || ang[0] > J1_MAX) continue;
        if (ang[1] < J2_MIN || ang[1] > J2_MAX) continue;
        if (ang[2] < J3_MIN || ang[2] > J3_MAX) continue;
        if (ang[3] < J4_MIN || ang[3] > J4_MAX) continue;
        if (ang[4] < J5_MIN || ang[4] > J5_MAX) continue;
        if (ang[5] < J6_MIN || ang[5] > J6_MAX) continue;
        memcpy(theta_out, ang, 6 * sizeof(float));
        return 0;
    }
    return 2;  /* 全部超限 */
}