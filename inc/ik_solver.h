/**
 ******************************************************************************
 * @file    ik_solver.h
 * @brief   6 轴机械臂逆运动学数值求解器（Levenberg-Marquardt）
 ******************************************************************************
 * @attention
 *  - 所有长度单位为 cm，角度单位为度
 *  - 使用前请根据实际机械臂参数修改 DH 参数及关节限位宏
 */

#ifndef __IK_SOLVER_H
#define __IK_SOLVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 圆周率定义 ---------- */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------- 角度弧度转换 ---------- */
#define DEG2RAD(deg) ((deg) * (float)M_PI / 180.0f)
#define RAD2DEG(rad) ((rad) * 180.0f / (float)M_PI)

/* ---------- DH 参数（单位 cm） ---------- */
#define X_01   5.0f
#define X_23  14.6f
#define Z_45   9.5f
#define Z_56   3.0f
#define X_56   5.0f
#define TOOL_Z 10.0f        /* 工具在末端坐标系下的 Z 偏移 */

/* ---------- 关节限位（度） ---------- */
#define J1_MIN  -90.0f
#define J1_MAX   90.0f
#define J2_MIN  -90.0f
#define J2_MAX   90.0f
#define J3_MIN  -90.0f
#define J3_MAX   90.0f
#define J4_MIN  -90.0f
#define J4_MAX   90.0f
#define J5_MIN    0.0f
#define J5_MAX  180.0f
#define J6_MIN  -90.0f
#define J6_MAX   90.0f

/* ---------- LM 算法参数 ---------- */
#define LM_MAX_ITER      200
#define LM_LAMBDA_INIT   1e-2f
#define LM_LAMBDA_FACTOR 10.0f
#define LM_TOL_F         1e-3f
#define LM_TOL_X         1e-3f
#define MAX_CANDIDATES   12       /* 最多保留的解组数 */

/* ---------- 公共接口 ---------- */

/**
 * @brief  6 轴带姿态逆运动学求解
 * @param  x, y, z    末端工具点位置 (cm)
 * @param  z_axis     末端 Z 轴方向向量（3维，内部自动归一化）
 * @param  x_axis     末端 X 轴方向向量（将自动正交化到与 Z 轴垂直）
 * @param  theta_out  输出关节角数组 (度，长度 6)
 * @retval 0: 成功找到符合关节限位的解
 *         1: 未找到数值解
 *         2: 所有解超出关节限位
 */
int ik6_pose(float x, float y, float z,
             const float z_axis[3], const float x_axis[3],
             float theta_out[6]);

#ifdef __cplusplus
}
#endif

#endif /* __IK_SOLVER_H */