/**
  ******************************************************************************
  * @file           : mag_calib.c
  * @brief          : 지자기 센서 캘리브레이션 (Hard-iron + Soft-iron)
  *                   타원체 피팅 알고리즘
  ******************************************************************************
  */

#include "magcalib.h"
#include <math.h>
#include <string.h>

#define NPARAM MAG_CALIB_NPARAM

/* ========== 내부 헬퍼 함수 ========== */

static void mat3_mul_vec(const double M[3][3], const double v[3], double out[3])
{
    out[0] = M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2];
    out[1] = M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2];
    out[2] = M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2];
}

static double vec3_dot(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static int mat3_inv(const double M[3][3], double inv[3][3])
{
    const double a = M[0][0], b = M[0][1], c = M[0][2];
    const double d = M[1][0], e = M[1][1], f = M[1][2];
    const double g = M[2][0], h = M[2][1], i = M[2][2];

    const double A =  (e*i - f*h);
    const double B = -(d*i - f*g);
    const double C =  (d*h - e*g);
    const double D = -(b*i - c*h);
    const double E =  (a*i - c*g);
    const double F = -(a*h - b*g);
    const double G =  (b*f - c*e);
    const double H = -(a*f - c*d);
    const double I =  (a*e - b*d);

    const double det = a*A + b*B + c*C;
    if (fabs(det) < 1e-18) return -1;

    const double invDet = 1.0 / det;

    inv[0][0] = A * invDet; inv[0][1] = D * invDet; inv[0][2] = G * invDet;
    inv[1][0] = B * invDet; inv[1][1] = E * invDet; inv[1][2] = H * invDet;
    inv[2][0] = C * invDet; inv[2][1] = F * invDet; inv[2][2] = I * invDet;
    return 0;
}

/**
  * @brief  3x3 대칭 행렬의 Jacobi 고유값 분해
  * @param  W: 입력 대칭 행렬 (파괴됨)
  * @param  V: 고유벡터 행렬 (열 = 고유벡터)
  * @param  d: 고유값 배열
  */
static void jacobi_eigen_sym3(double W[3][3], double V[3][3], double d[3])
{
    /* 1차원 배열로 변환하여 기존 jacobi 사용 */
    double A[9], Vf[9];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            A[i*3 + j] = W[i][j];

    memset(Vf, 0, sizeof(Vf));
    for (int i = 0; i < 3; i++) Vf[i*3 + i] = 1.0;
    for (int i = 0; i < 3; i++) d[i] = A[i*3 + i];

    const int n = 3;
    const int maxIter = 60;

    for (int iter = 0; iter < maxIter; iter++)
    {
        int p = 0, q = 1;
        double maxOff = 0.0;
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                double aij = fabs(A[i*n + j]);
                if (aij > maxOff) { maxOff = aij; p = i; q = j; }
            }
        }
        if (maxOff < 1e-12) break;

        double app = A[p*n + p];
        double aqq = A[q*n + q];
        double apq = A[p*n + q];

        double phi = 0.5 * atan2(2.0*apq, (aqq - app));
        double c = cos(phi);
        double s = sin(phi);

        for (int k = 0; k < n; k++)
        {
            double aik = A[p*n + k];
            double aqk = A[q*n + k];
            A[p*n + k] = c*aik - s*aqk;
            A[q*n + k] = s*aik + c*aqk;
        }
        for (int k = 0; k < n; k++)
        {
            double aki = A[k*n + p];
            double akq = A[k*n + q];
            A[k*n + p] = c*aki - s*akq;
            A[k*n + q] = s*aki + c*akq;
        }

        A[p*n + q] = 0.0;
        A[q*n + p] = 0.0;

        for (int k = 0; k < n; k++)
        {
            double vip = Vf[k*n + p];
            double viq = Vf[k*n + q];
            Vf[k*n + p] = c*vip - s*viq;
            Vf[k*n + q] = s*vip + c*viq;
        }

        d[p] = A[p*n + p];
        d[q] = A[q*n + q];
    }

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            V[i][j] = Vf[i*3 + j];
}

/**
  * @brief  대칭 양정치 행렬의 대칭 제곱근 계산
  *         W = V·D·Vᵀ  →  sqrt(W) = V·sqrt(D)·Vᵀ
  *         축 방향을 보존하는 보정 행렬을 생성
  * @param  W: 입력 대칭 양정치 행렬
  * @param  sqrtW: 출력 대칭 제곱근 행렬
  * @retval 0: 성공, -1: 양정치 아님
  */
static int mat3_sym_sqrt(double W[3][3], double sqrtW[3][3])
{
    double V[3][3], d[3];

    /* W를 고유값 분해 (W가 파괴되므로 복사본을 넘김) */
    double Wcopy[3][3];
    memcpy(Wcopy, W, sizeof(Wcopy));
    jacobi_eigen_sym3(Wcopy, V, d);

    /* 모든 고유값이 양수인지 확인 */
    for (int i = 0; i < 3; i++)
    {
        if (d[i] <= 0.0) return -1;
    }

    /* sqrtW = V · sqrt(D) · Vᵀ */
    double sd[3] = { sqrt(d[0]), sqrt(d[1]), sqrt(d[2]) };

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            double sum = 0.0;
            for (int k = 0; k < 3; k++)
            {
                sum += V[i][k] * sd[k] * V[j][k];
            }
            sqrtW[i][j] = sum;
        }
    }

    return 0;
}

static void jacobi_eigen_sym(double *A, double *V, double *d, int n)
{
    memset(V, 0, sizeof(double) * n * n);
    for (int i = 0; i < n; i++) V[i*n + i] = 1.0;
    for (int i = 0; i < n; i++) d[i] = A[i*n + i];

    const int maxIter = 60;
    for (int iter = 0; iter < maxIter; iter++)
    {
        int p = 0, q = 1;
        double maxOff = 0.0;
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                double aij = fabs(A[i*n + j]);
                if (aij > maxOff) { maxOff = aij; p = i; q = j; }
            }
        }
        if (maxOff < 1e-12) break;

        double app = A[p*n + p];
        double aqq = A[q*n + q];
        double apq = A[p*n + q];

        double phi = 0.5 * atan2(2.0*apq, (aqq - app));
        double c = cos(phi);
        double s = sin(phi);

        for (int k = 0; k < n; k++)
        {
            double aik = A[p*n + k];
            double aqk = A[q*n + k];
            A[p*n + k] = c*aik - s*aqk;
            A[q*n + k] = s*aik + c*aqk;
        }
        for (int k = 0; k < n; k++)
        {
            double aki = A[k*n + p];
            double akq = A[k*n + q];
            A[k*n + p] = c*aki - s*akq;
            A[k*n + q] = s*aki + c*akq;
        }

        A[p*n + q] = 0.0;
        A[q*n + p] = 0.0;

        for (int k = 0; k < n; k++)
        {
            double vip = V[k*n + p];
            double viq = V[k*n + q];
            V[k*n + p] = c*vip - s*viq;
            V[k*n + q] = s*vip + c*viq;
        }

        d[p] = A[p*n + p];
        d[q] = A[q*n + q];
    }
}

/* ========== API 함수 ========== */

void MagCalib_Init(MagCalibAccum_t *acc)
{
    memset(acc, 0, sizeof(*acc));
}

void MagCalib_Push(MagCalibAccum_t *acc, float Mx, float My, float Mz)
{
    double x = (double)Mx;
    double y = (double)My;
    double z = (double)Mz;

    double d[NPARAM];
    d[0] = x * x;
    d[1] = y * y;
    d[2] = z * z;
    d[3] = 2.0 * x * y;
    d[4] = 2.0 * x * z;
    d[5] = 2.0 * y * z;
    d[6] = 2.0 * x;
    d[7] = 2.0 * y;
    d[8] = 2.0 * z;
    d[9] = 1.0;

    for (int i = 0; i < NPARAM; i++)
    {
        for (int j = i; j < NPARAM; j++)
        {
            acc->S[i][j] += d[i] * d[j];
        }
    }
    acc->n++;
}

int MagCalib_Solve(const MagCalibAccum_t *acc, MagCalib_t *out)
{
    if (acc->n < 100) return -1;

    double Sfull[NPARAM * NPARAM];
    for (int i = 0; i < NPARAM; i++)
    {
        for (int j = 0; j < NPARAM; j++)
        {
            double v = (j >= i) ? acc->S[i][j] : acc->S[j][i];
            Sfull[i*NPARAM + j] = v;
        }
    }

    double V[NPARAM * NPARAM], eval[NPARAM];
    jacobi_eigen_sym(Sfull, V, eval, NPARAM);

    int kmin = 0;
    for (int k = 1; k < NPARAM; k++)
    {
        if (eval[k] < eval[kmin]) kmin = k;
    }

    double beta[NPARAM];
    for (int i = 0; i < NPARAM; i++) beta[i] = V[i*NPARAM + kmin];

    double Q[3][3] = {
        {beta[0], beta[3], beta[4]},
        {beta[3], beta[1], beta[5]},
        {beta[4], beta[5], beta[2]}
    };
    double p[3] = {beta[6], beta[7], beta[8]};
    double r = beta[9];

    double invQ[3][3];
    if (mat3_inv(Q, invQ) != 0) return -2;

    double invQp[3];
    mat3_mul_vec(invQ, p, invQp);

    double b[3] = {-invQp[0], -invQp[1], -invQp[2]};

    double Qb[3];
    mat3_mul_vec(Q, b, Qb);
    double s = vec3_dot(b, Qb) - r;

    if (s < 0.0)
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Q[i][j] = -Q[i][j];
        for (int i = 0; i < 3; i++) p[i] = -p[i];
        r = -r;

        if (mat3_inv(Q, invQ) != 0) return -3;
        mat3_mul_vec(invQ, p, invQp);
        b[0] = -invQp[0];
        b[1] = -invQp[1];
        b[2] = -invQp[2];
        mat3_mul_vec(Q, b, Qb);
        s = vec3_dot(b, Qb) - r;
        if (s <= 0.0) return -4;
    }

    /* W = Q / s (정규화된 타원체 행렬) */
    double W[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            W[i][j] = Q[i][j] / s;

    /* 핵심 변경: Cholesky → 대칭 제곱근 */
    double sqrtW[3][3];
    if (mat3_sym_sqrt(W, sqrtW) != 0) return -5;

    out->b[0] = b[0];
    out->b[1] = b[1];
    out->b[2] = b[2];
    memcpy(out->A, sqrtW, sizeof(sqrtW));

    return 0;
}

void MagCalib_Apply(const MagCalib_t *cal, float Mx, float My, float Mz, float *out)
{
    double v[3] = {
        (double)Mx - cal->b[0],
        (double)My - cal->b[1],
        (double)Mz - cal->b[2]
    };

    double result[3];
    mat3_mul_vec(cal->A, v, result);

    out[0] = (float)result[0];
    out[1] = (float)result[1];
    out[2] = (float)result[2];
}
