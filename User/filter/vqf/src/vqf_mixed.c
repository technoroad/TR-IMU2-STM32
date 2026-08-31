// Copyright (c) 2024 Hugo Chiang
// SPDX-License-Identifier: MIT

#include "libraries.h"

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;
extern VQF_Function vqf_func;

/* Private typedef -----------------------------------------------------------*/
// 変数型の選択
//#define DOUBLE
#define MIXED
//#define FLOAT

#if defined(DOUBLE)
typedef double vqf_real_t;
typedef double vqf_double_t;
#define EPS DBL_EPSILON
#elif defined(MIXED)
typedef float vqf_real_t;
typedef double vqf_double_t;
#define EPS DBL_EPSILON
#elif defined(FLOAT)
typedef float vqf_real_t;
typedef float vqf_double_t;
#define EPS FLT_EPSILON
#endif

typedef struct vqf_params_s {
    bool motionBiasEstEnabled;               // 動作中のジャイロバイアス推定を有効にするか
    bool restBiasEstEnabled;                 // 静止中のジャイロバイアス推定を有効にするか
    vqf_real_t biasSigmaInit;                // ジャイロバイアスの初期標準偏差
    vqf_real_t biasForgettingTime;           // ジャイロバイアスの忘却時間（推定の更新速度に影響）
    vqf_real_t biasClip;                     // ジャイロバイアスの最大値（クリップ制限）
    vqf_real_t biasSigmaMotion;              // 動作中のジャイロバイアス推定の標準偏差
    vqf_real_t biasVerticalForgettingFactor; // 垂直方向のバイアス忘却係数（姿勢安定性に影響）
    vqf_real_t biasSigmaRest;                // 静止中のジャイロバイアス推定の標準偏差
    vqf_real_t restMinT;                     // 静止判定に必要な最小時間
    vqf_real_t restFilterTau;                // 静止判定用フィルタの時定数
    vqf_real_t restThGyr;                    // 静止判定のジャイロ閾値
    vqf_real_t restThAcc;                    // 静止判定の加速度閾値
    vqf_real_t tauAcc;                       // 加速度の低パスフィルタの時定数（姿勢補正用）

    bool magDistRejectionEnabled;            // 磁気センサの外乱除去を有効にするか
    vqf_real_t tauMag;                       // 磁気センサの低パスフィルタの時定数（姿勢補正用）
    vqf_real_t magCurrentTau;                // 現在の磁気データのフィルタ時定数
    vqf_real_t magRefTau;                    // 参照磁気データのフィルタ時定数
    vqf_real_t magNormTh;                    // 磁気ベクトルのノルム（大きさ）による外乱判定閾値
    vqf_real_t magDipTh;                     // 磁気ベクトルの傾き（dip angle）による外乱判定閾値
    vqf_real_t magNewTime;                   // 新しい磁気データの更新時間
    vqf_real_t magNewFirstTime;              // 初回の磁気データ更新時間
    vqf_real_t magNewMinGyr;                 // 磁気データ更新に必要な最小ジャイロ値
    vqf_real_t magMinUndisturbedTime;        // 外乱のない状態が続く最小時間
    vqf_real_t magMaxRejectionTime;          // 磁気外乱の最大拒否時間
    vqf_real_t magRejectionFactor;           // 磁気外乱の拒否強度（フィルタの重み付けに影響）
} vqf_params_t;

typedef struct vqf_coeffs_s {
    vqf_real_t gyrTs;
    vqf_real_t accTs;
    vqf_real_t magTs;
    vqf_double_t accLpB[3];
    vqf_double_t accLpA[2];
    vqf_real_t kMag;
    vqf_real_t biasP0;
    vqf_real_t biasV;
    vqf_real_t biasMotionW;
    vqf_real_t biasVerticalW;
    vqf_real_t biasRestW;
    vqf_double_t restGyrLpB[3];
    vqf_double_t restGyrLpA[2];
    vqf_double_t restAccLpB[3];
    vqf_double_t restAccLpA[2];
    vqf_real_t kMagRef;
    vqf_double_t magNormDipLpB[3];
    vqf_double_t magNormDipLpA[2];
} vqf_coeffs_t ;

typedef struct vqf_state_s {
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    vqf_real_t delta;
    bool restDetected;
    bool magDistDetected;
    vqf_real_t lastAccLp[3];
    vqf_double_t accLpState[3*2];
    vqf_real_t lastAccCorrAngularRate;
    vqf_real_t kMagInit;
    vqf_real_t lastMagDisAngle;
    vqf_real_t lastMagCorrAngularRate;
    vqf_real_t bias[3];
    vqf_real_t biasP[9];
    vqf_double_t motionBiasEstRLpState[9*2];
    vqf_double_t motionBiasEstBiasLpState[2*2];
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    vqf_double_t restGyrLpState[3*2];
    vqf_real_t restLastAccLp[3];
    vqf_double_t restAccLpState[3*2];
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    vqf_double_t magNormDipLpState[2*2];
} vqf_state_t;

/* Private define ------------------------------------------------------------*/
#define M_SQRT2     1.41421356237309504880   // sqrt(2)s

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static vqf_params_t params;
static vqf_coeffs_t coeffs;
static vqf_state_t state;

/* Private function prototypes -----------------------------------------------*/
static void updateGyr(const vqf_real_t gyr[3]);
static void updateAcc(const vqf_real_t acc[3]);
static void getQuat3D(vqf_real_t out[4]);
static void getQuat6D(vqf_real_t out[4]);
static void initVqf(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);
static void init_params(void);
static void vqf_setup();
static void vqf_resetState(bool resetBias);
static void vqf_resetFilter();

static void VQF_Initialize();
static void VQF_ResetFilter();
static void UpdateGyro(double *gyro);
static void UpdateAccl(double *g);
static void get_quat_3d(float *out);
static void get_quat_6d(float *out);
static bool get_reset_detected();


void VQF_Mixed_RegisterFunctions(){
  vqf_func.Initialize                 = NULL;
  vqf_func.Reset                      = NULL;
  vqf_func.UpdateGyro                 = NULL;
  vqf_func.UpdateAccl                 = NULL;
  vqf_func.GetQuat3D                  = NULL;
  vqf_func.GetQuat6D                  = NULL;
  vqf_func.GetRestDetected            = NULL;

  vqf_func.Initialize                 = VQF_Initialize;
  vqf_func.Reset                      = VQF_ResetFilter;
  vqf_func.UpdateGyro                 = UpdateGyro;
  vqf_func.UpdateAccl                 = UpdateAccl;
  vqf_func.GetQuat3D                  = get_quat_3d;
  vqf_func.GetQuat6D                  = get_quat_6d;
  vqf_func.GetRestDetected            = get_reset_detected;
}

static void VQF_Initialize(double dt){
  init_params();
  initVqf((vqf_real_t)dt,(vqf_real_t)dt,-1);
}

static void VQF_ResetFilter(){
  vqf_resetFilter();
}

static void UpdateGyro(double *gyro){
  vqf_real_t gyr[3];

  for(int i= 0;i<3;i++){
    gyr[i] = (vqf_real_t)gyro[i];
  }


  // 100us
  updateGyr(gyr);

}

static void UpdateAccl(double *g){
  vqf_real_t acc[3];

  for(int i= 0;i<3;i++){
    acc[i] = (vqf_real_t)(g[i] *STANDARD_GRAVITY);
  }
  // 510us
  updateAcc(acc);
}

static void get_quat_3d(float *out){
  vqf_real_t qua[4];

  getQuat3D(qua);

  for (int i = 0; i < 4; i++) {
    out[i] = (float) qua[i];
  }
}


static void get_quat_6d(float *out){
  vqf_real_t qua[4];

  getQuat6D(qua);

  for (int i = 0; i < 4; i++) {
    out[i] = (float) qua[i];
  }
}

static bool get_reset_detected(){
  return state.restDetected;
}











static void init_params(void)
{
#if 0
  params.tauAcc = 3.0;
  params.tauMag = 9.0;
  params.motionBiasEstEnabled = true;
  params.restBiasEstEnabled = true;
  params.magDistRejectionEnabled = false;
  params.biasSigmaInit = 0.5;
  params.biasForgettingTime = 100.0;
  params.biasClip = 2.0;
  params.biasSigmaMotion = 0.1;
  params.biasVerticalForgettingFactor = 0.0001f; // 動作中に観測不能な重力軸方向バイアスを弱くゼロへ戻す係数（0で減衰無効）
  params.biasSigmaRest = 0.03;
  params.restMinT = 1.5;
  params.restFilterTau = 0.5;
  params.restThGyr = 2.0;
  params.restThAcc = 0.5;
  params.magCurrentTau = 0.05;
  params.magRefTau = 20.0;
  params.magNormTh = 0.1;
  params.magDipTh = 10.0;
  params.magNewTime = 20.0;
  params.magNewFirstTime = 5.0;
  params.magNewMinGyr = 20.0;
  params.magMinUndisturbedTime = 0.5;
  params.magMaxRejectionTime = 60.0;
  params.magRejectionFactor = 2.0;

#else
    params.tauAcc = 1.0;           // 高レートで応答性確保（既定3.0→1.0）
    params.tauMag = 9.0;           // 無効（magDistRejectionEnabled=false）
    params.motionBiasEstEnabled = true;
    params.restBiasEstEnabled   = true;
    params.magDistRejectionEnabled = false;

    params.biasSigmaInit = 0.2;    // 初期不確かさ（既定0.5→0.2）
    params.biasForgettingTime = 100.0; //
    params.biasClip = 2.0;         // 最大バイアス範囲（既定2.0→0.5）
    params.biasSigmaMotion = 0.05; // 動作時推定（既定0.1→0.05）
    params.biasSigmaRest = 0.02;   // 静止時推定（既定0.03→0.02）
    params.biasVerticalForgettingFactor = 0.0001f; // 動作中に観測不能な重力軸方向バイアスを弱くゼロへ戻す係数（0で減衰無効）

    params.restMinT = 0.8;         // 静止判定時間（既定1.5→0.8）
    params.restFilterTau = 0.3;    // 静止検出LPF（既定0.5→0.3）
    params.restThGyr = 0.8;        // ジャイロ閾値（既定2.0→0.8）
    params.restThAcc = 0.3;        // 加速度閾値（既定0.5→0.3）
#endif
}

static void vqf_fill_real(vqf_real_t *dst, size_t n, vqf_real_t val)
{
    for (size_t i = 0; i < n; i++) {
        dst[i] = val;
    }
}

static void vqf_fill_double(vqf_double_t *dst, size_t n, vqf_double_t val)
{
    for (size_t i = 0; i < n; i++) {
        dst[i] = val;
    }
}
static vqf_real_t vqf_square(vqf_real_t x) {
    return x*x;
}

static vqf_real_t vqf_max(vqf_real_t a, vqf_real_t b) {
    return a > b ? a : b;
}

#if 0
static vqf_real_t vqf_min(vqf_real_t a, vqf_real_t b) {
    return a < b ? a : b;
}
#endif

__STATIC_FORCEINLINE vqf_real_t norm3(const vqf_real_t *v)
{
    const vqf_real_t a = v[0] * v[0];
    const vqf_real_t b = v[1] * v[1];
    const vqf_real_t c = v[2] * v[2];
    return sqrt(a + b + c);
}

__STATIC_FORCEINLINE void normalize4(vqf_real_t *v)
{
    const vqf_real_t n =
        sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);

    if (n < EPS) {
        return;
    }

    const vqf_real_t inv = 1.0 / n;
    v[0] *= inv;
    v[1] *= inv;
    v[2] *= inv;
    v[3] *= inv;
}

static void normalize3(vqf_real_t vec[3])
{
    vqf_real_t n = norm3(vec);
    if (n < EPS) {
        return;
    }
    vec[0] /= n;
    vec[1] /= n;
    vec[2] /= n;
}

static void clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max)
{
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) {
            vec[i] = min;
        } else if (vec[i] > max) {
            vec[i] = max;
        }
    }
}


static inline void quatMultiply(const vqf_real_t *q1, const vqf_real_t *q2, vqf_real_t *out)
{
    vqf_real_t a0 = q1[0], a1 = q1[1], a2 = q1[2], a3 = q1[3];
    vqf_real_t b0 = q2[0], b1 = q2[1], b2 = q2[2], b3 = q2[3];

    vqf_real_t w = a0*b0 - a1*b1 - a2*b2 - a3*b3;
    vqf_real_t x = a0*b1 + a1*b0 + a2*b3 - a3*b2;
    vqf_real_t y = a0*b2 - a1*b3 + a2*b0 + a3*b1;
    vqf_real_t z = a0*b3 + a1*b2 - a2*b1 + a3*b0;

    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

static void quatSetToIdentity(vqf_real_t out[4])
{
    out[0] = 1;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}

#if 0
static void quatApplyDelta(vqf_real_t q[], vqf_real_t delta, vqf_real_t out[])
{
    // out = quatMultiply([cos(delta/2), 0, 0, sin(delta/2)], q)
    // sin and cos can be replaced by arm_sin_f32 and arm_cos_f32 from CMSIS-DSP
    vqf_real_t c = cos(delta/2.0);
    vqf_real_t s = sin(delta/2.0);
    vqf_real_t w = c * q[0] - s * q[3];
    vqf_real_t x = c * q[1] - s * q[2];
    vqf_real_t y = c * q[2] + s * q[1];
    vqf_real_t z = c * q[3] + s * q[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}
#endif

static void quatRotate(const vqf_real_t *q, const vqf_real_t *v, vqf_real_t *out)
{
    const vqf_real_t qw = q[0];
    const vqf_real_t qx = q[1];
    const vqf_real_t qy = q[2];
    const vqf_real_t qz = q[3];

    // t = 2 * (qv × v)
    const vqf_real_t tx = 2 * (qy * v[2] - qz * v[1]);
    const vqf_real_t ty = 2 * (qz * v[0] - qx * v[2]);
    const vqf_real_t tz = 2 * (qx * v[1] - qy * v[0]);

    // out = v + qw * t + qv × t
    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}



static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts)
{
    // assert(Ts > 0);
    if (tau < (vqf_real_t)(0.0)) {
        return 0; // k=0 for negative tau (disable update)
    } else if (tau == (vqf_real_t)(0.0)) {
        return 1; // k=1 for tau=0
    } else {
        return 1 - exp(-Ts/tau);  // fc = 1/(2*pi*tau)
    }
}

static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_double_t outB[], vqf_double_t outA[])
{
    // assert(tau > 0);
    // assert(Ts > 0);
    // second order Butterworth filter based on https://stackoverflow.com/a/52764064
    vqf_double_t fc = (M_SQRT2 / (2.0*M_PI))/(vqf_double_t)(tau); // time constant of dampened, non-oscillating part of step response
    // tan_fast can be replaced by sin/cos from CMSIS_DSP lib
    vqf_double_t C = tan(M_PI*fc*(vqf_double_t)(Ts));
    // sqrt can be replaced by arm_sqrt_f32 from CMSIS_DSP
    vqf_double_t D = C*C + sqrt(2)*C + 1;
    vqf_double_t b0 = C*C/D;
    outB[0] = b0;
    outB[1] = 2*b0;
    outB[2] = b0;
    // a0 = 1.0
    outA[0] = 2*(C*C-1)/D; // a1
    // sqrt can be replaced by arm_sqrt_f32 from CMSIS_DSP
    outA[1] = (1-sqrt(2)*C+C*C)/D; // a2
}

#if 0
static void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const vqf_double_t b_old[],
                                         const vqf_double_t a_old[], const vqf_double_t b_new[],
                                         const vqf_double_t a_new[], vqf_double_t state[])
{
    if (isnan(state[0])) {
        return;
    }
    for (size_t i = 0; i < N; i++) {
        state[0+2*i] = state[0+2*i] + (b_old[0] - b_new[0])*last_y[i];
        state[1+2*i] = state[1+2*i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0])*last_y[i];
    }
}
#endif

static void filterInitialState(vqf_real_t x0, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t out[])
{
    // initial state for steady state (equivalent to scipy.signal.lfilter_zi, obtained by setting y=x=x0 in the filter
    // update equation)
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

static vqf_real_t filterStep(vqf_real_t x, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t state[2])
{
    // difference equations based on scipy.signal.lfilter documentation
    // assumes that a0 == 1.0
    vqf_double_t y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_double_t b[3],
                    const vqf_double_t a[2], vqf_double_t state[], vqf_real_t out[])
{
    // assert(N>=2);

    // to avoid depending on a single sample, average the first samples (for duration tau)
    // and then use this average to calculate the filter initial state
    if (isnan(state[0])) { // initialization phase
        if (isnan(state[1])) { // first sample
            state[1] = 0; // state[1] is used to store the sample count
            for(size_t i = 0; i < N; i++) {
                state[2+i] = 0; // state[2+i] is used to store the sum
            }
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) {
            for(size_t i = 0; i < N; i++) {
               filterInitialState(out[i], b, a, state+2*i);
            }
        }
        return;
    }

    for (size_t i = 0; i < N; i++) {
        out[i] = filterStep(x[i], b, a, state+2*i);
    }
}

static void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t *out)
{
    out[0] = scale;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
    out[4] = scale;
    out[5] = 0.0;
    out[6] = 0.0;
    out[7] = 0.0;
    out[8] = scale;
}

static void matrix3Multiply(const vqf_real_t * in1, const vqf_real_t * in2, vqf_real_t *out)
{
    out[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6];
    out[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7];
    out[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
    out[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6];
    out[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7];
    out[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
    out[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6];
    out[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7];
    out[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
}

static void matrix3MultiplyTpsFirst(const vqf_real_t * in1, const vqf_real_t * in2, vqf_real_t *out)
{
    out[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6];
    out[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7];
    out[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
    out[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6];
    out[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7];
    out[5] = in1[1]*in2[2] + in1[4]*in2[5] + in1[7]*in2[8];
    out[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6];
    out[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7];
    out[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
}

static void matrix3MultiplyTpsSecond(const vqf_real_t * in1, const vqf_real_t * in2, vqf_real_t *out)
{
    out[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2];
    out[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5];
    out[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
    out[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2];
    out[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
    out[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
    out[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2];
    out[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5];
    out[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
}

static bool matrix3Inv(const vqf_real_t *in, vqf_real_t *out)
{
    // ローカルに読み出し（レジスタ展開される）
    vqf_real_t a = in[0], b = in[1], c = in[2];
    vqf_real_t d = in[3], e = in[4], f = in[5];
    vqf_real_t g = in[6], h = in[7], i = in[8];

    // まず必要な 2×2 の組み合わせを先に計算
    vqf_double_t ei_fh = e*i - f*h;
    vqf_double_t fg_di = f*g - d*i;
    vqf_double_t dh_eg = d*h - e*g;

    // ここで cofactors の 1行目を算出（A,B,C）
    vqf_double_t A =  ei_fh;
    vqf_double_t B =  fg_di;      // -(d*i - f*g)
    vqf_double_t C =  dh_eg;

    // det = a*A + b*B + c*C
    vqf_double_t det = a*A + b*B + c*C;

    if (det >= -EPS && det <= EPS) {
        for (int j = 0; j < 9; j++) out[j] = 0;
        return false;
    }

    // 逆数を1回だけ求める（除算 1回化）
    vqf_double_t invDet = 1.0 / det;

    // 残りの cofactors（2行目・3行目）
    vqf_double_t D = (c*h - b*i);
    vqf_double_t E = (a*i - c*g);
    vqf_double_t F = (b*g - a*h);

    vqf_double_t G = (b*f - c*e);
    vqf_double_t H = (c*d - a*f);
    vqf_double_t I = (a*e - b*d);

    // 転置せずに並べる（cofactor 行列は最終で転置＝adjugate）
    out[0] = A * invDet;
    out[1] = D * invDet;
    out[2] = G * invDet;
    out[3] = B * invDet;
    out[4] = E * invDet;
    out[5] = H * invDet;
    out[6] = C * invDet;
    out[7] = F * invDet;
    out[8] = I * invDet;

    return true;
}







static void updateGyr(const vqf_real_t gyr[3])
{
    // rest detection
    if (params.restBiasEstEnabled || params.magDistRejectionEnabled) {
        filterVec(gyr, 3, params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA,
                  state.restGyrLpState, state.restLastGyrLp);
        state.restLastSquaredDeviations[0] = vqf_square(gyr[0] - state.restLastGyrLp[0])
                + vqf_square(gyr[1] - state.restLastGyrLp[1]) + vqf_square(gyr[2] - state.restLastGyrLp[2]);

        vqf_real_t biasClip = params.biasClip*(vqf_real_t)(M_PI/180.0);
        if (state.restLastSquaredDeviations[0] >= vqf_square(params.restThGyr*(vqf_real_t)(M_PI/180.0))
                // fabs can be replaced by arm_abs_f32 from CMSIS-DSP
                || fabs(state.restLastGyrLp[0]) > biasClip || fabs(state.restLastGyrLp[1]) > biasClip
                || fabs(state.restLastGyrLp[2]) > biasClip) {
            state.restT = 0.0;
            state.restDetected = false;
        }
    }

    // remove estimated gyro bias
    vqf_real_t gyrNoBias[3] = {gyr[0]-state.bias[0], gyr[1]-state.bias[1], gyr[2]-state.bias[2]};

    // gyroscope prediction step
    vqf_real_t gyrNorm = norm3(gyrNoBias);
    vqf_real_t angle = gyrNorm * coeffs.gyrTs;
    if (gyrNorm > EPS) {
        // sin cos can be replaced by arm_sin_f32 and arm_cos_f32 from CMSIS-DSP
        vqf_real_t c = cos(angle/2.0);
        vqf_real_t s = sin(angle/2.0)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        quatMultiply(state.gyrQuat, gyrStepQuat, state.gyrQuat);
        normalize4(state.gyrQuat);
    }
}

static void updateAcc(const vqf_real_t acc[3])
{
    // ignore [0 0 0] samples
    if (acc[0] == (vqf_real_t)(0.0) && acc[1] == (vqf_real_t)(0.0) && acc[2] == (vqf_real_t)(0.0)) {
        return;
    }

    // rest detection
    if (params.restBiasEstEnabled) {
        filterVec(acc, 3, params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA,
                  state.restAccLpState, state.restLastAccLp);

        state.restLastSquaredDeviations[1] = vqf_square(acc[0] - state.restLastAccLp[0])
                + vqf_square(acc[1] - state.restLastAccLp[1]) + vqf_square(acc[2] - state.restLastAccLp[2]);

        if (state.restLastSquaredDeviations[1] >= vqf_square(params.restThAcc)) {
            state.restT = 0.0;
            state.restDetected = false;
        } else {
            state.restT += coeffs.accTs;
            if (state.restT >= params.restMinT) {
                state.restDetected = true;
            }
        }
    }

    vqf_real_t accEarth[3];

    // filter acc in inertial frame
    quatRotate(state.gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.accLpState, state.lastAccLp);

    // transform to 6D earth frame and normalize
    quatRotate(state.accQuat, state.lastAccLp, accEarth);
    normalize3(accEarth);

    // 100us

    // inclination correction
    vqf_real_t accCorrQuat[4];
    // sqrt can be replaced by arm_sqrt_f32 from CMSIS_DSP
    vqf_real_t q_w = sqrt((accEarth[2]+1)/2);
    if (q_w > 1e-6) {
        accCorrQuat[0] = q_w;
        accCorrQuat[1] = 0.5*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5*accEarth[0]/q_w;
        accCorrQuat[3] = 0;
    } else {
        // to avoid numeric issues when acc is close to [0 0 -1], i.e. the correction step is close (<= 0.00011°) to 180°:
        accCorrQuat[0] = 0;
        accCorrQuat[1] = 1;
        accCorrQuat[2] = 0;
        accCorrQuat[3] = 0;
    }
    quatMultiply(accCorrQuat, state.accQuat, state.accQuat);
    normalize4(state.accQuat);

    // calculate correction angular rate to facilitate debugging
    // acos can be replaced by 2*arctan( sqrt(1-x*x) / x )
    state.lastAccCorrAngularRate = acos(accEarth[2])/coeffs.accTs;

    // 80 ~ 100us

//    ElapsedMicroseconds(&calc_sw);
    // before 350us / after 278us
    // bias estimation
    if (params.motionBiasEstEnabled || params.restBiasEstEnabled) {
        vqf_real_t biasClip = params.biasClip*(vqf_real_t)(M_PI/180.0);

        vqf_real_t accGyrQuat[4];
        vqf_real_t R[9];
        vqf_real_t biasLp[2];

        // get rotation matrix corresponding to accGyrQuat
        getQuat6D(accGyrQuat);
        R[0] = 1 - 2*vqf_square(accGyrQuat[2]) - 2*vqf_square(accGyrQuat[3]); // r11
        R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]); // r12
        R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]); // r13
        R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]); // r21
        R[4] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[3]); // r22
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]); // r23
        R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]); // r31
        R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]); // r32
        R[8] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[2]); // r33

        // calculate R*b_hat (only the x and y component, as z is not needed)
        biasLp[0] = R[0]*state.bias[0] + R[1]*state.bias[1] + R[2]*state.bias[2];
        biasLp[1] = R[3]*state.bias[0] + R[4]*state.bias[1] + R[5]*state.bias[2];

        // low-pass filter R and R*b_hat
        filterVec(R, 9, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstRLpState, R);
        filterVec(biasLp, 2, params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA, state.motionBiasEstBiasLpState,
                  biasLp);

        // set measurement error and covariance for the respective Kalman filter update
        vqf_real_t w[3];
        vqf_real_t e[3];
        if (state.restDetected && params.restBiasEstEnabled) {
            e[0] = state.restLastGyrLp[0] - state.bias[0];
            e[1] = state.restLastGyrLp[1] - state.bias[1];
            e[2] = state.restLastGyrLp[2] - state.bias[2];
            matrix3SetToScaledIdentity(1.0, R);
            vqf_fill_real(w, 3, coeffs.biasRestW);
            // std::fill(w, w+3, coeffs.biasRestW);
        } else if (params.motionBiasEstEnabled) {
            e[0] = -accEarth[1]/coeffs.accTs + biasLp[0] - R[0]*state.bias[0] - R[1]*state.bias[1] - R[2]*state.bias[2];
            e[1] = accEarth[0]/coeffs.accTs + biasLp[1] - R[3]*state.bias[0] - R[4]*state.bias[1] - R[5]*state.bias[2];
            e[2] = - R[6]*state.bias[0] - R[7]*state.bias[1] - R[8]*state.bias[2];
            w[0] = coeffs.biasMotionW;
            w[1] = coeffs.biasMotionW;
            w[2] = coeffs.biasVerticalW;
        } else {
            vqf_fill_real(w, 3, -1);
            // std::fill(w, w+3, -1); // disable update
        }
//        vq_time = ElapsedMicroseconds(&calc_sw);
        // Kalman filter update
        // step 1: P = P + V (also increase covariance if there is no measurement update!)
        if (state.biasP[0] < coeffs.biasP0) {
            state.biasP[0] += coeffs.biasV;
        }
        if (state.biasP[4] < coeffs.biasP0) {
            state.biasP[4] += coeffs.biasV;
        }
        if (state.biasP[8] < coeffs.biasP0) {
            state.biasP[8] += coeffs.biasV;
        }
#if 0
        if (w[0] >= 0) {
            // clip disagreement to -2..2 °/s
            // (this also effectively limits the harm done by the first inclination correction step)
            clip(e, 3, -biasClip, biasClip);

            // step 2: K = P R^T inv(W + R P R^T)
            vqf_real_t K[9];
            matrix3MultiplyTpsSecond(state.biasP, R, K); // K = P R^T
            matrix3Multiply(R, K, K); // K = R P R^T
            K[0] += w[0];
            K[4] += w[1];
            K[8] += w[2]; // K = W + R P R^T
            matrix3Inv(K, K); // K = inv(W + R P R^T)
            matrix3MultiplyTpsFirst(R, K, K); // K = R^T inv(W + R P R^T)
            matrix3Multiply(state.biasP, K, K); // K = P R^T inv(W + R P R^T)

            // step 3: bias = bias + K (y - R bias) = bias + K e
            state.bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
            state.bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
            state.bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

            // step 4: P = P - K R P
            matrix3Multiply(K, R, K); // K = K R
            matrix3Multiply(K, state.biasP, K); // K = K R P
            for(size_t i = 0; i < 9; i++) {
                state.biasP[i] -= K[i];
            }

            // clip bias estimate to -2..2 °/s
            clip(state.bias, 3, -biasClip, biasClip);
        }
#else
        if (w[0] >= 0) {
            // clip disagreement to -2..2 °/s
            // (this also effectively limits the harm done by the first inclination correction step)
            clip(e, 3, -biasClip, biasClip);

            // step 2: K = P R^T inv(W + R P R^T)
            static vqf_real_t K1[9],K2[9];
            matrix3MultiplyTpsSecond(state.biasP, R, K1); // K = P R^T
            matrix3Multiply(R, K1, K2); // K = R P R^T
            K2[0] += w[0];
            K2[4] += w[1];
            K2[8] += w[2]; // K = W + R P R^T
            matrix3Inv(K2, K1); // K = inv(W + R P R^T)
            matrix3MultiplyTpsFirst(R, K1, K2); // K = R^T inv(W + R P R^T)
            matrix3Multiply(state.biasP, K2, K1); // K = P R^T inv(W + R P R^T)

            // step 3: bias = bias + K (y - R bias) = bias + K e
            state.bias[0] += K1[0]*e[0] + K1[1]*e[1] + K1[2]*e[2];
            state.bias[1] += K1[3]*e[0] + K1[4]*e[1] + K1[5]*e[2];
            state.bias[2] += K1[6]*e[0] + K1[7]*e[1] + K1[8]*e[2];

            // step 4: P = P - K R P
            matrix3Multiply(K1, R, K2); // K = K R
            matrix3Multiply(K2, state.biasP, K1); // K = K R P
            for(size_t i = 0; i < 9; i++) {
                state.biasP[i] -= K1[i];
            }

            // clip bias estimate to -2..2 °/s
            clip(state.bias, 3, -biasClip, biasClip);
        }
#endif

//            vq_time = ElapsedMicroseconds(&calc_sw);
    }
}

static void getQuat3D(vqf_real_t out[4])
{
    memcpy(out, state.gyrQuat, sizeof(state.gyrQuat));
    // std::copy(state.gyrQuat, state.gyrQuat+4, out);
}

static void getQuat6D(vqf_real_t out[4])
{
    quatMultiply(state.accQuat, state.gyrQuat, out);
}

//static void getQuat9D(vqf_real_t out[4])
//{
//    quatMultiply(state.accQuat, state.gyrQuat, out);
//    quatApplyDelta(out, state.delta, out);
//}
//

#if 0
static void updateMag(const vqf_real_t mag[3])
{
    // ignore [0 0 0] samples
    if (mag[0] == (vqf_real_t)(0.0) && mag[1] == (vqf_real_t)(0.0) && mag[2] == (vqf_real_t)(0.0)) {
        return;
    }

    vqf_real_t magEarth[3];

    // bring magnetometer measurement into 6D earth frame
    vqf_real_t accGyrQuat[4];
    getQuat6D(accGyrQuat);
    quatRotate(accGyrQuat, mag, magEarth);

    if (params.magDistRejectionEnabled) {
        state.magNormDip[0] = norm3(magEarth);
        // asin can be replace by 2*arctan(x / sqrt(1-x*x))
        state.magNormDip[1] = -asin(magEarth[2]/state.magNormDip[0]);

        if (params.magCurrentTau > 0) {
            filterVec(state.magNormDip, 2, params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB,
                      coeffs.magNormDipLpA, state.magNormDipLpState, state.magNormDip);
        }

        // magnetic disturbance detection
        // fabs can be replaced by arm_abs_f32 from CMSIS-DSP
        if (fabs(state.magNormDip[0] - state.magRefNorm) < params.magNormTh*state.magRefNorm
                && fabs(state.magNormDip[1] - state.magRefDip) < params.magDipTh*(vqf_real_t)(M_PI/180.0)) {
            state.magUndisturbedT += coeffs.magTs;
            if (state.magUndisturbedT >= params.magMinUndisturbedTime) {
                state.magDistDetected = false;
                state.magRefNorm += coeffs.kMagRef*(state.magNormDip[0] - state.magRefNorm);
                state.magRefDip += coeffs.kMagRef*(state.magNormDip[1] - state.magRefDip);
            }
        } else {
            state.magUndisturbedT = 0.0;
            state.magDistDetected = true;
        }

        // new magnetic field acceptance
        // fabs can be replaced by arm_abs_f32 from CMSIS-DSP
        if (fabs(state.magNormDip[0] - state.magCandidateNorm) < params.magNormTh*state.magCandidateNorm
                && fabs(state.magNormDip[1] - state.magCandidateDip) < params.magDipTh*(vqf_real_t)(M_PI/180.0)) {
            if (norm3(state.restLastGyrLp) >= params.magNewMinGyr*M_PI/180.0) {
                state.magCandidateT += coeffs.magTs;
            }
            state.magCandidateNorm += coeffs.kMagRef*(state.magNormDip[0] - state.magCandidateNorm);
            state.magCandidateDip += coeffs.kMagRef*(state.magNormDip[1] - state.magCandidateDip);

            if (state.magDistDetected && (state.magCandidateT >= params.magNewTime || (
                    state.magRefNorm == 0.0 && state.magCandidateT >= params.magNewFirstTime))) {
                state.magRefNorm = state.magCandidateNorm;
                state.magRefDip = state.magCandidateDip;
                state.magDistDetected = false;
                state.magUndisturbedT = params.magMinUndisturbedTime;
            }
        } else {
            state.magCandidateT = 0.0;
            state.magCandidateNorm = state.magNormDip[0];
            state.magCandidateDip = state.magNormDip[1];
        }
    }

    // calculate disagreement angle based on current magnetometer measurement
    state.lastMagDisAngle = atan2(magEarth[0], magEarth[1]) - state.delta;

    // make sure the disagreement angle is in the range [-pi, pi]
    if (state.lastMagDisAngle > (vqf_real_t)(M_PI)) {
        state.lastMagDisAngle -= (vqf_real_t)(2*M_PI);
    } else if (state.lastMagDisAngle < (vqf_real_t)(-M_PI)) {
        state.lastMagDisAngle += (vqf_real_t)(2*M_PI);
    }

    vqf_real_t k = coeffs.kMag;

    if (params.magDistRejectionEnabled) {
        // magnetic disturbance rejection
        if (state.magDistDetected) {
            if (state.magRejectT <= params.magMaxRejectionTime) {
                state.magRejectT += coeffs.magTs;
                k = 0;
            } else {
                k /= params.magRejectionFactor;
            }
        } else {
            state.magRejectT = vqf_max(state.magRejectT - params.magRejectionFactor*coeffs.magTs, (vqf_real_t)(0.0));
        }
    }

    // ensure fast initial convergence
    if (state.kMagInit != (vqf_real_t)(0.0)) {
        // make sure that the gain k is at least 1/N, N=1,2,3,... in the first few samples
        if (k < state.kMagInit) {
            k = state.kMagInit;
        }

        // iterative expression to calculate 1/N
        state.kMagInit = state.kMagInit/(state.kMagInit+1);

        // disable if t > tauMag
        if (state.kMagInit*params.tauMag < coeffs.magTs) {
            state.kMagInit = 0.0;
        }
    }

    // first-order filter step
    state.delta += k*state.lastMagDisAngle;
    // calculate correction angular rate to facilitate debugging
    state.lastMagCorrAngularRate = k*state.lastMagDisAngle/coeffs.magTs;

    // make sure delta is in the range [-pi, pi]
    if (state.delta > (vqf_real_t)(M_PI)) {
        state.delta -= (vqf_real_t)(2*M_PI);
    } else if (state.delta < (vqf_real_t)(-M_PI)) {
        state.delta += (vqf_real_t)(2*M_PI);
    }
}

//void update(const vqf_real_t gyr[3], const vqf_real_t acc[3])
//{
//    updateGyr(gyr);
//    updateAcc(acc);
//}
//
//void update(const vqf_real_t gyr[3], const vqf_real_t acc[3], const vqf_real_t mag[3])
//{
//    updateGyr(gyr);
//    updateAcc(acc);
//    updateMag(mag);
//}

// void updateBatch(const vqf_real_t gyr[], const vqf_real_t acc[], const vqf_real_t mag[], size_t N,
//                       vqf_real_t out6D[], vqf_real_t out9D[], vqf_real_t outDelta[], vqf_real_t outBias[],
//                       vqf_real_t outBiasSigma[], bool outRest[], bool outMagDist[])
// {
//     for (size_t i = 0; i < N; i++) {
//         if (mag) {
//             update(gyr+3*i, acc+3*i, mag+3*i);
//         } else {
//             update(gyr+3*i, acc+3*i);
//         }
//         if (out6D) {
//             getQuat6D(out6D+4*i);
//         }
//         if (out9D) {
//             getQuat9D(out9D+4*i);
//         }
//         if (outDelta) {
//             outDelta[i] = state.delta;
//         }
//         if (outBias) {
//             memcpy(outBias+3*i, state.bias, sizeof(state.bias));
//             // std::copy(state.bias, state.bias+3, outBias+3*i);
//         }
//         if (outBiasSigma) {
//             outBiasSigma[i] = getBiasEstimate({0,0,0});
//         }
//         if (outRest) {
//             outRest[i] = state.restDetected;
//         }
//         if (outMagDist) {
//             outMagDist[i] = state.magDistDetected;
//         }
//     }
// }
//vqf_real_t getDelta()
//{
//    return state.delta;
//}
//
//vqf_real_t getBiasEstimate(vqf_real_t out[3])
//{
//    if (out) {
//        memcpy(out, state.bias, sizeof(state.bias));
//        // std::copy(state.bias, state.bias+3, out);
//    }
//    // use largest absolute row sum as upper bound estimate for largest eigenvalue (Gershgorin circle theorem)
//    // and clip output to biasSigmaInit
//    // fabs can be replaced by arm_abs_f32 from CMSIS-DSP
//    vqf_real_t sum1 = fabs(state.biasP[0]) + fabs(state.biasP[1]) + fabs(state.biasP[2]);
//    vqf_real_t sum2 = fabs(state.biasP[3]) + fabs(state.biasP[4]) + fabs(state.biasP[5]);
//    vqf_real_t sum3 = fabs(state.biasP[6]) + fabs(state.biasP[7]) + fabs(state.biasP[8]);
//    vqf_real_t P = vqf_min(vqf_max(vqf_max(sum1, sum2), sum3), coeffs.biasP0);
//    // convert standard deviation from 0.01deg to rad
//    // sqrt can be replaced by arm_sqrt_f32 from CMSIS_DSP
//    return sqrt(P)*(vqf_real_t)(M_PI/100.0/180.0);
//}
//
//void setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma)
//{
//    state.bias[0] = bias[0];
//    state.bias[1] = bias[1];
//    state.bias[2] = bias[2];
//    // std::copy(bias, bias+3, state.bias);
//    if (sigma > 0) {
//        vqf_real_t P = vqf_square(sigma*(vqf_real_t)(180.0*100.0/M_PI));
//        matrix3SetToScaledIdentity(P, state.biasP);
//    }
//}
//
//bool vqf_getRestDetected()
//{
//    return state.restDetected;
//}
//
//bool getMagDistDetected()
//{
//    return state.magDistDetected;
//}
//
//void getRelativeRestDeviations(vqf_real_t out[2])
//{
//    // sqrt can be replaced by arm_sqrt_f32 from CMSIS_DSP
//    out[0] = sqrt(state.restLastSquaredDeviations[0]) / (params.restThGyr*(vqf_real_t)(M_PI/180.0));
//    out[1] = sqrt(state.restLastSquaredDeviations[1]) / params.restThAcc;
//}
//
//vqf_real_t getMagRefNorm()
//{
//    return state.magRefNorm;
//}
//
//vqf_real_t getMagRefDip()
//{
//    return state.magRefDip;
//}
//
//void setMagRef(vqf_real_t norm, vqf_real_t dip)
//{
//    state.magRefNorm = norm;
//    state.magRefDip = dip;
//}
//
//void setMotionBiasEstEnabled(bool enabled)
//{
//    if (params.motionBiasEstEnabled == enabled) {
//        return;
//    }
//    params.motionBiasEstEnabled = enabled;
//    vqf_fill_double(state.motionBiasEstRLpState, 9*2, NAN);
//    // std::fill(state.motionBiasEstRLpState, state.motionBiasEstRLpState + 9*2, NAN);
//    vqf_fill_double(state.motionBiasEstBiasLpState, 2*2, NAN);
//    // std::fill(state.motionBiasEstBiasLpState, state.motionBiasEstBiasLpState + 2*2, NAN);
//}
//
//void setRestBiasEstEnabled(bool enabled)
//{
//    if (params.restBiasEstEnabled == enabled) {
//        return;
//    }
//    params.restBiasEstEnabled = enabled;
//    state.restDetected = false;
//
//    vqf_fill_real(state.restLastSquaredDeviations, 2, 0.0);
//    // std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations + 2, 0.0);
//    state.restT = 0.0;
//    vqf_fill_real(state.restLastGyrLp, 3, 0.0);
//    // std::fill(state.restLastGyrLp, state.restLastGyrLp + 3, 0.0);
//    vqf_fill_double(state.restGyrLpState, 3*2, NAN);
//    // std::fill(state.restGyrLpState, state.restGyrLpState + 3*2, NAN);
//    vqf_fill_real(state.restLastAccLp, 3, 0.0);
//    // std::fill(state.restLastAccLp, state.restLastAccLp + 3, 0.0);
//    vqf_fill_double(state.restAccLpState, 3*2, NAN);
//    // std::fill(state.restAccLpState, state.restAccLpState + 3*2, NAN);
//}
//
//void setMagDistRejectionEnabled(bool enabled)
//{
//    if (params.magDistRejectionEnabled == enabled) {
//        return;
//    }
//    params.magDistRejectionEnabled = enabled;
//    state.magDistDetected = true;
//    state.magRefNorm = 0.0;
//    state.magRefDip = 0.0;
//    state.magUndisturbedT = 0.0;
//    state.magRejectT = params.magMaxRejectionTime;
//    state.magCandidateNorm = -1.0;
//    state.magCandidateDip = 0.0;
//    state.magCandidateT = 0.0;
//    vqf_fill_double(state.magNormDipLpState, 2*2, NAN);
//    // std::fill(state.magNormDipLpState, state.magNormDipLpState + 2*2, NAN);
//}
//
//void setTauAcc(vqf_real_t tauAcc)
//{
//    if (params.tauAcc == tauAcc) {
//        return;
//    }
//    params.tauAcc = tauAcc;
//    vqf_double_t newB[3];
//    vqf_double_t newA[3];
//
//    filterCoeffs(params.tauAcc, coeffs.accTs, newB, newA);
//    filterAdaptStateForCoeffChange(state.lastAccLp, 3, coeffs.accLpB, coeffs.accLpA, newB, newA, state.accLpState);
//
//    // For R and biasLP, the last value is not saved in the state.
//    // Since b0 is small (at reasonable settings), the last output is close to state[0].
//    vqf_real_t R[9];
//    for (size_t i = 0; i < 9; i++) {
//        R[i] = state.motionBiasEstRLpState[2*i];
//    }
//    filterAdaptStateForCoeffChange(R, 9, coeffs.accLpB, coeffs.accLpA, newB, newA, state.motionBiasEstRLpState);
//    vqf_real_t biasLp[2];
//    for (size_t i = 0; i < 2; i++) {
//        biasLp[i] = state.motionBiasEstBiasLpState[2*i];
//    }
//    filterAdaptStateForCoeffChange(biasLp, 2, coeffs.accLpB, coeffs.accLpA, newB, newA, state.motionBiasEstBiasLpState);
//
//    memcpy(coeffs.accLpB, newB, sizeof(newB));
//    // std::copy(newB, newB+3, coeffs.accLpB);
//    memcpy(coeffs.accLpA, newA, sizeof(newA));
//    // std::copy(newA, newA+2, coeffs.accLpA);
//}
//
//void setTauMag(vqf_real_t tauMag)
//{
//    params.tauMag = tauMag;
//    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);
//}
//
//void setRestDetectionThresholds(vqf_real_t thGyr, vqf_real_t thAcc)
//{
//    params.restThGyr = thGyr;
//    params.restThAcc = thAcc;
//}
#endif

static void vqf_resetFilter(){
    vqf_resetState(false);
}

static void vqf_resetState(bool resetBias)
{
    quatSetToIdentity(state.gyrQuat);
    quatSetToIdentity(state.accQuat);
    state.delta = 0.0;

    state.restDetected = false;
    state.magDistDetected = true;

    vqf_fill_real(state.lastAccLp, 3, 0);
    // std::fill(state.lastAccLp, state.lastAccLp+3, 0);
    vqf_fill_double(state.accLpState, 3*2, NAN);
    // std::fill(state.accLpState, state.accLpState + 3*2, NAN);
    state.lastAccCorrAngularRate = 0.0;

    state.kMagInit = 1.0;
    state.lastMagDisAngle = 0.0;
    state.lastMagCorrAngularRate = 0.0;

    if (resetBias) {
        vqf_fill_real(state.bias, 3, 0);
        // std::fill(state.bias, state.bias+3, 0);

        matrix3SetToScaledIdentity(coeffs.biasP0, state.biasP);
    }


    vqf_fill_double(state.motionBiasEstRLpState, 9*2, NAN);
    // std::fill(state.motionBiasEstRLpState, state.motionBiasEstRLpState + 9*2, NAN);
    vqf_fill_double(state.motionBiasEstBiasLpState, 2*2, NAN);
    // std::fill(state.motionBiasEstBiasLpState, state.motionBiasEstBiasLpState + 2*2, NAN);


    vqf_fill_real(state.restLastSquaredDeviations, 2, 0.0);
    // std::fill(state.restLastSquaredDeviations, state.restLastSquaredDeviations + 2, 0.0);
    state.restT = 0.0;
    vqf_fill_real(state.restLastGyrLp, 3, 0.0);
    // std::fill(state.restLastGyrLp, state.restLastGyrLp + 3, 0.0);
    vqf_fill_double(state.restGyrLpState, 3*2, NAN);
    // std::fill(state.restGyrLpState, state.restGyrLpState + 3*2, NAN);
    vqf_fill_real(state.restLastAccLp, 3, 0.0);
    // std::fill(state.restLastAccLp, state.restLastAccLp + 3, 0.0);
    vqf_fill_double(state.restAccLpState, 3*2, NAN);
    // std::fill(state.restAccLpState, state.restAccLpState + 3*2, NAN);

    state.magRefNorm = 0.0;
    state.magRefDip = 0.0;
    state.magUndisturbedT = 0.0;
    state.magRejectT = params.magMaxRejectionTime;
    state.magCandidateNorm = -1.0;
    state.magCandidateDip = 0.0;
    state.magCandidateT = 0.0;
    vqf_fill_real(state.magNormDip, 2, 0);
    // std::fill(state.magNormDip, state.magNormDip + 2, 0);
    vqf_fill_double(state.magNormDipLpState, 2*2, NAN);
    // std::fill(state.magNormDipLpState, state.magNormDipLpState + 2*2, NAN);
}

static void vqf_setup()
{
    // assert(coeffs.gyrTs > 0);
    // assert(coeffs.accTs > 0);
    // assert(coeffs.magTs > 0);

    filterCoeffs(params.tauAcc, coeffs.accTs, coeffs.accLpB, coeffs.accLpA);

    coeffs.kMag = gainFromTau(params.tauMag, coeffs.magTs);

    coeffs.biasP0 = vqf_square(params.biasSigmaInit*100.0);
    // the system noise increases the variance from 0 to (0.1 °/s)^2 in biasForgettingTime seconds
    coeffs.biasV = vqf_square(0.1*100.0)*coeffs.accTs/params.biasForgettingTime;


    vqf_real_t pMotion = vqf_square(params.biasSigmaMotion*100.0);
    coeffs.biasMotionW = vqf_square(pMotion) / coeffs.biasV + pMotion;
    coeffs.biasVerticalW = coeffs.biasMotionW / vqf_max(params.biasVerticalForgettingFactor, (vqf_real_t)(1e-10));


    vqf_real_t pRest = vqf_square(params.biasSigmaRest*100.0);
    coeffs.biasRestW = vqf_square(pRest) / coeffs.biasV + pRest;

    filterCoeffs(params.restFilterTau, coeffs.gyrTs, coeffs.restGyrLpB, coeffs.restGyrLpA);
    filterCoeffs(params.restFilterTau, coeffs.accTs, coeffs.restAccLpB, coeffs.restAccLpA);

    coeffs.kMagRef = gainFromTau(params.magRefTau, coeffs.magTs);
    if (params.magCurrentTau > 0) {
        filterCoeffs(params.magCurrentTau, coeffs.magTs, coeffs.magNormDipLpB, coeffs.magNormDipLpA);
    } else {
        vqf_fill_double(coeffs.magNormDipLpB, 3, NAN);
        // std::fill(coeffs.magNormDipLpB, coeffs.magNormDipLpB + 3, NAN);
        vqf_fill_double(coeffs.magNormDipLpA, 2, NAN);
        // std::fill(coeffs.magNormDipLpA, coeffs.magNormDipLpA + 2, NAN);
    }

    vqf_resetState(true);
}


static void initVqf(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    coeffs.gyrTs = gyrTs;
    coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    coeffs.magTs = magTs > 0 ? magTs : gyrTs;

    vqf_setup();
}
