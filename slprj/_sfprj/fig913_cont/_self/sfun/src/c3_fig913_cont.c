/* Include files */

#include "fig913_cont_sfun.h"
#include "c3_fig913_cont.h"
#include "mwmathutil.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c3_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c3_IN_Left                     ((uint8_T)1U)
#define c3_IN_Right                    ((uint8_T)2U)
#define c3_IN_Stop                     ((uint8_T)3U)
#define c3_IN_Straight                 ((uint8_T)4U)
#define c3_const_v                     (35.0)
#define c3_const_dt                    (0.01)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;

/* Function Declarations */
static void initialize_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void initialize_params_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void enable_c3_fig913_cont(SFc3_fig913_contInstanceStruct *chartInstance);
static void disable_c3_fig913_cont(SFc3_fig913_contInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c3_fig913_cont
  (SFc3_fig913_contInstanceStruct *chartInstance);
static void set_sim_state_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_st);
static void c3_set_sim_state_side_effects_c3_fig913_cont
  (SFc3_fig913_contInstanceStruct *chartInstance);
static void finalize_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void sf_gateway_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void mdl_start_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void zeroCrossings_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void derivatives_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static void outputs_c3_fig913_cont(SFc3_fig913_contInstanceStruct *chartInstance);
static void initSimStructsc3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance);
static real_T c3_mrdivide(SFc3_fig913_contInstanceStruct *chartInstance, real_T
  c3_A, real_T c3_B);
static real_T c3_rdivide(SFc3_fig913_contInstanceStruct *chartInstance, real_T
  c3_b_x, real_T c3_b_y);
static real_T c3_div(SFc3_fig913_contInstanceStruct *chartInstance, real_T
                     c3_b_x, real_T c3_b_y);
static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber);
static const mxArray *c3_emlrt_marshallOut(SFc3_fig913_contInstanceStruct
  *chartInstance);
static const mxArray *c3_b_emlrt_marshallOut(SFc3_fig913_contInstanceStruct
  *chartInstance, const real_T c3_b_u);
static const mxArray *c3_c_emlrt_marshallOut(SFc3_fig913_contInstanceStruct
  *chartInstance, const uint8_T c3_b_u);
static void c3_emlrt_marshallIn(SFc3_fig913_contInstanceStruct *chartInstance,
  const mxArray *c3_b_u);
static real_T c3_b_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_dir, const char_T *c3_identifier);
static real_T c3_c_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_u, const emlrtMsgIdentifier *c3_parentId);
static uint8_T c3_d_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_fig913_cont, const char_T
  *c3_identifier);
static uint8_T c3_e_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_u, const emlrtMsgIdentifier *c3_parentId);
static const mxArray *c3_f_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_setSimStateSideEffectsInfo, const char_T
  *c3_identifier);
static const mxArray *c3_g_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_cos(SFc3_fig913_contInstanceStruct *chartInstance, real_T *c3_b_x);
static void c3_applyScalarFunctionInPlace(SFc3_fig913_contInstanceStruct
  *chartInstance, real_T *c3_b_x);
static void c3_b_cos(SFc3_fig913_contInstanceStruct *chartInstance, real_T
                     *c3_b_x);
static void c3_sin(SFc3_fig913_contInstanceStruct *chartInstance, real_T *c3_b_x);
static void c3_b_applyScalarFunctionInPlace(SFc3_fig913_contInstanceStruct
  *chartInstance, real_T *c3_b_x);
static void c3_b_sin(SFc3_fig913_contInstanceStruct *chartInstance, real_T
                     *c3_b_x);
static void init_dsm_address_info(SFc3_fig913_contInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc3_fig913_contInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_doSetSimStateSideEffects = 0U;
  chartInstance->c3_setSimStateSideEffectsInfo = NULL;
  chartInstance->c3_is_active_c3_fig913_cont = 0U;
  chartInstance->c3_is_c3_fig913_cont = c3_IN_NO_ACTIVE_CHILD;
  *chartInstance->c3_dir = 1.2566370614359172;
  chartInstance->c3_v = 35.0;
  chartInstance->c3_dt = 0.01;
  *chartInstance->c3_x = -30.0;
  *chartInstance->c3_y = 35.0;
}

static void initialize_params_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c3_fig913_cont(SFc3_fig913_contInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_fig913_cont(SFc3_fig913_contInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static const mxArray *get_sim_state_c3_fig913_cont
  (SFc3_fig913_contInstanceStruct *chartInstance)
{
  const mxArray *c3_st = NULL;
  c3_st = NULL;
  sf_mex_assign(&c3_st, c3_emlrt_marshallOut(chartInstance), false);
  return c3_st;
}

static void set_sim_state_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  c3_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_st));
  chartInstance->c3_doSetSimStateSideEffects = 1U;
  sf_mex_destroy(&c3_st);
}

static void c3_set_sim_state_side_effects_c3_fig913_cont
  (SFc3_fig913_contInstanceStruct *chartInstance)
{
  if (chartInstance->c3_doSetSimStateSideEffects != 0) {
    chartInstance->c3_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  sf_mex_destroy(&chartInstance->c3_setSimStateSideEffectsInfo);
}

static void sf_gateway_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  boolean_T c3_stateChanged;
  c3_set_sim_state_side_effects_c3_fig913_cont(chartInstance);
  _sfTime_ = sf_get_time(chartInstance->S);
  if (ssIsMajorTimeStep(chartInstance->S) != 0U) {
    chartInstance->c3_lastMajorTime = _sfTime_;
    c3_stateChanged = (boolean_T)0;
    if (chartInstance->c3_is_active_c3_fig913_cont == 0U) {
      chartInstance->c3_is_active_c3_fig913_cont = 1U;
      c3_stateChanged = true;
      chartInstance->c3_is_c3_fig913_cont = c3_IN_Stop;
    } else {
      switch (chartInstance->c3_is_c3_fig913_cont) {
       case c3_IN_Left:
        if (*chartInstance->c3_u == 0.0) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Stop;
        } else if (*chartInstance->c3_d >= -*chartInstance->c3_e) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Straight;
        } else {
          if (*chartInstance->c3_d <= -*chartInstance->c3_e) {
            c3_stateChanged = true;
            chartInstance->c3_is_c3_fig913_cont = c3_IN_Left;
          }
        }
        break;

       case c3_IN_Right:
        if (*chartInstance->c3_u == 0.0) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Stop;
        } else if (*chartInstance->c3_d <= *chartInstance->c3_e) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Straight;
        } else {
          if (*chartInstance->c3_d >= *chartInstance->c3_e) {
            c3_stateChanged = true;
            chartInstance->c3_is_c3_fig913_cont = c3_IN_Right;
          }
        }
        break;

       case c3_IN_Stop:
        if ((*chartInstance->c3_u == 1.0) && (*chartInstance->c3_d >=
             *chartInstance->c3_e)) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Right;
        } else if ((*chartInstance->c3_u == 1.0) && ((real_T)
                    (-*chartInstance->c3_e <= *chartInstance->c3_d) <=
                    *chartInstance->c3_e)) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Straight;
        } else {
          if ((*chartInstance->c3_u == 1.0) && (*chartInstance->c3_d <=
               -*chartInstance->c3_e)) {
            c3_stateChanged = true;
            chartInstance->c3_is_c3_fig913_cont = c3_IN_Left;
          }
        }
        break;

       case c3_IN_Straight:
        if (*chartInstance->c3_u == 0.0) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Stop;
        } else if (*chartInstance->c3_d >= *chartInstance->c3_e) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Right;
        } else if (*chartInstance->c3_d <= -*chartInstance->c3_e) {
          c3_stateChanged = true;
          chartInstance->c3_is_c3_fig913_cont = c3_IN_Left;
        } else {
          if ((real_T)(-*chartInstance->c3_e <= *chartInstance->c3_d) <=
              *chartInstance->c3_e) {
            c3_stateChanged = true;
            chartInstance->c3_is_c3_fig913_cont = c3_IN_Straight;
          }
        }
        break;

       default:
        /* Unreachable state, for coverage only */
        chartInstance->c3_is_c3_fig913_cont = c3_IN_NO_ACTIVE_CHILD;
        break;
      }
    }

    if (c3_stateChanged) {
      ssSetSolverNeedsReset(chartInstance->S);
    }
  }

  _sfTime_ = sf_get_time(chartInstance->S);
  switch (chartInstance->c3_is_c3_fig913_cont) {
   case c3_IN_Left:
   case c3_IN_Right:
   case c3_IN_Stop:
   case c3_IN_Straight:
    break;

   default:
    /* Unreachable state, for coverage only */
    chartInstance->c3_is_c3_fig913_cont = c3_IN_NO_ACTIVE_CHILD;
    break;
  }

  *chartInstance->c3_x_out = *chartInstance->c3_x;
  *chartInstance->c3_y_out = *chartInstance->c3_y;
}

static void mdl_start_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void zeroCrossings_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  boolean_T c3_stateChanged;
  real_T *c3_zcVar;
  c3_zcVar = (real_T *)(ssGetNonsampledZCs_wrapper(chartInstance->S) + 0);
  _sfTime_ = sf_get_time(chartInstance->S);
  if (chartInstance->c3_lastMajorTime == _sfTime_) {
    *c3_zcVar = -1.0;
  } else {
    c3_stateChanged = (boolean_T)0;
    if (chartInstance->c3_is_active_c3_fig913_cont == 0U) {
      c3_stateChanged = true;
    } else {
      switch (chartInstance->c3_is_c3_fig913_cont) {
       case c3_IN_Left:
        if (*chartInstance->c3_u == 0.0) {
          c3_stateChanged = true;
        } else if (*chartInstance->c3_d >= -*chartInstance->c3_e) {
          c3_stateChanged = true;
        } else {
          if (*chartInstance->c3_d <= -*chartInstance->c3_e) {
            c3_stateChanged = true;
          }
        }
        break;

       case c3_IN_Right:
        if (*chartInstance->c3_u == 0.0) {
          c3_stateChanged = true;
        } else if (*chartInstance->c3_d <= *chartInstance->c3_e) {
          c3_stateChanged = true;
        } else {
          if (*chartInstance->c3_d >= *chartInstance->c3_e) {
            c3_stateChanged = true;
          }
        }
        break;

       case c3_IN_Stop:
        if ((*chartInstance->c3_u == 1.0) && (*chartInstance->c3_d >=
             *chartInstance->c3_e)) {
          c3_stateChanged = true;
        } else if ((*chartInstance->c3_u == 1.0) && ((real_T)
                    (-*chartInstance->c3_e <= *chartInstance->c3_d) <=
                    *chartInstance->c3_e)) {
          c3_stateChanged = true;
        } else {
          if ((*chartInstance->c3_u == 1.0) && (*chartInstance->c3_d <=
               -*chartInstance->c3_e)) {
            c3_stateChanged = true;
          }
        }
        break;

       case c3_IN_Straight:
        if (*chartInstance->c3_u == 0.0) {
          c3_stateChanged = true;
        } else if (*chartInstance->c3_d >= *chartInstance->c3_e) {
          c3_stateChanged = true;
        } else if (*chartInstance->c3_d <= -*chartInstance->c3_e) {
          c3_stateChanged = true;
        } else {
          if ((real_T)(-*chartInstance->c3_e <= *chartInstance->c3_d) <=
              *chartInstance->c3_e) {
            c3_stateChanged = true;
          }
        }
        break;

       default:
        /* Unreachable state, for coverage only */
        chartInstance->c3_is_c3_fig913_cont = c3_IN_NO_ACTIVE_CHILD;
        break;
      }
    }

    if (c3_stateChanged) {
      *c3_zcVar = 1.0;
    } else {
      *c3_zcVar = -1.0;
    }
  }
}

static void derivatives_c3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  real_T c3_d0;
  real_T *c3_x_dot;
  real_T *c3_y_dot;
  real_T *c3_dir_dot;
  c3_dir_dot = (real_T *)(ssGetdX_wrapper(chartInstance->S) + 2);
  c3_y_dot = (real_T *)(ssGetdX_wrapper(chartInstance->S) + 1);
  c3_x_dot = (real_T *)(ssGetdX_wrapper(chartInstance->S) + 0);
  *c3_x_dot = 0.0;
  *c3_y_dot = 0.0;
  *c3_dir_dot = 0.0;
  _sfTime_ = sf_get_time(chartInstance->S);
  switch (chartInstance->c3_is_c3_fig913_cont) {
   case c3_IN_Left:
    c3_d0 = *chartInstance->c3_dir;
    c3_cos(chartInstance, &c3_d0);
    *c3_x_dot = c3_mrdivide(chartInstance, c3_const_v * c3_d0, 2.0);
    c3_d0 = *chartInstance->c3_dir;
    c3_sin(chartInstance, &c3_d0);
    *c3_y_dot = c3_mrdivide(chartInstance, c3_const_v * c3_d0, 2.0);
    *c3_dir_dot = 3.1415926535897931;
    break;

   case c3_IN_Right:
    c3_d0 = *chartInstance->c3_dir;
    c3_cos(chartInstance, &c3_d0);
    *c3_x_dot = c3_mrdivide(chartInstance, c3_const_v * c3_d0, 2.0);
    c3_d0 = *chartInstance->c3_dir;
    c3_sin(chartInstance, &c3_d0);
    *c3_y_dot = c3_mrdivide(chartInstance, c3_const_v * c3_d0, 2.0);
    *c3_dir_dot = -3.1415926535897931;
    break;

   case c3_IN_Stop:
    *c3_x_dot = 0.0;
    *c3_y_dot = 0.0;
    *c3_dir_dot = 0.0;
    break;

   case c3_IN_Straight:
    c3_d0 = *chartInstance->c3_dir;
    c3_cos(chartInstance, &c3_d0);
    *c3_x_dot = c3_const_v * c3_d0;
    c3_d0 = *chartInstance->c3_dir;
    c3_sin(chartInstance, &c3_d0);
    *c3_y_dot = c3_const_v * c3_d0;
    *c3_dir_dot = 0.0;
    break;

   default:
    /* Unreachable state, for coverage only */
    chartInstance->c3_is_c3_fig913_cont = c3_IN_NO_ACTIVE_CHILD;
    break;
  }
}

static void outputs_c3_fig913_cont(SFc3_fig913_contInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
  switch (chartInstance->c3_is_c3_fig913_cont) {
   case c3_IN_Left:
   case c3_IN_Right:
   case c3_IN_Stop:
   case c3_IN_Straight:
    break;

   default:
    /* Unreachable state, for coverage only */
    chartInstance->c3_is_c3_fig913_cont = c3_IN_NO_ACTIVE_CHILD;
    break;
  }

  *chartInstance->c3_x_out = *chartInstance->c3_x;
  *chartInstance->c3_y_out = *chartInstance->c3_y;
}

static void initSimStructsc3_fig913_cont(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static real_T c3_mrdivide(SFc3_fig913_contInstanceStruct *chartInstance, real_T
  c3_A, real_T c3_B)
{
  return c3_rdivide(chartInstance, c3_A, c3_B);
}

static real_T c3_rdivide(SFc3_fig913_contInstanceStruct *chartInstance, real_T
  c3_b_x, real_T c3_b_y)
{
  return c3_div(chartInstance, c3_b_x, c3_b_y);
}

static real_T c3_div(SFc3_fig913_contInstanceStruct *chartInstance, real_T
                     c3_b_x, real_T c3_b_y)
{
  (void)chartInstance;
  return c3_b_x / c3_b_y;
}

static void init_script_number_translation(uint32_T c3_machineNumber, uint32_T
  c3_chartNumber, uint32_T c3_instanceNumber)
{
  (void)(c3_machineNumber);
  (void)(c3_chartNumber);
  (void)(c3_instanceNumber);
}

const mxArray *sf_c3_fig913_cont_get_eml_resolved_functions_info(void)
{
  const mxArray *c3_nameCaptureInfo = NULL;
  c3_nameCaptureInfo = NULL;
  sf_mex_assign(&c3_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c3_nameCaptureInfo;
}

static const mxArray *c3_emlrt_marshallOut(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  const mxArray *c3_b_y;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_createcellmatrix(5, 1), false);
  sf_mex_setcell(c3_b_y, 0, c3_b_emlrt_marshallOut(chartInstance,
    *chartInstance->c3_dir));
  sf_mex_setcell(c3_b_y, 1, c3_b_emlrt_marshallOut(chartInstance,
    *chartInstance->c3_x));
  sf_mex_setcell(c3_b_y, 2, c3_b_emlrt_marshallOut(chartInstance,
    *chartInstance->c3_y));
  sf_mex_setcell(c3_b_y, 3, c3_c_emlrt_marshallOut(chartInstance,
    chartInstance->c3_is_active_c3_fig913_cont));
  sf_mex_setcell(c3_b_y, 4, c3_c_emlrt_marshallOut(chartInstance,
    chartInstance->c3_is_c3_fig913_cont));
  return c3_b_y;
}

static const mxArray *c3_b_emlrt_marshallOut(SFc3_fig913_contInstanceStruct
  *chartInstance, const real_T c3_b_u)
{
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 0, 0U, 0U, 0U, 0), false);
  return c3_b_y;
}

static const mxArray *c3_c_emlrt_marshallOut(SFc3_fig913_contInstanceStruct
  *chartInstance, const uint8_T c3_b_u)
{
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", &c3_b_u, 3, 0U, 0U, 0U, 0), false);
  return c3_b_y;
}

static void c3_emlrt_marshallIn(SFc3_fig913_contInstanceStruct *chartInstance,
  const mxArray *c3_b_u)
{
  *chartInstance->c3_dir = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("dir", c3_b_u, 0)), "dir");
  *chartInstance->c3_x = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("x", c3_b_u, 1)), "x");
  *chartInstance->c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("y", c3_b_u, 2)), "y");
  chartInstance->c3_is_active_c3_fig913_cont = c3_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c3_fig913_cont", c3_b_u,
       3)), "is_active_c3_fig913_cont");
  chartInstance->c3_is_c3_fig913_cont = c3_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_c3_fig913_cont", c3_b_u, 4)),
    "is_c3_fig913_cont");
  sf_mex_assign(&chartInstance->c3_setSimStateSideEffectsInfo,
                c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(
    "setSimStateSideEffectsInfo", c3_b_u, 5)), "setSimStateSideEffectsInfo"),
                true);
  sf_mex_destroy(&c3_b_u);
}

static real_T c3_b_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_dir, const char_T *c3_identifier)
{
  real_T c3_b_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = (const char *)c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_b_y = c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_dir), &c3_thisId);
  sf_mex_destroy(&c3_b_dir);
  return c3_b_y;
}

static real_T c3_c_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_u, const emlrtMsgIdentifier *c3_parentId)
{
  real_T c3_b_y;
  real_T c3_d1;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_b_u), &c3_d1, 1, 0, 0U, 0, 0U, 0);
  c3_b_y = c3_d1;
  sf_mex_destroy(&c3_b_u);
  return c3_b_y;
}

static uint8_T c3_d_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_fig913_cont, const char_T
  *c3_identifier)
{
  uint8_T c3_b_y;
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = (const char *)c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_b_y = c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_fig913_cont), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_fig913_cont);
  return c3_b_y;
}

static uint8_T c3_e_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_b_y;
  uint8_T c3_u0;
  (void)chartInstance;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_b_u), &c3_u0, 1, 3, 0U, 0, 0U, 0);
  c3_b_y = c3_u0;
  sf_mex_destroy(&c3_b_u);
  return c3_b_y;
}

static const mxArray *c3_f_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_setSimStateSideEffectsInfo, const char_T
  *c3_identifier)
{
  const mxArray *c3_b_y = NULL;
  emlrtMsgIdentifier c3_thisId;
  c3_b_y = NULL;
  c3_thisId.fIdentifier = (const char *)c3_identifier;
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  sf_mex_assign(&c3_b_y, c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_setSimStateSideEffectsInfo), &c3_thisId), false);
  sf_mex_destroy(&c3_b_setSimStateSideEffectsInfo);
  return c3_b_y;
}

static const mxArray *c3_g_emlrt_marshallIn(SFc3_fig913_contInstanceStruct
  *chartInstance, const mxArray *c3_b_u, const emlrtMsgIdentifier *c3_parentId)
{
  const mxArray *c3_b_y = NULL;
  (void)chartInstance;
  (void)c3_parentId;
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_duplicatearraysafe(&c3_b_u), false);
  sf_mex_destroy(&c3_b_u);
  return c3_b_y;
}

static void c3_cos(SFc3_fig913_contInstanceStruct *chartInstance, real_T *c3_b_x)
{
  c3_applyScalarFunctionInPlace(chartInstance, c3_b_x);
}

static void c3_applyScalarFunctionInPlace(SFc3_fig913_contInstanceStruct
  *chartInstance, real_T *c3_b_x)
{
  c3_b_cos(chartInstance, c3_b_x);
}

static void c3_b_cos(SFc3_fig913_contInstanceStruct *chartInstance, real_T
                     *c3_b_x)
{
  (void)chartInstance;
  *c3_b_x = muDoubleScalarCos(*c3_b_x);
}

static void c3_sin(SFc3_fig913_contInstanceStruct *chartInstance, real_T *c3_b_x)
{
  c3_b_applyScalarFunctionInPlace(chartInstance, c3_b_x);
}

static void c3_b_applyScalarFunctionInPlace(SFc3_fig913_contInstanceStruct
  *chartInstance, real_T *c3_b_x)
{
  c3_b_sin(chartInstance, c3_b_x);
}

static void c3_b_sin(SFc3_fig913_contInstanceStruct *chartInstance, real_T
                     *c3_b_x)
{
  (void)chartInstance;
  *c3_b_x = muDoubleScalarSin(*c3_b_x);
}

static void init_dsm_address_info(SFc3_fig913_contInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc3_fig913_contInstanceStruct
  *chartInstance)
{
  chartInstance->c3_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c3_x = (real_T *)(ssGetContStates_wrapper(chartInstance->S) + 0);
  chartInstance->c3_x_out = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c3_y = (real_T *)(ssGetContStates_wrapper(chartInstance->S) + 1);
  chartInstance->c3_y_out = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c3_d = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c3_e = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c3_u = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c3_dir = (real_T *)(ssGetContStates_wrapper(chartInstance->S) +
    2);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c3_fig913_cont_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3415009521U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1164116433U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2815298379U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3403521306U);
}

mxArray* sf_c3_fig913_cont_get_post_codegen_info(void);
mxArray *sf_c3_fig913_cont_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("YEX5HsujB6zWUBWLip3EsE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxData);
  }

  {
    mxArray* mxPostCodegenInfo = sf_c3_fig913_cont_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c3_fig913_cont_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_fig913_cont_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("early");
  mxArray *fallbackReason = mxCreateString("plant_model_chart");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c3_fig913_cont_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c3_fig913_cont_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c3_fig913_cont(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[5],M[38],T\"dir\",},{M[5],M[7],T\"x\",},{M[5],M[8],T\"y\",},{M[8],M[0],T\"is_active_c3_fig913_cont\",},{M[9],M[0],T\"is_c3_fig913_cont\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_fig913_cont_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sZc5nnS7pfXx9HuSKIhsCQB";
}

static void sf_opaque_initialize_c3_fig913_cont(void *chartInstanceVar)
{
  initialize_params_c3_fig913_cont((SFc3_fig913_contInstanceStruct*)
    chartInstanceVar);
  initialize_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c3_fig913_cont(void *chartInstanceVar)
{
  enable_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_fig913_cont(void *chartInstanceVar)
{
  disable_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_zeroCrossings_c3_fig913_cont(void *chartInstanceVar)
{
  zeroCrossings_c3_fig913_cont((SFc3_fig913_contInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_derivatives_c3_fig913_cont(void *chartInstanceVar)
{
  derivatives_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_outputs_c3_fig913_cont(void *chartInstanceVar)
{
  outputs_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_fig913_cont(void *chartInstanceVar)
{
  sf_gateway_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c3_fig913_cont(SimStruct* S)
{
  return get_sim_state_c3_fig913_cont((SFc3_fig913_contInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c3_fig913_cont(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c3_fig913_cont((SFc3_fig913_contInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c3_fig913_cont(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_fig913_contInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_fig913_cont_optimization_info();
    }

    finalize_c3_fig913_cont((SFc3_fig913_contInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_fig913_cont((SFc3_fig913_contInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_fig913_cont(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_fig913_cont((SFc3_fig913_contInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c3_fig913_cont(SimStruct *S)
{
  ssSetStatesModifiedOnlyInUpdate(S, 0);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_fig913_cont_optimization_info(sim_mode_is_rtw_gen
      (S), sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,3);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,3,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 3);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,3);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,3,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,3,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,3);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1365967913U));
  ssSetChecksum1(S,(2348319367U));
  ssSetChecksum2(S,(1106476761U));
  ssSetChecksum3(S,(2416918916U));
  ssSetNumContStates(S,3);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c3_fig913_cont(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c3_fig913_cont(SimStruct *S)
{
  SFc3_fig913_contInstanceStruct *chartInstance;
  chartInstance = (SFc3_fig913_contInstanceStruct *)utMalloc(sizeof
    (SFc3_fig913_contInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc3_fig913_contInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c3_fig913_cont;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c3_fig913_cont;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c3_fig913_cont;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_fig913_cont;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_fig913_cont;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c3_fig913_cont;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c3_fig913_cont;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_fig913_cont;
  chartInstance->chartInfo.zeroCrossings =
    sf_opaque_zeroCrossings_c3_fig913_cont;
  chartInstance->chartInfo.outputs = sf_opaque_outputs_c3_fig913_cont;
  chartInstance->chartInfo.derivatives = sf_opaque_derivatives_c3_fig913_cont;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_fig913_cont;
  chartInstance->chartInfo.mdlStart = mdlStart_c3_fig913_cont;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_fig913_cont;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_start_c3_fig913_cont(chartInstance);
}

void c3_fig913_cont_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c3_fig913_cont(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_fig913_cont(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_fig913_cont(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_fig913_cont_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
