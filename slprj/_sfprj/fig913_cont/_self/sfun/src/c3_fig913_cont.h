#ifndef __c3_fig913_cont_h__
#define __c3_fig913_cont_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc3_fig913_contInstanceStruct
#define typedef_SFc3_fig913_contInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  real_T c3_lastMajorTime;
  uint8_T c3_is_active_c3_fig913_cont;
  uint8_T c3_is_c3_fig913_cont;
  real_T c3_v;
  real_T c3_dt;
  uint8_T c3_doSetSimStateSideEffects;
  const mxArray *c3_setSimStateSideEffectsInfo;
  void *c3_fEmlrtCtx;
  real_T *c3_x;
  real_T *c3_x_out;
  real_T *c3_y;
  real_T *c3_y_out;
  real_T *c3_d;
  real_T *c3_e;
  real_T *c3_u;
  real_T *c3_dir;
} SFc3_fig913_contInstanceStruct;

#endif                                 /*typedef_SFc3_fig913_contInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_fig913_cont_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_fig913_cont_get_check_sum(mxArray *plhs[]);
extern void c3_fig913_cont_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
