#ifndef rt_VTDelayfindtDInterpolate_h_
#define rt_VTDelayfindtDInterpolate_h_
#include "rtwtypes.h"
#include "multiword_types.h"

real_T rt_VTDelayfindtDInterpolate(
  real_T x,real_T* uBuf,int_T bufSz,int_T head,int_T tail,int_T* pLast,real_T t,
  real_T tStart,boolean_T discrete,boolean_T minorStepAndTAtLastMajorOutput,
  real_T initOutput,real_T* appliedDelay);

#endif
