#ifndef LHS_Robot_h_
#define LHS_Robot_h_
#ifndef LHS_Robot_COMMON_INCLUDES_
#define LHS_Robot_COMMON_INCLUDES_
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "sfrtif/sim_shared_utils.h"
#include "sfrtif/sfc_sdi.h"
#endif
#include "LHS_Robot_types.h"
#include <stddef.h>
#include <string.h>
#include "rtGetNaN.h"
#include "model_reference_types.h"
#include "rtw_modelmap.h"
typedef struct { real_T jj2wyov2xd ; real_T dkvequ3xkh ; real_T mynkjmqsxb ;
real_T grzis1hxku ; real_T lvf0ctgkrt ; uint32_T kqbzfectwq ; uint32_T
dsaqz2ojao ; uint32_T d1dwtqjbmf ; boolean_T byeq1yucbz ; } leld2fnqig ;
typedef struct { real_T agpivbt0hr ; real_T ic4s3kvc0r ; real_T cukzdhyjcg ;
real_T gxb5qmm2pn ; real_T jk2kl2xhqx ; struct { void * AQHandles ; }
blzyu3a0rs ; struct { void * AQHandles ; } i30rc1bker ; uint32_T nbb3xoep3f [
4 ] ; uint32_T ffs1dxran1 [ 4 ] ; uint32_T prnq0s3lqy ; uint32_T gfnjintt4q ;
uint32_T kk4m0uwudl ; uint32_T jeihvtzxco ; uint32_T btploezjrc ; boolean_T
iopribspkq ; boolean_T noqne5tmfl ; boolean_T msm5msyt5h ; } bhduwxqtz1 ;
typedef struct { real_T p2ggtkfarh ; real_T gcd5lf35va ; } k4a404kthw ;
typedef struct { real_T p2ggtkfarh ; real_T gcd5lf35va ; } jd2odbvxqe ;
typedef struct { boolean_T p2ggtkfarh ; boolean_T gcd5lf35va ; } ae5m15ljke ;
struct audy4wot3le_ { real_T P_5 ; real_T P_6 ; real_T P_7 ; real_T P_8 ;
real_T P_9 ; real_T P_10 ; real_T P_11 ; real_T P_12 ; real_T P_13 ; real_T
P_14 ; real_T P_15 ; real_T P_16 ; real_T P_17 ; real_T P_18 ; real_T P_19 ;
real_T P_20 ; real_T P_21 ; real_T P_22 ; real_T P_23 ; int8_T P_24 ; int8_T
P_25 ; int8_T P_26 ; int8_T P_27 ; } ; struct dykrpvl4hp { struct
SimStruct_tag * _mdlRefSfcnS ; struct { real_T mr_nonContSig0 [ 1 ] ; }
NonContDerivMemory ; ssNonContDerivSigInfo nonContDerivSignal [ 1 ] ; const
rtTimingBridge * timingBridge ; struct { rtwCAPI_ModelMappingInfo mmi ;
rtwCAPI_ModelMapLoggingInstanceInfo mmiLogInstanceInfo ; void * dataAddress [
4 ] ; int32_T * vardimsAddress [ 4 ] ; RTWLoggingFcnPtr loggingPtrs [ 4 ] ;
sysRanDType * systemRan [ 4 ] ; int_T systemTid [ 4 ] ; } DataMapInfo ;
struct { int_T mdlref_GlobalTID [ 4 ] ; time_T tStart ; } Timing ; } ;
typedef struct { leld2fnqig rtb ; bhduwxqtz1 rtdw ; kizrp3vx0c rtm ; }
d1b0notwieb ; extern real_T rtP_E_max ; extern real_T rtP_Ki ; extern real_T
rtP_Kp ; extern real_T rtP_c_L ; extern real_T rtP_m_L ; extern void
bdyao54c3n ( SimStruct * _mdlRefSfcnS , int_T mdlref_TID0 , int_T mdlref_TID1
, int_T mdlref_TID2 , int_T mdlref_TID3 , kizrp3vx0c * const gi5soirers ,
leld2fnqig * localB , bhduwxqtz1 * localDW , k4a404kthw * localX , void *
sysRanPtr , int_T contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI ,
const char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ;
extern void mr_LHS_Robot_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) ; extern mxArray * mr_LHS_Robot_GetDWork ( const
d1b0notwieb * mdlrefDW ) ; extern void mr_LHS_Robot_SetDWork ( d1b0notwieb *
mdlrefDW , const mxArray * ssDW ) ; extern void
mr_LHS_Robot_RegisterSimStateChecksum ( SimStruct * S ) ; extern mxArray *
mr_LHS_Robot_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * LHS_Robot_GetCAPIStaticMap ( void ) ; extern
void mlqxpobxpc ( kizrp3vx0c * const gi5soirers , bhduwxqtz1 * localDW ) ;
extern void g4z4h50w20 ( bhduwxqtz1 * localDW , k4a404kthw * localX ) ;
extern void hqewqjevpv ( bhduwxqtz1 * localDW , k4a404kthw * localX ) ;
extern void jx5xxtsvgz ( kizrp3vx0c * const gi5soirers , bhduwxqtz1 * localDW
) ; extern void lhnw1a4qd5 ( real_T * cdpwyev5qkw , leld2fnqig * localB ,
k4a404kthw * localX , jd2odbvxqe * localXdot ) ; extern void eavqqyvzgg ( kizrp3vx0c * const gi5soirers , leld2fnqig * localB , bhduwxqtz1 * localDW ) ; extern void LHS_Robot ( kizrp3vx0c * const gi5soirers , const real_T * cdpwyev5qk , real_T * cdpwyev5qkw , real_T * hlksnyzmq0 , real_T * kgenb0btne , leld2fnqig * localB , bhduwxqtz1 * localDW , k4a404kthw * localX ) ; extern void hqag25h24y ( kizrp3vx0c * const gi5soirers ) ;
#endif
