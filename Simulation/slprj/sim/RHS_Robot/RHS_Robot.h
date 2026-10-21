#ifndef RHS_Robot_h_
#define RHS_Robot_h_
#ifndef RHS_Robot_COMMON_INCLUDES_
#define RHS_Robot_COMMON_INCLUDES_
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
#include "RHS_Robot_types.h"
#include <stddef.h>
#include <string.h>
#include "rtGetNaN.h"
#include "model_reference_types.h"
#include "rtw_modelmap.h"
typedef struct { real_T c0jccsulew ; real_T ov1cdcuct5 ; real_T hvcyjeza13 ;
real_T imhnrjvb5m ; real_T pumjeov5x4 ; uint32_T hd3hazoav3 ; uint32_T
hfhsa2gech ; uint32_T mbpxij3efb ; boolean_T k0r5wxm4fm ; } cjq1gwrxkz ;
typedef struct { real_T onjeckwmao ; real_T jzvmx3qjzz ; real_T corra0dc3o ;
real_T ifvfgbjylw ; real_T dlfg2iqplp ; struct { void * AQHandles ; }
bhjlwssmgd ; struct { void * AQHandles ; } dourqi4cm5 ; uint32_T lv5djcekmn [
4 ] ; uint32_T acqhs0xjmf [ 4 ] ; uint32_T g0jtbcc43w ; uint32_T kq0lwermq5 ;
uint32_T atdh2lza40 ; uint32_T imiczw551d ; uint32_T kubw0tzzkg ; boolean_T
iif0u5seic ; boolean_T olynyirzn2 ; boolean_T gamhlg3p0o ; } pzcxryh4y3 ;
typedef struct { real_T gkm05mq2uf ; real_T li4zimgc0e ; } kcqz4a1hsb ;
typedef struct { real_T gkm05mq2uf ; real_T li4zimgc0e ; } bzogrrroc4 ;
typedef struct { boolean_T gkm05mq2uf ; boolean_T li4zimgc0e ; } ovrrmmcoo5 ;
struct hhityjduzsf_ { real_T P_5 ; real_T P_6 ; real_T P_7 ; real_T P_8 ;
real_T P_9 ; real_T P_10 ; real_T P_11 ; real_T P_12 ; real_T P_13 ; real_T
P_14 ; real_T P_15 ; real_T P_16 ; real_T P_17 ; real_T P_18 ; real_T P_19 ;
real_T P_20 ; real_T P_21 ; real_T P_22 ; real_T P_23 ; int8_T P_24 ; int8_T
P_25 ; int8_T P_26 ; int8_T P_27 ; } ; struct jgaffmfrln { struct
SimStruct_tag * _mdlRefSfcnS ; struct { real_T mr_nonContSig0 [ 1 ] ; }
NonContDerivMemory ; ssNonContDerivSigInfo nonContDerivSignal [ 1 ] ; const
rtTimingBridge * timingBridge ; struct { rtwCAPI_ModelMappingInfo mmi ;
rtwCAPI_ModelMapLoggingInstanceInfo mmiLogInstanceInfo ; void * dataAddress [
4 ] ; int32_T * vardimsAddress [ 4 ] ; RTWLoggingFcnPtr loggingPtrs [ 4 ] ;
sysRanDType * systemRan [ 4 ] ; int_T systemTid [ 4 ] ; } DataMapInfo ;
struct { int_T mdlref_GlobalTID [ 4 ] ; time_T tStart ; } Timing ; } ;
typedef struct { cjq1gwrxkz rtb ; pzcxryh4y3 rtdw ; lqonxzrhbw rtm ; }
beudxm5cnjh ; extern real_T rtP_E_max ; extern real_T rtP_Ki ; extern real_T
rtP_Kp ; extern real_T rtP_c_R ; extern real_T rtP_m_R ; extern void
mxa1lwsr3v ( SimStruct * _mdlRefSfcnS , int_T mdlref_TID0 , int_T mdlref_TID1
, int_T mdlref_TID2 , int_T mdlref_TID3 , lqonxzrhbw * const mcfta5e4au ,
cjq1gwrxkz * localB , pzcxryh4y3 * localDW , kcqz4a1hsb * localX , void *
sysRanPtr , int_T contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI ,
const char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ;
extern void mr_RHS_Robot_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) ; extern mxArray * mr_RHS_Robot_GetDWork ( const
beudxm5cnjh * mdlrefDW ) ; extern void mr_RHS_Robot_SetDWork ( beudxm5cnjh *
mdlrefDW , const mxArray * ssDW ) ; extern void
mr_RHS_Robot_RegisterSimStateChecksum ( SimStruct * S ) ; extern mxArray *
mr_RHS_Robot_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * RHS_Robot_GetCAPIStaticMap ( void ) ; extern
void itiuuk0lsp ( lqonxzrhbw * const mcfta5e4au , pzcxryh4y3 * localDW ) ;
extern void hkwwlx5qkq ( pzcxryh4y3 * localDW , kcqz4a1hsb * localX ) ;
extern void l4vpeawoem ( pzcxryh4y3 * localDW , kcqz4a1hsb * localX ) ;
extern void n2liu3fiso ( lqonxzrhbw * const mcfta5e4au , pzcxryh4y3 * localDW
) ; extern void mpqdzjte1t ( real_T * bg5sfuocyj , cjq1gwrxkz * localB ,
kcqz4a1hsb * localX , bzogrrroc4 * localXdot ) ; extern void fqhmpvlpgz ( lqonxzrhbw * const mcfta5e4au , cjq1gwrxkz * localB , pzcxryh4y3 * localDW ) ; extern void RHS_Robot ( lqonxzrhbw * const mcfta5e4au , const real_T * k1ynov3occ , real_T * bg5sfuocyj , real_T * ihqcing44o , real_T * hnh3no20sd , cjq1gwrxkz * localB , pzcxryh4y3 * localDW , kcqz4a1hsb * localX ) ; extern void mkmhbwfpnh ( lqonxzrhbw * const mcfta5e4au ) ;
#endif
