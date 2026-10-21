#ifndef Robot_h_
#define Robot_h_
#ifndef Robot_COMMON_INCLUDES_
#define Robot_COMMON_INCLUDES_
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#include "sfrtif/sim_shared_utils.h"
#include "sfrtif/sfc_sdi.h"
#endif
#include "Robot_types.h"
#include "RHS_Robot.h"
#include "LHS_Robot.h"
#include <stddef.h>
#include <string.h>
#include "model_reference_types.h"
#include "rtw_modelmap.h"
#include "rt_nonfinite.h"
typedef struct { real_T bt3v4tz3vc [ 3 ] ; real_T n1d1o410q5 [ 2 ] ; real_T
kx3zp1vjx0 [ 2 ] ; real_T gwlif0cb5c ; real_T niwmxjzarl ; real_T khur02rovy
; real_T oxj1pfzvxc ; real_T hp05tz33h3 ; real_T pff5tdsjfx ; real_T
d3rdrsfoaq ; real_T c0aucmrtra ; real_T jh0rwtpvlt ; real_T chs2aeoyso [ 3 ]
; real_T m4sjem2ccc ; real_T ffcbt3fdhz ; real_T dslbpcsy0y ; real_T
k3gs1qqdla [ 3 ] ; } nwsnpeh0gv ; typedef struct { real_T mbip04lzsp [ 2 ] ;
real_T d3ilz453za ; real_T h3332uj4aw ; real_T fdoeme1zt3 ; real_T eouiwnmlg2
[ 3 ] ; real_T lp5agtuvxc ; real_T h5jvqgtpml ; real_T dqwwcuyuq4 [ 90001 ] ;
real_T c504xcsrsm [ 6145 ] ; struct { void * AQHandles ; } hxbid0vsn5 ; void
* kwbfl5dt0f [ 6 ] ; struct { void * AQHandles ; } c4u1lpdbv5 ; struct { void
* AQHandles ; } lxh3eyrkbd ; struct { void * AQHandles ; } hs0ewiu05t ;
struct { void * AQHandles ; } lwk4kxq0uo ; void * cvx5zcltdt [ 6 ] ; struct {
void * AQHandles ; } fooxvrwdmt ; struct { void * AQHandles ; } bvhuic1igg ;
struct { void * AQHandles ; } fnv04adgun ; uint32_T e1ksmroidr ; uint32_T
eefjghohnx ; uint32_T fpr4n1vl1w ; uint32_T cxg03genxm ; int_T lksq110q2k [ 8
] ; int_T l22bn1g3va [ 12 ] ; int8_T esosaj0uae ; beudxm5cnjh byox3ydkm3 ;
d1b0notwieb nsxv3ehu00 ; } c4q3wsctrr ; typedef struct { real_T nvfwub2g2d [
3 ] ; real_T pf4a12cvup [ 2 ] ; kcqz4a1hsb e0zwirfcjr ; k4a404kthw prlozmkf4f
; } m12nvksmt4 ; typedef struct { real_T nvfwub2g2d [ 3 ] ; real_T pf4a12cvup
[ 2 ] ; bzogrrroc4 e0zwirfcjr ; jd2odbvxqe prlozmkf4f ; } gx4bh4h51m ;
typedef struct { boolean_T nvfwub2g2d [ 3 ] ; boolean_T pf4a12cvup [ 2 ] ;
ovrrmmcoo5 e0zwirfcjr ; ae5m15ljke prlozmkf4f ; } jeltdsf5bd ; struct
lskqh0u45pg_ { real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 ; real_T
P_17 ; real_T P_18 ; real_T P_19 ; real_T P_20 ; real_T P_21 ; real_T P_22 ;
real_T P_23 ; real_T P_24 ; real_T P_25 ; } ; struct pomxc0yuro { struct
SimStruct_tag * _mdlRefSfcnS ; struct { real_T mr_nonContSig0 [ 1 ] ; }
NonContDerivMemory ; ssNonContDerivSigInfo nonContDerivSignal [ 1 ] ; const
rtTimingBridge * timingBridge ; struct { rtwCAPI_ModelMappingInfo mmi ;
rtwCAPI_ModelMapLoggingInstanceInfo mmiLogInstanceInfo ; void * dataAddress [
2 ] ; int32_T * vardimsAddress [ 2 ] ; RTWLoggingFcnPtr loggingPtrs [ 2 ] ;
rtwCAPI_ModelMappingInfo * childMMI [ 2 ] ; sysRanDType * systemRan [ 5 ] ;
int_T systemTid [ 5 ] ; } DataMapInfo ; struct { int_T mdlref_GlobalTID [ 6 ]
; time_T tStart ; } Timing ; } ; typedef struct { nwsnpeh0gv rtb ; c4q3wsctrr
rtdw ; ov4zevcfb3 rtm ; } ebrc2jzfjdz ; extern real_T rtP_CAM_LAT ; extern
real_T rtP_E_max ; extern real_T rtP_Ki ; extern real_T rtP_Kp ; extern
real_T rtP_L_track ; extern real_T rtP_R_wheel ; extern real_T rtP_bias_th ;
extern real_T rtP_bias_x ; extern real_T rtP_bias_y ; extern real_T rtP_c_L ;
extern real_T rtP_c_R ; extern real_T rtP_m_L ; extern real_T rtP_m_R ;
extern void mx5pxzwgbk ( SimStruct * _mdlRefSfcnS ,
ssNonContDerivSigFeedingOutports * * mr_nonContOutputArray , int_T
mdlref_TID0 , int_T mdlref_TID1 , int_T mdlref_TID2 , int_T mdlref_TID3 ,
int_T mdlref_TID4 , int_T mdlref_TID5 , ov4zevcfb3 * const ptfalyqw1f ,
nwsnpeh0gv * localB , c4q3wsctrr * localDW , m12nvksmt4 * localX , void *
sysRanPtr , int_T contextTid , rtwCAPI_ModelMappingInfo * rt_ParentMMI ,
const char_T * rt_ChildPath , int_T rt_ChildMMIIdx , int_T rt_CSTATEIdx ) ;
extern void mr_Robot_MdlInfoRegFcn ( SimStruct * mdlRefSfcnS , char_T *
modelName , int_T * retVal ) ; extern mxArray * mr_Robot_GetDWork ( const
ebrc2jzfjdz * mdlrefDW ) ; extern void mr_Robot_SetDWork ( ebrc2jzfjdz *
mdlrefDW , const mxArray * ssDW ) ; extern void
mr_Robot_RegisterSimStateChecksum ( SimStruct * S ) ; extern mxArray *
mr_Robot_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * Robot_GetCAPIStaticMap ( void ) ; extern
void ab0nm2yrwy ( ov4zevcfb3 * const ptfalyqw1f , c4q3wsctrr * localDW ) ;
extern void lo32uwf2vy ( ov4zevcfb3 * const ptfalyqw1f , real_T nexdntiky0 ,
const real_T oesgikpmir [ 3 ] , real_T kio141y2si , real_T kdqgjhotff ,
real_T ehd2kdkuss , c4q3wsctrr * localDW , m12nvksmt4 * localX ) ; extern
void kuqhzhrnbu ( ov4zevcfb3 * const ptfalyqw1f , real_T nexdntiky0 , const
real_T oesgikpmir [ 3 ] , real_T kio141y2si , real_T kdqgjhotff , real_T
ehd2kdkuss , c4q3wsctrr * localDW , m12nvksmt4 * localX ) ; extern void
g2ma4hm2ad ( ov4zevcfb3 * const ptfalyqw1f , c4q3wsctrr * localDW ) ; extern
void kcv51qakwg ( ov4zevcfb3 * const ptfalyqw1f , const real_T oesgikpmir [ 3
] , c4q3wsctrr * localDW ) ; extern void pcqiaapqwu ( const real_T *
ia32rubutl , const real_T * ncr3vfbgqc , nwsnpeh0gv * localB , c4q3wsctrr *
localDW , gx4bh4h51m * localXdot , m12nvksmt4 * localX ) ; extern void
mggwjwnbfc ( ov4zevcfb3 * const ptfalyqw1f , const real_T * ia32rubutl ,
const real_T * ncr3vfbgqc , nwsnpeh0gv * localB , c4q3wsctrr * localDW ,
m12nvksmt4 * localX ) ; extern void Robot ( ov4zevcfb3 * const ptfalyqw1f ,
real_T mpj4qdwwdl [ 3 ] , const real_T oesgikpmir [ 3 ] , nwsnpeh0gv * localB
, c4q3wsctrr * localDW , m12nvksmt4 * localX ) ; extern void krgsypt3b4 ( c4q3wsctrr * localDW , ov4zevcfb3 * const ptfalyqw1f ) ;
#endif
