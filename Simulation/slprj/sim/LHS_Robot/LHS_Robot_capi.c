#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "LHS_Robot_capi_host.h"
#define sizeof(...) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 17
#endif
#ifndef SS_INT64
#define SS_INT64 18
#endif
#else
#include "builtin_typeid_types.h"
#include "LHS_Robot.h"
#include "LHS_Robot_capi.h"
#include "LHS_Robot_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static rtwCAPI_States rtBlockStates [ ] = { { 0 , 1 ,
TARGET_STRING ( "LHS_Robot/Left Motor" ) , TARGET_STRING ( "" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 1 , 0 ,
TARGET_STRING ( "LHS_Robot/Left MT Estimator/Integrator" ) , TARGET_STRING ( ""
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 2 , - 1 ,
TARGET_STRING ( "LHS_Robot/Left speed controller/Filter/Disc. Forward Euler Filter/Filter" ) , TARGET_STRING ( "DSTATE" ) , "" , 0 , 0 , 0 , 0 , 1 , 0 , - 1 , 0 } , { 3 , - 1 , TARGET_STRING ( "LHS_Robot/Left speed controller/Integrator/Discrete/Integrator" ) , TARGET_STRING ( "DSTATE" ) , "" , 0 , 0 , 0 , 0 , 1 , 0 , - 1 , 0 } , { 0 , - 1 , ( NULL ) , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 1 , 0 , 0 , 1 } ;
#ifndef HOST_CAPI_BUILD
static void LHS_Robot_InitializeDataAddr ( void * dataAddr [ ] , bhduwxqtz1 *
localDW , k4a404kthw * localX ) { dataAddr [ 0 ] = ( void * ) ( & localX ->
gcd5lf35va ) ; dataAddr [ 1 ] = ( void * ) ( & localX -> p2ggtkfarh ) ;
dataAddr [ 2 ] = ( void * ) ( & localDW -> agpivbt0hr ) ; dataAddr [ 3 ] = ( void * ) ( & localDW -> ic4s3kvc0r ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void LHS_Robot_InitializeVarDimsAddr ( int32_T * vardimsAddr [ ] ) {
vardimsAddr [ 0 ] = ( NULL ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void LHS_Robot_InitializeLoggingFunctions ( RTWLoggingFcnPtr
loggingPtrs [ ] ) { loggingPtrs [ 0 ] = ( NULL ) ; loggingPtrs [ 1 ] = ( NULL
) ; loggingPtrs [ 2 ] = ( NULL ) ; loggingPtrs [ 3 ] = ( NULL ) ; }
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } }
;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } } ; static uint_T rtDimensionArray [ ] = { 1 , 1
} ; static const real_T rtcapiStoredFloats [ ] = { 0.0 , 0.02 } ; static
rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 0 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 2 , ( uint8_T ) 0 } } ; static int_T rtContextSystems [ 4 ] ; static rtwCAPI_LoggingMetaInfo loggingMetaInfo [ ] = { { 0 , 0 , "" , 0 } } ; static rtwCAPI_ModelMapLoggingStaticInfo mmiStaticInfoLogging = { 4 , rtContextSystems , loggingMetaInfo , 0 , ( NULL ) , { 0 , ( NULL ) , ( NULL ) } , 0 , ( NULL ) } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 0 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockStates , 4 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 1435876931U , 3196076125U , 3959102191U , 3667112841U } , & mmiStaticInfoLogging , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * LHS_Robot_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
static void LHS_Robot_InitializeSystemRan ( kizrp3vx0c * const gi5soirers ,
sysRanDType * systemRan [ ] , bhduwxqtz1 * localDW , int_T systemTid [ ] ,
void * rootSysRanPtr , int rootTid ) { UNUSED_PARAMETER ( gi5soirers ) ;
UNUSED_PARAMETER ( localDW ) ; systemRan [ 0 ] = ( sysRanDType * )
rootSysRanPtr ; systemRan [ 1 ] = ( NULL ) ; systemRan [ 2 ] = ( NULL ) ;
systemRan [ 3 ] = ( NULL ) ; systemTid [ 1 ] = gi5soirers -> Timing .
mdlref_GlobalTID [ 1 ] ; systemTid [ 2 ] = gi5soirers -> Timing .
mdlref_GlobalTID [ 2 ] ; systemTid [ 3 ] = gi5soirers -> Timing .
mdlref_GlobalTID [ 0 ] ; systemTid [ 0 ] = rootTid ; rtContextSystems [ 0 ] =
0 ; rtContextSystems [ 1 ] = 0 ; rtContextSystems [ 2 ] = 0 ;
rtContextSystems [ 3 ] = 0 ; }
#endif
#ifndef HOST_CAPI_BUILD
void LHS_Robot_InitializeDataMapInfo ( kizrp3vx0c * const gi5soirers ,
bhduwxqtz1 * localDW , k4a404kthw * localX , void * sysRanPtr , int_T
contextTid ) { rtwCAPI_SetVersion ( gi5soirers -> DataMapInfo . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( gi5soirers -> DataMapInfo . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( gi5soirers -> DataMapInfo . mmi , &
mmiStaticInfoLogging ) ; LHS_Robot_InitializeDataAddr ( gi5soirers ->
DataMapInfo . dataAddress , localDW , localX ) ; rtwCAPI_SetDataAddressMap ( gi5soirers -> DataMapInfo . mmi , gi5soirers -> DataMapInfo . dataAddress ) ; LHS_Robot_InitializeVarDimsAddr ( gi5soirers -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetVarDimsAddressMap ( gi5soirers -> DataMapInfo . mmi , gi5soirers -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetPath ( gi5soirers -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetFullPath ( gi5soirers -> DataMapInfo . mmi , ( NULL ) ) ; LHS_Robot_InitializeLoggingFunctions ( gi5soirers -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetLoggingPtrs ( gi5soirers -> DataMapInfo . mmi , gi5soirers -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetInstanceLoggingInfo ( gi5soirers -> DataMapInfo . mmi , & gi5soirers -> DataMapInfo . mmiLogInstanceInfo ) ; rtwCAPI_SetChildMMIArray ( gi5soirers -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( gi5soirers -> DataMapInfo . mmi , 0 ) ; LHS_Robot_InitializeSystemRan ( gi5soirers , gi5soirers -> DataMapInfo . systemRan , localDW , gi5soirers -> DataMapInfo . systemTid , sysRanPtr , contextTid ) ; rtwCAPI_SetSystemRan ( gi5soirers -> DataMapInfo . mmi , gi5soirers -> DataMapInfo . systemRan ) ; rtwCAPI_SetSystemTid ( gi5soirers -> DataMapInfo . mmi , gi5soirers -> DataMapInfo . systemTid ) ; rtwCAPI_SetGlobalTIDMap ( gi5soirers -> DataMapInfo . mmi , & gi5soirers -> Timing . mdlref_GlobalTID [ 0 ] ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void LHS_Robot_host_InitializeDataMapInfo ( LHS_Robot_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
