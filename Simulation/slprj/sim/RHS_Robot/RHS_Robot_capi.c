#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "RHS_Robot_capi_host.h"
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
#include "RHS_Robot.h"
#include "RHS_Robot_capi.h"
#include "RHS_Robot_private.h"
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
TARGET_STRING ( "RHS_Robot/Right Motor" ) , TARGET_STRING ( "" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 1 , 0 ,
TARGET_STRING ( "RHS_Robot/Right MT Estimator/Integrator" ) , TARGET_STRING ( ""
) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 2 , - 1 ,
TARGET_STRING ( "RHS_Robot/Right Speed Controller/Filter/Disc. Forward Euler Filter/Filter" ) , TARGET_STRING ( "DSTATE" ) , "" , 0 , 0 , 0 , 0 , 1 , 0 , - 1 , 0 } , { 3 , - 1 , TARGET_STRING ( "RHS_Robot/Right Speed Controller/Integrator/Discrete/Integrator" ) , TARGET_STRING ( "DSTATE" ) , "" , 0 , 0 , 0 , 0 , 1 , 0 , - 1 , 0 } , { 0 , - 1 , ( NULL ) , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 1 , 0 , 0 , 1 } ;
#ifndef HOST_CAPI_BUILD
static void RHS_Robot_InitializeDataAddr ( void * dataAddr [ ] , pzcxryh4y3 *
localDW , kcqz4a1hsb * localX ) { dataAddr [ 0 ] = ( void * ) ( & localX ->
li4zimgc0e ) ; dataAddr [ 1 ] = ( void * ) ( & localX -> gkm05mq2uf ) ;
dataAddr [ 2 ] = ( void * ) ( & localDW -> onjeckwmao ) ; dataAddr [ 3 ] = ( void * ) ( & localDW -> jzvmx3qjzz ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void RHS_Robot_InitializeVarDimsAddr ( int32_T * vardimsAddr [ ] ) {
vardimsAddr [ 0 ] = ( NULL ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void RHS_Robot_InitializeLoggingFunctions ( RTWLoggingFcnPtr
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
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 0 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 1 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 2 , ( uint8_T ) 0 } } ; static int_T rtContextSystems [ 4 ] ; static rtwCAPI_LoggingMetaInfo loggingMetaInfo [ ] = { { 0 , 0 , "" , 0 } } ; static rtwCAPI_ModelMapLoggingStaticInfo mmiStaticInfoLogging = { 4 , rtContextSystems , loggingMetaInfo , 0 , ( NULL ) , { 0 , ( NULL ) , ( NULL ) } , 0 , ( NULL ) } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 0 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockStates , 4 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 1080152732U , 3116893746U , 3137021212U , 3467382145U } , & mmiStaticInfoLogging , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * RHS_Robot_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
static void RHS_Robot_InitializeSystemRan ( lqonxzrhbw * const mcfta5e4au ,
sysRanDType * systemRan [ ] , pzcxryh4y3 * localDW , int_T systemTid [ ] ,
void * rootSysRanPtr , int rootTid ) { UNUSED_PARAMETER ( mcfta5e4au ) ;
UNUSED_PARAMETER ( localDW ) ; systemRan [ 0 ] = ( sysRanDType * )
rootSysRanPtr ; systemRan [ 1 ] = ( NULL ) ; systemRan [ 2 ] = ( NULL ) ;
systemRan [ 3 ] = ( NULL ) ; systemTid [ 1 ] = mcfta5e4au -> Timing .
mdlref_GlobalTID [ 1 ] ; systemTid [ 2 ] = mcfta5e4au -> Timing .
mdlref_GlobalTID [ 2 ] ; systemTid [ 3 ] = mcfta5e4au -> Timing .
mdlref_GlobalTID [ 0 ] ; systemTid [ 0 ] = rootTid ; rtContextSystems [ 0 ] =
0 ; rtContextSystems [ 1 ] = 0 ; rtContextSystems [ 2 ] = 0 ;
rtContextSystems [ 3 ] = 0 ; }
#endif
#ifndef HOST_CAPI_BUILD
void RHS_Robot_InitializeDataMapInfo ( lqonxzrhbw * const mcfta5e4au ,
pzcxryh4y3 * localDW , kcqz4a1hsb * localX , void * sysRanPtr , int_T
contextTid ) { rtwCAPI_SetVersion ( mcfta5e4au -> DataMapInfo . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( mcfta5e4au -> DataMapInfo . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( mcfta5e4au -> DataMapInfo . mmi , &
mmiStaticInfoLogging ) ; RHS_Robot_InitializeDataAddr ( mcfta5e4au ->
DataMapInfo . dataAddress , localDW , localX ) ; rtwCAPI_SetDataAddressMap ( mcfta5e4au -> DataMapInfo . mmi , mcfta5e4au -> DataMapInfo . dataAddress ) ; RHS_Robot_InitializeVarDimsAddr ( mcfta5e4au -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetVarDimsAddressMap ( mcfta5e4au -> DataMapInfo . mmi , mcfta5e4au -> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetPath ( mcfta5e4au -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetFullPath ( mcfta5e4au -> DataMapInfo . mmi , ( NULL ) ) ; RHS_Robot_InitializeLoggingFunctions ( mcfta5e4au -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetLoggingPtrs ( mcfta5e4au -> DataMapInfo . mmi , mcfta5e4au -> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetInstanceLoggingInfo ( mcfta5e4au -> DataMapInfo . mmi , & mcfta5e4au -> DataMapInfo . mmiLogInstanceInfo ) ; rtwCAPI_SetChildMMIArray ( mcfta5e4au -> DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArrayLen ( mcfta5e4au -> DataMapInfo . mmi , 0 ) ; RHS_Robot_InitializeSystemRan ( mcfta5e4au , mcfta5e4au -> DataMapInfo . systemRan , localDW , mcfta5e4au -> DataMapInfo . systemTid , sysRanPtr , contextTid ) ; rtwCAPI_SetSystemRan ( mcfta5e4au -> DataMapInfo . mmi , mcfta5e4au -> DataMapInfo . systemRan ) ; rtwCAPI_SetSystemTid ( mcfta5e4au -> DataMapInfo . mmi , mcfta5e4au -> DataMapInfo . systemTid ) ; rtwCAPI_SetGlobalTIDMap ( mcfta5e4au -> DataMapInfo . mmi , & mcfta5e4au -> Timing . mdlref_GlobalTID [ 0 ] ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void RHS_Robot_host_InitializeDataMapInfo ( RHS_Robot_host_DataMapInfo_T *
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
