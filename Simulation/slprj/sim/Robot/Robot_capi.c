#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Robot_capi_host.h"
#define sizeof(...) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 19
#endif
#ifndef SS_INT64
#define SS_INT64 20
#endif
#else
#include "builtin_typeid_types.h"
#include "Robot.h"
#include "Robot_capi.h"
#include "Robot_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static rtwCAPI_States rtBlockStates [ ] = { { 0 , 0 ,
TARGET_STRING ( "Robot/Integrator" ) , TARGET_STRING ( "" ) , TARGET_STRING ( ""
) , 0 , 0 , 0 , 0 , 0 , 1 , - 1 , 0 } , { 1 , 3 , TARGET_STRING ( "Robot/Bluetooth Delay/Variable\nTransport Delay" ) , TARGET_STRING ( "" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 , 1 , - 1 , 0 } , { 0 , - 1 , ( NULL ) , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 , 0 , - 1 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 0 , 1 } ;
#ifndef HOST_CAPI_BUILD
static void Robot_InitializeDataAddr ( void * dataAddr [ ] , c4q3wsctrr *
localDW , m12nvksmt4 * localX ) { dataAddr [ 0 ] = ( void * ) ( & localX ->
nvfwub2g2d [ 0 ] ) ; dataAddr [ 1 ] = ( void * ) ( & localX -> pf4a12cvup [ 0
] ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void Robot_InitializeVarDimsAddr ( int32_T * vardimsAddr [ ] ) {
vardimsAddr [ 0 ] = ( NULL ) ; }
#endif
#ifndef HOST_CAPI_BUILD
static void Robot_InitializeLoggingFunctions ( RTWLoggingFcnPtr loggingPtrs [
] ) { loggingPtrs [ 0 ] = ( NULL ) ; loggingPtrs [ 1 ] = ( NULL ) ; }
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } }
;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_VECTOR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } } ; static
uint_T rtDimensionArray [ ] = { 3 , 1 , 2 , 1 } ; static const real_T
rtcapiStoredFloats [ ] = { 0.0 } ; static rtwCAPI_FixPtMap rtFixPtMap [ ] = {
{ ( NULL ) , ( NULL ) , rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , }
; static rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , ( int8_T ) 0 , ( uint8_T ) 0 } } ; static int_T rtContextSystems [ 5 ] ; static rtwCAPI_LoggingMetaInfo loggingMetaInfo [ ] = { { 0 , 0 , "" , 0 } } ; static rtwCAPI_ModelMapLoggingStaticInfo mmiStaticInfoLogging = { 5 , rtContextSystems , loggingMetaInfo , 0 , ( NULL ) , { 0 , ( NULL ) , ( NULL ) } , 0 , ( NULL ) } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals , 0 , ( NULL ) , 0 , ( NULL ) , 0 } , { ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockStates , 2 } , { rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float" , { 696874642U , 981371786U , 439127923U , 1623566673U } , & mmiStaticInfoLogging , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ; const rtwCAPI_ModelMappingStaticInfo * Robot_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
static void Robot_InitializeSystemRan ( ov4zevcfb3 * const ptfalyqw1f ,
sysRanDType * systemRan [ ] , c4q3wsctrr * localDW , int_T systemTid [ ] ,
void * rootSysRanPtr , int rootTid ) { UNUSED_PARAMETER ( ptfalyqw1f ) ;
UNUSED_PARAMETER ( localDW ) ; systemRan [ 0 ] = ( sysRanDType * )
rootSysRanPtr ; systemRan [ 1 ] = ( NULL ) ; systemRan [ 2 ] = ( NULL ) ;
systemRan [ 3 ] = ( NULL ) ; systemRan [ 4 ] = ( NULL ) ; systemTid [ 1 ] =
ptfalyqw1f -> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 2 ] = ptfalyqw1f
-> Timing . mdlref_GlobalTID [ 0 ] ; systemTid [ 3 ] = ptfalyqw1f -> Timing .
mdlref_GlobalTID [ 0 ] ; systemTid [ 4 ] = ptfalyqw1f -> Timing .
mdlref_GlobalTID [ 0 ] ; systemTid [ 0 ] = rootTid ; rtContextSystems [ 0 ] =
0 ; rtContextSystems [ 1 ] = 0 ; rtContextSystems [ 2 ] = 0 ;
rtContextSystems [ 3 ] = 0 ; rtContextSystems [ 4 ] = 0 ; }
#endif
#ifndef HOST_CAPI_BUILD
void Robot_InitializeDataMapInfo ( ov4zevcfb3 * const ptfalyqw1f , c4q3wsctrr
* localDW , m12nvksmt4 * localX , void * sysRanPtr , int_T contextTid ) {
rtwCAPI_SetVersion ( ptfalyqw1f -> DataMapInfo . mmi , 1 ) ;
rtwCAPI_SetStaticMap ( ptfalyqw1f -> DataMapInfo . mmi , & mmiStatic ) ;
rtwCAPI_SetLoggingStaticMap ( ptfalyqw1f -> DataMapInfo . mmi , &
mmiStaticInfoLogging ) ; Robot_InitializeDataAddr ( ptfalyqw1f -> DataMapInfo
. dataAddress , localDW , localX ) ; rtwCAPI_SetDataAddressMap ( ptfalyqw1f
-> DataMapInfo . mmi , ptfalyqw1f -> DataMapInfo . dataAddress ) ;
Robot_InitializeVarDimsAddr ( ptfalyqw1f -> DataMapInfo . vardimsAddress ) ;
rtwCAPI_SetVarDimsAddressMap ( ptfalyqw1f -> DataMapInfo . mmi , ptfalyqw1f
-> DataMapInfo . vardimsAddress ) ; rtwCAPI_SetPath ( ptfalyqw1f ->
DataMapInfo . mmi , ( NULL ) ) ; rtwCAPI_SetFullPath ( ptfalyqw1f ->
DataMapInfo . mmi , ( NULL ) ) ; Robot_InitializeLoggingFunctions ( ptfalyqw1f
-> DataMapInfo . loggingPtrs ) ; rtwCAPI_SetLoggingPtrs ( ptfalyqw1f ->
DataMapInfo . mmi , ptfalyqw1f -> DataMapInfo . loggingPtrs ) ;
rtwCAPI_SetInstanceLoggingInfo ( ptfalyqw1f -> DataMapInfo . mmi , &
ptfalyqw1f -> DataMapInfo . mmiLogInstanceInfo ) ; rtwCAPI_SetChildMMIArray ( ptfalyqw1f -> DataMapInfo . mmi , ptfalyqw1f -> DataMapInfo . childMMI ) ; rtwCAPI_SetChildMMIArrayLen ( ptfalyqw1f -> DataMapInfo . mmi , 2 ) ; Robot_InitializeSystemRan ( ptfalyqw1f , ptfalyqw1f -> DataMapInfo . systemRan , localDW , ptfalyqw1f -> DataMapInfo . systemTid , sysRanPtr , contextTid ) ; rtwCAPI_SetSystemRan ( ptfalyqw1f -> DataMapInfo . mmi , ptfalyqw1f -> DataMapInfo . systemRan ) ; rtwCAPI_SetSystemTid ( ptfalyqw1f -> DataMapInfo . mmi , ptfalyqw1f -> DataMapInfo . systemTid ) ; rtwCAPI_SetGlobalTIDMap ( ptfalyqw1f -> DataMapInfo . mmi , & ptfalyqw1f -> Timing . mdlref_GlobalTID [ 0 ] ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Robot_host_InitializeDataMapInfo ( Robot_host_DataMapInfo_T * dataMap ,
const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; dataMap -> childMMI [ 0 ] = & ( dataMap -> child0 . mmi ) ;
LHS_Robot_host_InitializeDataMapInfo ( & ( dataMap -> child0 ) ,
"Robot/Model" ) ; dataMap -> childMMI [ 1 ] = & ( dataMap -> child1 . mmi ) ;
RHS_Robot_host_InitializeDataMapInfo ( & ( dataMap -> child1 ) ,
"Robot/Model1" ) ; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , dataMap ->
childMMI ) ; rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 2 ) ; }
#ifdef __cplusplus
}
#endif
#endif
