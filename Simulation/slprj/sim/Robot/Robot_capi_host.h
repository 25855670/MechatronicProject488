#ifndef Robot_cap_host_h__
#define Robot_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"
#include "LHS_Robot_capi_host.h"
#include "RHS_Robot_capi_host.h"
typedef struct { rtwCAPI_ModelMappingInfo mmi ; rtwCAPI_ModelMappingInfo *
childMMI [ 2 ] ; LHS_Robot_host_DataMapInfo_T child0 ;
RHS_Robot_host_DataMapInfo_T child1 ; } Robot_host_DataMapInfo_T ;
#ifdef __cplusplus
extern "C" {
#endif
void Robot_host_InitializeDataMapInfo ( Robot_host_DataMapInfo_T * dataMap ,
const char * path ) ;
#ifdef __cplusplus
}
#endif
#endif
#endif
