// ---------------------------------------------------------------------------------------------------------------------
// Company:      Visteon Inc / UTEQ.
// ---------------------------------------------------------------------------------------------------------------------
// Copyright:    This software is ERUIZ3 property.
//               Duplication or disclosure without ERUIZ3 written authorization is prohibited.
// ---------------------------------------------------------------------------------------------------------------------
// Project:      Application Library
// Language:     ANSI-C
// ---------------------------------------------------------------------------------------------------------------------
// Component:    Funcion Test Source
// ---------------------------------------------------------------------------------------------------------------------
// $Revision: $
// ---------------------------------------------------------------------------------------------------------------------
// $Log: $
//
// ---------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
// Body Identification
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Included files to resolve specific definitions in this file
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local macros
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local types
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local data
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Exported data
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Constant exported data
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local function prototypes
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local constants
//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

//======================================================================================================================
// LOCAL FUNCTIONS
//======================================================================================================================


//----------------------------------------------------------------------------------------------------------------------
/// @brief Invoke Falla
///
/// @param bool data
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------

#include "FunctionTest.h"

void Interface_Falla(Hay_Falla)
{
    /* Imprime el estado de falla*/
    printf("¿La falla esta presente?");
    printf(Hay_Falla?"true":"false");
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief Invoke Reporte
///
/// @param Corto_t data
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------

void Reporte_De_Falla(Data)
{
    bool falla = false;
    if(SIN_CORTOS != Data)
    {
        falla = true;
    }
    Interface_Falla(falla);
}
/**==========================================================================*/
/* Revision Notes=================================================*/
/**====================================================================================================================================
 **      CDSID         Date        Traceability              Description
 **====================================================================================================================================
 **      eruiz3      26-Feb-2022      0000001      Initial Template
 **====================================================================================================================================*/
 /* end of file =============================================================*/