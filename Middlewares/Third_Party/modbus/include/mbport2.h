#ifndef _MB_PORT_H
#define _MB_PORT_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif



/* ----------------------- Type definitions ---------------------------------*/

typedef enum
{
    EV_READY,                   /*!< Startup finished. */
    EV_FRAME_RECEIVED,          /*!< Frame received. */
    EV_EXECUTE,                 /*!< Execute function. */
    EV_FRAME_SENT,              /*!< Frame sent. */
    EV_NULL
} eMBEventType;



#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif


/*********************** END OF FILE ******************************************************/
