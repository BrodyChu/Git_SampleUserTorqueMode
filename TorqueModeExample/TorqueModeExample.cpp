#include "NexMotion.h"
#include "NexMotionError.h"

#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//#define UNDER_WIN32_SIMULATION

int main()
{
    RTN_ERR ret                = 0;
    I32_T   devIndex           = 0;
    I32_T   retDevID           = 0;
    I32_T   retGroupCount      = 0;
    I32_T   retGroupAxisCount  = 0;
    I32_T   countGroupAxis     = 0;
    I32_T   countGroup         = 0;
    I32_T   initGroupAxisCount = 0;
    I32_T   groupIndex         = 0;
    I32_T   retDevState        = 0;
    Pos_T   cmdPosAcs          = { 0 };
    I32_T   ctrlMode           = 0;
    BOOL_T  isSupport          = FALSE;
    ATorq_T torq               = { 0 };
    I32_T   groupState         = 0;
    I32_T   groupIdxMask       = 0;
    AVel_T  velAcs             = { 0 };
	I32_T	timeSetTorque = 4000;
	I32_T	stepTorque = 1;
	I32_T	maxTorque = 100;

#ifdef UNDER_WIN32_SIMULATION
    I32_T devType   = NMC_DEVICE_TYPE_SIMULATOR;
    U32_T sleepTime = 500;
#else
    I32_T devType   = NMC_DEVICE_TYPE_ETHERCAT;
    U32_T sleepTime = 2000;
#endif

    printf( "=======================================================================================================================\n");
    printf( "** This is an example of how to use APIs of torque mode to control a general robot.\n" );
    printf( "**** Notification: The device must have at least one group.\n" );
    printf( "**** Notification: The user must check whether the correct configuration file (.ini ) exists next to the executable file.\n");
    printf( "=======================================================================================================================\n\n");
    //=================================================
    //              Device open up
    //=================================================
    printf( "Start to openup device...\n" );

    ret = NMC_DeviceOpenUp( devType, devIndex, &retDevID );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_DeviceOpenUp: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }
    printf( "\nDevice open up succeed, device ID: %d.\n", retDevID );

    //get device state
    ret = NMC_DeviceGetState( retDevID, &retDevState );
    if( retDevState != NMC_DEVICE_STATE_OPERATION || ret != 0 )
    {
        printf( "ERROR! Device open up failed, device ID: %d.\n", retDevID );
        goto ERR_SHUTDOWN;
    }
    printf( "Device ID %d: state is OPERATION.\n", retDevID );

    //=================================================
    //              Get device infomation
    //=================================================
    //Get amount of GROUP
    ret = NMC_DeviceGetGroupCount( retDevID, &retGroupCount );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_DeviceGetGroupCount: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }

    if( retGroupCount == 0 )
    {
        printf( "ERROR! NMC_DeviceGetGroupCount = 0\n" );
        goto ERR_SHUTDOWN;
    }
    printf( "Get group count succeed, device has %d group.\n", retGroupCount );

    //Get amount of AXIS of each GROUP

    for( countGroup = 0; countGroup < retGroupCount; ++countGroup )
    {
        ret = NMC_DeviceGetGroupAxisCount( retDevID, countGroupAxis, &retGroupAxisCount );
        if( ret != ERR_NEXMOTION_SUCCESS )
        {
            printf( "ERROR! NMC_DeviceGetGroupAxisCount: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
            goto ERR_SHUTDOWN;
        }
        printf( "Get group axis count succeed, group index %d has %d axis.\n", countGroupAxis, retGroupAxisCount );
    }

    //=================================================
    //       Clean alarm of drives of each group
    //=================================================
    for( countGroup = 0; countGroup < retGroupCount; ++countGroup )
    {
        ret = NMC_GroupResetState( retDevID, countGroup );
        if( ret != ERR_NEXMOTION_SUCCESS )
        {
            printf( "ERROR! NMC_GroupResetDriveAlmAll: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
            goto ERR_SHUTDOWN;
        }
    }

    Sleep( sleepTime );

    //=================================================
    //       Enable all single axes and groups
    //=================================================
    ret = NMC_DeviceEnableAll( retDevID );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_DeviceEnableAll: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }
    printf( "\nReady to enable all single axes and groups...\n" );

    Sleep( sleepTime );

	for( countGroup = 0; countGroup < retGroupCount; ++countGroup )
	{
		ret = NMC_GroupGetState( retDevID, countGroup, &retDevState );
		if( retDevState != NMC_GROUP_STATE_STAND_STILL )
		{
			printf( "ERROR! Group state is not STAND STILL, go to shutdown\n" );
			goto ERR_SHUTDOWN;
		}
	}

    //=================================================
    //            Change operation mode
    //=================================================
    ret = NMC_GroupIsCtrlModeSupported( retDevID, groupIndex, NMC_GROUP_CTRL_MODE_USER_TORQ, &isSupport );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_GroupIsCtrlModeSupported: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }
    
    printf( "User torque mode is support or not(0: No, 1: Yes): %d\n", isSupport );

    if( isSupport == TRUE )
    {
		ret = NMC_GroupGetActTorq( retDevID, groupIndex, &torq );
		if( ret != ERR_NEXMOTION_SUCCESS )
		{
			printf( "ERROR! NMC_GroupGetActTorq: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
			goto ERR_SHUTDOWN;
		}

        ret = NMC_GroupSetCmdTorqFC( retDevID, groupIndex, &torq );
        if( ret != ERR_NEXMOTION_SUCCESS )
        {
            printf( "ERROR! NMC_GroupSetCmdTorqFC: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
            goto ERR_SHUTDOWN;
        }

        Sleep( 50 );

		//set control mode
		ctrlMode = NMC_GROUP_CTRL_MODE_USER_TORQ;
        ret = NMC_GroupSetCtrlMode( retDevID, groupIndex, ctrlMode );
        if( ret != ERR_NEXMOTION_SUCCESS )
        {
            printf( "ERROR! NMC_GroupSetCtrlMode: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
            goto ERR_SHUTDOWN;
        }

		Sleep( 50 );

		//enable fast control
		ret = NMC_GroupFastCtrlEnable( retDevID, groupIndex, NMC_GROUP_FAST_CTRL_ENTRY_ITEM_TORQUE, TRUE );
		if( ret != ERR_NEXMOTION_SUCCESS )
		{
			printf( "ERROR! NMC_GroupFastCtrlEnable: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
			goto ERR_SHUTDOWN;
		}
    }
    else
        goto ERR_SHUTDOWN;

    Sleep( 50 );

    ret = NMC_GroupGetCtrlMode( retDevID, groupIndex, &ctrlMode );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_GroupGetCtrlMode: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }

    printf( "Control mode=%d\n", ctrlMode ); //Should be 1

    ret = NMC_GroupGetState( retDevID, groupIndex, &groupState );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_GroupGetState: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }

    printf( "Group(%d) state=%d\n", groupIndex, groupState ); //Should be 16


	//=================================================
    //            Set torque and get torque
    //=================================================
	//get actual torque and print it
    ret = NMC_GroupGetActTorq( retDevID, groupIndex, &torq );
    if( ret != ERR_NEXMOTION_SUCCESS )
    {
        printf( "ERROR! NMC_GroupGetActTorq: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
        goto ERR_SHUTDOWN;
    }
	for( I32_T i = 0; i < retGroupAxisCount; i++ )
	{
		printf( "Group(%d) Joint(%d)'s actual torque = %lf\n", groupIndex, i, torq.torq[i] );
	}

	//plan torque and print it
	bool isDone = false;
	for( I32_T t = 0; t < timeSetTorque; t++ )
	{
		for( I32_T i = 0; i < retGroupAxisCount; i++ )
		{
			if( t < ( I32_T ) ( timeSetTorque / 2 ) )
			{
				torq.torq[i] += stepTorque;
				if( torq.torq[i] > maxTorque )
					torq.torq[i] = maxTorque;
			}
			else
			{
				torq.torq[i] -= stepTorque;
				if( torq.torq[i] <= 0 )
					isDone = true;
			}

			printf( "Group(%d) Joint(%d)'s torque command = %lf\n", groupIndex, i, torq.torq[i] );
		}
		ret = NMC_GroupSetCmdTorqFC( retDevID, groupIndex, &torq );
		if( ret != ERR_NEXMOTION_SUCCESS )
		{
			printf( "ERROR! NMC_GroupSetCmdTorqFC: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
			goto ERR_SHUTDOWN;
		}

		if( isDone )
			break;
	}

    Sleep( 2000 );

	//get actual torque and print it
	ret = NMC_GroupGetActTorq( retDevID, groupIndex, &torq );
	if( ret != ERR_NEXMOTION_SUCCESS )
	{
		printf( "ERROR! NMC_GroupGetActTorq: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );
		goto ERR_SHUTDOWN;
	}
	for( I32_T i = 0; i < retGroupAxisCount; i++ )
		printf( "Group(%d) Joint(%d)'s actual torque = %lf\n", groupIndex, i, torq.torq[i] );

	Sleep( 1000 );

ERR_SHUTDOWN:
    //=================================================
    //       Disable all single axes and groups
    //=================================================
    ret = NMC_DeviceDisableAll( retDevID );
    if( ret != ERR_NEXMOTION_SUCCESS )
        printf( "ERROR! NMC_DeviceDisableAll: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );

    printf( "\nReady to disable all single axes and groups...\n" );

    Sleep( sleepTime );

    //=================================================
    //              Shutdown device
    //=================================================
    ret = NMC_DeviceShutdown( retDevID );
    if( ret != ERR_NEXMOTION_SUCCESS )
        printf( "ERROR! NMC_DeviceShutdown: (%d)%s.\n", ret, NMC_GetErrorDescription( ret, NULL, 0 ) );

    printf( "\nDevice shutdown succeed.\n" );

    system( "pause" );
    return 0;
}