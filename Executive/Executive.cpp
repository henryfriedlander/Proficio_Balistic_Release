#include <stdio.h>
#include "Dragonfly.h"
#include "Dragonfly_config.h"


int main( int argc, char *argv[])
{
	try 
	{
		Dragonfly_Module mod( 0, 0);
		mod.ConnectToMMM();
		mod.Subscribe( MT_FORCE_FEEDBACK);
		mod.Subscribe( MT_POSITION_FEEDBACK);
		mod.Subscribe( MT_TRIAL_STATUS_FEEDBACK);
		mod.Subscribe( MT_RT_POSITION_FEEDBACK);
		mod.Subscribe( MT_TASK_STATE_CONFIG);
		
		// TODO -- here is where one should read from the .config file and send out the next specifications for the next trial instead randomly selecting
		
		MDF_TRIAL_INPUT trial_input_data;
		double targetPositions [4] = {0.10, 0.15, 0.20};
		double forceThresholds [4] = {0.01, 0.02, 0.03, 0.04};
		int directions [3] = {0,1,2};
		int UpOrDownDirs [2] = {-1,1};
		
		double tDist = targetPositions[rand() % 3];
		double tError = 0.03;
		
		int UpOrDown = UpOrDownDirs[rand() % 2];
		trial_input_data.XorYorZ = directions[rand() % 2];
		trial_input_data.forceThreshold = forceThresholds[rand() % 4];
		trial_input_data.targetDistance = tDist;
		trial_input_data.UpOrDown = UpOrDown;
		trial_input_data.targetError = tError;
		CMessage trial_input_M( MT_TRIAL_INPUT);
		trial_input_M.SetData( &trial_input_data, sizeof(trial_input_data));
		mod.SendMessageDF( &trial_input_M);
		cout << "XorYorZ " << trial_input_data.XorYorZ << endl << "force threshold " << trial_input_data.forceThreshold  << endl << "target distance " << trial_input_data.targetDistance << endl << "UpOrDown " << trial_input_data.UpOrDown << endl;
		cout << "Sent out data" << endl;
		
        std::cout << "Consumer running...\n" << std::endl;
        
		while( 1) 
		{
            CMessage M;
			mod.ReadMessage( &M);
			//std::cout << "Received message " << M.msg_type << std::endl;
            
			switch( M.msg_type) {
				case MT_TASK_STATE_CONFIG:
					MDF_TASK_STATE_CONFIG task_state_data;
					M.GetData( &task_state_data);
					if(strcmp(task_state_data.fdbk_display_color, "yellow") == 0){
						// populate messages
						// TODO read from config file to populate messages
						trial_input_data.XorYorZ = directions[rand() % 2];
						trial_input_data.forceThreshold = forceThresholds[rand() % 4];
						trial_input_data.targetDistance = targetPositions[rand() % 3];
						trial_input_data.UpOrDown = UpOrDownDirs[rand() % 2];
						trial_input_data.targetError = 0.03;
						
						cout << "XorYorZ " << trial_input_data.XorYorZ << endl << "force threshold " << trial_input_data.forceThreshold  << endl << "target distance " << trial_input_data.targetDistance << endl << "UpOrDown " << trial_input_data.UpOrDown << endl;
						cout << "Sent out data" << endl;
						
						MDF_TASK_STATE_CONFIG task_state_data;
						task_state_data.target[0] = (trial_input_data.targetDistance - trial_input_data.targetError) * 1240 / 0.2;
						task_state_data.target[1] = (trial_input_data.targetDistance + trial_input_data.targetError) * 1240 / 0.2;
						task_state_data.direction = trial_input_data.UpOrDown + trial_input_data.XorYorZ;
						
						// Send out messages
						CMessage trial_input_M( MT_TRIAL_INPUT);
						trial_input_M.SetData( &trial_input_data, sizeof(trial_input_data));
						mod.SendMessageDF( &trial_input_M);
						
						CMessage task_state_config_M( MT_TASK_STATE_CONFIG);
						task_state_config_M.SetData( &task_state_data, sizeof(task_state_data));
						mod.SendMessageDF( &task_state_config_M);
					}
					break;
			}
		}
	}
	catch( UPipeClosedException &e)
	{
		MyCString s;
		e.AppendTraceToString( s);
		std::cout << "UPipeClosedException: " << s.GetContent() << std::endl;
	}
	catch( UPipeException &e)
	{
		MyCString s;
		e.AppendTraceToString( s);
		std::cout << "UPipeException: " << s.GetContent() << std::endl;
	}
	catch( MyCException &e)
	{
		MyCString s;
		e.AppendTraceToString( s);
		std::cout << "MyCException: " << s.GetContent() << std::endl;
	}
	catch(...)
	{
		MyCString s;
		std::cout << "Unknown Exception!" << std::endl;
	}
	
	std::cout << "Exiting cleanly." << std::endl;
}
