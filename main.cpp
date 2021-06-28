#include "pc/pchandler.hpp"
#include "state/state.hpp"
#include <filesystem>

//using std::filesystem::current_path;

#define NUM_FILES 757

int main()
{
    std::string framePath = "../Dev_DATA/PC/frame";
    std::string statePath = "../Dev_DATA/State/state";

    pcHandler pchandler;
    Velocity Lim(10,10,10);
    stateHandler stateHandler(Lim);
    char tmp[566];
    getcwd(tmp, 256);
    std::cout << tmp << std::endl;
    for(int i=1; i < NUM_FILES; ++i)
    {
        try{
            stateHandler.loadState(statePath + std::to_string(i) + ".csv");

            State changeState;
            changeState = stateHandler.detectMotion();
            if(!changeState.isnull()) //Something in the state was changed.
            {
                stateHandler.logStateChange();
                Velocity vel = stateHandler.detectVelocity();
            }
            //We only load clouds if detectVelocity did not throw an exception.
            auto cloud = pchandler.loadPoints(framePath + std::to_string(i) + ".csv");

            pchandler.viewTop(cloud);

            //Perform actions on cloud


        }catch(velocityAlert alert) //an exception indicating the velocity limit was exceeded has been raised.
        {
            stateHandler.logVelLimit(alert.delta);
        }


    }








}