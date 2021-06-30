#include "state.hpp"
#include <ios>
#include <stdlib.h>
#include <sstream>

#define logFile "../syracuse.log"


void stateHandler::logStateChange()
{
    std::ofstream  myLog(logFile,std::ios_base::app);
    if(!myLog.is_open()) 
        throw std::runtime_error("Could not open log file in logstateChange");

    myLog << "State changed from: R - " << previous.R << " Slew - " << previous.Slew << " h - " << previous.h;
    myLog << " To state: R - " << current.R << " Slew - " << current.Slew << " h - " << current.h << std::endl;;
    myLog.close();

}

void stateHandler::logVelLimit(Velocity delta)
{
    std::ofstream  myLog(logFile,std::ios_base::app);
    if(!myLog.is_open()) 
        throw std::runtime_error("Could not open log file in logstateChange");

    myLog << "Velocity exceeded, current limit: R - " << velocityLimit.R << " Slew - " << velocityLimit.Slew << " h - " << velocityLimit.h;
    myLog << " Exceeded by: R - " << delta.R << " Slew - " << delta.Slew << " h - " << delta.h << std::endl;;
    myLog.close();
}

void stateHandler::loadState(std::string filePath)
{
     // Create an input filestream
    std::ifstream myFile(filePath);

    // Make sure the file is open
    if(!myFile.is_open()) 
        throw std::runtime_error("Could not open file in function loadState");

    std::string line;
    // Iterate over all lines of the file, process the points and add them to the cloud.
    if(std::getline(myFile, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        State state;
        ss >> state.R;
        ss >> state.Slew;
        ss >> state.h;

        previous = current;
        current = state;

    }

    myFile.close();
    //Replace all commas with whitespace so we can parse better.
    
}


State stateHandler::detectMotion()
{
    return current - previous;
}

Velocity stateHandler::detectVelocity()
{
    State delta = detectMotion();
    Velocity vel;

    vel.R = delta.R/CTIME;
    vel.Slew = delta.Slew/CTIME;
    vel.h = delta.h/CTIME;

    if(vel > velocityLimit) //Check #1 - if velocity is higher than velocity limit
    {
        throw velocityAlert("Velocity limit exceeded!", vel - velocityLimit);
    }else if(-vel > velocityLimit) //check #2 - if negative velocity is higher than velocity limit, this is equal to abs(velocity) > velocityLimit
    {
        throw velocityAlert("Velocity limit exceeded!", -vel - velocityLimit);
    }

    return vel;
}


stateHandler::stateHandler(Velocity velLim):velocityLimit(velLim){
        std::ofstream  myLog(logFile,std::ios_base::trunc);
    };
