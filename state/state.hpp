#ifndef STATE_HANDLER_HPP
#define STATE_HANDLER_HPP

#include <stdexcept>
#include <string>
#include <fstream>
#include <algorithm>

#define CTIME 0.2



//State (Velocity) class represents the state or the velocity at the R,Slew and h parameters.
class State
{
    public:

    float R;
    float Slew;
    float h;
    //Different constructors for the State/Velocity class
    State():R(0),Slew(0),h(0){};
    State(float R, float Slew, float h):R(R),Slew(Slew),h(h){};
    State(const State& a):R(a.R),Slew(a.Slew),h(a.h){};

    State operator-(State b)
    {
        State newState;
        newState. R = R - b.R;
        newState.Slew = Slew - b.Slew;
        newState.h = h - b.h;
        return newState;
    }

    State operator-()
    {
        return State(-R,-Slew,-h);
    }

    bool operator>(State b)
    {
        return (R > b.R || Slew > b.Slew || h > b.h);
    }

    //check if all parameters are zero.
    bool isnull()
    {
        return R || Slew || h ? false : true;
    }


};

typedef State Velocity;


//An exception class designed to notify of a deviation of the speed beyond the provided limit.
class velocityAlert : std::runtime_error
{
    public:

    Velocity delta;
    velocityAlert(std::string msg, Velocity diff):runtime_error(msg),delta(diff){};
};



//the State handler holds the current and the previous state, in order to calculate speed.
class stateHandler
{
    State current, previous; //Current and previous States.
    Velocity velocityLimit; //the provided velocity limit of all 3 motion parameters.


    public:
    //Constructor with the velocity limit.
    stateHandler(Velocity velLim);

    //Load state from the provided filePath and update the current/previous member states.
    void loadState(std::string filePath);

    //Detect motion by taking the diffrence between current and previous states and return the delta.
    State detectMotion();

    //Calculate the motion velocity by taking the delta in coordinates and dividing the by cycle time (CTIME = 0.2 as default)
    //If velocity exceeds velocity limit raise an exception of type velocityAlert
    Velocity detectVelocity();

    //Log a state change to the logFile
    void logStateChange();

    //Log a velocity limit violation in the logFile
    void logVelLimit(Velocity delta);

};


#endif // STATE_HANDLER_HPP