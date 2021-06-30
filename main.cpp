#include "pc/pchandler.hpp"
#include "state/state.hpp"
#include <filesystem>

//using std::filesystem::current_path;

#define NUM_FILES 757



void get_batch(stateHandler&, pcHandler&, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& ,const int&);

std::string framePath = "../Dev_DATA/PC/frame";
std::string statePath = "../Dev_DATA/State/state";


int main()
{


    pcHandler pchandler;
    Velocity Lim(10,10,10);
    stateHandler statehandler(Lim);
    char tmp[566];
    getcwd(tmp, 256);
    std::cout << tmp << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> batch;
    int batch_size = 2; //Determines the size of the batch



    for(int i=1; i <= NUM_FILES; i+=batch_size)
    {
        get_batch(statehandler, pchandler, batch, batch_size);

        //Execute actions on the batch.


        //Cluster
        auto clusters = pchandler.cluster(batch,0.3);
        pchandler.viewSide(clusters);
        auto slice = pchandler.sliceZ(batch,0,40); //Slice from Z=0 to Z=40

        pchandler.viewTop(slice);
 
        auto slice2 = pchandler.sliceX(slice, 5,10); //Slice the previous slice from X=5 to X=10
        pchandler.viewTop(slice2);



    }

}





void get_batch(stateHandler& statehandler, pcHandler& pchandler,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& batch, const int& batch_size)
{

    static int batcher = 1;
    //Clear the batch of old/rogue elements.
    batch.erase(batch.begin(),batch.end());

    for(int j=0; j<batch_size; ++j)
    {
        if(batcher+j >= NUM_FILES)
            return; //This is to ensure we dont try to open non-existent files.

        try{
            statehandler.loadState(statePath + std::to_string(batcher+j) + ".csv");

            State changeState;
            changeState = statehandler.detectMotion();
            if(!changeState.isnull()) //Something in the state was changed.
            {
                statehandler.logStateChange();
                Velocity vel = statehandler.detectVelocity();
            }
            //We only load clouds if detectVelocity did not throw an exception.
            auto cloud = pchandler.loadPoints(framePath + std::to_string(batcher+j) + ".csv");
            batch.push_back(cloud);

        }catch(velocityAlert alert) //an exception indicating the velocity limit was exceeded has been raised.
        {
            statehandler.logVelLimit(alert.delta);
        }
    }

    batcher += batch_size;
}