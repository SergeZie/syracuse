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

            auto clusters = pchandler.cluster(cloud,0.5); //Provide the cloud, and a distance of 0.5 to check for clusters

            //View the clusters
            pchandler.viewTop(clusters);


            //Slice at X=11.0
            auto slice = pchandler.sliceX(cloud,11.0);

            pchandler.viewSide(slice);
            //Slice at Y=17.0
            slice = pchandler.sliceY(cloud, 17.0);

            pchandler.viewSide(slice);
            //Slice at Z=7.5
            slice = pchandler.sliceZ(cloud, 7.5);

            pchandler.viewTop(slice);

            //Create a batch out of a single cloud point (divide it into sub-clouds):

            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> batch;

            pcl::PointCloud<pcl::PointXYZ>::Ptr one(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr two(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr three(new pcl::PointCloud<pcl::PointXYZ>);
            //Divide our cloud into a batch of three clouds
            for(int i=0; i<cloud->points.size(); ++i)
            {
                switch(i % 3)
                {
                    case 0:
                        one->points.push_back(cloud->points[i]);
                        break;
                    case 1:
                        two->points.push_back(cloud->points[i]);
                        break;
                    case 2:
                        three->points.push_back(cloud->points[i]);
                        break;
                    default:
                        break;
                }
            }

            std::cout << "Done populating the batch" << std::endl;
            batch.push_back(one);
            batch.push_back(two);
            batch.push_back(three);
            //Cluster the batch
            auto batch_clusters = pchandler.cluster(batch);
            //View the clusters
            pchandler.viewTop(batch_clusters);
            //Slice the batch according to x=12.0
            auto batch_slice = pchandler.sliceX(batch,12.0);
            //View the batch slice
            pchandler.viewTop(batch_slice);


            


        }catch(velocityAlert alert) //an exception indicating the velocity limit was exceeded has been raised.
        {
            stateHandler.logVelLimit(alert.delta);
        }


    }








}