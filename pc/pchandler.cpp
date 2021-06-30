#include "pchandler.hpp"
#include <new>
#include <sstream>
#include <stdexcept>

enum CameraAngle
{
	XY, TopDown, Side, FPS
};

void pcHandler::initSideView()
{
    int distance = 40;
    if(topViewer)
    {
        topViewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        return;
    }
    pcl::visualization::PCLVisualizer::Ptr viewSide(new pcl::visualization::PCLVisualizer("Side Viewer"));
    
    topViewer = viewSide;

    topViewer->setBackgroundColor (0, 0, 0);
    topViewer->initCameraParameters();
    topViewer->setCameraPosition(0, -distance, 0, 0, 0, 1);

    topViewer->addCoordinateSystem (1.0);
}

void pcHandler::initTopView()
{
    int distance = 40;
    if(topViewer)
    {
        topViewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        return;
    }
        

    pcl::visualization::PCLVisualizer::Ptr viewTop(new pcl::visualization::PCLVisualizer("Top-Down Viewer"));
    topViewer = viewTop;
 

    topViewer->setBackgroundColor (0, 0, 0);
    topViewer->initCameraParameters();
    topViewer->setCameraPosition(0, 0, distance, 1, 0, 1);

    topViewer->addCoordinateSystem (1.0);
}

pcHandler::pcHandler():topViewer(nullptr)
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::loadPoints(std::string filePath)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Make sure point is not null
    if(!cloud) 
        throw std::runtime_error("Could not create new cloud in function loadPoints");
    

    // Create an input filestream
    std::ifstream myFile(filePath);

    // Make sure the file is open
    if(!myFile.is_open()) 
        throw std::runtime_error("Could not open file in function loadPoints");

    std::string line;
    // Iterate over all lines of the file, process the points and add them to the cloud.
    while(std::getline(myFile, line))
    {
        //Replace all commas with whitespace so we can parse better.
        std::replace(line.begin(), line.end(), ',', ' ');

        std::stringstream ss(line);
        float x, y, z;
    
        ss >> x;
        ss >> y;
        ss >> z;

        //std::cout << "X: " << x << "Y: " << y << "Z: " << z << std::endl;
        pcl::PointXYZ point(x,y,z);

        cloud->push_back(point);
        
    }

    myFile.close();

    return cloud;
}




inline void setupView(pcl::visualization::PCLVisualizer::Ptr viewer, std::string name, Color color)
{

  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
  	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);

}

inline void spinView(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    viewer->resetStoppedFlag();

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    //viewer->close();
}

inline void batchView(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, pcl::visualization::PCLVisualizer::Ptr viewer, std::string name)
{
    int i = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud : clouds)
    {
        Color colors[6] = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1), Color(1,0,1)};
        std::string nameId = name + std::to_string(i);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, nameId);
        setupView(viewer, nameId, colors[i%6]);
        ++i;
        
    }

}



void pcHandler::viewTop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    Color color(1,0,0);
    initTopView();
    topViewer->addPointCloud<pcl::PointXYZ>(cloud, "Top view");
    setupView(topViewer, "Top view", color);
    spinView(topViewer);

}


void pcHandler::viewTop(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds)
{
    initTopView();
    batchView(clouds, topViewer, "Batch top viewer");
    spinView(topViewer);

}

void pcHandler::viewSide(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Color color(0,0,1);
    initSideView();
    topViewer->addPointCloud<pcl::PointXYZ>(cloud, "Side view");
    setupView(topViewer, "Side view", color);
    spinView(topViewer);
}


void pcHandler::viewSide(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds)
{
    initSideView();
    batchView(clouds, topViewer, "Batch Side viewer");
    spinView(topViewer);

}



  /*
    We will be using a KD-Tree to store our points so that we can efficiently (O(logn))

  */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcHandler::cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float clusterTolerance, const int minSize, const int maxSize)
{
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusterIndices;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    if(!tree)
        throw std::runtime_error("Could not create kdTree");


    tree->setInputCloud(cloud);

    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    //We have a vector of indices, iterate over them and cluster all bundled indices together in a cloud.
    for (pcl::PointIndices getIndices : clusterIndices) 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    return clusters;

}


//Merge the batch of points into a big cloud and then run the clustering algorithm on it.
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcHandler::cluster(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float clusterTolerance, const int minSize, const int maxSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto cloud : clouds)
    {
        mergedCloud->points.insert(mergedCloud->points.end(), cloud->points.begin(), cloud->points.end());
    }
    mergedCloud->width = mergedCloud->points.size();
    mergedCloud->height = 1;
    mergedCloud->is_dense = true;

    return cluster(mergedCloud, clusterTolerance, minSize, maxSize);
}

//Slice a plane according to a given point.
pcl::PointCloud<pcl::PointXYZ>::Ptr slice(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float& pointMin, const float& pointMax, const char* axis)
{
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());

    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    if(!range_cond || !cloud_projected)
        throw std::runtime_error("Failed to allocate new object in 'slice'");

    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (axis, pcl::ComparisonOps::GT, pointMin)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (axis, pcl::ComparisonOps::LT, pointMax)));


    pcl::ConditionalRemoval<pcl::PointXYZ> proj;

    proj.setInputCloud(cloud);
    proj.setCondition(range_cond);
    proj.filter(*cloud_projected);

    return cloud_projected;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::sliceX(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float& x1, const float& x2)
{
    const float min = x1 < x2 ? x1 : x2;
    const float max = x1 >= x2 ? x1 : x2;

    return slice(cloud,min,max,"x");

}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::sliceY(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float& y1, const float& y2)
{
    const float min = y1 < y2 ? y1 : y2;
    const float max = y1 >= y2 ? y1 : y2;

    return slice(cloud,min,max,"y");
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::sliceZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float& z1, const float& z2)
{
    const float min = z1 < z2 ? z1 : z2;
    const float max = z1 >= z2 ? z1 : z2;

    return slice(cloud,min,max,"z");
}


//Concat the individual slices and return the big slice.
pcl::PointCloud<pcl::PointXYZ>::Ptr sliceBatch(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float pointMin, const float pointMax,const char* axis)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr bigSlice(new pcl::PointCloud<pcl::PointXYZ>);

    for(auto cloud : clouds)
    {
        *bigSlice += *slice(cloud,pointMin, pointMax, axis);
    }
    return bigSlice;

}
pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::sliceX(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float& x1, const float& x2)
{
    const float min = x1 < x2 ? x1 : x2;
    const float max = x1 >= x2 ? x1 : x2;

    return sliceBatch(clouds,min,max,"x");

}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::sliceY(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float& y1, const float& y2)
{

    const float min = y1 < y2 ? y1 : y2;
    const float max = y1 >= y2 ? y1 : y2;

    return sliceBatch(clouds,min,max,"y");
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pcHandler::sliceZ(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float& z1, const float& z2)
{
    const float min = z1 < z2 ? z1 : z2;
    const float max = z1 >= z2 ? z1 : z2;

    return sliceBatch(clouds,min,max,"z");
}


