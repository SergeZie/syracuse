#ifndef POINT_CLOUD_HANDLER_HPP
#define POINT_CLOUD_HANDLER_HPP


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

struct Color
{

	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};

//Since we are given data with XYZ points only we will be using PointXYZ and not a template.
class pcHandler
{

    pcl::visualization::PCLVisualizer::Ptr topViewer;
    pcl::visualization::PCLVisualizer::Ptr sideViewer;
    
    void initSideView();
    void initTopView();


    public:


    pcHandler();


    /* function cluster
    @in: filepath - the file path pointing to the next point data file to load.
    @out: a pointer to a point cloud that was loaded.

    The cluster function accepts a point cloud, runs a eucledian distance clustering algorithm and 
    returns a vector of point clouds which contains the clusters.
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPoints(std::string filePath);


    /* function cluster
    @in:cloud - pointer to a point cloud of type PointXYZ
    @in: clusterTolerance - the distance around which we consider a point to be part of the same cluster.
    @in: minSize - minimum Cluster size.
    @in: maxSize - Maximum cluster Size.
    @out: a vector containing point clouds which represent the different clusters.

    The cluster function accepts a point cloud, runs a eucledian distance clustering algorithm and 
    returns a vector of point clouds which contains the clusters.
    */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float clusterTolerance = .25, const int minSize = 5, const int maxSize = 500);


    /* function cluster (For batches)
    @in:cloud - vector of pointers to point clouds of type PointXYZ
    @in: clusterTolerance - the distance around which we consider a point to be part of the same cluster.
    @in: minSize - minimum Cluster size.
    @in: maxSize - Maximum cluster Size.
    @out: a vector containing point clouds which represent the different clusters.

    The cluster function accepts a point cloud, runs a eucledian distance clustering algorithm and 
    returns a vector of point clouds which contains the clusters.
    */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float clusterTolerance = .25, const int minSize = 5, const int maxSize = 500);
    
    
    
    /*
    function viewTop
    @in:cloud - point cloud of type PointXYZ

    Render the given pointcloud in a Top view.
    */
    void viewTop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /*
    function viewTop (for batches)
    @in:cloud - vector of point clouds of type PointXYZ

    Render the given pointclouds in a Top view.
    */
    void viewTop(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);

    /*
    function viewSide
    @in:cloud - point cloud of type PointXYZ

    Render the given pointcloud in a Side view.
    */
    void viewSide(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*
    function viewSide (for batches)
    @in:cloud - vector of pointers to point clouds of type PointXYZ

    Render the given pointclouds in a Side view.
    */
    void viewSide(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);



    /*
    function sliceX 
    @in:cloud - pointer to a point cloud of type PointXYZ
    @in: x - the value of x around which to slice.
    @out: a point cloud of sliced points.
    
    Slice a YZ plane at the given X point.
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr sliceX(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float x);

    /*
    function sliceY 
    @in:cloud - pointer to a point cloud of type PointXYZ
    @in: y - the value of y around which to slice.
    @out: a point cloud of sliced points.
    
    Slice a XZ plane at the given Y point.
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr sliceY(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float y);


    /*
    function sliceZ 
    @in:cloud - pointer to a point cloud of type PointXYZ
    @in: z - the value of z around which to slice.
    @out: a point cloud of sliced points.
    
    Slice a XZ plane at the given Z point.
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr sliceZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float z);


    pcl::PointCloud<pcl::PointXYZ>::Ptr sliceX(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float x);


    pcl::PointCloud<pcl::PointXYZ>::Ptr sliceY(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float y);


    pcl::PointCloud<pcl::PointXYZ>::Ptr sliceZ(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds, const float z);



};



#endif // POINT_CLOUD_HANDLER_HPP