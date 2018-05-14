#include "cook_basis.h"
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "alignment.h"
#include <time.h>
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointP;
typedef pcl::PointCloud<pcl::PointXYZ>  PointC;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalP;
typedef pcl::PointCloud<pcl::Normal> NormalC;

// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{
  clock_t startTime,endTime;
  startTime= clock();
  if (argc < 3)
  {
    printf ("No target PCD file given!\n");
    return (-1);
  }

  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  InputStream(object_templates,argv[1]);

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile (argv[2], *cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 0.8;
  PrePassThrough(cloud,cloud,0,depth_limit);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.0016;//0.0026  //0.003
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  DownSampleCloud(cloud,tempCloud,voxel_grid_size);

  // Remove Outliers from the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  RemoveOutlier(tempCloud,cloud_sor_voxel,50,0.3);
  cloud = cloud_sor_voxel;

  int num_cluster;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  num_cluster = ExtractClusters(cloud,clusters,0.005,60000,500);

  //num_cluster = ExtractClusters(cloud,0.005,60000,500);
  int alig_model_count=0;
  std::vector<PlateInformation> infor;
  std::vector<PlateInformation> infor_all;
  float diff = 0.0;
  float xmin,xmax;
  int index_large_diff;
  for (int i=0;i<num_cluster;i++)
  {
    findXMinMax(clusters[i],xmin,xmax);
    if(xmax-xmin > diff)
    {
      diff = xmax-xmin;
      index_large_diff=i;
    }
  }
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = clusters.begin() + index_large_diff;
  clusters.erase(it);

  for(int i=0;i<num_cluster-1;i++)
  {
    std::cout<< "-------------cluster " << i << "-----------------" << std::endl;
    alig_model_count+=AlignAllModels(clusters[i],object_templates,0.000010,infor);
    infor_all.insert(infor_all.end(),infor.begin(),infor.end());
  }

  std::cout<<"the count of models that have been aligned: " << alig_model_count << std::endl;
  std::cout<<"plate information:" << std::endl;
  for(int i=0;i<alig_model_count;i++)
  {
    std::cout << " " <<std::endl;
    std::cout << "plate NO." << i << std::endl;
    std::cout<<"x: "<< infor_all[i].x << " cm " << std::endl;
    std::cout<<"y: "<< infor_all[i].y << " cm " << std::endl;
    std::cout<<"z: "<< infor_all[i].z << " cm " << std::endl;
    std::cout<<"r: "<< infor_all[i].radius << " cm " << std::endl;
  }
  endTime = clock();
  std::cout << "Total time : " << (double)(endTime-startTime)/ CLOCKS_PER_SEC << "s" << std::endl;
//  pcl::io::savePCDFileBinary ("sence_after_extract.pcd", *cloud);
//  ShowCloud(cloud);
  return (0);
}
