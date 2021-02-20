#include <iostream>
#include <io.h>
#include <time.h>
#include <string>
#include <random>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/registration/icp.h>
#include <pcl/point_representation.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <pcl/console/time.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>

using namespace std;
using namespace pcl;

float cloudCurvature[40000];
int cloudSortInd[40000];
int cloudNeighborPicked[40000];
int cloudLabel[40000];

void getPCDfilelist(string path, vector<string>& files, vector<double>& time_list);
bool extractGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original, pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_ground, pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_remained, const float in_distance_thre, const float lowerlimit, const float higherlimit);
bool extractselfcar(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original, pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_car, pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_remaining, const float car_width, const float car_length, const float car_height);
bool setregion(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original, pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_remaining, const float xlim, const float ylim, const float zlim);
bool performEuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original, pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_out, std::vector<pcl::PointIndices>& cluster_indices, const float dist_thred, const int MinClusterSize, const int MaxClusterSize);
bool performRegionGrowingClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original, std::vector<pcl::PointIndices>& cluster_indices, const int NumberOfNeighbours, const int MinClusterSize, const int MaxClusterSize);
float pairAlign(const pcl::PointCloud<PointXYZ>::Ptr cloud_src, const pcl::PointCloud<PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<PointXYZ>::Ptr output, Eigen::Matrix4f& final_transform, bool downsample, float gridsize);
void merge_left_right(const pcl::PointCloud<PointXYZ>::Ptr cloud_left, const pcl::PointCloud<PointXYZ>::Ptr cloud_right, const pcl::PointCloud<PointXYZ>::Ptr cloud_out);
void get_2d_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_in, pcl::PointCloud<PointXYZ>::Ptr& pt_out, int N_Scan, float visible_dist);

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::visualization::Camera camera;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform = Eigen::Matrix4f::Identity(), GlobalTransform_temp = Eigen::Matrix4f::Identity(), pairTransform1 = Eigen::Matrix4f::Identity(), pairTransform2 = Eigen::Matrix4f::Identity();

void lidar_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_in, pcl::PointCloud<PointXYZI>::Ptr& cornerPointsSharp,
    pcl::PointCloud<PointXYZI>::Ptr& cornerPointsLessSharp,
    pcl::PointCloud<PointXYZI>::Ptr& surfPointsFlat,
    pcl::PointCloud<PointXYZI>::Ptr& surfPointsLessFlat, float visible_dist);


int main() {
    string path1 = "D:\\Thesis\\EECE5554\\Final Project\\pcd1";
    string path2 = "D:\\Thesis\\EECE5554\\Final Project\\pcd2";
    vector<string> filelist1;
    vector<string> filelist2;
    vector<double> time_list1, time_list2;
    getPCDfilelist(path1, filelist1, time_list1);
    getPCDfilelist(path2, filelist2, time_list2);
    cout << filelist1.size() << endl;
    cout << filelist2.size() << endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr selfcar(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloudremaining(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_ptcloudremaining(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_ptcloudremaining_afterextraction(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_right(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_right(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
    
    static int itnum = 0;
    static int iterstep = 6;
    static int maxrange = 45;
    static float gridsize = 0.4f;
    static float dist_thred = 0.6f;
    static int NumberOfNeighbours = 30;
    static double density_thred = 12;
    static double previous_time = 0, time = 0, dt = 0;

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    Eigen::Matrix4f accmTrans = init_guess;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3f current_position(0, 0, 0);
    Eigen::Vector3f previous_position(0, 0, 0);
    double velocity = 0;
    vector<double> vel_list(0);
    pcl::PointCloud<PointXYZ>::Ptr result(new pcl::PointCloud<PointXYZ>), source, target;

    
    vtkObject::GlobalWarningDisplayOff();

    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 200, 0, 1, 0);
    //reader.read<pcl::PointXYZ>(filelist1[0], *scene);
    for (int i = iterstep; i < filelist1.size(); i += iterstep) {

        reader.read<pcl::PointXYZ>(filelist1[i - iterstep], *src1);
        reader.read<pcl::PointXYZ>(filelist2[i - iterstep], *src2);
        reader.read<pcl::PointXYZ>(filelist1[i], *tgt1);
        reader.read<pcl::PointXYZ>(filelist2[i], *tgt2);

        *src = *src1;
        *tgt = *tgt1;
        //merge_left_right(src1, src2, src);
        //merge_left_right(tgt1, tgt2, tgt);
        //get_2d_scan(src1, src, 12, 60);
        //get_2d_scan(tgt1, tgt, 12, 60);

        pcl::PointCloud<PointXYZI>::Ptr cornerPointsSharp_src(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr cornerPointsLessSharp_src(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr surfPointsFlat_src(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlat_src(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::PointCloud<PointXYZI>::Ptr cornerPointsSharp_tgt(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr cornerPointsLessSharp_tgt(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr surfPointsFlat_tgt(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlat_tgt(new pcl::PointCloud<pcl::PointXYZI>());

        previous_time = time_list1[i - iterstep];
        time = time_list1[i];
        dt = time - previous_time;
        //viewer->addPointCloud<pcl::PointXYZ>(src);
        //viewer->spinOnce();
        //setregion(src1, src1, maxrange, maxrange, 5);
        //setregion(tgt1, tgt1, maxrange, maxrange, 5);
        extractGroundPlane(src, ground, src, 0.5, -2.2, -1);
        extractGroundPlane(tgt, ground, tgt, 0.5, -2.2, -1);
        //cout << (*src1).size() << endl;
        //extractselfcar(src1, selfcar, src1, 3, 5, 3);
        //extractselfcar(tgt1, selfcar, tgt1, 3, 5, 3);
        if (i > 1) {
            viewer->setBackgroundColor(0, 0, 0);
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            //viewer->removeShape("sence_txt");
        }

        lidar_registration(src, cornerPointsSharp_src, cornerPointsLessSharp_src, surfPointsFlat_src, surfPointsLessFlat_src,30);
        lidar_registration(tgt, cornerPointsSharp_tgt, cornerPointsLessSharp_tgt, surfPointsFlat_tgt, surfPointsLessFlat_tgt,30);

        pcl::PointCloud<PointXYZ>::Ptr temp(new pcl::PointCloud<PointXYZ>);

        
       
        std::vector<pcl::PointIndices> cluster_indices1;
        std::vector<pcl::PointIndices> cluster_indices2;

        //performEuclideanClusterExtraction(src1, src1, cluster_indices1, dist_thred, 200, 2000);

        pcl::PointCloud<pcl::PointXYZ>::Ptr surfPointsFlatXYZ_src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfPointsLessFlatXYZ_src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cornerPointsSharpXYZ_src(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cornerPointsLessXYZ_src(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*surfPointsFlat_src, *surfPointsFlatXYZ_src);
        copyPointCloud(*surfPointsLessFlat_src, *surfPointsLessFlatXYZ_src);
        copyPointCloud(*cornerPointsSharp_src, *cornerPointsLessXYZ_src);
        copyPointCloud(*cornerPointsLessSharp_src, *cornerPointsSharpXYZ_src);

        pcl::PointCloud<pcl::PointXYZ>::Ptr surfPointsFlatXYZ_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr surfPointsLessFlatXYZ_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cornerPointsSharpXYZ_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cornerPointsLessXYZ_tgt(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*surfPointsFlat_tgt, *surfPointsFlatXYZ_tgt);
        copyPointCloud(*surfPointsLessFlat_tgt, *surfPointsLessFlatXYZ_tgt);
        copyPointCloud(*cornerPointsSharp_tgt, *cornerPointsLessXYZ_tgt);
        copyPointCloud(*cornerPointsLessSharp_tgt, *cornerPointsSharpXYZ_tgt);
        float score1, score2;
        score1=pairAlign(cornerPointsSharpXYZ_src, cornerPointsSharpXYZ_tgt, temp, pairTransform1, false, 0.2);
        score2=pairAlign(surfPointsFlatXYZ_src, surfPointsFlatXYZ_tgt, temp, pairTransform2, false, 0.2);

        Eigen::Matrix3f m1= pairTransform1.block(0,0,3,3);
        Eigen::Vector3f euler1 = m1.eulerAngles(2, 1, 0);
        Eigen::Matrix3f m2 = pairTransform2.block(0, 0, 3, 3);
        Eigen::Vector3f euler2 = m2.eulerAngles(2, 1, 0);
        float r=0;
        if (score1 < score2)r = 0;
        else r = 1;

        float yaw1 = euler1(0),yaw2=euler2(0),yaw;
        if(yaw1 > M_PI/2) yaw1+=-M_PI;
        else if (yaw1 <- M_PI/2)yaw1 += M_PI;
        if (yaw2 > M_PI / 2) yaw2 += -M_PI;
        else if (yaw2 < -M_PI / 2)yaw2 += M_PI;
        yaw = (1 - r) * yaw1 + r * yaw2;
        cout <<"yaw\t:"<< yaw << "score1\t:" << score1 << "score2\t:" << score2 << endl;
        Eigen::Matrix3f m;
        m << cos(-yaw), sin(-yaw), 0, -sin(-yaw), cos(-yaw), 0, 0, 0, 1;
        pairTransform.block(0, 0, 3, 3) = m;
        pairTransform(0, 3) = (1 - r) * pairTransform1(0, 3) + r * pairTransform2(0, 3);
        pairTransform(1, 3) = (1 - r) * pairTransform1(1, 3) + r * pairTransform2(1, 3);
        pairTransform(2,3) = 0;
        cout << pairTransform << endl;
        GlobalTransform *= pairTransform.inverse();
        cout << GlobalTransform << endl;
        pcl::transformPointCloud(*tgt, *result, GlobalTransform);

        if (i % (10 * iterstep) == 0) {
            *scene += *result;
        }
            //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h1(tgt, 255, 255, 255);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h2(surfPointsFlatXYZ_tgt, 0, 0, 255);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h3(surfPointsLessFlatXYZ_tgt, 0, 155, 155);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h4(cornerPointsLessXYZ_tgt, 155, 155, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h5(cornerPointsSharpXYZ_tgt, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h6(scene, 255, 255, 255);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h7(result, 255, 0, 0);

            int v1(1);
            int v2(2);
            viewer->addCoordinateSystem(5);
            //viewer->createViewPort(0, 0, 1, 1.0, v1);
            viewer->createViewPort(0, 0, 0.5, 1.0, v1);
            viewer->setBackgroundColor(0, 0, 0, v1);
            viewer->addText("current_view", 10, 10, "txt1", v1);

            viewer->createViewPort(0.5, 0, 1.0, 1.0, v2);
            viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
            viewer->addText("whole_scene", 10, 10, "txt2", v2);

            //viewer->addPointCloud<pcl::PointXYZ>(tgt, cloud_in_color_h1, "tgt", v1);
            //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tgt", v1);
            viewer->addPointCloud<pcl::PointXYZ>(surfPointsFlatXYZ_tgt, cloud_in_color_h2, "1", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "1", v1);
            viewer->addPointCloud<pcl::PointXYZ>(surfPointsLessFlatXYZ_tgt, cloud_in_color_h3, "2", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "2", v1);
            viewer->addPointCloud<pcl::PointXYZ>(cornerPointsLessXYZ_tgt, cloud_in_color_h4, "3", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "3", v1);
            viewer->addPointCloud<pcl::PointXYZ>(cornerPointsSharpXYZ_tgt, cloud_in_color_h5, "4", v1);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "4", v1);
            

            viewer->addPointCloud<pcl::PointXYZ>(scene, cloud_in_color_h6, "scene", v2);
            viewer->addPointCloud<pcl::PointXYZ>(result, cloud_in_color_h7, "result", v2);

            //viewer->addPointCloud<pcl::PointXYZ>(scene, cloud_in_color_h2, "tgt_v2");
            viewer->spinOnce();
    }
    return (0);
}


bool setregion(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_remaining, const float xlim = 0, const float ylim = 0., const float zlim = 0.) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-xlim, -ylim, -2.5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(xlim, ylim, -2.5 + zlim, 1.0));
    boxFilter.setInputCloud(pt_original);
    boxFilter.setNegative(false);
    boxFilter.filter(*pt_remaining);
    return true;
}


void getPCDfilelist(string path, vector<string>& files, vector<double>& time_list)
{
    intptr_t hFile = 0;
    struct _finddata_t fileinfo;
    string p;
    double time0 = 0;
    double previous_time = 0;

    int n = 0;
    if ((hFile = _findfirst(p.assign(path).append("\\*.*").c_str(), &fileinfo)) != -1) {
        do {
            if ((strcmp(fileinfo.name, ".") != 0) && (strcmp(fileinfo.name, "..") != 0))
            {
                if (n == 0) {
                    files.push_back(path + "\\" + fileinfo.name);
                    stringstream ss;
                    string name = fileinfo.name;
                    ss << name.substr(0, name.length() - 4);
                    ss >> time0;
                    //cout << "name=" << fileinfo.name <<"\t"<< name.substr(0, name.length() - 3) << "   time = " <<  time0 << endl;
                    time_list.push_back(time0);
                }
                else {
                    files.push_back(path + "\\" + fileinfo.name);
                    stringstream ss;
                    string name = fileinfo.name;
                    ss << name.substr(0, name.length() - 4);
                    ss >> previous_time;
                    //cout << "name=" << fileinfo.name << "\t" << name.substr(0, name.length() - 3) <<"   time = " << previous_time-time0 << endl;
                    time_list.push_back(previous_time - time0);
                }
                n++;
            }
        } while (_findnext(hFile, &fileinfo) == 0);
    }
}

bool extractselfcar(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_car,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_remaining,
    const float car_width = 0., const float car_length = 0., const float car_height = 0.) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::CropBox<pcl::PointXYZ> boxFilter;    boxFilter.setMin(Eigen::Vector4f(-0.5 * car_length, -0.5 * car_width, -2.5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(0.5 * car_length, 0.5 * car_width, -2.5 + car_height, 1.0));
    boxFilter.setInputCloud(pt_original);
    boxFilter.setNegative(false);
    boxFilter.filter(*pt_car);
    boxFilter.setNegative(true);
    boxFilter.filter(*pt_remaining);
    return true;
}

bool extractGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_ground,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_remaining,
    const float in_distance_thre = 0.2, const float lowerlimit = 0., const float higherlimit = 0.)
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pttemp(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(pt_original);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(lowerlimit, higherlimit);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud1);
    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud2);
    //plane segmentation
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    //seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(in_distance_thre);
    seg.setInputCloud(cloud1);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud1);
    extract.setNegative(false);
    extract.setIndices(inliers);
    extract.filter(*pt_ground);
    extract.setNegative(true);
    extract.filter(*pt_remaining);
    *pt_remaining += *cloud2;
    return true;
}

bool performEuclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_original,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_out,
    std::vector<pcl::PointIndices>& cluster_indices,
    const float dist_thred = 0.2, const int MinClusterSize = 200, const int MaxClusterSize = 25000) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pt_original);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(dist_thred);
    ec.setMinClusterSize(MinClusterSize);
    ec.setMaxClusterSize(MaxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pt_original);
    ec.extract(cluster_indices);

    int num = cluster_indices.size();
    default_random_engine e;
    uniform_real_distribution<float> u(0, 255);
    int j = 0;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    //if (itnum > 0) { viewer->removeAllPointClouds(); viewer->removeAllShapes(); }
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloudremaining(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        min_point_AABB.x = std::numeric_limits <float>::max();
        min_point_AABB.y = std::numeric_limits <float>::max();
        min_point_AABB.z = std::numeric_limits <float>::max();
        max_point_AABB.x = -std::numeric_limits <float>::max();
        max_point_AABB.y = -std::numeric_limits <float>::max();
        max_point_AABB.z = -std::numeric_limits <float>::max();
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(pt_original->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            if (pt_original->points[*pit].x <= min_point_AABB.x) min_point_AABB.x = pt_original->points[*pit].x;
            if (pt_original->points[*pit].y <= min_point_AABB.y) min_point_AABB.y = pt_original->points[*pit].y;
            if (pt_original->points[*pit].z <= min_point_AABB.z) min_point_AABB.z = pt_original->points[*pit].z;
            if (pt_original->points[*pit].x >= max_point_AABB.x) max_point_AABB.x = pt_original->points[*pit].x;
            if (pt_original->points[*pit].y >= max_point_AABB.y) max_point_AABB.y = pt_original->points[*pit].y;
            if (pt_original->points[*pit].z >= max_point_AABB.z) max_point_AABB.z = pt_original->points[*pit].z;
        }
        double volume = abs((min_point_AABB.x - max_point_AABB.x) * (min_point_AABB.y - max_point_AABB.y) * (min_point_AABB.z - max_point_AABB.z));
        double density = cloud_cluster->points.size() / volume;
        //cout << density << endl;
        if (abs(min_point_AABB.x - max_point_AABB.x) < 6 && abs(min_point_AABB.y - max_point_AABB.y) < 6 &&  max_point_AABB.z < 2) {
            string id = std::to_string(e());
            viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h1(cloud_cluster, u(e), u(e), u(e));//赋予显示点云的颜色
        //viewer->addPointCloud(cloud_cluster, cloud_in_color_h1, to_string(j));
        j++;
        *ptcloudremaining += *cloud_cluster;
    }
    *pt_out = *ptcloudremaining;

    return true;
}

void lidar_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& pt_in, pcl::PointCloud<PointXYZI>::Ptr& cornerPointsSharp,
    pcl::PointCloud<PointXYZI>::Ptr& cornerPointsLessSharp,
    pcl::PointCloud<PointXYZI>::Ptr& surfPointsFlat,
    pcl::PointCloud<PointXYZI>::Ptr& surfPointsLessFlat,
    float visible_dist = 100)
{

    int ptsize = pt_in->points.size();
    float start_angle= atan2(pt_in->points[0].x, pt_in->points[0].y);
    float hori_angle = 0, scanID = 0;
    int count = ptsize;
    int N_SCANS = 16;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    PointXYZI point;
    std::vector<pcl::PointCloud<PointXYZI> > laserCloudScans(N_SCANS);

    int StartNSCAN = 0;
    for (int i = 0; i < ptsize; i++) {
        double x = pt_in->points[i].x;
        double y = pt_in->points[i].y;
        double z = pt_in->points[i].z;
        point.x = pt_in->points[i].x;
        point.y = pt_in->points[i].y;
        point.z = pt_in->points[i].z;
        double r = sqrt(x * x + y * y);
        double d = sqrt(x * x + y * y + z * z);
        float ori = (atan2(x, y)- start_angle);
        float ori_I = ori / 2 / M_PI;
        scanID = (round(atan2(z, r) * 180 / M_PI) + 15)/2;
        point.intensity = scanID + ori_I;
        if (scanID >= StartNSCAN && scanID < N_SCANS && scanID == round(scanID) && d <= visible_dist) {
            laserCloudScans[scanID- StartNSCAN].push_back(point);
            //cout << scanID << endl;
        }
        //cout << "surfPointsFlat" << point.x << "\t" << point.y << "\t" << point.z << "\t" << point.intensity << endl;
    }
    N_SCANS -= StartNSCAN;

    pcl::PCDWriter writer;
    pcl::PointCloud<PointXYZI>::Ptr laserCloud(new pcl::PointCloud<PointXYZI>());
    for (int i = 0; i < N_SCANS; i++) {
        *laserCloud += laserCloudScans[i];
        //std::stringstream ss;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudScans_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        //copyPointCloud(laserCloudScans[i], *laserCloudScans_xyz);
        //cout << laserCloudScans[i].size() << endl;
        //ss << "SCAN_" << i << ".pcd";
        //writer.write<pcl::PointXYZ>(ss.str(), *laserCloudScans_xyz, false);
        //if (i == 7)viewer->addPointCloud(laserCloudScans_xyz);
    }
    ptsize = laserCloud->points.size();
    for (int i = 5; i < ptsize - 5; i++) {
        // 对所有的激光点一个一个求出在该点前后5个点(10点)的偏差，作为cloudCurvature点云数据的曲率
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
            + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
            + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
            + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
            + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
            + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
            + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
            + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
            + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
            + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
            + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
            + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
            + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
            + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
            + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
            + laserCloud->points[i + 5].z;
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        int scanCount = -1;
        if (int(laserCloud->points[i].intensity) != scanCount) {
            scanCount = int(laserCloud->points[i].intensity);

            if (scanCount > 0 && scanCount < N_SCANS) {
                scanStartInd[scanCount] = i + 5;
                scanEndInd[scanCount - 1] = i - 5;
            }
        }
    }
    scanStartInd[0] = 5;
    scanEndInd.back() = ptsize - 5;
    for (int i = 5; i < ptsize - 6; i++) {
        float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
        float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
        float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.1) {

            float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                laserCloud->points[i].y * laserCloud->points[i].y +
                laserCloud->points[i].z * laserCloud->points[i].z);

            float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

            if (depth1 > depth2) {
                diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
                diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
                diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            }
            else {
                diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
        float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
        float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        float dis = laserCloud->points[i].x * laserCloud->points[i].x
            + laserCloud->points[i].y * laserCloud->points[i].y
            + laserCloud->points[i].z * laserCloud->points[i].z;

        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
            cloudNeighborPicked[i] = 1;
        }
    }

    for (int i = 0; i < N_SCANS; i++) {
        pcl::PointCloud<PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointXYZI>);
        for (int j = 0; j < 6; j++) {
            int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;
            int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

            for (int k = sp + 1; k <= ep; k++) {
                for (int l = k; l >= sp + 1; l--) {
                    if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
                        int temp = cloudSortInd[l - 1];
                        cloudSortInd[l - 1] = cloudSortInd[l];
                        cloudSortInd[l] = temp;
                    }
                }
            }

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1) {

                    largestPickedNum++;
                    if (largestPickedNum <= 2) {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp->push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20) {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    }
                    else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        float diffX = laserCloud->points[ind + l].x
                            - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y
                            - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                            - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x
                            - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y
                            - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                            - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1) {

                    cloudLabel[ind] = -1;
                    surfPointsFlat->push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        float diffX = laserCloud->points[ind + l].x
                            - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y
                            - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                            - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x
                            - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y
                            - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                            - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointXYZI> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        *surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    //cout<<"cornerPointsSharp" <<cornerPointsSharp->size()<<endl;
    //cout << "cornerPointsLessSharp" << cornerPointsLessSharp->size() << endl;
    //cout << "cornerPointsLessSharp" << surfPointsFlat->size() << endl;
    //cout << "surfPointsLessFlat" << surfPointsLessFlat->points[0].intensity << endl;

}


float pairAlign(const pcl::PointCloud<PointXYZ>::Ptr cloud_src, const pcl::PointCloud<PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<PointXYZ>::Ptr output, Eigen::Matrix4f& final_transform, bool downsample = false, float gridsize = 0.1)
{
    pcl::PointCloud<PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<PointNormal>);
    pcl::PointCloud<PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<PointNormal>);
    pcl::NormalEstimation<PointXYZ, PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;

    src = cloud_src;
    tgt = cloud_tgt;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(1);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-9);
    icp.setRANSACOutlierRejectionThreshold(0.6);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(*output, final_transform);
    //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    final_transform = icp.getFinalTransformation();
    return icp.getFitnessScore();
}

void merge_left_right(const pcl::PointCloud<PointXYZ>::Ptr cloud_left, const pcl::PointCloud<PointXYZ>::Ptr cloud_right, const pcl::PointCloud<PointXYZ>::Ptr cloud_out) {
    Eigen::Matrix4f tform_cam;
    tform_cam<<1, 0, 0, 0, 0, 1, 0, -1.32, 0, 0, 1, 0, 0, 0, 0, 1;
    //cout << tform_cam << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_align(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_right, *pt_align, tform_cam);
    *cloud_out = *cloud_left + *pt_align;
}

void get_2d_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr & pt_in, pcl::PointCloud<PointXYZ>::Ptr & pt_out,int N_Scan=7, float visible_dist = 100){
        int ptsize = pt_in->points.size();
        float start_angle = atan2(pt_in->points[0].x, pt_in->points[0].y);
        float hori_angle = 0, scanID = 0;
        int count = ptsize;
        int N_SCANS = 16;
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);
        PointXYZI point;
        std::vector<pcl::PointCloud<PointXYZI> > laserCloudScans(N_SCANS);

        for (int i = 0; i < ptsize; i++) {
            double x = pt_in->points[i].x;
            double y = pt_in->points[i].y;
            double z = pt_in->points[i].z;
            point.x = pt_in->points[i].x;
            point.y = pt_in->points[i].y;
            point.z = pt_in->points[i].z;
            double r = sqrt(x * x + y * y);
            double d = sqrt(x * x + y * y + z * z);
            float ori = (atan2(x, y) - start_angle);
            float ori_I = ori / 2 / M_PI;
            scanID = (round(atan2(z, r) * 180 / M_PI) + 15) / 2;
            point.intensity = scanID + ori_I;
            if (scanID >= 0 && scanID < N_SCANS && scanID == round(scanID) && d <= visible_dist) {
                laserCloudScans[scanID].push_back(point);
            }
        }
        pcl::PCDWriter writer;
        pcl::PointCloud<PointXYZ>::Ptr laserCloudScans_xyz(new pcl::PointCloud<PointXYZ>());
        copyPointCloud(laserCloudScans[N_Scan], *pt_out);
}

