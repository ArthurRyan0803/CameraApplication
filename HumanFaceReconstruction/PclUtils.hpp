#pragma once

#include <Eigen/Eigen>

#include <pcl/pcl_base.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/surface/mls.h>

#include <opencv2/calib3d.hpp>

#include <Utils.hpp>
#include <QDebug>
#include <unordered_set>
#include <mutex>
#include <thread>


template<typename PointT>
class PclPointUtils
{
public:
    static void translatePCLPoint(PointT& point, const Eigen::Vector3f& translation)
    {
        point.x += translation[0]; point.y += translation[1]; point.z += translation[2];
    }
};


template<typename PointT>
class PclPointCloudUtils
{
public:
    PclPointCloudUtils() = delete;
    ~PclPointCloudUtils() = delete;

    static Eigen::Vector3f cloudsCentroid(const std::vector<std::shared_ptr<pcl::PointCloud<PointT>>>& clouds_ptrs)
    {
        Eigen::Vector3f accumulator {0, 0, 0};
        float points_count = 0;
        for(auto& cloud_ptr: clouds_ptrs)
        {
            for(auto& point: cloud_ptr->points)
            {
                accumulator[0] += point.x;
                accumulator[1] += point.y;
                accumulator[2] += point.z;
            }
            points_count += static_cast<float>(cloud_ptr->size());
        }
        auto center = accumulator / points_count;
        return center;
    }

    static void translatePointCloud(const std::shared_ptr<pcl::PointCloud<PointT>>& cloud_ptr, const Eigen::Vector3f& vector)
    {
        for(auto& point: cloud_ptr->points)
        {
            PclPointUtils<PointT>::translatePCLPoint(point, vector);
        }
    }

    static void pclTransformCloud(
        std::shared_ptr<pcl::PointCloud<PointT>>& cloud_in, std::shared_ptr<pcl::PointCloud<PointT>>& cloud_out,
        const cv::Mat& rmat, const cv::Mat& tvec
    )
    {
	    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
        transform.block<3, 3>(0, 0) = Utils::matrixCast<double, float, 3>(rmat);
        transform.block<3, 1>(0, 3) = Utils::vectorCast<double, float, 3>(tvec);
    
        pcl::transformPointCloud<PointT>(*cloud_in, *cloud_out, transform);
    }

    static void voxelDownsample(
        const std::shared_ptr<pcl::PointCloud<PointT>>& cloud_in, pcl::PointCloud<PointT>& cloud_out,
        float leaf_x, float leaf_y, float leaf_z
    )
    {
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setLeafSize(leaf_x, leaf_y, leaf_z);
        voxel_filter.setInputCloud(cloud_in);
        voxel_filter.filter(cloud_out);
    }

    static void statisticalOutliersRemove(
        const std::shared_ptr<pcl::PointCloud<PointT>>& cloud_in, pcl::PointCloud<PointT>& cloud_out,
        int neighbor_num, double stddev
    )
    {
	    pcl::StatisticalOutlierRemoval<PointT> sor;
	    sor.setInputCloud(cloud_in);
	    sor.setMeanK(neighbor_num);
	    sor.setStddevMulThresh(stddev);
	    sor.filter(cloud_out);
    }

    static void cloudsIntersection(
        const std::shared_ptr<pcl::PointCloud<PointT>>& cloud1, 
        const std::shared_ptr<pcl::PointCloud<PointT>>& cloud2, 
        pcl::PointCloud<PointT>& intersection_cloud1,
        pcl::PointCloud<PointT>& intersection_cloud2,
        float dist_threshold, float& max_distance
    )
    {
        pcl::KdTreeFLANN<PointT> search_tree;
        search_tree.setInputCloud(cloud2);

        std::vector<int> indices_temp(1);
        std::vector<float> distances_temp(1);
        std::unordered_set<int> cloud2_indices_set;

        intersection_cloud1.clear();
        intersection_cloud2.clear();

        for(int i=0; i < cloud1->size(); i++)
        {
            auto& point = cloud1->points[i];
            auto count = search_tree.nearestKSearch(point, 1, indices_temp, distances_temp);
            if(count == 1 && distances_temp[0] <= dist_threshold)
            {
                intersection_cloud1.push_back(point);
                intersection_cloud2.push_back(cloud2->points.at(indices_temp[0]));
                max_distance = std::max({max_distance, distances_temp[0]});
            }
        }
    }

    static void getCloudsCorrespondences(
        const std::shared_ptr<pcl::PointCloud<PointT>>& source_cloud,  const std::shared_ptr<pcl::PointCloud<PointT>>& target_cloud,
        pcl::PointCloud<PointT>& source_correspondences_cloud, pcl::PointCloud<PointT>& target_correspondences_cloud,
        double max_dist_threshold, bool use_reciprocal_correspondence_ = true
    )
    {
        pcl::Correspondences correspondences_;

        pcl::registration::CorrespondenceEstimation<PointT, PointT> estimation;
        estimation.setInputSource(source_cloud);
        estimation.setInputTarget(target_cloud);

        if (use_reciprocal_correspondence_)
            estimation.determineReciprocalCorrespondences(correspondences_, max_dist_threshold);
        else
            estimation.determineCorrespondences(correspondences_, max_dist_threshold);
        
        source_correspondences_cloud.clear();
        target_correspondences_cloud.clear();

        for(auto correspondence: correspondences_)
        {
            auto source_index = correspondence.index_query;
            auto target_index = correspondence.index_match;
            source_correspondences_cloud.push_back(source_cloud->at(source_index));
            target_correspondences_cloud.push_back(target_cloud->at(target_index));
        }
    }

    static void projectCloudToPixelCoordinates(
        const pcl::PointCloud<PointT>& cloud, const cv::Mat& r_mat, const cv::Mat& t_vec, const cv::Mat& cam_matrix, 
        const std::vector<double>& dist_coefficients, cv::OutputArray image_points
    )
    {
        std::vector<cv::Point3f> phy_points;
        for(const auto& pt: cloud.points)
            phy_points.push_back(cv::Point3f(pt.x, pt.y, pt.z));

        cv::projectPoints(phy_points, r_mat, t_vec, cam_matrix, dist_coefficients, image_points);
    }

    //static void mlsSmoothWithoutNormals(
    //    const pcl::PointCloud<PointT>& cloud_in, const pcl::PointCloud<PointT>& cloud_out,
    //    int polynomial_order, double search_radius, double squared_gaussian, int threads = 8
    //)
    //{
    //    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();

    //    pcl::MovingLeastSquares<PointT, PointT> mls;

    //    mls.setComputeNormals(false);
    //    mls.setInputCloud(cloud_in);
    //    mls.setPolynomialFit(true);
    //    mls.setPolynomialOrder(polynomial_order);
    //    mls.setNumberOfThreads(threads);
    //    mls.setSearchRadius(search_radius);
    //    mls.setSqrGaussParam(squared_gaussian);


    //}
};


template<typename ColorPointT>
class PclColorPointCloudUtils
{
public:
    static void avgColor(const std::shared_ptr<pcl::PointCloud<ColorPointT>>& cloud, const std::vector<int>& indices, float& r, float& g, float& b)
    {
        r = 0; b = 0; g = 0;

        for(auto i: indices)
        {
            const auto& p = cloud->points.at(i);
            r += p.r; g += p.g; b += p.b;
        }

        r /= indices.size();
        g /= indices.size();
        b /= indices.size();
    }

    static void avgGray(const std::shared_ptr<pcl::PointCloud<ColorPointT>>& cloud, const std::vector<int>& indices, uint8_t avg_gray)
    {
        double gray = 0;

        for(auto i: indices)
        {
            const auto& p = cloud->points.at(i);
            gray += (p.r + p.g + p.b) / 3.0;
        }

        avg_gray = static_cast<uint8_t>(gray / indices.size());
    }

    static void cloudsEdgeColorSmooth(
        const std::shared_ptr<pcl::PointCloud<ColorPointT>>& cloud1, 
        const std::shared_ptr<pcl::PointCloud<ColorPointT>>& cloud2, 
        float smooth_radius, float smooth_ratio
    )
    {
        assert(smooth_ratio < 1);

        std::array<pcl::KdTreeFLANN<ColorPointT>, 2> search_trees;
        search_trees[0].setInputCloud(cloud1);
        search_trees[1].setInputCloud(cloud2);

        std::mutex mutex;
        std::vector<std::shared_ptr<std::thread>> threads;

        for(auto i: {0, 1})
        {
            auto thread = std::make_shared<std::thread>(
                [&cloud1, &cloud2, &search_trees, i, &smooth_radius, &smooth_ratio, &mutex]
                {
                    float r = 255, g = 0, b = 0;
                    std::vector<int> indices_1, indices_2;
                    std::vector<float> distances_temp;

                    const pcl::KdTreeFLANN<ColorPointT>& self_tree = search_trees[i];
                    const pcl::KdTreeFLANN<ColorPointT>& reciprocal_tree = search_trees[1 - i];
                    auto self_cloud = i == 0 ? cloud1: cloud2;
                    auto reciprocal_cloud = i == 0 ? cloud2: cloud1;

                    for(auto& pt: self_cloud->points)
                    {
                        // Neighbors count of the current point cloud.
                        int self_neighbors = self_tree.radiusSearch(pt, smooth_radius, indices_1, distances_temp);
                        // Neighbors count of another point cloud.
                        int reciprocal_neighbors = reciprocal_tree.radiusSearch(pt, smooth_radius, indices_2, distances_temp);

                        if(reciprocal_neighbors == 0 || self_neighbors == 0)
                            continue;

                        uint8_t self_gary = 0, reciprocal_gray = 0;

                        avgGray(self_cloud, indices_1, self_gary);
                        avgGray(reciprocal_cloud, indices_2, reciprocal_gray);

                        
                    }
                }
            );

            threads.push_back(thread);
        }

        for(auto& thd: threads)
            thd->join();
    }

    template<typename PointT>
    static void pclSetColor(pcl::PointCloud<PointT>& cloud, uint8_t r, uint8_t g, uint8_t b)
    {
        for(auto& pt: cloud.points)
        {
            pt.r = r; pt.g = g; pt.b = b;
        }
    }
};
