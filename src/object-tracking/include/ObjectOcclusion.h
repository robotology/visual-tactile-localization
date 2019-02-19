#ifndef OBJECTOCCLUSION_H
#define OBJECTOCCLUSION_H

#include <MeshModel.h>

#include <Eigen/Dense>

#include <SuperimposeMesh/SICAD.h>

#include <opencv2/opencv.hpp>

class ObjectOcclusion
{
public:
    ObjectOcclusion(std::unique_ptr<MeshModel> mesh_model, const std::string cut_method, const double occlusion_scale);

    void findOcclusionArea();

    void drawOcclusionArea(cv::Mat& image);

    std::tuple<bool, bool, cv::Mat> removeOcclusion(const cv::Mat& mask_in);

    virtual std::pair<bool, Eigen::MatrixXd> getOcclusionPose() = 0;

    virtual std::tuple<bool, Eigen::VectorXd, Eigen::VectorXd> getCameraPose() = 0;

    void reset();

protected:
    std::vector<cv::Point> enlargeConvexHull(const std::vector<cv::Point>& contour, const double& scaling_factor);

    std::unique_ptr<MeshModel> mesh_model_;

    std::unique_ptr<SICAD> object_sicad_;

    bool occlusion_area_set_ = false;

    std::vector<cv::Point> occlusion_area_;

    const double occlusion_scale_;

    const std::string cut_method_;
};

#endif /* OBJECTOCCLUSION_H */
