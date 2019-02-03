#ifndef OBJECTOCCLUSION_H
#define OBJECTOCCLUSION_H

#include <MeshModel.h>

#include <Eigen/Dense>

#include <SuperimposeMesh/SICAD.h>

#include <opencv2/opencv.hpp>

using Object2DCoordinates = std::vector<std::pair<int, int>>;

class ObjectOcclusion
{
public:
    ObjectOcclusion(std::unique_ptr<MeshModel> mesh_model, const std::string cut_method);

    void findOcclusionArea();

    void drawOcclusionArea(cv::Mat& image);

    std::pair<bool, Object2DCoordinates> removeOcclusionCoordinates(const Object2DCoordinates& object_coordinates);

    virtual std::pair<bool, Eigen::MatrixXd> getOcclusionPose() = 0;

    virtual std::tuple<bool, Eigen::VectorXd, Eigen::VectorXd> getCameraPose() = 0;

    void reset();

protected:
    std::unique_ptr<MeshModel> mesh_model_;

    std::unique_ptr<SICAD> object_sicad_;

    bool occlusion_area_set_ = false;

    std::vector<cv::Point> occlusion_area_;

    const std::string cut_method_;
};

#endif /* OBJECTOCCLUSION_H */
