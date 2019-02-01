#include <ObjectOcclusion.h>

#include "opencv2/imgproc.hpp"

using namespace Eigen;


ObjectOcclusion::ObjectOcclusion(std::unique_ptr<MeshModel> mesh_model, const std::string cut_method) :
    mesh_model_(std::move(mesh_model)),
    cut_method_(cut_method)
{ }


void ObjectOcclusion::reset()
{
    occlusion_area_set_ = false;
}


bool ObjectOcclusion::findOcclusionArea()
{
    // Get occlusion pose
    bool valid_pose;
    MatrixXd pose;
    std::tie(valid_pose, pose) = getOcclusionPose();

    if (!valid_pose)
        return occlusion_area_set_;

    // Get camera pose
    bool valid_camera_pose;
    VectorXd camera_origin;
    VectorXd camera_orientation;
    std::tie(valid_camera_pose, camera_origin, camera_orientation) = getCameraPose();

    if (!valid_camera_pose)
        return occlusion_area_set_;

    // Get the sicad model pose
    bool valid_sicad_pose;
    std::vector<Superimpose::ModelPoseContainer> sicad_poses;
    std::tie(valid_sicad_pose, sicad_poses) = mesh_model_->getModelPose(pose);

    if (!valid_sicad_pose)
        return occlusion_area_set_;

    // Render the occlusion mask
    cv::Mat occlusion_mask;
    object_sicad_->superimpose(sicad_poses[0], camera_origin.data(), camera_orientation.data(), occlusion_mask);

    // Convert to gray scale
    cv::cvtColor(occlusion_mask, occlusion_mask, CV_BGR2GRAY);


    if (cut_method_ == "convex_hull")
    {
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(occlusion_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Only one contour has to be found
        if (contours.size() != 1)
            return occlusion_area_set_;

        // Find the convex hull
        cv::convexHull(contours[0], occlusion_area_);
    }

    occlusion_area_set_ = true;

    return true;
}


std::pair<bool, Object2DCoordinates> ObjectOcclusion::removeOcclusionCoordinates(const Object2DCoordinates& object_coordinates)
{
    if (!findOcclusionArea())
        return std::make_pair(false, object_coordinates);

    Object2DCoordinates output_coordinates;
    for (auto coord : object_coordinates)
    {
        if (cv::pointPolygonTest(occlusion_area_, cv::Point(coord.first, coord.second), false) < 0)
            output_coordinates.push_back(coord);
    }

    return std::make_pair(true, output_coordinates);
}
