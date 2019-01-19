#ifndef ICUBPOINTCLOUD_H
#define ICUBPOINTCLOUD_H

#include <PointCloudModel.h>
#include <PointCloudPrediction.h>
#include <SFM.h>

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <GazeController.h>

#include <string>
#include <memory>

class iCubPointCloud : PointCloudModel
{
public:
    iCubPointCloud
        (
            const string port_prefix,
            const string SFM_context_name,
            const string SFM_config_name,
            const string IOL_object_name,
            const string obj_mesh_file,
            std::unique_ptr<PointCloudPrediction> prediction,
            const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix
        );

    virtual ~iCubPointCloud();

    /*
     * To crop out external undesired object, e.g. the hand of the robot, when extracting the point cloud of the object.
     */
    virtual void setExternalObjectBoundingBox(std::pair<int, int> top_left, std::pair<int, int> bottom_right);

    /*
     * Set the last estimate available.
     */
    virtual void setObjectEstimate(Eigen::Ref<const Eigen::VectorXd>& pose);

    std::pair<bool, bfl::Data> measure() const override;

    bool freezeMeasurements() override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:

    /*
     * Retrieve the object bounding box according to iCub OPC (objects property collector).
     * Return a boolean indicating the outcome, a pair of top-left u-v coordinates and a pair of bottom-right u-v coordinates.
     */
    std::tuple<bool, std::pair<int, int>, std::pair<int, int>> retrieveObjectBoundingBox(const string obj_name);

    /*
     * Evaluate the 2D coordinates of the object.
     */
    std::pair<bool, std::vector<std::pair<int, int>>> getObject2DCoordinates(std::size_t stride_u, std::size_t stride_v);

    /**
     * This is iCub SFM.
     */
    SFM sfm_;

    /**
     * Object bounding box (top-left, bottom-right) of the target object.
     * Note: this is relative to the **left** iCub eye.
     */
    std::pair<int, int> obj_bbox_tl_;
    std::pair<int, int> obj_bbox_br_;
    bool obj_bbox_set_;

    /**
     * Bounding box (top-left, bottom-right) of an **undesired** external object.
     * Note: this is relative to the **left** iCub eye.
     */
    std::pair<int, int> ext_obj_bbox_tl_;
    std::pair<int, int> ext_obj_bbox_br_;
    bool ext_obj_bbox_set_;

    /**
     * IOL object category name (required to initialize the bounding box of the object).
     */
    const std::string IOL_object_name_;

    /**
     * RPC clients.
     */
    RpcClient opc_rpc_client_;

    /**
     * Local copy of measurements.
     * A vector of size 3 * L with L the number of points in the point cloud.
     */
    Eigen::VectorXd measurement_;

    /**
     * Last object estimate, to be used as hint to evaluate a better bounding box.
     * The vector contains 3 Cartesian coordinates and a ZYX Euler angles parametrization.
     */
    Eigen::VectorXd last_estimate_;
    bool obj_estimate_set_;

    /**
     * Interface to iCub cameras, required to predict the 2D object bounding box.
     */
    GazeController gaze_;
};

#endif /* ICUBPOINTCLOUD_H */
