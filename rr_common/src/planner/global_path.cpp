/**
 * Used to determine the cost for a plan (list of poses) based on the distance from a global
 * path in the world. It takes a nav_msgs::Path and compares the plan to the global path.
 * @Note: Assumes plan is "close" to path and we are progressing forward along global path
 *
 * @author Brian Cochran
 */

#include <rr_common/planning/global_path.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <tuple>
#include <parameter_assertions/assertions.h>
#include <numeric>
#include <rr_common/planning/vectordtw.h>



namespace rr {

GlobalPath::GlobalPath(ros::NodeHandle nh) : has_global_path_(false), accepting_updates_(true), closest_point_to_robot_index_(0), listener_(new tf::TransformListener) {
    std::string global_path_topic;
    assertions::getParam(nh, "global_path_topic", global_path_topic);
    assertions::getParam(nh, "robot_base_frame", robot_base_frame_);
    assertions::getParam(nh, "cost_scaling_factor", cost_scaling_factor_, { assertions::greater_eq(0.0) });

    global_path_sub_ = nh.subscribe(global_path_topic, 1, &GlobalPath::SetPathMessage, this);
}

double GlobalPath::CalculateCost(const std::vector<PathPoint>& plan, const bool viz) {
    if (!has_global_path_) {
        return 0.0;
    }
    std::vector<tf::Point> sample_path(plan.size());
    std::transform(plan.begin(), plan.end(), sample_path.begin(), [](const PathPoint &pose) {return tf::Point(pose.pose.x, pose.pose.y, 0);});
    tf::Point sample_start = sample_path[0];
    std::vector<double> distances_to_origin(global_path_.size());
    std::transform(global_path_.begin(), global_path_.end(), distances_to_origin.begin(), [sample_start](const tf::Point& global_pnt){
        return GlobalPath::GetPointDistance(global_pnt, sample_start);
    });
    //segement start
    int seg_start_index = std::min_element(distances_to_origin.begin(), distances_to_origin.begin()) - distances_to_origin.begin();
    vector<double> sample_adj_dist = GlobalPath::adjacent_distances(sample_path);
    double sample_length = std::accumulate(sample_adj_dist.begin(), sample_adj_dist.end(), 0.0);
    //find last point using cumulative distance
    double upper_cum_limit = std::fmod((global_cum_dist_[seg_start_index] + sample_length), global_cum_dist_[global_cum_dist_.size() - 1]);
    int seg_end_index = std::upper_bound(global_cum_dist_.begin(), global_cum_dist_.end(), upper_cum_limit) - global_cum_dist_.begin();
    //Assume sample_path.length < global_path.length
    std::vector<tf::Point> global_segment;
    if (seg_start_index <= seg_end_index) {
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.begin() + seg_end_index);
    } else {
        global_segment = std::vector<tf::Point>(global_path_.begin() + seg_start_index, global_path_.end());
        global_segment.insert(global_segment.end(), global_path_.begin(), global_path_.begin() + seg_end_index);
    }

    // Create Point lists for the dtw
    std::vector<Point> global_points(global_segment.size());
    std::transform(global_segment.begin(), global_segment.end(), global_points.begin(), [](const tf::Point& tf_pt) {
        return Point(tf_pt.getX(), tf_pt.getY(), 0);
    });
    std::vector<Point> sample_points(sample_path.size());
    std::transform(sample_path.begin(), sample_path.end(), sample_points.begin(), [](const tf::Point& tf_pt) {
        return Point(tf_pt.getX(), tf_pt.getY(), 0);
    });
    //ensure they are the same length for fast dwt
    int points_len = std::min(global_points.size(), sample_points.size());
    if (global_points.size() != sample_points.size()) {
        global_points.resize(points_len);
        sample_points.resize(points_len);
    }

    //publish if visulization is enabled
    if(viz) {

    }

    // dtw
    VectorDTW dtw1(points_len, points_len / 10);
    double dtw_val = dtw1.fastdynamic(global_points, sample_points);
    return dtw_val;
}

std::vector<double> GlobalPath::adjacent_distances(const std::vector<tf::Point>& path) {
    std::vector<double> distances(1, 0.0);
    std::transform(path.begin(), path.end() - 1, path.begin() + 1, std::back_inserter(distances), &GlobalPath::GetPointDistance);
    return distances;
}

void GlobalPath::PreProcess() {
    ROS_ERROR_STREAM("Started Pre Process");
    if (!has_global_path_) {
        return;
    }
    this->LookupPathTransform();
    //find nearest point to robot
    tf::Pose w_robotPose = robot_to_path_transform_ * tf::Pose(tf::createQuaternionFromYaw(0), tf::Vector3(0, 0, 0));

    std::tuple<unsigned int, double> closestIndexPoint = this->FindNearestPathPointIndex(0, w_robotPose);
    closest_point_to_robot_index_ = std::get<0>(closestIndexPoint);
    ROS_ERROR_STREAM("Finished Pre Process");
}

void GlobalPath::LookupPathTransform() {
    accepting_updates_ = false;
    try {
        listener_->waitForTransform(global_path_msg_.header.frame_id, robot_base_frame_, ros::Time(0), ros::Duration(.05));
        listener_->lookupTransform(global_path_msg_.header.frame_id, robot_base_frame_, ros::Time(0), robot_to_path_transform_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
    }
    accepting_updates_ = true;
}

double GlobalPath::GetPointDistance(tf::Point point1, tf::Point point2){
    return point1.distance(point2);
}

std::tuple<unsigned int, double> GlobalPath::FindNearestPathPointIndex(unsigned int startIndex, tf::Pose inputPose) {
    ROS_ERROR_STREAM("Start of Find Nearest Path Point Index");
    if (!has_global_path_) {
        return std::make_tuple(0, 0.0);
    }

    unsigned int currIndex = startIndex;
    unsigned int nextIndex = (currIndex + 1 ) % global_path_msg_.poses.size();

    tf::Pose w_currPathPose;
    tf::poseMsgToTF(global_path_msg_.poses[currIndex].pose, w_currPathPose);
    double distCurr = GlobalPath::GetPointDistance(inputPose.getOrigin(), w_currPathPose.getOrigin());

    tf::Pose w_nextPathPose;
    tf::poseMsgToTF(global_path_msg_.poses[nextIndex].pose, w_nextPathPose);
    double distNext = GlobalPath::GetPointDistance(inputPose.getOrigin(), w_nextPathPose.getOrigin());

    do {
        if (distNext >= distCurr) {
            return std::make_tuple(currIndex, distCurr);
        }

        currIndex = nextIndex;
        nextIndex = (currIndex + 1 ) % global_path_msg_.poses.size();

        w_currPathPose = w_nextPathPose;
        distCurr = distNext;

        tf::poseMsgToTF(global_path_msg_.poses[nextIndex].pose, w_nextPathPose);
        distNext = GlobalPath::GetPointDistance(inputPose.getOrigin(), w_nextPathPose.getOrigin());
    } while (currIndex != startIndex); //full loop around

    ROS_ERROR_STREAM("End of Find Nearest Path Point Index");
    return std::make_tuple(0, 0.0); //nothing found
}

void GlobalPath::SetPathMessage(const nav_msgs::Path& global_path_msg) {
    if (!accepting_updates_) {
        return;
    }
    has_global_path_ = true;
    global_path_msg_ = nav_msgs::Path(global_path_msg);
    global_path_ = std::vector<tf::Point> (global_path_msg.poses.size());
    // Convert Type
    std::transform(global_path_msg.poses.begin(), global_path_msg.poses.end(), global_path_.begin(),
                   [](const auto &poseStamped) {
        tf::Pose tf_pose;
        tf::poseMsgToTF(poseStamped.pose, tf_pose);
        return tf_pose.getOrigin();
    });
    // Get Distances
    std::vector<double> adj_dist = GlobalPath::adjacent_distances(global_path_);
    // Get Cumalitive Distance
    global_cum_dist_ = std::vector<double>(adj_dist.size());
    std::partial_sum(adj_dist.begin(), adj_dist.end(), global_cum_dist_.begin());
    updated_ = true;
}

}  // namespace rr