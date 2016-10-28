#include "FindCircle.h"

std::string FindCircle::lookupLabel(float id) {

    if (id < 3.5f){
        return "bread_b";
    }
    else if (id < 7.f){
        return "bread_t";
    }
    else if (id < 13.f){
        return "cheese";
    }else{
        return "ham";
    }


    // if (id < 3.7f) {
    //     return "egg";
    // } else if (id > 7.5f) {
    //     return "ham";
    // } else {
    //     return "tomato";
    // }
}

void FindCircle::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    if (image->bpp != msg->step / msg->width || image->width != msg->width || image->height != msg->height) {
        delete image;
        ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.", image->width, image->height, image->bpp, msg->width, msg->height, msg->step / msg->width);
        image = new CRawImage(msg->width, msg->height, msg->step / msg->width);
    }

    memcpy(image->data, (void*) &msg->data[0], msg->step * msg->height);

    circle_detection::detection_results_array tracked_objects;
    tracked_objects.header = msg->header;
    visualization_msgs::MarkerArray marker_list;

    //search image for circles
    for (int i = 0; i < MAX_PATTERNS; i++) {
        lastSegmentArray[i] = currentSegmentArray[i];
        currentSegmentArray[i] = detectorArray[i]->findSegment(image, lastSegmentArray[i]);
        objectArray[i].valid = false;

        if (currentSegmentArray[i].valid) {
            objectArray[i] = trans->transform(currentSegmentArray[i]);

            // Filter error values
            if (isnan(objectArray[i].x)) continue;
            if (isnan(objectArray[i].y)) continue;
            if (isnan(objectArray[i].z)) continue;
            if (isnan(objectArray[i].roll)) continue;
            if (isnan(objectArray[i].pitch)) continue;
            if (isnan(objectArray[i].yaw)) continue;

            // temp value to hold current detection
            circle_detection::detection_results objectsToAdd;

            // Convert to ROS standard Coordinate System
            objectsToAdd.pose.position.x = -objectArray[i].y;
            objectsToAdd.pose.position.y = -objectArray[i].z;
            objectsToAdd.pose.position.z = objectArray[i].x;

            // Convert YPR to Quaternion
            tf::Quaternion q;
            q.setRPY(objectArray[i].roll, objectArray[i].pitch, objectArray[i].yaw);
            objectsToAdd.pose.orientation.x = q.getX();
            objectsToAdd.pose.orientation.y = q.getY();
            objectsToAdd.pose.orientation.z = q.getZ();
            objectsToAdd.pose.orientation.w = q.getW();

            // This needs to be replaced with a unique label as ratio is unreliable
            // objectsToAdd.uuid = generateUUID(startup_time_str,floor(objectArray[i].bwratio));
            objectsToAdd.uuid = lookupLabel(objectArray[i].bwratio);
            objectsToAdd.roundness = objectArray[i].roundness;
            objectsToAdd.bwratio = objectArray[i].bwratio;
            objectsToAdd.esterror = objectArray[i].esterror;

            // Hardcoded values need to be replaced
            objectsToAdd.objectsize.x = 3;
            objectsToAdd.objectsize.y = 3;
            objectsToAdd.objectsize.z = 0;

            tracked_objects.tracked_objects.push_back(objectsToAdd);

            // Generate RVIZ marker for visualisation
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.id = floor(objectArray[i].bwratio);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::MODIFY;
            marker.pose = objectsToAdd.pose;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(0.2);
            marker_list.markers.push_back(marker);
        }
    }

    if ((marker_list.markers.size() > 0) && (tracked_objects.tracked_objects.size() > 0)) {

        if (tracked_objects.header.frame_id.find("left") != std::string::npos) {
            pubLeft.publish(tracked_objects);
        } else {
            pubRight.publish(tracked_objects);
        }
        vis_pub.publish(marker_list);
    }
    memcpy((void*) &msg->data[0], image->data, msg->step * msg->height);
    imdebug.publish(msg);
}

void FindCircle::cameraInfoCallBack(const sensor_msgs::CameraInfo &msg) {
    trans->updateParams(msg.K[2], msg.K[5], msg.K[0], msg.K[4]);
}

FindCircle::FindCircle(void) {
    nh = new ros::NodeHandle("~");
    defaultImageWidth = 1280;
    defaultImageHeight = 800;
    circleDiameter = 0.049;
}

FindCircle::~FindCircle(void) {
    delete image;
    for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
    delete trans;
}

void FindCircle::init(int argc, char* argv[]) {

    if (nh->getParam("camera", topic)) {
        std::transform(topic.begin(), topic.end(), topic.begin(), ::tolower);
    } else {
        if (argc != 2) {
            ROS_FATAL("Please supply whether you are subscribing to the left or right camera (R/L)");
            return;
        } else {
            if (argv[1][0] == 'l' || argv[1][0] == 'L') {
                topic = "left";
            } else if (argv[1][0] == 'r' || argv[1][0] == 'R') {
                topic = "right";
            } else {
                ROS_FATAL("Please supply whether you are subscribing to the left or right camera (R/L)");
                return;
            }
        }
    }

    image_transport::ImageTransport it(*nh);
    image = new CRawImage(this->defaultImageWidth, this->defaultImageHeight, 4);
    trans = new CTransformation(circleDiameter, nh);
    for (int i = 0; i < MAX_PATTERNS; i++) {
        detectorArray[i] = new CCircleDetect(this->defaultImageWidth, this->defaultImageHeight);
    }
    cameraInfo = nh->subscribe("/cameras/right_hand_camera/camera_info", 1, &FindCircle::cameraInfoCallBack, this);
    image->getSaveNumber();
    image_transport::Subscriber subim = it.subscribe("/cameras/" + topic + "_hand_camera/image", 1, &FindCircle::imageCallback, this);

    imdebug = it.advertise("/circledetection/"+topic+"/rgb/processedimage", 1);
    pubLeft = nh->advertise<circle_detection::detection_results_array>("/circledetection/left_circleArray", 1);
    pubRight = nh->advertise<circle_detection::detection_results_array>("/circledetection/right_circleArray", 1);
    vis_pub = nh->advertise<visualization_msgs::MarkerArray>("/circledetection/" + topic + "/rviz_marker", 1);
    lookup = new tf::TransformListener();
    ROS_DEBUG("Server running");
    ros::spin();
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "circle_detector", ros::init_options::AnonymousName);

    FindCircle *detector = new FindCircle();

    //Attempt to start detector
    detector->init(argc, argv);
    //Clean up
    detector->~FindCircle();
    return 0;
}