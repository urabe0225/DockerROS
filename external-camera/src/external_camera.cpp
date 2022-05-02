#include "ros/ros.h"
#include "std_msgs/String.h"
#include <external_camera/c_state.h>
#include <external_camera/c_req.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>
#include <time.h>
#include <signal.h>

class State{
public:
  virtual ~State(){
  }
  virtual void publishCameraState(char* date) = 0;
};


class StandbyState: public State{
private:
  const char* STANDBY; // camera mode
  ros::Publisher standby_pub;
public:
  StandbyState(ros::Publisher& standby_pub):
    STANDBY("Standby"){
    this->standby_pub = standby_pub;
    ROS_INFO_STREAM("StandbyState constructor was called.");
    }
    void publishCameraState(char* date){
      external_camera::c_state msg;
      msg.time = std::string(date);
      msg.c_mode = STANDBY;
      standby_pub.publish(msg);
      ROS_INFO_STREAM("publish a standby state message." );
    }
};
class MonitorState: public State{
private:
  const char* MONITOR;
  ros::Publisher monitor_pub;

  cv::VideoCapture cap;
  const int WIDTH; // 背景画像や差分画像の幅 (pixel)
  const int HEIGHT; // 背景画像や差分画像の高さ (pixel)

  cv::Mat frame, intrinsic, distortion, mapx, mapy;
  cv::FileStorage fs;
public:
  MonitorState(ros::Publisher& monitor_pub):
    MONITOR("Monitor"), WIDTH(1920), HEIGHT(1080){
    if(!cap.open(0)){
      throw "cannot open video capture device.";
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

    // カメラの歪みデータの読み込み
    if(!(fs.open("/root/catkin_ws/src/external_camera/camera.xml", cv::FileStorage::READ))){
      throw "cannot open calibration data file.";
    }
    fs["intrinsic"] >> intrinsic;
    fs["distortion"] >> distortion;

    cap >> frame;
    cv::initUndistortRectifyMap (intrinsic, distortion, cv::Matx33d::eye(), intrinsic, frame.size(), CV_32FC1, mapx, mapy);
    
    this->monitor_pub = monitor_pub;
    ROS_INFO_STREAM("MonitorState constructor was called.");
  }
  void publishCameraState(char* date){

    cap >> frame;

    if(frame.empty()){
      throw "cannot capture the image.";
    }

    cv::imshow("win", frame);
    const int key = cv::waitKey(1);

    ROS_INFO_STREAM("publish a monitor state message.");
  }

};
class ErrorState: public State{
private:
  const char* ERROR;
  ros::Publisher error_pub;

public:
  ErrorState(ros::Publisher& error_pub):
    ERROR("Error"){
    this->error_pub = error_pub;
    ROS_INFO_STREAM("ErrorState constructor was called.");
  }
  void publishCameraState(char* date){
    external_camera::c_state msg;
    msg.time = std::string(date);
    msg.c_mode = ERROR;

    error_pub.publish(msg);
    ROS_INFO_STREAM("publish a error state message.");
  }

};

class ExternalCameraNode{
private:
  State* camera_state;

  const char* STANDBY; // "Standby" mode
  const char* MONITOR; // "Monitor" mode
  const char* ERROR; // "Error" mode

  const char* TOPIC_STATE_REQUEST;
  const char* TOPIC_STATE_PUBLISH;
  ros::Subscriber req_sub;
  ros::Publisher state_pub;

  char date_tmp[20];
  struct tm *date;
  time_t now;
  int year, month, day, hour, minute, second;

  const int FREQUENCY;
  const int QUEUE_SIZE;

  void changeCameraState(State* camera_state){
    delete this->camera_state;
    this->camera_state = camera_state;
  }

  void receiveInternalError(){
    changeCameraState(new ErrorState(state_pub));
    ROS_INFO_STREAM("changed the camera state into ERROR.");
  }

  void receiveMonitorRequest(){
    try{
      changeCameraState(new MonitorState(state_pub));
    }catch(const char *e){
      ROS_ERROR("Exception: %s", e);
      receiveInternalError();
      return;
    }
    ROS_INFO_STREAM("changed the camera state into MONITOR.");
  }

  void receiveStandbyRequest(){
    changeCameraState(new StandbyState(state_pub));
    ROS_INFO_STREAM("changed the camera state into STANDBY.");
  }

  void requestCallback(const external_camera::c_req::ConstPtr& msg){
    char* command = const_cast<char*>(msg->c_cmd.c_str());
    if(strcmp(command, MONITOR) == 0 && typeid(*camera_state) == typeid(StandbyState)){
      receiveMonitorRequest();
    }
    else if(strcmp(command, STANDBY) == 0 && typeid(*camera_state) == typeid(MonitorState)){
      receiveStandbyRequest();
    }

  }
  void makeDateString(){
    time(&now);
    date = localtime(&now);

    year = date->tm_year + 1900;
    month = date->tm_mon + 1;
    day = date->tm_mday;
    hour = date->tm_hour;
    minute = date->tm_min;
    second = date->tm_sec;
    sprintf(date_tmp, "%4d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
  }

public:
  ExternalCameraNode():
    STANDBY("Standby"), MONITOR("Monitor"), ERROR("Error"),
    TOPIC_STATE_REQUEST("sub_topic"), TOPIC_STATE_PUBLISH("pub_topic"),
    FREQUENCY(1), QUEUE_SIZE(100){

    ROS_INFO_STREAM("==> Constructor was called.");
    ros::NodeHandle nh("~");
    req_sub = nh.subscribe<external_camera::c_req>(TOPIC_STATE_REQUEST, QUEUE_SIZE, &ExternalCameraNode::requestCallback, this);
    state_pub = nh.advertise<external_camera::c_state>(TOPIC_STATE_PUBLISH, QUEUE_SIZE);

    camera_state = new StandbyState(state_pub);
    ROS_INFO_STREAM("set the camera state into STANDBY.");
  }
  ~ExternalCameraNode(){
    ROS_INFO_STREAM("==> Destructor was called.");
    // camera_stateインスタンスの削除
    if(camera_state != NULL){
      delete camera_state;
      ROS_INFO_STREAM("camera state instance was deleted.");
    }
  }
  void mainLoop(){
    ros::Rate loop_rate(FREQUENCY);

    while (ros::ok()){
      makeDateString();
      try{
        camera_state->publishCameraState(date_tmp);
      }
      catch(const char *e){
        ROS_ERROR("Exception: %s", e);
        ROS_INFO_STREAM("Error_Date: " << date_tmp);
        //receiveInternalError();
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "external_camera_node");
  ExternalCameraNode ecn;
  ecn.mainLoop();
}
