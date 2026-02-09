/**
 * @file /include/zl_Crane_AutomaticLift_trunk/main_window.hpp
 *
 * @brief Qt based gui for zl_Crane_AutomaticLift_trunk.
 *
 * @date November 2010
 **/
#ifndef zl_Crane_AutomaticLift_trunk_MAIN_WINDOW_H
#define zl_Crane_AutomaticLift_trunk_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#pragma once
#include<QMainWindow>
#include<QWidget>
#include<QAction>
#include<QDateTime>
#include<QTimer>
#include<iostream>
#include <image_transport/image_transport.h>
#include <thread>
#include <future>
#include <mutex>
#include <chrono>
#include <atomic>
#include <QSettings>
#include <QVariant>
#include <iomanip>
#include <fstream>
#include <string>
#include<time.h>


#undef slots
// #include <siamrpn/siamRPN_tracker.hpp>
#define slots Q_SLOTS

#include<QMessageBox>
#include <ros/ros.h>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <QtGui/QOpenGLFramebufferObject>
#include <zl_manipulation/AutoCraneArmManipulation.h>

#include "QLabelEx.hpp"
#include "clientusbtocancommunication.hpp"
#include "PTZ_TCPClientSocket.hpp"

#include "GLwidget.hpp"
#include "subscriberThread.hpp"

#include "ObstacleDetection.hpp"
#include "CameraObject.hpp"
#include <nlohmann/json.hpp>
#include "receivertk.hpp"
#include "udpRtk.hpp"
#include "mqtt_client.hpp"
#include "JointState.pb.hpp"

//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
#include "HookLoadAntiCollisionDetector.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <HookLoadAntiCollisionDetection/processbackend.h>
#include <HookLoadAntiCollisionDetection/common.h>
#include <HookLoadAntiCollisionDetection/CraneStatus.h>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
// --- 根据 VTK 版本自动适配控件类型和渲染窗口 ---
#if VTK_MAJOR_VERSION < 8
    // VTK 6.x 或 7.x 逻辑
    #include <QVTKWidget.h>
    #include <vtkRenderWindow.h>
    typedef QVTKWidget VTKWidgetType;
    #define VTK_WINDOW_TYPE vtkRenderWindow
#else
    // VTK 8.x 或 9.x 逻辑
    #include <QVTKOpenGLWidget.h>
    #include <vtkGenericOpenGLRenderWindow.h>
    typedef QVTKOpenGLWidget VTKWidgetType;
    #define VTK_WINDOW_TYPE vtkGenericOpenGLRenderWindow
#endif


/*****************************************************************************
** Namespace
*****************************************************************************/
//using namespace FrameWork;
using namespace cv;
using namespace std;
using json = nlohmann::json;

namespace zl_Crane_AutomaticLift_trunk {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class MainWindow : public QMainWindow {
  Q_OBJECT

private:
  Ui::MainWindowDesign ui;
  QAction *openAction;
  QLabel *msgLabel;
  QLabel *ShowTimeLabel;

  clientUsbToCanCommunication *m_clsClientUsbToCan;
  bool m_bUsbToCan;
  QThread *thread;
  int timer_display_id;
  int robot_visual_display_id;

  ompl::app::AutoCraneArmManipulation *auto_crane_manipulation_; //motion planning
  ompl::app::joint_trajectory_t solution_trajectory_;
  ompl::app::joint_trajectory_point_t last_sent_point_;
  std::vector<ObstacleInforStr> last_added_collision_objects_;
  QTimer *planning_scene_visualization_timer_;
  QTimer *app_update_timer_;
  const size_t collision_check_range_;
  std::mutex update_solution_trajectory_mutex_;

  std::atomic<bool> is_replanning_;
  std::atomic<bool> is_predicted_collision_;
  std::atomic<bool> is_running_predict_collision_;
  std::atomic<bool> is_stop_send_trajectory_;
  std::atomic<int> predicted_collision_index_;
  std::atomic<int> sent_joint_trajectory_index_;
  std::atomic<int> replan_start_index_;
  std::atomic<int> slow_down_end_index_;


  //std::thread start_swing_tread_;
  zl_common::app::zl::PickPlaceTask ppt_;//the default mode is ONCE.
  std::ofstream joint_trajectory_command_file_saver_;
  ompl::app::joint_state_t current_joint_state_;
  ompl::app::joint_state_t previous_joint_state_;

private:
  std::thread open_vedio_stream_thread_;
  bool is_open_video_player_display_{false};

  std::thread start_object_position_compute_thread_;
  bool is_start_object_position_compute_{false};

  std::thread send_control_command_package_;
  bool is_send_control_command_package_{false};

  std::thread send_control_command_package_autodrving;
  bool is_send_control_command_package_autodrving{false};

  std::thread start_pcan_data_receive_thread_;
  bool is_start_pcan_data_receive_thread_{false};

  std::thread start_cramera_real_ptz_control_thread_;
  bool is_start_cramera_real_ptz_control_thread_{false};

  std::thread start_data_update_display_thread_;
  bool is_start_data_update_display_thread_{false};

  std::thread send_joint_trajectory_thread_;
  std::thread realtime_collision_check_thread_;

  bool is_start_obstacles_detect_thread_{false};
  std::thread start_obstacles_detect_thread_;

  bool is_start_scence_visualization_thread_{false};
  std::thread start_scence_visualization_thread_;

  bool is_start_voice_control_thread_{false};
  std::thread start_voice_control_thread_;


  std::thread start_play_voice_thread_;
  bool is_start_play_voice_thread_{false};

  int m_image_width_{0};
  int m_image_height_{0};

  QPoint start_label_vedio_point_;
  int m_lift_goods_select_mode_{1};
  int m_lift_work_mode_{0};

  QRadioButton *m_hook_mode;      //hook mode
  QRadioButton *m_lift_mode;      //lift mode
  QButtonGroup *GenderGroup;

  QRadioButton *m_checkedBoard_mode;      //checkedBoard mode
  QRadioButton *m_anything_mode;      //anything mode
  QButtonGroup *m_TargetGroup;

public:
  zl_common::app::CameraObject *Camera_;
  zl_common::app::QLabelEx *label_vedioStream_;
  ompl::app::GLWidget *glWidget_;//motion planning visualization

  // Load planner JSON output and start trajectory tracking.
  // Returns false if the JSON is invalid or the trajectory is empty.
  bool loadPlannerJsonAndTrack(const std::string& json_path, double dt_sec = 0.1);

  QReadWriteLock m_frame_mutex;
  QReadWriteLock m_param_mutex;
  QReadWriteLock m_hookheight_mutex;
  QReadWriteLock m_target_value_mutex;
  QReadWriteLock m_box_mutex;
  Mutex m_pathplan_mutex;
  QReadWriteLock m_list_laser_mutex;      //线程互斥
  Mutex m_label_paint_mutex;

  Mat m_gframe_;
  bool is_selectObjectFlag_{false};
  bool is_start_crane_trajectory_track_{false};
  bool is_directly_visual_location_{false};
  int m_trajectory_point_size{0};

  Rect2d siam_detect_preBox_;

  int m_vedioRoi_x;       //视频图像框选目标左上角坐标
  int m_vedioRoi_y;
  int m_vedioRoi_w;
  int m_vedioRoi_h;
  float reality_center_point_x{0};
  float reality_center_point_y{0};
  float reality_center_point_z{0};

  float camera_pitch_angle_{0.0};  //相机扫描倾角
  float camera_rotate_angle_{0.0};
  int m_focusIndex{0};


  // SiamRPNTracker *siamrpn_tracker_;
  // torch::jit::script::Module m_temple_net;
  // torch::jit::script::Module m_detect_net;
  vector<int> m_vec_stretchWay;
  QSettings *m_psetting;
  double camera_frame_rate_{0.0};

  zl_common::app::QLabelEx *label_img_;
  zl_common::app::QLabelEx *label_vedio_;

  PTZ_TCPClientSocket *m_clsClientToYunTai{nullptr};     //云台通讯定义
  std::string m_strPTZcontrolCommand;
  zl_common::app::HParameter *hcfg_;

  std::string base_path{""};
  QString configFileName{QString::null};        //HPConfig's pathName

  double object_init_world_x;
  double object_init_world_y;

  bool is_detect_Chessboard_success{false};
  double image_targetRelativeLen_x_;
  double image_targetRelativeLen_y_;

  double m_tempCopyTargetC_{0.0};
  double m_tempCopyTargetR_{0.0};
  bool m_isDetectTargetFlag{false};

  double g_TargetC{0.0};
  double g_TargetR{0.0};

  vector<double> g_TargetC4{0.0};
  vector<double> g_TargetR4{0.0};

  bool is_trajectory_track_slow_to_stop_complete_{false};
  float m_hook_height{1.6};
  float m_object_weight{0};
  float m_object_height{0.0};
  float m_object_len{1.8};
  float m_object_width{1.4};
  float m_rope_height{2.5};
  float m_goods_height{1};

  int m_lift_work_status_{0};
  CraneStateStr m_stCraneState;

  int play_voice_flag{100};
  double m_dtargetRotationAngle_{0.0};
  double m_dtargetLuffLen_{0.0};

private:
  ros::Publisher chatter_publisher;
  image_transport::Publisher image_publisher;
  ros::Publisher calib_publisher_;
  ros::Publisher roll_publisher_;

  vector<ObjResult> calib_obstacles_datasets;//objects_res;
  ros::NodeHandle nh_;
  subscriberThread *ros_thread;
  ObstacleDetection *m_obstacle_detect_class;
  receiveRTK *m_rtk_;
  udpRtk *rtk;
  vector<double> rtkData;
  int dispatchtask_id{0};


  int m_merge_num_{0};
  pcl::PointCloud<pcl::PointXYZ> temp_laser_pointcloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_laser_cloud_frame_{new pcl::PointCloud<pcl::PointXYZ>};
  std::list<pcl::PointCloud<pcl::PointXYZ>> m_list_laser_pointclouds_;
  std::vector<ObstacleInforStr> m_output_obstacle_Informs_;

public:
  bool on_path_planing{false};
  bool m_pathplan_stop{false};
  bool m_start_pathPlan_thread_flag{false};
  bool m_set_new_pathplan_start_position{false};
  bool m_xubaike_pathplan{false};
  bool pathend_but_no_jingzhundingwei{false};
  bool avoidstopstart{false};
  std::thread thread_start_pathPlan;

  mqtt_client* mqtt_client_;
  //mqtt_client_ = new mqtt_client();
  bool isStarted{false};

  //20220418yt  jiqunxietong
  QVector<float> g_cranepostrue;
  vector<double> g_craneposition;
  int workstate{0};
  bool icrane_ok{false};
  bool north_board{false};
  //20220418  jiqunxietong

  zl_common::app::avoidcommandtask avoidcommandtask_;
  zl_common::app::dispatchtask dispatchtask_;
  zl_common::app::commandtask commandtask_;
  zl_common::app::pointPostrue current_goal;//存放当前规划目标
  zl_common::app::structPointXYZ current_goal_position;//存放当前规划目标

  //语音调度参数
  float rotate_up_left_x{0.0};
  float rotate_up_right_x{0.0};
  float luff_up_x{0.0};
  float luff_down_x{0.0};
  float wind_up_x{0.0};
  float wind_down_x{0.0};
  int emergency_stop{0};

  double targetsInfors_x{0};
  double targetsInfors_y{0};
  double targetsInfors_z{0};

public:
  bool is_start_rtk_data_update_thread_;
  std::thread start_rtk_data_update_thread_;

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void connect_Camera_Pcan();
  void elimination_mismatch_measure_three(bool &g_bErrorValue,  bool &is_compute_mean);
  void start_siamRPN_accurate_position();

  void Function_open_Vedio_Stream_Thread();
  void Function_SiamRPN_Accurate_Position_Thread();
  void Function_Send_Control_Command_Package_Thread();
  void Function_pcan_data_receive_Thread();
  void Function_cramera_real_ptz_control_Thread();
  void Function_data_update_display_Thread();
  void Function_start_obstacles_detect_Thread();
  void Function_start_scence_visualization_Thread();

  void Function_Send_Control_Command_Package_Thread_TestRTK();
  void Function_Send_Control_Command_Package_Thread_TestRTK_test();
  void Function_Send_Control_Command_Package_Thread_TestRTK_direct();

  void Function_RTK_data_receive_Thread();

  void Function_play_voice_Thread();

  void Function_voice_control_Thread();

signals:
  void dealthread();
  void eraseRect();
  void showMB();
  void buttonClicked(int);
signals:
  //void SendDataSignal(tutorial::RobotData rd);

private Q_SLOTS:
  void TimeUpdate();
  void visualize_Crane_Robot();
  void Timer_visualize_lift_pathplan_scence();
  //  void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;
  void Start_LabelVedio_Point_Slot(QPoint point);
  void Stop_LabelVedio_Point_Slot(QPoint point);
  void Start_Labelimg_Point_Slot(QPoint point);

  void on_Scene_reconstruction_visualization_triggered();
  void on_Accurate_positioning_triggered();
  void on_path_plan_triggered();
  void on_suspended_triggered();

  void on_Trajectory_tracking_triggered();

  void Timer_pathplan_scence_Update();
  void showMessageBox();

  void sendJointTrajectory(std::promise<bool>* start_RTCC_promise);
  bool sendJointTrajectoryPoint(ompl::app::joint_trajectory_point_t point);
  void predictCollision();

  void on_attach_lift_param_setBtn_clicked();

  void on_attach_clicked();

  void on_exit_btn_triggered();

  void on_beep_triggered();

  void on_auto_drive_finished_clicked();
  void on_hook_hang_or_off_finished_clicked();
  void on_posture_adjust_finished_clicked();
  void on_pushButton_one_clicked();
  void on_pushButton_two_clicked();

  void on_pushButton_clicked();

  void on_pushButton_2_clicked();

  void on_pushButton_3_clicked();

  void on_zhuzi1_clicked();

  void on_zhuzi2_clicked();

  void on_zhuzi3_clicked();

  void on_zhuzi4_clicked();

  void on_qiangban1_clicked();

  void on_qiangban2_clicked();

  void on_qiangban3_clicked();

  //void on_pushButton_14_clicked();

  void on_qiangban4_clicked();

  void on_pushButton_6_clicked();

  void on_up_hook_clicked();

  void on_stop_hook_clicked();

  void on_down_hook_clicked();

  void on_pushButton_7_clicked();

  void on_pushButton_5_clicked();

  void on_rotate_left_clicked();

  void on_rotate_stop_clicked();

  void on_rotate_right_clicked();

  void on_luff_up_clicked();

  void on_luff_stop_clicked();

  void on_luff_down_clicked();

  void on_zhuzi_clicked();

  void on_pushButton_9_clicked();

  void on_gaizi_clicked();

  void on_test_liang_clicked();

  void readTxt();

  void writetxt(int id);


  void on_start_target_voice_clicked();

  void on_task_wancheng_voice_clicked();

  void on_recome_safe_voice_clicked();

  void on_pushButton_8_clicked();

  void on_pushButton_10_clicked();

  void on_pushButton_11_clicked();

  void on_pushButton_12_clicked();

  void on_pushButton_13_clicked();

  void on_respose_tower_clicked();

  //void on_board_qian_clicked();

  void on_voice_control_clicked();

  void on_gaizi_hou_clicked();
  
  void on_rotation_right_clicked();

  void on_rotation_left_clicked();

  void on_luff_up_2_clicked();

  void on_luff_down_2_clicked();

  void on_wind_up_clicked();

  void on_wind_down_clicked();

public slots :
  void on_lift_type_Judge(int id);
  void on_target_liftMode_Judge(int id);
  void receive_laser_data();

private:
  void SetStatusBarState(const QString &text, bool is_good);
  void SetToolBarButtonIsEnableState();
  void SetSiamInitialBbox(cv::Mat &img, Rect2d bbox);

  void get_object_cameraCoordinate_based_cameraPixs(const int &_x, const int &_y);
  void get_transform_from_world_to_camera(Eigen::Isometry3f &out_pose);

  void get_ptz_Offset_initial_angle();
  bool Get_Crane_Stretch_Arm_Pitch_Number(vector<int> stretchWay, int &pitchNum);
  void get_accurate_location_offset_parameter();
  QVariant readConfigFileEx(QString configFileName, QString qstrnodename, QString qstrkeyname);

  void clear_all_can_send_data();
  void Send02D0DataCommand();
  void Send02D1DataCommand();
  void Send01D1DataCommand();
  void Send01D0DataCommand();
  void reset_all_parameter_to_initial_status();

  void leftRotation();
  void rightRotation();
  void downAmp();
  void upAmp();
  void upLuff();
  void downLuff();
  void stopRotion();
  void stopAmp();
  void stopLuff();

  void read_lift_weight_form_data();
  void start_Obstacles_Detect_Thread();

  void control_cramera_deflect_to_object_place(float x_angle, float y_angle, bool bflag);
  void filter_collision_objects(const std::vector<ObstacleInforStr> &detected_obstacles, std::vector<ObstacleInforStr> &filtered_obstacles);

  void space2jointCoordinate(double x,double y);
  void jointDiff(double targetRotatonAngel,double targetLuffLength);
  void joint2spaceCoordinate(double rotationAngel,double crane_luff_length);
  void currSpaceAndJointCompute();
  void currSpaceAndJointCompute_luntai();

public:
  void start_path_plan_Thread();
  void on_stop_crane_control();
  //  void get_env_scene();
  void set_new_pathplan_start_positon();
  //jiqunxietong yt add 20220418

  //void sendRobotStatusTo(QVector<float> jointstates, vector<double> GPSPosition, int workstatus, float g_ArmLen, int taskid, double headangle);//状态发送
  void sendTaskStatusTo(int taskid, int stage, int status);//调度任务
  void sendAvoidTaskStatusTo(int taskid, int status);//避让
  void resolveMqttData(mqtt::const_message_ptr msg);
  void sendAutodrvingStatusTo(int taskid,int state);
  void sendCommandTaskStatusTo(int commandid, int status,string message);
  void sendRegeisterTo(double x, double y, double z);
  void sendIcraneStatusTo(int commandid, int status, string message);
  void resetavoidcommandtask_();
  void resetdispatchtask_();
  void resetcommandtask_();
  zl_common::app::pointPostrue position_to_postrue_transform(zl_common::app::structPointXYZ position);
  void play_voice(int index);
  //以上为接收回复消息以及初始化各类任务状态函数

  //jiqunxietong
  void Function_Start_PathPlan_Thread();
  void resetexcavator_task_();
  void resetcompoud_task_();


  void Function_Send_Control_Command_Package_Thread_TestRTK_dizuo();
  void sendRobotStatusTo(QVector<float> jointstates, vector<double> GPSPosition, int workstatus, float g_ArmLen, int taskid, double headangle, vector<double> GPSPositionL);

//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
public:
  VTKWidgetType *qvtkWidget_;
  pcl::visualization::PCLVisualizer::Ptr pclviewer_;
  HookLoadAntiCollisionDetector* collisionWorker_;
  QThread* collisionWorkerThread_;
  void initPCLViewer();// 初始化 PCL 可视化器的核心函数

public slots:
  void updateVisualization(CollisionResult result, 
                           CloudVisVector visInfo);

};

}  // namespace zl_Crane_AutomaticLift_trunk

#endif // zl_Crane_AutomaticLift_trunk_MAIN_WINDOW_H
