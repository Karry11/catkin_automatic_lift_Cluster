/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <main_window.hpp>
#include "main_window.hpp"
#include <qmetatype.h>
// #include<Zl_livox_merge_with_camera/Zl_livox_merge_with_camera.h>


#define OBJECT_HEIGHT   0.1
#define HOOK_WEIGHT     0.8       //1.5t
/*****************************************************************************
** Namespaces
*****************************************************************************/
extern list<Mat>  gframeList_;
extern QReadWriteLock g_ImageListMutex;
//QReadWriteLock g_frame_mutex;
zl_common::app::CRANE_PARAM *crane_parameter_;
namespace zl_Crane_AutomaticLift_trunk {

using namespace Qt;

const double baseFocus[27] = { 1877.68, 2081.22, 2636.52, 3405.36, 4174.49,5237.85,6113.43,7025.73,7801.9,8714.89,9525.58,10240.0,11070.27,11941.69,
                               12800.0,13562.91,14124.14,15170.37,15753.85,16516.13,17282.7,18285.71,19692.31,19504.76,21557.89,23141.24,24526.95};//焦距
const double cameraFov[27][2] = {{28.6, 22.25}, {26.2, 20.255},{21.226, 16.24}, {16.736, 12.71}, {13.782, 10.424},{11.062,8.25},{9.509,7.196},{8.292,6.32},
                                 {7.477,5.711},{6.702,5.171},{6.136,4.716},{5.711,4.289},{5.285,4.033},{4.901,3.719},{4.574,3.491},{4.318,3.291},
                                 {4.147,3.148},{3.862,2.891},{3.719,2.791},{3.548,2.619},{3.391,2.491},{3.205,2.362},{2.977,2.305},{3.005,2.2333},
                                 {2.72,2.09},{2.534,1.919},{2.391,1.833}};//焦段对应的市场角，

const double pixlsWidth[][2] = {{0,0}, {4, 2},{7, 7}, {20, 16}, {27, 23}, {38, 41}, {45, 42}, {56, 54}, {63, 60}, {72, 70}, {81, 79},
                                {91, 90}, {98, 101}, {108, 107}, {116, 118}, {123, 139}, {131, 134}, {140, 144}, {148, 156},
                                {157, 170}, {164, 173}, {170, 177}, {177, 195}, {184, 204}, {201, 220}, {216, 236}, {233, 259}};

const double directAngle[][2] = {{0,0}, {0.11,0.055},{0.152,0.152},{0.336,0.269},{0.371,0.316},{0.416,0.448},{0.422,0.394},{0.457,0.44},
                                 {0.463,0.441},{0.473, 0.46},{0.487,0.475},{0.509,0.504},{0.507,0.523},{0.518,0.513},{0.519,0.528},{0.52,0.587},
                                 {0.531,0.544},{0.529,0.544},{0.538,0.567},{0.545,0.59},{0.544,0.574},{0.533,0.555},{0.515,0.567},{0.54,0.599},
                                 {0.534,0.585},{0.535,0.584},{0.544,0.605}};
// lc::lc_merge  *me = new lc::lc_merge();
// mosaic::image_mosaic im ;
cv::Mat depth_img;

int  rtkflag1 =0;


bool qiangban_hook_down = false;
bool zhuzi_bool = false;
bool zuozi_bool = false;
bool gaizi_bool = false;
bool qiangban_bool = false;
int yt_test = 4;
vector<double> gcranepositionL;
int num_qiangban = 0;



/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      collision_check_range_(50)
{
    // 注册自定义类型，确保信号槽能跨线程传递
    qRegisterMetaType<CollisionResult>("CollisionResult");
    qRegisterMetaType<CloudVisVector>("CloudVisVector");
    //************************************************UI界面设置以及信号槽connect处理*******************************************//
    ui.setupUi(this);             // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    setWindowIcon(QIcon(":/images/icon.png"));

    //yt add 20220418 jiqunxietong start
    qRegisterMetaType<mqtt::const_message_ptr>("mqtt::const_message_ptr");
    //initial task information
    resetcommandtask_();
    resetavoidcommandtask_();
    resetdispatchtask_();
    current_goal.amplitude = 0;
    current_goal.roller = 0;
    current_goal.rotate = 0;
    current_goal_position.point_x = 0;
    current_goal_position.point_y = 0;
    current_goal_position.point_z = 0;
    //yt add 20220418 jiqunxietong  end

    //临时部件
    msgLabel  = new QLabel(this);
    ShowTimeLabel  = new QLabel(this);
    ui.statusbar->addWidget(msgLabel);

    QPalette pa;
    pa.setColor(QPalette::Background,  QColor(85, 85, 85));
    ui.statusbar->setPalette(pa);
    ui.statusbar->setStyleSheet("QStatusBar::item{border: 0px}");
    SetStatusBarState(QString::fromLocal8Bit("提示：人工操控模式！"),  false);

    //set timer to  display the realtime
    QTimer *timer_display = new QTimer(this);
    timer_display->start(1000);
    connect(timer_display,SIGNAL(timeout()), this, SLOT(TimeUpdate()));

    SetToolBarButtonIsEnableState();

    ui.tabWidget_display->setCurrentIndex(0);
    ui.tabWidget_param->setCurrentIndex(0);
    ui.tabWidget_display->setStyleSheet(
                "QTabWidget#tabWidget{background-color:rgb(245,245,245);}\
                QTabBar::tab{background-color:rgb(200,200,200);color:rgb(0,0,0);}\
                QTabBar::tab::selected{background-color:rgb(0,255,0);color:rgb(0,0,0);}"
                );
    ui.pathplan_tabWidget->setStyleSheet(
                "QTabWidget#tabWidget{background-color:rgb(245,245,245);}\
                QTabBar::tab{background-color:rgb(200,200,200);color:rgb(0,0,0);}\
                QTabBar::tab::selected{background-color:rgb(0,255,0);color:rgb(0,0,0);}"
                );
    ui.tabWidget_param->setStyleSheet(
                "QTabWidget#tabWidget{background-color:rgb(245,245,245);}\
                QTabBar::tab{background-color:rgb(200,200,200);color:rgb(0,0,0);}\
                QTabBar::tab::selected{background-color:rgb(0,255,0);color:rgb(0,0,0);}"
                );

    GenderGroup = new QButtonGroup(this);
    GenderGroup->addButton(ui.hook_mode,0);
    GenderGroup->addButton(ui.lift_mode,1);
    ui.hook_mode->setChecked(true);
    connect (GenderGroup, SIGNAL(buttonClicked(int)), this, SLOT(on_lift_type_Judge(int)));

    m_TargetGroup = new QButtonGroup(this);
    m_TargetGroup->addButton(ui.checkboard_mode,0);
    m_TargetGroup->addButton(ui.anything_mode,1);
    ui.anything_mode->setChecked(true);
    connect (m_TargetGroup, SIGNAL(buttonClicked(int)), this, SLOT(on_target_liftMode_Judge(int)));

    label_vedio_ = new zl_common::app::QLabelEx(this);
    label_img_ = new zl_common::app::QLabelEx(this);
    connect(label_vedio_,SIGNAL(StartPointSignal(QPoint)),this,SLOT(Start_LabelVedio_Point_Slot(QPoint)));         //关联开始坐标的信号
    connect(label_vedio_,SIGNAL(StopPointSignal(QPoint)),this,SLOT(Stop_LabelVedio_Point_Slot(QPoint)));         //关联结束坐标的信号
    connect(label_img_,SIGNAL(ClickPointSignal(QPoint)),this,SLOT(Start_Labelimg_Point_Slot(QPoint)));
    connect(this,SIGNAL(eraseRect()),label_vedio_,SLOT(eraseRectRectBox()));
    //ui.scence_gridLayout->addWidget(label_img_);
    //ui.vedio_gridLayout->addWidget(label_vedio_);             //自定义label添加到布局
    //************************************************UI界面设置以及信号槽connect处理*******************************************//
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //************************************************若干类对象实例化*******************************************//
    base_path = getenv("HOME");
    std::string init_file_path = base_path + "/config.ini";
    char *initFile = const_cast<char*>(init_file_path.c_str());

    init_file_path = base_path + "/HPconfig.ini";               //initial HPconfig data
    configFileName = QString::fromStdString(init_file_path);

    Camera_ = new zl_common::app::CameraObject();
    m_obstacle_detect_class = new ObstacleDetection();

    rtk = new udpRtk();
    rtk->init();

    //pcan初始化
    m_clsClientUsbToCan = new clientUsbToCanCommunication();
    m_clsClientToYunTai = new PTZ_TCPClientSocket();
    crane_parameter_ = new  zl_common::app::CRANE_PARAM;
    hcfg_ = new zl_common::app::HParameter;
    m_psetting = new QSettings(configFileName, QSettings::IniFormat);

    // m_clsClientUsbToCan->OpenUsbCanDevice();
    //创建mqtt通讯机制 add by yt start
    mqtt_client_ = new mqtt_client();
    connect(mqtt_client_->cb,&callback::sendMsgToClient,this,&MainWindow::resolveMqttData);
    //路径规划
    glWidget_ = new ompl::app::GLWidget(ui.pathplan_openGLWidget);
    ui.pathplan_gridLayout->addWidget(glWidget_, 0, 0, 1, 1);

    auto_crane_manipulation_ = new ompl::app::AutoCraneArmManipulation();
    auto_crane_manipulation_->set_auto_crane_link_length(16.9);

    //  auto_crane_manipulation_->add_cube_collision_object("aaa", 3.5, 2.0, 10.0, -2.0, 5.0, 5, 0, 0, 0);
    //  auto_crane_manipulation_->add_cube_collision_object("abc", 3.5, 2.0, 10.0, 0.0, 7.0, 5, 0, 0, 0);
    //  auto_crane_manipulation_->add_cube_collision_object("aab", 1, 5.0, 16.0, 6.0, 10.0, 8, 0, 0, 0);
    //  auto_crane_manipulation_->add_cube_collision_object("a", 3, 0.5, 16.0, 8.0, 0.0, 8, 0, 0, 0);

    current_joint_state_.positions.push_back(0.2);//(1.56017);
    current_joint_state_.positions.push_back(1.2);//(0.684344);
    current_joint_state_.positions.push_back(10);//(14.88);

    std::cerr <<" get_ptz_Offset_initial_angle is preparing." << std::endl;
    //  visualize_Crane_Robot();
    get_ptz_Offset_initial_angle();
    std::cerr << "get_ptz_Offset_initial_angle is finished. "<< std::endl;
    m_obstacle_detect_class->InitialParaConfig(initFile);
    //************************************************若干类对象实例化*******************************************//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    is_selectObjectFlag_ = false;
    m_vedioRoi_x = 0;       //视频图像框选目标左上角坐标
    m_vedioRoi_y = 0;
    m_vedioRoi_w = 0;
    m_vedioRoi_h = 0;

    current_joint_state_.positions.push_back(0.2);//(1.56017);
    current_joint_state_.positions.push_back(1.2);//(0.684344);
    current_joint_state_.positions.push_back(10);//(14.88);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //*********************************************路径规划相关变量与对象初始化**************************************//
    //  motion planning visualization widget
    planning_scene_visualization_timer_ = new QTimer;
    planning_scene_visualization_timer_->setInterval(100);
    connect(planning_scene_visualization_timer_, &QTimer::timeout, this, &MainWindow::Timer_visualize_lift_pathplan_scence);
    //  app_update_timer_ = new QTimer;
    //  app_update_timer_->setInterval(200);
    //  connect(app_update_timer_, &QTimer::timeout, this, &MainWindow::Timer_pathplan_scence_Update);
    planning_scene_visualization_timer_->start();

    is_replanning_.store(false);
    is_predicted_collision_.store(false);
    is_running_predict_collision_.store(false);
    predicted_collision_index_.store(0);
    sent_joint_trajectory_index_.store(0);
    current_joint_state_.positions.resize(3);
    current_joint_state_.velocities.resize(3);
    previous_joint_state_.positions.resize(3);
    previous_joint_state_.velocities.resize(3);

    //  m_pathplan_stop = false;
    //*********************************************路径规划相关变量与对象初始化**************************************//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //****************************************	精准识别与定位对象初始化************************************************//
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //initial siam tracking
    // m_temple_net = torch::jit::load("/usr/local/model/init_model.pt");
    // m_detect_net = torch::jit::load("/usr/local/model/trace_model.pt");
    // siamrpn_tracker_ = new SiamRPNTracker(m_temple_net, m_detect_net);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros_thread = new subscriberThread();
    thread = new QThread(this);
    ros_thread->moveToThread(thread);
    thread->start();
    connect(this, SIGNAL(dealthread()), ros_thread, SLOT(thread_Spin()));
    Q_EMIT dealthread();
    connect(ros_thread, SIGNAL(Signal()), this, SLOT(receive_laser_data()));
    //  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  m_clsClientToYunTai->initlizePTZParam(hcfg_->ptz_str_pitch_initial_angle, hcfg_->ptz_str_rotate_initial_angle);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ui.edit_hook_height->setText("1.6");
    ui.edit_goods_w->setText("1.4");
    ui.edit_goods_h->setText("0.5");
    ui.edit_goods_len->setText("1.8");
    ui.edit_goods_weight->setText("5");
    ui.edit_sling_height->setText("1.5");
    ui.edit_target_height->setText("1");

    ui.edit_circle_length->setText("3");
    ui.edit_circle_radius->setText("1");
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(!m_clsClientUsbToCan->OpenUsbCanDevice())
    {
        std::cerr<<"pcan initial success"<<std::endl;
    }
    reset_all_parameter_to_initial_status();

    if(start_pcan_data_receive_thread_.joinable())
    {
        start_pcan_data_receive_thread_.join();
    }
    is_start_pcan_data_receive_thread_ = true;
    start_pcan_data_receive_thread_ = std::thread(&MainWindow::Function_pcan_data_receive_Thread, this);

    if(start_play_voice_thread_.joinable())
    {
        start_play_voice_thread_.join();
    }
    is_start_play_voice_thread_ = true;
    start_play_voice_thread_ = std::thread(&MainWindow::Function_play_voice_Thread, this);


    //  std::cerr <<" Function_pcan_data_receive_Thread is started." << std::endl;
    //    if(start_cramera_real_ptz_control_thread_.joinable())
    //    {
    //        start_cramera_real_ptz_control_thread_.join();
    //    }
    //    is_start_cramera_real_ptz_control_thread_ = true;
    //    start_cramera_real_ptz_control_thread_ = std::thread(&MainWindow::Function_cramera_real_ptz_control_Thread, this);

    if(start_data_update_display_thread_.joinable())
    {
        start_data_update_display_thread_.join();
    }
    is_start_data_update_display_thread_ = true;
    start_data_update_display_thread_ = std::thread(&MainWindow::Function_data_update_display_Thread, this);

    if(start_rtk_data_update_thread_.joinable())
    {
        start_rtk_data_update_thread_.join();
    }
    is_start_rtk_data_update_thread_ = true;
    start_rtk_data_update_thread_ = std::thread(&MainWindow::Function_RTK_data_receive_Thread, this);

    //  std::cerr <<" Function_data_update_display_Thread is started." << std::endl;

//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
    qvtkWidget_ = new VTKWidgetType(ui.pcl_holder);
    // 2. 手动同步尺寸 (假设你希望它和 ui 里的占位符一样大)
    qvtkWidget_->setGeometry(0, 0, ui.pcl_holder->width(), ui.pcl_holder->height());
    // 3. 必须让 VTK 知道窗口大小变了
    qvtkWidget_->GetRenderWindow()->SetSize(ui.pcl_holder->width(), ui.pcl_holder->height());
    // 初始化 PCL Visualizer 并关联到 QVTKWidget
    initPCLViewer(); 

    // --- 线程设置 ---
    collisionWorker_ = new HookLoadAntiCollisionDetector();
    collisionWorkerThread_ = new QThread();
    collisionWorker_->moveToThread(collisionWorkerThread_);
    // 连接信号：当子线程有新数据时，更新 UI
    connect(collisionWorkerThread_, &QThread::started, collisionWorker_, &HookLoadAntiCollisionDetector::process);
    connect(collisionWorker_, &HookLoadAntiCollisionDetector::dataUpdated, this, &MainWindow::updateVisualization);
    collisionWorkerThread_->start();
}

MainWindow::~MainWindow()
{
    on_exit_btn_triggered();
}

void MainWindow::SetStatusBarState(const QString &text, bool is_good)
{
    QPalette pa;
    QFont font;//实例化QFont对象

    font.setFamily("黑体");//字体
    font.setBold(true);
    if(is_good)
        pa.setColor(QPalette::WindowText, Qt::black);
    else
        pa.setColor(QPalette::WindowText, Qt::green);
    msgLabel->setPalette(pa);
    msgLabel->setFont(font);
    msgLabel->setText(text);
}

void MainWindow::TimeUpdate()
{
    QDateTime CurrentTime=QDateTime::currentDateTime();
    QString strTime =  CurrentTime.toString(" yyyy年MM月dd日 hh:mm:ss "); //设置显示的格式

    QPalette pa;
    QFont font;//实例化QFont对象

    font.setFamily("宋体");//字体
    font.setBold(true);
    pa.setColor(QPalette::WindowText, Qt::green);
    ShowTimeLabel->setPalette(pa);
    ShowTimeLabel->setFont(font);
    ShowTimeLabel->setText(QString::fromLocal8Bit(strTime.toStdString().c_str()));

    statusBar()->addPermanentWidget(ShowTimeLabel);
    statusBar()->setSizeGripEnabled(true); //设置是否显示右边的大小控制点

}

//void MainWindow::timerEvent(QTimerEvent *event)
//{
//  if(event->timerId() == timer_display_id)
//  {
//    TimeUpdate();
//  }

//  if(event->timerId() == robot_visual_display_id)
//  {
//    visualize_Crane_Robot();
//  }
//}

void MainWindow::SetToolBarButtonIsEnableState()
{
    ui.toolBar->setStyleSheet(
                "QToolBar{background: rgb(60, 60, 60)}"
                "QToolButton{ color:white }"
                "QToolButton:hover{ background-color:rgb(0,255,0); color:white }"
                "QToolButton:pressed{ background-color:red; color:white }"
                );

    //  ui.path_plan->setEnabled(false);
    //  ui.Trajectory_tracking->setEnabled(false);
    //  ui.Accurate_positioning->setEnabled(false);
    //  ui.suspended->setEnabled(false);

    qApp->processEvents();
}

void MainWindow::on_lift_type_Judge(int id)
{
    if(id == 0)
    {
        m_lift_work_mode_ = 0;
    }
    else if(id == 1)
    {
        m_lift_work_mode_ = 1;
    }
}

void MainWindow::on_target_liftMode_Judge(int id)
{
    if(id == 0)
    {
        m_lift_goods_select_mode_ = 0;       //棋盘格吊装模式
    }
    else if(id == 1)
    {
        m_lift_goods_select_mode_ = 1;       //任意目标模式
    }
}


//开启障碍物检测线程
void MainWindow::start_Obstacles_Detect_Thread()
{
    if(!is_start_obstacles_detect_thread_)
    {
        is_start_obstacles_detect_thread_ = true;
        if(start_obstacles_detect_thread_.joinable())
        {
            start_obstacles_detect_thread_.join();
        }
        start_obstacles_detect_thread_ = std::thread(&MainWindow::Function_start_obstacles_detect_Thread, this);
    }
}


// 获取到数据后
void MainWindow::receive_laser_data()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr _laser_cloud_frame_ = ros_thread->laser_cloud_;

    m_list_laser_mutex.lockForWrite();
    m_list_laser_pointclouds_.push_back(*_laser_cloud_frame_);
    if(m_merge_num_ < 10)
    {
        m_merge_num_++;
        temp_laser_pointcloud += (*_laser_cloud_frame_);
    }
    if(m_merge_num_ == 10)
    {
        *m_laser_cloud_frame_ = temp_laser_pointcloud;
        m_merge_num_ = 0;
        temp_laser_pointcloud.clear();
    }

    if(m_list_laser_pointclouds_.size() > PTS_LENG)
    {
        //qDebug("*************laser data count out of maxsize!***************\n");
        m_list_laser_pointclouds_.pop_front();
    }
    m_list_laser_mutex.unlock();
}

void MainWindow::Function_start_obstacles_detect_Thread()
{
    double groundDistance = 0.0;
    pcl::PointCloud<pcl::PointXYZ> temp_laser_pointcloud;
    int merge_num = 10;

    m_obstacle_detect_class->GetObjectsSize(m_hook_height, m_object_width, m_object_len, m_object_height, m_rope_height,m_goods_height, m_lift_work_mode_);

    while(is_start_obstacles_detect_thread_)
    {
        m_list_laser_mutex.lockForRead();
        //        std::list<pcl::PointCloud<pcl::PointXYZ>>::iterator pointcloud_itor = m_list_laser_pointclouds_.end()-1;
        usleep(10*1000);
        temp_laser_pointcloud.clear();
        if((m_list_laser_pointclouds_.size() < PTS_LENG*0.5))//&& m_list_laser_pointclouds_.size()>merge_num)
        {
            qDebug("laser date false!");
            usleep(10*1000);
            m_list_laser_mutex.unlock();
            continue;
        }
        std::list<pcl::PointCloud<pcl::PointXYZ>>::iterator pointcloud_itor = m_list_laser_pointclouds_.end();
        m_list_laser_mutex.unlock();

        for(int num=0; num<merge_num; num++)
        {
            pointcloud_itor--;
            temp_laser_pointcloud += (*pointcloud_itor);
        }
        *m_laser_cloud_frame_ = temp_laser_pointcloud;

        m_stCraneState.bcAngle = crane_parameter_->crane_luff_Angle_;
        m_stCraneState.bcLength = crane_parameter_->crane_mainarm_length_;
        m_stCraneState.bfLength = crane_parameter_->crane_luff_length;
        m_stCraneState.hzAngle = crane_parameter_->crane_rotation_Angle_;
        m_stCraneState.focus = 0;//test value

        int res = 0;

        //      if(laserVerticalFlag == true)//确保雷达处于垂直朝下
        {
            m_obstacle_detect_class->GetPlaneHight(temp_laser_pointcloud.makeShared(), groundDistance);
        }

        if(m_obstacle_detect_class->m_nMode != m_lift_work_mode_ || m_obstacle_detect_class->m_hook_height != m_hook_height
                || m_obstacle_detect_class->m_goods_height != m_goods_height ||m_obstacle_detect_class->m_object_len != m_object_len
                || m_obstacle_detect_class->m_object_height != m_object_height ||m_obstacle_detect_class->m_object_width != m_object_width
                || m_obstacle_detect_class->m_rope_height != m_rope_height)
        {
            m_obstacle_detect_class->GetObjectsSize(m_hook_height, m_object_width, m_object_len, m_object_height,
                                                    m_rope_height,m_goods_height, m_lift_work_mode_);
        }

        double hook_or_lift_height = 0.0;

        m_param_mutex.lockForRead();
        double crane_hook_height = crane_parameter_->crane_hook_to_ground_height_;
        m_param_mutex.unlock();

        if(groundDistance != 0)
        {
            res = m_obstacle_detect_class->ObstacleDetected(temp_laser_pointcloud.makeShared(), m_stCraneState, m_output_obstacle_Informs_, hook_or_lift_height, groundDistance, crane_hook_height);
        }
        m_hookheight_mutex.lockForWrite();
        if(groundDistance != 0)
        {
            crane_parameter_->laser_camera_to_ground_height_ = groundDistance;
        }
        m_hookheight_mutex.unlock();

        usleep(1000 * 1000);
    }

    is_start_obstacles_detect_thread_ = false;
    qDebug("DoObstacleDetected thread exited.");
}

void MainWindow::control_cramera_deflect_to_object_place(float x_angle,float y_angle, bool bflag)
{
    QString rotate_x_angle_str,rotate_y_angle_str;

    int hundred_num = int(x_angle / 100);
    int ten_num = int(x_angle - (100 * hundred_num));
    int percent_num = int((x_angle - (100 * hundred_num) - ten_num)*100);

    rotate_x_angle_str = "5A 04 82 00 " + QString("%1").arg(hundred_num, 2, 10, QChar('0')) + " " +
            QString("%1").arg(ten_num, 2, 10, QChar('0')) + " " + QString("%1").arg(percent_num, 2, 10, QChar('0'))
            + " 00 AA";
    std::cerr << "rotate_x_angle_str : " << rotate_x_angle_str.toStdString() << std::endl;
    m_clsClientToYunTai->sendByteDataToPTZ(rotate_x_angle_str);

    usleep(100 * 1000);

    hundred_num = int(y_angle / 100);
    ten_num = int(y_angle - (100 * hundred_num));
    percent_num = int((y_angle - (100 * hundred_num) - ten_num)*100);

    if(bflag == true)
    {
        rotate_y_angle_str = "5A 04 81 01 " + QString("%1").arg(hundred_num, 2, 10, QChar('0')) + " " +
                QString("%1").arg(ten_num, 2, 10, QChar('0')) + " " + QString("%1").arg(percent_num, 2, 10, QChar('0'))
                + " 00 AA";
    }
    else
    {

        rotate_y_angle_str = "5A 04 81 00 " + QString("%1").arg(hundred_num, 2, 10, QChar('0')) + " " +
                QString("%1").arg(ten_num, 2, 10, QChar('0')) + " " + QString("%1").arg(percent_num, 2, 10, QChar('0'))
                + " 00 AA";
    }
    std::cerr << "rotate_y_angle_str : " << rotate_y_angle_str.toStdString() << std::endl;
    m_clsClientToYunTai->sendByteDataToPTZ(rotate_y_angle_str);

    //异常处理，判断云台运动是否到位
    double first_hori_angle = 0.0, first_pitch_angle = 0.0, second_hori_angle = 0.0, second_pitch_angle = 0.0;
    m_clsClientToYunTai->RecvData(m_strPTZcontrolCommand, second_hori_angle, first_hori_angle, first_pitch_angle, second_pitch_angle);

    std::cerr<<"--------相机已偏转-------"<<std::endl;
}

void MainWindow::connect_Camera_Pcan()
{
    Camera_->OpenCamera();
    //****************** 打开USB-CAN设备 **************************//
    m_bUsbToCan = m_clsClientUsbToCan->OpenUsbCanDevice();
    if (m_bUsbToCan)
    {
        QMessageBox::information(this, tr("错误"), tr("CAN_Initialize is failed!code = \"%1\"").arg(m_bUsbToCan));
        return;
    }

    //创建线程函数进行视频图像实时显示
    if(open_vedio_stream_thread_.joinable())
    {
        open_vedio_stream_thread_.join();
    }

    is_open_video_player_display_ = true;
    open_vedio_stream_thread_ = std::thread(&MainWindow::Function_open_Vedio_Stream_Thread, this);


    //start Obstacle thread
    //  startObstacleThread();
}

void MainWindow::Function_open_Vedio_Stream_Thread()
{
    Mat image;

    while(is_open_video_player_display_)
    {
        g_ImageListMutex.lockForRead();
        if(gframeList_.size()  <= 0)
        {
            g_ImageListMutex.unlock();
            continue;
        }

        std::list<cv::Mat>::iterator it;
        it = gframeList_.end();
        it--;
        image  = (*(it));

        gframeList_.pop_front();
        gframeList_.clear(); // 丢掉旧的帧
        g_ImageListMutex.unlock();

        m_frame_mutex.lockForWrite();
        m_gframe_ = image.clone();
        m_frame_mutex.unlock();

        if(m_image_width_ == 0)
        {
            m_image_width_ = image.cols;
            m_image_height_ = image.rows;
        }

        //std::cerr <<"g_initBox.empty() ="<<g_initBox.empty()<<"g_SelObjectFlag ="<<g_SelObjectFlag<<std::endl;
        if( is_selectObjectFlag_)
        {
            cv::Rect2d uRoiImgBox;
            uRoiImgBox.x = m_vedioRoi_x;
            uRoiImgBox.y = m_vedioRoi_y;
            uRoiImgBox.width = m_vedioRoi_w;
            uRoiImgBox.height = m_vedioRoi_h;
            //          g_first_bbox = uRoiImgBox;

            std::cerr <<"--------------DoThreadVedioDisplay:初始化孪生网络------------------"<<std::endl;
            // SetSiamInitialBbox(image, uRoiImgBox);
            is_selectObjectFlag_ = false;
        }

        if(is_start_object_position_compute_ /*&& g_bErrorValue*/)
        {
            m_box_mutex.lockForRead();
            cv::rectangle(image, siam_detect_preBox_, Scalar(255, 0, 255), 2, 1);
            cv::putText(image, "FPS : " + std::to_string(camera_frame_rate_), cv::Point(10,40), cv::FONT_HERSHEY_COMPLEX_SMALL,2,cv::Scalar(0,255,255),2,8,0);
            cv::Point centerPoint(siam_detect_preBox_.x + siam_detect_preBox_.width / 2, siam_detect_preBox_.y + siam_detect_preBox_.height / 2);
            m_box_mutex.unlock();

            circle(image, Point(centerPoint.x,centerPoint.y) ,10 , CV_RGB(255,0,255), 10, 8);
        }
        cv::line(image, cvPoint(1, image.rows / 2), cvPoint(image.cols, image.rows / 2 ), Scalar(0,0,255), 2);
        cv::line(image, cvPoint(image.cols / 2, 1), cvPoint(image.cols / 2, image.rows), Scalar(0,0,255), 2);
        circle(image, Point(image.cols / 2,image.rows / 2) ,10 , CV_RGB(0,0,255), 4, 5);
        //      imwrite("/home/xurui/dstImage.jpg", image);
        QImage img_frame = QImage((const unsigned char*)(image.data), image.cols, image.rows, QImage::Format_RGB888);

        m_label_paint_mutex.lock();
        label_vedio_->setPixmap(QPixmap::fromImage(img_frame.scaled(label_vedio_->size())));
        m_label_paint_mutex.unlock();

        usleep(100 * 1000);
    }

    is_open_video_player_display_ = false;
    qDebug("DoThreadVedioDisplay thread exited.");
}

// void MainWindow::SetSiamInitialBbox(cv::Mat &img, Rect2d bbox)
// {
//     if(siamrpn_tracker_ == NULL)
//     {
//         torch::jit::script::Module temple_net = torch::jit::load("/usr/local/model/init_model.pt");
//         torch::jit::script::Module detect_net = torch::jit::load("/usr/local/model/trace_model.pt");
//         siamrpn_tracker_ = new SiamRPNTracker(temple_net, detect_net);
//     }

//     //    if(!g_system_running)
//     //    {
//     //        startObstacleThread();
//     //    }

//     siamrpn_tracker_->init(img, bbox);
// }


bool MainWindow::Get_Crane_Stretch_Arm_Pitch_Number(vector<int> stretchWay, int &pitchNum)
{
    int num = stretchWay.size();
    int notOneCount = 0;
    if(num <= 0)
    {
        std::cerr << "stretchWay read failed ... " << std::endl;
        return false;
    }

    for(int i = 0; i < num; i++)
    {
        if(stretchWay[i] > 1)
        {
            notOneCount+=1;
        }
    }

    pitchNum = notOneCount;
    return true;
}


QVariant MainWindow::readConfigFileEx(QString configFileName, QString qstrnodename, QString qstrkeyname)
{
    QVariant qvar = -1;
    QFileInfo fileInfo(configFileName);
    if(!fileInfo.exists())
    {
        std::cerr << "configFile is not exist!.." << std::endl;
        return qvar;
    }

    if (m_psetting)
    {
        qvar = m_psetting->value(QString("/%1/%2").arg(qstrnodename).arg(qstrkeyname));
    }
    return qvar;
}


void MainWindow::get_accurate_location_offset_parameter()
{
    hcfg_->camera_xdire_angle = readConfigFileEx(configFileName, "CARERA_INSTALL_OFFSET", "camera_xdire_angle").toFloat();
    hcfg_->camera_xdire_angle = readConfigFileEx(configFileName, "CARERA_INSTALL_OFFSET", "camera_ydire_angle").toFloat();

    hcfg_->camera_to_armcenter_x = readConfigFileEx(configFileName, "CARERA_INSTALL_OFFSET", "camera_to_armcenter_x").toFloat();
    hcfg_->camera_to_armcenter_y = readConfigFileEx(configFileName, "CARERA_INSTALL_OFFSET", "camera_to_armcenter_y").toFloat();


    int pitch_number = -1;
    QString condition_type(QString::null);
    if(Get_Crane_Stretch_Arm_Pitch_Number(m_vec_stretchWay, pitch_number))
    {
        switch(pitch_number)
        {
        case 0:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_NULL" : "HOISTING_MODE_STRETCH_NULL"; break;
        case 1:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_ONE" : "HOISTING_MODE_STRETCH_ONE"; break;
        case 2:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_TWO" : "HOISTING_MODE_STRETCH_TWO"; break;
        case 3:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_THREE" : "HOISTING_MODE_STRETCH_THREE"; break;
        case 4:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_FOUR" : "HOISTING_MODE_STRETCH_FOUR"; break;
        case 5:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_FIVE" : "HOISTING_MODE_STRETCH_FIVE"; break;
        case 6:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_SIX" : "HOISTING_MODE_STRETCH_SIX"; break;
        case 7:
            condition_type = m_lift_work_mode_ == 0 ? "EMPTY_HOOK_STRETCH_SEVEN" : "HOISTING_MODE_STRETCH_SEVEN"; break;
        }

        hcfg_->left_rotate_and_luff_up_rotation_amend = readConfigFileEx(configFileName, condition_type, "left_rotate_and_luff_up_rotation_amend").toFloat();
        hcfg_->left_rotate_and_luff_up_luffing_amend = readConfigFileEx(configFileName, condition_type, "left_rotate_and_luff_up_luffing_amend").toFloat();
        hcfg_->left_rotate_and_luff_down_rotation_amend = readConfigFileEx(configFileName, condition_type, "left_rotate_and_luff_down_rotation_amend").toFloat();
        hcfg_->left_rotate_and_luff_down_luffing_amend = readConfigFileEx(configFileName, condition_type, "left_rotate_and_luff_down_luffing_amend").toFloat();
        hcfg_->right_rotate_and_luff_up_rotation_amend = readConfigFileEx(configFileName, condition_type, "right_rotate_and_luff_up_rotation_amend").toFloat();
        hcfg_->right_rotate_and_luff_up_luffing_amend = readConfigFileEx(configFileName, condition_type, "right_rotate_and_luff_up_luffing_amend").toFloat();
        hcfg_->right_rotate_and_luff_down_rotation_amend = readConfigFileEx(configFileName, condition_type, "right_rotate_and_luff_down_rotation_amend").toFloat();
        hcfg_->right_rotate_and_luff_down_luffing_amend = readConfigFileEx(configFileName, condition_type, "right_rotate_and_luff_down_luffing_amend").toFloat();
    }
}

void MainWindow::get_ptz_Offset_initial_angle()
{
    hcfg_->ptz_pitch_initial_angle = readConfigFileEx(configFileName, "PTZ_INITIAL_ANGLE", "ptz_pitch_angle_offset").toFloat();
    hcfg_->ptz_rotate_initial_angle = readConfigFileEx(configFileName, "PTZ_INITIAL_ANGLE", "ptz_rotate_angle_offset").toFloat();

    hcfg_->ptz_str_pitch_initial_angle = readConfigFileEx(configFileName, "PTZ_INITIAL_ANGLE", "ptz_pitch_initial_angle").toString();
    hcfg_->ptz_str_rotate_initial_angle = readConfigFileEx(configFileName, "PTZ_INITIAL_ANGLE", "ptz_rotate_initial_angle").toString();

    hcfg_->camera_transform_ptz_x = readConfigFileEx(configFileName, "COORDIDATE_TRANSFORM_OFFSET", "camera_transform_ptz_x").toFloat();
    hcfg_->camera_transform_ptz_y = readConfigFileEx(configFileName, "COORDIDATE_TRANSFORM_OFFSET", "camera_transform_ptz_y").toFloat();
    hcfg_->camera_transform_ptz_z = readConfigFileEx(configFileName, "COORDIDATE_TRANSFORM_OFFSET", "camera_transform_ptz_z").toFloat();
    hcfg_->ptz_tansform_arm_header_x = readConfigFileEx(configFileName, "COORDIDATE_TRANSFORM_OFFSET", "ptz_tansform_arm_header_x").toFloat();
    hcfg_->ptz_tansform_arm_header_y = readConfigFileEx(configFileName, "COORDIDATE_TRANSFORM_OFFSET", "ptz_tansform_arm_header_y").toFloat();
    hcfg_->ptz_tansform_arm_header_z = readConfigFileEx(configFileName, "COORDIDATE_TRANSFORM_OFFSET", "ptz_tansform_arm_header_z").toFloat();
}


void MainWindow::get_transform_from_world_to_camera(Eigen::Isometry3f& out_pose)
{
    Eigen::Isometry3f main_link_end_to_world_tf,main_link_world_to_armHeader_tf;
    std::cerr <<"主臂高度 主臂高度 crane_arm_to_ground_height_"<<crane_parameter_->crane_arm_to_ground_height_<<std::endl;
    //设置臂长
    auto_crane_manipulation_->set_auto_crane_link_length(crane_parameter_->crane_mainarm_length_);

    if (auto_crane_manipulation_->get_main_link_end_to_world_tf(crane_parameter_->crane_rotation_Angle_ * M_PI/180,
                                                                crane_parameter_->crane_luff_Angle_ * M_PI/180,//crane_parameter_->crane_luff_Angle_
                                                                crane_parameter_->crane_arm_to_ground_height_,//crane_parameter_->crane_arm_to_ground_height_
                                                                main_link_end_to_world_tf))
    {
        main_link_world_to_armHeader_tf = main_link_end_to_world_tf.inverse();
    }
    else
    {
        throw "get_main_link_end_to_world_tf failed ...";
    }

    //臂尖坐标系到云台固定安装面中心点转换
    Eigen::Isometry3f tf_2 = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI/2 + crane_parameter_->crane_luff_Angle_*M_PI/180,
                                                                 Eigen::Vector3f::UnitX())
                                               *Eigen::AngleAxisf(M_PI/2,
                                                                  Eigen::Vector3f::UnitZ()));
    //    tf_2.pretranslate(Eigen::Vector3f (-0.07,0.64,0.97));//平移向量(-0.54,0.29,0.55 )(0.54,-0.29,-0.55 )//臂尖到云台安装面
    tf_2.pretranslate(Eigen::Vector3f ( -hcfg_->ptz_tansform_arm_header_x, -hcfg_->ptz_tansform_arm_header_y, -hcfg_->ptz_tansform_arm_header_z));//平移向量

    //云台固定安装面中心点到相机中心点坐标转换
    Eigen::Isometry3f tf_3 = Eigen::Isometry3f::Identity();
    //    tf_3.pretranslate(Eigen::Vector3f ( -0.052,0.0062,0.477 ));//平移向量
    tf_3.pretranslate(Eigen::Vector3f ( -hcfg_->camera_transform_ptz_x, -hcfg_->camera_transform_ptz_y, -hcfg_->camera_transform_ptz_z));//平移向量
    //out_pose = tf_3 * tf_2 * main_link_world_to_armHeader_tf;
    out_pose = tf_3 * tf_2 * main_link_world_to_armHeader_tf;
}

//倾斜相机到世界坐标系
void MainWindow::get_object_cameraCoordinate_based_cameraPixs(const int &_x,const int &_y)
{
    if (_x <= 0 || _x >= m_gframe_.cols || _y <= 0 || _y >= m_gframe_.rows)
    {
        std::cerr << "x or y is out of range ...." << std::endl;
        return;
    }

    if(crane_parameter_->laser_camera_to_ground_height_ < 1e-6)
    {
        std::cerr << "g_camerato_ground_height : " << crane_parameter_->laser_camera_to_ground_height_ << " do not get from livox ..." << std::endl;
        return;
    }

    std::cerr << "g_camerato_ground_height : " << crane_parameter_->laser_camera_to_ground_height_ << std::endl;
    float xx=0.0,yy=0.0;
    float j = _x + 0.5;
    float i = _y + 0.5;

    if (j > m_gframe_.cols / 2)
        xx = crane_parameter_->laser_camera_to_ground_height_ / tan((90 - camera_pitch_angle_) * M_PI/180 - atan((j - m_gframe_.cols / 2) * 2 * tan((cameraFov[m_focusIndex][0]) * M_PI/180) / m_gframe_.cols));
    else
        xx = crane_parameter_->laser_camera_to_ground_height_ / tan((90 - camera_rotate_angle_) * M_PI/180 + atan((m_gframe_.cols / 2 - j) * 2 * tan((cameraFov[m_focusIndex][0]) * M_PI/180 ) / m_gframe_.cols));
    yy = sqrt(crane_parameter_->laser_camera_to_ground_height_*crane_parameter_->laser_camera_to_ground_height_ + xx*xx) * (i - m_gframe_.rows / 2) * tan((cameraFov[m_focusIndex][1]) * M_PI/180) * 2 / m_gframe_.rows;
    // 转换成二维坐标系坐标
    yy = -yy;
    std::cerr << "xx : " << xx << " yy : " << yy << std::endl;

    //#ifdef USE_RIDE
    //    xx = xx*(1 - OBJECT_HEIGHT / g_camerato_ground_height);
    //    yy = yy*(1 - OBJECT_HEIGHT / g_camerato_ground_height);
    //#endif
    //transform to camera coordination system
    Eigen::Vector2f slipe_camera_pose(xx,yy);
    Eigen::Isometry2f slipe_to_camera_transform;
    Camera_->get_transform_from_rotateCamera_to_initCamera(slipe_to_camera_transform, camera_rotate_angle_);
    Eigen::Vector2f camera_2d_pose = slipe_to_camera_transform * slipe_camera_pose;

    std::cerr << "camera_x : " << camera_2d_pose[0] <<" " <<"camera_y : " << -camera_2d_pose[1] << std::endl;
    Eigen::Vector3f start_position(camera_2d_pose[0],-camera_2d_pose[1],crane_parameter_->laser_camera_to_ground_height_-OBJECT_HEIGHT);

    //transform to armHeader coordination system
    Eigen::Isometry3f camera_to_armHead_transform;
    Camera_->get_transform_from_camera_to_armHead(camera_to_armHead_transform, hcfg_, crane_parameter_);
    Eigen::Vector3f armHead_position = camera_to_armHead_transform * start_position;

    reality_center_point_x = armHead_position[0];
    reality_center_point_y = armHead_position[1];
    reality_center_point_z = armHead_position[2];

    std::cerr << "x : " << armHead_position[0] << " y : " << armHead_position[1] << " z : " << armHead_position[2] << std::endl;
    Eigen::Isometry3f main_link_end_to_world_tf;

    //设置臂长
    auto_crane_manipulation_->set_auto_crane_link_length(crane_parameter_->crane_mainarm_length_);

    if (auto_crane_manipulation_->get_main_link_end_to_world_tf(crane_parameter_->crane_rotation_Angle_ * M_PI/180,
                                                                crane_parameter_->crane_luff_Angle_ * M_PI/180,//crane_parameter_->crane_luff_Angle_
                                                                crane_parameter_->crane_arm_to_ground_height_,
                                                                main_link_end_to_world_tf))
    {
        std::cerr << "main_link_end_to_world_tf: " << main_link_end_to_world_tf.translation()[0] << " " << main_link_end_to_world_tf.translation()[1] << " " << main_link_end_to_world_tf.translation()[2] << std::endl;
        Eigen::Vector3f world_pose = main_link_end_to_world_tf  * armHead_position;

        std::cerr << "world_pose_x : " << world_pose[0] << std::endl;
        std::cerr << "world_pose_y : " << world_pose[1] << std::endl;
        std::cerr << "world_pose_z : " << world_pose[2] << std::endl;
        object_init_world_x = world_pose[0];
        object_init_world_y = world_pose[1];
    }
    else
    {
        throw "get_main_link_end_to_world_tf failed ...";
    }
}

void MainWindow::elimination_mismatch_measure_three(bool &bErrorValue, bool &is_compute_mean)
{
    if(is_compute_mean)
    {
        for(int i=0; i<16; i++)
        {
            g_TargetC4.push_back(g_TargetC);
            g_TargetR4.push_back(g_TargetR);
        }
        is_compute_mean=false;
    }
    else
    {
        if(fabs(g_TargetC-(std::accumulate(std::begin(g_TargetC4),std::end(g_TargetC4),0.0))/g_TargetC4.size())< 0.8 &&  fabs(g_TargetR-(std::accumulate(std::begin(g_TargetR4),std::end(g_TargetR4),0.0))/g_TargetR4.size())<0.4)
        {
            g_TargetC4.push_back(g_TargetC);
            g_TargetC4.erase(g_TargetC4.begin());
            g_TargetR4.push_back(g_TargetR);
            g_TargetR4.erase(g_TargetR4.begin());

            bErrorValue = true;
        }
        else
        {
            g_TargetC = (std::accumulate(std::begin(g_TargetC4),std::end(g_TargetC4),0.0))/g_TargetC4.size();
            g_TargetR = (std::accumulate(std::begin(g_TargetR4),std::end(g_TargetR4),0.0))/g_TargetR4.size();
            bErrorValue = false;
        }
    }
}

void MainWindow::start_siamRPN_accurate_position()
{
    g_TargetC4.clear();
    g_TargetR4.clear();
    g_TargetC = 0.0;
    g_TargetR= 0.0;

    get_accurate_location_offset_parameter();

    if(is_trajectory_track_slow_to_stop_complete_ == true)
    {
        sleep(5);
    }

    //创建线程函数进行视频图像实时显示
    if(start_object_position_compute_thread_.joinable())
    {
        start_object_position_compute_thread_.join();
    }

    if(send_control_command_package_.joinable())
    {
        send_control_command_package_.join();
    }

    is_start_object_position_compute_ = true;
    is_send_control_command_package_ = true;
    //start_object_position_compute_thread_ = std::thread(&MainWindow::Function_SiamRPN_Accurate_Position_Thread, this);
    //is_send_control_command_package_autodrving = true;
    //send_control_command_package_autodrving = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread, this);
    std::cerr<<"22222222222222222"<<std::endl;
    //send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK, this);
    send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
    //send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_direct, this);
}

// void MainWindow::Function_SiamRPN_Accurate_Position_Thread()
// {
//     Mat img;
//     int64 tickCount,tockcount;
//     //    GetCondtionOffsetParam();      //get the stretchway of current crane
//     qDebug("start Function_SiamRPN_Accurate_Position_Thread...");

//     int cout_num = 0;
//     cv::Size patternSize = cvSize(4, 6);
//     vector<Point2f> detectCorners;
//     double scale = 0.7;

//     bool bFound = false;
//     bool is_have_mismatch_value= true;
//     bool is_compute_object_mean_measure = true;

//     while(is_start_object_position_compute_)
//     {
//         cout_num++;
//         Rect2d pred_bbox;
//         //      bool isSiamMatch = false;
//         m_frame_mutex.lockForRead();
//         if(m_gframe_.empty())
//         {
//             m_frame_mutex.unlock();
//             usleep(50 * 1000);
//             continue;
//         }
//         qDebug ("---------------m_gframe_.clone() start-----------------");
//         img = m_gframe_.clone();
//         m_frame_mutex.unlock();
//         qDebug( "---------------m_gframe_.clone() end-----------------");
//         if(!bFound && m_lift_goods_select_mode_ == 0)
//         {
//             //********************* 检测棋盘格 *****************************
//             Mat gray_frame, binary_frame, frameTemp;
//             cv::resize(img, frameTemp, cv::Size(0, 0), scale, scale);
//             imwrite(base_path + "pictrue1.jpg",frameTemp);
//             cvtColor(frameTemp, gray_frame, COLOR_BGR2GRAY);
//             adaptiveThreshold(gray_frame, binary_frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 25, 20);

//             //        erode(binary_frame, binary_frame, element);
//             //          imwrite(base_path + "pictrue.jpg",binary_frame);
//             bFound = findChessboardCorners(binary_frame, patternSize, detectCorners, 3);

//             if (bFound && detectCorners.size() == (patternSize.height*patternSize.width))
//             {
//                 int baseWidth = patternSize.width;
//                 int baseHeight = patternSize.height;

//                 cv::Rect2d uRoiImgBox;
//                 uRoiImgBox.x = detectCorners[baseWidth*baseHeight / 2 - 1].x / scale - 60;
//                 uRoiImgBox.y = detectCorners[baseWidth*baseHeight / 2 - 1].y / scale - 60;
//                 uRoiImgBox.width = 120;
//                 uRoiImgBox.height = 120;

//                 SetSiamInitialBbox(img, uRoiImgBox);
//                 is_detect_Chessboard_success = true;
//             }
//             else
//             {
//                 bFound = false;
//                 continue;
//             }
//         }

//         m_hookheight_mutex.lockForRead();
//         double _camerato_ground_height = crane_parameter_->laser_camera_to_ground_height_;
//         m_hookheight_mutex.unlock();

//         m_param_mutex.lockForRead();
//         double _dAmplitudeValue = crane_parameter_->crane_luff_length;
//         double _dAngleValue = crane_parameter_->crane_luff_Angle_;
//         double _dRotationAngle = crane_parameter_->crane_rotation_Angle_;
//         m_param_mutex.unlock();

//         if(!img.empty())
//         {
//             tickCount = cv::getTickCount();
//             siamrpn_tracker_->update(img, pred_bbox);
//             tockcount = cv::getTickCount() - tickCount;

//             if((pred_bbox.width ==0) || (pred_bbox.height ==0))
//             {
//                 continue;
//             }

//             m_box_mutex.lockForWrite();
//             siam_detect_preBox_ = pred_bbox;
//             camera_frame_rate_ = 1.0 / (tockcount / cv::getTickFrequency());
//             m_box_mutex.unlock();

//             cv::rectangle(img, siam_detect_preBox_, Scalar(255, 0, 255), 2, 1);
//             cv::Point centerPoint(siam_detect_preBox_.x + siam_detect_preBox_.width / 2, siam_detect_preBox_.y + siam_detect_preBox_.height / 2);
//             circle(img, Point(centerPoint.x,centerPoint.y) ,10 , CV_RGB(255,0,255), 10, 8);

//             cv::line(img, cvPoint(1, img.rows / 2), cvPoint(img.cols, img.rows / 2 ), Scalar(0,0,255), 2);
//             cv::line(img, cvPoint(img.cols / 2, 1), cvPoint(img.cols / 2, img.rows), Scalar(0,0,255), 2);
//             circle(img, Point(img.cols / 2,img.rows / 2) ,10 , CV_RGB(0,0,255), 4, 5);

//             zl_common::app::MyPoint _targetCrossPoint;
//             double distance_x_ = 0.0, distance_y_ = 0.0;

//             _targetCrossPoint.m_pointX = pred_bbox.x + pred_bbox.width / 2;
//             _targetCrossPoint.m_pointY = pred_bbox.y + pred_bbox.height / 2;

//             image_targetRelativeLen_x_ = (img.cols / 2 - _targetCrossPoint.m_pointX) * (_camerato_ground_height - m_object_height)/baseFocus[m_focusIndex]/* + pixls_xx*/;     //目标几何中心相对图像中心像素偏移量
//             image_targetRelativeLen_y_ = (img.rows / 2 - _targetCrossPoint.m_pointY) * (_camerato_ground_height - m_object_height)/baseFocus[m_focusIndex]/* - pixls_yy*/;

//             distance_x_ = image_targetRelativeLen_x_ - hcfg_->camera_to_armcenter_x - _camerato_ground_height*tan(/*0.5*/hcfg_->camera_xdire_angle*M_PI/180);
//             distance_y_ = _dAmplitudeValue + image_targetRelativeLen_y_ - hcfg_->camera_to_armcenter_y*cos(_dAngleValue* M_PI/180) + 0.131*sin(_dAngleValue* M_PI/180) + _camerato_ground_height*tan(hcfg_->camera_ydire_angle*M_PI/180) + 0.2 * sin(_dAngleValue* M_PI/180);

//             double temp_angle = atan(distance_x_ / distance_y_) * 180 / 3.14159;
//             if(fabs(temp_angle) <= 0.5)
//             {
//                 continue;
//             }
//             if(distance_x_ < 0)      //右回转
//             {
//                 //起变幅
//                 if(image_targetRelativeLen_y_ <0)
//                 {
//                     g_TargetC = _dRotationAngle + atan(distance_x_ / distance_y_) * 180 / 3.14159 + hcfg_->right_rotate_and_luff_up_rotation_amend;//*+ 0.75*/;
//                     g_TargetR = sqrt(pow(distance_y_, 2) + pow(distance_x_, 2)) + hcfg_->right_rotate_and_luff_up_luffing_amend;// + 0.15/*- 0.85*/;
//                 }
//                 else//落变幅
//                 {
//                     g_TargetC = _dRotationAngle + atan(distance_x_ / distance_y_) * 180 / 3.14159 + hcfg_->right_rotate_and_luff_down_rotation_amend;//*+ 0.75*/;
//                     g_TargetR = sqrt(pow(distance_y_, 2) + pow(distance_x_, 2)) + hcfg_->right_rotate_and_luff_down_luffing_amend;// + 0.15/*- 0.85*/;
//                 }
//             }
//             else        //左回转
//                 //起变幅
//                 if(image_targetRelativeLen_y_ <0)
//                 {
//                     g_TargetC = _dRotationAngle + atan(distance_x_ / distance_y_) * 180 / 3.14159 + hcfg_->left_rotate_and_luff_up_rotation_amend;//- 0.25/* + 0.4*/;
//                     g_TargetR = sqrt(pow(distance_y_, 2) + pow(distance_x_, 2)) + hcfg_->left_rotate_and_luff_up_luffing_amend;// + 0.15/*- 0.85*/;
//                 }
//                 else//落变幅
//                 {
//                     g_TargetC = _dRotationAngle + atan(distance_x_ / distance_y_) * 180 / 3.14159 + hcfg_->left_rotate_and_luff_down_rotation_amend;//- 0.25/* + 0.4*/;
//                     g_TargetR = sqrt(pow(distance_y_, 2) + pow(distance_x_, 2)) + hcfg_->left_rotate_and_luff_down_luffing_amend;// + 0.15/*- 0.73*/;
//                 }
//         }

//         //跟丢策略3-前4 or n次正确的均值
//         if(cout_num > 40) //&& fabs(g_TargetC - crane_parameter_->crane_rotation_Angle_) < 2 && fabs(g_TargetR - g_dAmplitudeValue) < 0.4)
//         {
//             elimination_mismatch_measure_three(is_have_mismatch_value, is_compute_object_mean_measure);
//         }

//         m_target_value_mutex.lockForWrite();
//         m_tempCopyTargetC_ = g_TargetC;
//         m_tempCopyTargetR_ = g_TargetR;
//         m_isDetectTargetFlag = true;
//         m_target_value_mutex.unlock();

//         if(cout_num > 6000)
//         {
//             cout_num = 41;
//         }
//         usleep(50 * 1000);
//     }

//     is_start_object_position_compute_ = false;
// }

void MainWindow::Send01D0DataCommand()
{
    QString strSendID01D0 = "01D0";
    QString strSendData01D0 = "";
    for (int i = 0; i < 8; i++)
    {
        strSendData01D0 += m_clsClientUsbToCan->Data01D0[i];
    }
    qDebug()<<"ID:"<<strSendID01D0<<"Data:"<<strSendData01D0<<endl;
    m_clsClientUsbToCan->SendCanData(strSendID01D0, strSendData01D0);
}

//发送精准定位控制命令
void MainWindow::Send01D1DataCommand()
{
    QString strSendID01D1 = "01D1";
    QString strSendData01D1 = "";
    for (int i = 0; i < 8; i++)
    {
        strSendData01D1 += m_clsClientUsbToCan->Data01D1[i];
    }
    m_clsClientUsbToCan->SendCanData(strSendID01D1, strSendData01D1);
}

void MainWindow::Send02D0DataCommand()
{
    QString strSendID02D0 = "02D0";
    QString strSendData02D0 = "";
    for (int i = 0; i < 8; i++)
    {
        strSendData02D0 += m_clsClientUsbToCan->Data02D0[i];
    }
    m_clsClientUsbToCan->SendCanData(strSendID02D0, strSendData02D0);
}

void MainWindow::Send02D1DataCommand()
{
    QString strSendID02D1 = "02D1";
    QString strSendData02D1 = "";
    for (int i = 0; i < 8; i++)
    {
        strSendData02D1 += m_clsClientUsbToCan->Data02D1[i];
    }
    m_clsClientUsbToCan->SendCanData(strSendID02D1, strSendData02D1);
}

void MainWindow::Function_Send_Control_Command_Package_Thread()
{
    std::cerr<<"yyyyyyyyyyyyyyyyyyyy"<<std::endl;
    double _dHook_to_ground_height = 0.0;
    sleep(3);
    bool _bRotation_locate_finished = false;
    bool _bLuff_locate_finished = false;
    bool _bWinch_locate_finished = false;
    bool _bContinue_control_luff = false;

    bool hook_up_over = false;
    float roatate_stop_threshold = 0.0;
    while(is_send_control_command_package_autodrving)
    {
        //std::cerr<<"yyyyyyyyyyyyyyyyyyyy"<<std::endl;
        rtk->process();//获取数据

        if(rtk->hookData ==0.0)
        {
            std::cerr<<"the i_crane data is error"<<std::endl;
            continue;
        }
        //dHookHeightCali_ =rtk->hookData -  targetRtkHeight;
        //crane_parameter_->crane_hook_to_ground_height_ =  rtk->hookData - 48.36;

        _dHook_to_ground_height = rtk->hookData - 48.36;
        //    if(m_lift_work_mode_ == 1)
        //    {
        //      _dHook_to_ground_height = crane_parameter_->laser_hook_to_ground_height - m_rope_height - m_object_height;
        //      if(_dHook_to_ground_height <= 0)
        //        _dHook_to_ground_height = 0.2;
        //    }
        //    if(m_lift_work_mode_ == 0)
        //    {
        //      _dHook_to_ground_height = crane_parameter_->crane_hook_to_ground_height_;
        //      if(_dHook_to_ground_height <= 0)
        //        _dHook_to_ground_height = 0.2;
        //    }

        m_lift_work_status_ = 1;
        m_target_value_mutex.lockForRead();
        //    double tempCopyTargetR = m_tempCopyTargetR_;
        //    double tempCopyTargetC = m_tempCopyTargetC_;
        double tempCopyTargetR = 8.0;
        double tempCopyTargetC = 0;
        if(m_lift_work_mode_ == 1)
        {
            tempCopyTargetC = 180;
        }

        //std::cerr<<"currect r"<<tempCopyTargetC<<std::endl;
        //    if(m_lift_work_mode_ == 1)
        //    {
        //      tempCopyTargetR = 6;
        //      tempCopyTargetC = 60;
        //    }
        //    else if(m_lift_work_mode_ == 2)
        //    {
        //      tempCopyTargetR = 8;
        //      tempCopyTargetC = 90;
        //    }
        //    else if(m_lift_work_mode_ == 3)
        //    {
        //      tempCopyTargetR = 6;
        //      tempCopyTargetC = 120;
        //    }
        //    else if(m_lift_work_mode_ == 4)
        //    {
        //      tempCopyTargetR = 8;
        //      tempCopyTargetC = 150;
        //    }
        //    else if(m_lift_work_mode_ == 5)
        //    {
        //      tempCopyTargetR = 6;
        //      tempCopyTargetC = 190;
        //    }
        //    else if(m_lift_work_mode_ == 6)
        //    {
        //      tempCopyTargetR = 6;
        //      tempCopyTargetC = 50;
        //    }
        //    else if(m_lift_work_mode_ == 7)
        //    {
        //      tempCopyTargetR = 6;
        //      tempCopyTargetC = 330;
        //    }
        //    double tempCopyTargetR = 8;
        //    double tempCopyTargetC = 180;
        bool isDetectTargetFlag  = true;
        m_target_value_mutex.unlock();

        m_param_mutex.lockForRead();
        double dRotationAngle_ = crane_parameter_->crane_rotation_Angle_;
        if((tempCopyTargetC >= 0) && (tempCopyTargetC <90))
        {
            if((dRotationAngle_ > 270) && (dRotationAngle_ <=360))
            {
                dRotationAngle_ = dRotationAngle_ - 360;
            }
        }
        if((dRotationAngle_ >= 0) && (dRotationAngle_ <90))
        {
            if((tempCopyTargetC > 270) && (tempCopyTargetC <=360))
            {
                tempCopyTargetC = tempCopyTargetC - 360;
            }
        }
        double dAmplitudeValue_ = crane_parameter_->crane_luff_length;
        double dAngleValue_ = crane_parameter_->crane_luff_Angle_;
        double dHookHeightCali_ = crane_parameter_->crane_hook_to_ground_height_ + 2.3;
        m_param_mutex.unlock();
        //double goalHeight = crane_parameter_->crane_arm_to_ground_height_ - g_dHookHeight;  //吊钩到臂尖的高度 g_dHookHeight
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

        if(hook_up_over == false)
        {
            if(_dHook_to_ground_height < 9.4)//7.0
            {

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                Send01D1DataCommand();
            }
            else
            {
                std::cerr<<"start up rope end"<<std::endl;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
                Send01D1DataCommand();
                hook_up_over = true;
            }
        }
        else
        {
            //std::cerr<<"start up rope end"<<std::endl;
            hook_up_over = true;
        }
        if(hook_up_over == false)
        {
            continue;
        }

        //回转方向指令更新
        if (!_bRotation_locate_finished)
        {
            //高速回转
            //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 0);1.2 2.5 1.5 2 2 1.5

            //0高速档位
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 0);

            if (/*m_temptargetRelativeLen_x < 0 || */(tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
            {
                roatate_stop_threshold = 0.5;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                // qDebug()<<"-----------------右回转--------------------------";
            }
            else if(/*m_temptargetRelativeLen_x > 0 || */(tempCopyTargetC - dRotationAngle_) > 0)
            {
                roatate_stop_threshold = 0.5;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                //qDebug()<<"-----------------左回转--------------------------";
            }
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
        }

        //变幅方向指令更新
        if(!_bLuff_locate_finished)
        {
            //std::cerr<<"dAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
            if((dAmplitudeValue_ - tempCopyTargetR) > 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅起方向
                //qDebug()<<"-----------------变幅起--------------------------";
            }
            else if((dAmplitudeValue_ - tempCopyTargetR) < 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(-0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
                //qDebug()<<"-----------------变幅落--------------------------";
            }
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
        }

        //卷方向指令更新
        if(!_bWinch_locate_finished)
        {
            double d_height = 0.0;
            if(m_lift_work_mode_ == 0)
            {
                d_height = m_object_height;
            }
            else if(m_lift_work_mode_ == 1)
            {
                d_height = m_goods_height;
            }

            //if ((g_camerato_ground_height - 0.7) < (goalHeight))
            m_hookheight_mutex.lockForRead();
            //      if (!_bRotation_locate_finished || !_bLuff_locate_finished)
            //      {

            //        if (fabs(_dHook_to_ground_height - d_height) <= 6)
            //        {
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
            //        }
            //        else if (fabs(_dHook_to_ground_height - d_height) > 6 && fabs(_dHook_to_ground_height - d_height) < 6.1)
            //        {
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
            //        }
            //        else
            //        {
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
            //        }
            //      }
            //      else
            //      {
            //        if (fabs(_dHook_to_ground_height - d_height) > 0.9)
            //        {
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
            //          m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
            //        }
            //      }
        }
        m_hookheight_mutex.unlock();
        //回转方向停止指令判断
        //x direction:target's distance is detected camera
        if (!_bRotation_locate_finished)
        {
            //目标回转角度和当前回转角度比较  小于阈值发送刹车动作
            //if((m_isDetectTargetFlag && distance_x < 0.05) || (m_isDetectTargetFlag && (fabs(crane_parameter_->crane_rotation_Angle_ - m_tempCopyTargetC) < 0.3)))
            if(fabs(dRotationAngle_ - tempCopyTargetC) <= roatate_stop_threshold && isDetectTargetFlag)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                //            usleep(100 * 1000);
                Send01D1DataCommand();
                _bRotation_locate_finished = true;		//回转定位完成标志
                qDebug()<<"-----------------回转已完成！--------------------------";
                qDebug()<<"-----------------回转已完成！--------------------------";
            }
        }
        //变幅方向停止指令判断
        //y direction:target's distance is detected camera
        if (!_bLuff_locate_finished)
        {
            if((fabs(tempCopyTargetR - dAmplitudeValue_) <= 0.2 && isDetectTargetFlag))
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);    //变幅停

                usleep(50 * 1000);
                Send01D1DataCommand();
                _bLuff_locate_finished = true; //变幅方向停止指令判断
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
            }
        }

        //send target amplitude and current amplitude diff value to control system...
        //进行变幅速度的控制
        QString high_bit,low_bit;
        double distance_y = fabs(image_targetRelativeLen_y_ - 0.219*sin(dAngleValue_* M_PI/180) - hcfg_->camera_to_armcenter_x*cos(dAngleValue_* M_PI/180)) + crane_parameter_->laser_camera_to_ground_height_*tan(0.22*M_PI/180);
        if(m_isDetectTargetFlag)
        {
            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(distance_y) * 10), high_bit, low_bit);
            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        }
        else
        {
            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(tempCopyTargetR - dAmplitudeValue_) * 10), high_bit, low_bit);
            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        }

        //卷扬方向停止指令判断
        if (_bRotation_locate_finished && _bLuff_locate_finished)
        {
            //后面如果回转到位之后，在判断需不需要开启变幅
            if((fabs(tempCopyTargetR - dAmplitudeValue_) >= 0.25 && isDetectTargetFlag && !_bContinue_control_luff))
            {
                _bLuff_locate_finished = false;//变幅没有到位
                _bContinue_control_luff = true;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);    //变幅启
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 1);

                usleep(50 * 1000);
                Send01D1DataCommand();
                qDebug()<<"-----------------变幅继续！--------------------------";
                qDebug()<<"-----------------变幅继续！--------------------------";
            }

            m_hookheight_mutex.lockForRead();
            //      if (((m_lift_work_mode_ ==0) && ((fabs(_dHook_to_ground_height - m_object_height) < 0.9) || (fabs(dHookHeightCali_ - m_object_height) < 0.9))) ||
            //          ((m_lift_work_mode_ ==1) && ((fabs(_dHook_to_ground_height - m_goods_height) < 0.9 || (fabs(dHookHeightCali_ - m_goods_height) < 0.9)))))
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);	//卷扬停
                sleep(2);
                //Send01D1DataCommand();
                if(!_bWinch_locate_finished)
                {
                    qDebug()<<"targetRelativeLen_x:"<<QString::number(image_targetRelativeLen_x_)<<endl;
                    qDebug()<<"targetRelativeLen_y:"<<QString::number(image_targetRelativeLen_y_)<<endl;
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";

                    is_start_play_voice_thread_ = true;
                    play_voice_flag = 6;
                    m_lift_work_status_ = 2;
                    is_trajectory_track_slow_to_stop_complete_ = false;
                    reset_all_parameter_to_initial_status();
                    m_clsClientToYunTai->recovery_camera_to_init_position(hcfg_->ptz_str_pitch_initial_angle, hcfg_->ptz_str_rotate_initial_angle);
                    emit eraseRect();
                    m_clsClientUsbToCan->clearCommand();

                    std::cerr<<"reset all command success"<<std::endl;
                }
                // precise positioning is already finished, send flag command
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                Send01D0DataCommand();

                usleep(2000 * 1000);        //internal 2s,clear command and exit all threads

                //              g_system_running = false;
                //              g_obstacle_detect_enable = false;
                is_selectObjectFlag_ = false;
            }
            m_hookheight_mutex.unlock();
        }

        if(!_bWinch_locate_finished)
            Send01D1DataCommand();
        usleep(200*100);
    }

    is_send_control_command_package_autodrving = false;
    send_control_command_package_autodrving.detach();
    //  if(dispatchtask_.autodrvingt == false)
    //  {

    //      sendTaskStatusTo(dispatchtask_.id,-1,6);
    //      sendTaskStatusTo(dispatchtask_.id,-1,7);
    //      sendAutodrvingStatusTo(1,1);
    //      //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 0, 1);//无人驾驶状态标志位
    //      Send01D1DataCommand();
    //  }
    //  else
    //  {
    sendTaskStatusTo(dispatchtask_.id,2,6);

    //  }
    //sendTaskStatusTo(dispatchtask_.id,2,12);
    //  if(dispatchtask_.autodrvingt == false)
    //  {
    //      dispatchtask_.autodrvingt = true;
    //      sendAutodrvingStatusTo(1,1);
    //  }
}

void MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK()
{
    double _dHook_to_ground_height = 0.0;
    //sleep(3);
    bool _crane_gear_switch_flag = true;
    bool _bRotation_locate_finished = false;
    bool _bLuff_locate_finished = false;
    bool _bWinch_locate_finished = false;
    bool _bContinue_control_luff = false;
    bool icrane_ok_once = false;
    icrane_ok = false;
    bool _bPreControlRotation = false;
    double _dPreControlRotationAngle = 0.0;
    m_target_value_mutex.lockForRead();

    //算一次目标回转变幅
    vector<double> targetPoint;
    vector<double> targetRotAmp;

    current_goal_position.point_x =3123582.610;//3123589.042
    current_goal_position.point_y =396198.120;//396188.493

    //    current_goal_position.point_x =3123589.042;//3123589.042
    //    current_goal_position.point_y =396188.493;//396188.493

    targetPoint.push_back(current_goal_position.point_x);
    targetPoint.push_back(current_goal_position.point_y);
    //rtk->process();//获取数据
    //targetRotAmp =rtk->getRotationAmp(targetPoint);
    //targetRotAmp =rtk->getRotationAmp2(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

    double tempCopyTargetC;
    double tempCopyTargetR;

    m_target_value_mutex.unlock();

    vector<double> currRotAmp;


    bool currCompuFlag =false;

    double rotationCompose =0.0;
    bool _bone_enable_flag = false;
    double d_height = 2.0;

    double predHookHeightCali_ = crane_parameter_->crane_hook_to_ground_height_ + 5;

    double targetRtkHeight =0.0;
    targetRtkHeight =47.77;

    while(is_send_control_command_package_)
    {

        m_lift_work_status_ = 1;

        double dHookHeightCali_ =0.0;

        rtk->process();//获取数据

        if(rtk->hookData ==0.0)
        {
            std::cerr<<"the i_crane data is error"<<std::endl;
            continue;
        }
        dHookHeightCali_ =rtk->hookData -  targetRtkHeight;


        if(currCompuFlag ==false)
            targetRotAmp =rtk->getCurrRotationAmp(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if(targetRotAmp.size()==2)
        {
            currCompuFlag =true;
            tempCopyTargetC = targetRotAmp[0];
            tempCopyTargetR = targetRotAmp[1];
        }


        if(targetRotAmp.size() !=2)
        {
            std::cerr<<"the target rotationAmp is error"<<std::endl;
            continue;
        }

        //        m_param_mutex.lockForRead();

        if(rtk->armHeadData.size()!=2)
        {
            std::cerr<<"the armHead data is error!!!!!!!!!!"<<std::endl;
            std::cerr<<"the size is ="<<rtk->armHeadData.size()<<std::endl;
            continue;
        }

        currRotAmp =rtk->getCurrRotationAmp(rtk->armHeadData,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if(currRotAmp.size() !=2)
        {
            std::cerr<<"the curr rotAmp is error!!!!!!!!!!"<<std::endl;
            continue;
        }

        tempCopyTargetC = 360 - tempCopyTargetC;


        //ZAT1100's parameter is (width:0.8, height:1.05), the electric crane's parameter is  (width:0.35, height:0.5)
        //double dRotationAngle_ = currRotAmp[0] - atan(0.35/(crane_parameter_->crane_luff_length))*180/M_PI + 0.7;
        //double dAmplitudeValue_ = currRotAmp[1] - 0.5*sin(crane_parameter_->crane_luff_Angle_) - 0.15*cos(crane_parameter_->crane_luff_Angle_) + 0.6;

        //        std::cerr<<"atan(0.98/(crane_parameter_->crane_luff_length))*180/M_PI = "<<atan(0.98/(crane_parameter_->crane_luff_length - 0.3))*180/M_PI<<std::endl;
        //std::cerr<<"currRotAmp"<<currRotAmp[0]<<std::endl;


        double dAmplitudeValue_ = currRotAmp[1] + 0.1;/* + 0.85* sin(crane_parameter_->crane_luff_Angle_*M_PI/180) + 0.15*cos(crane_parameter_->crane_luff_Angle_*M_PI/180);*/
        double dRotationAngle_ = 360 - currRotAmp[0] + 2.0; /*+ atan(0.98/(dAmplitudeValue_ - 0.6))*180/M_PI;*/
        //double dRotationAngle_ = 360 - crane_parameter_->crane_rotation_Angle_ + rotationCompose;

        //        double dAmplitudeValue_ = crane_parameter_->crane_luff_length;
        //        double dRotationAngle_ = crane_parameter_->crane_rotation_Angle_;

        if((tempCopyTargetC > 0) && (tempCopyTargetC <90))
        {
            if((dRotationAngle_ > 270) && (dRotationAngle_ <=360))
            {
                dRotationAngle_ = dRotationAngle_ - 360;
            }
        }
        if((dRotationAngle_ > 0) && (dRotationAngle_ <90))
        {
            if((tempCopyTargetC > 270) && (tempCopyTargetC <=360))
            {
                tempCopyTargetC = tempCopyTargetC - 360;
            }
        }

        //        std::cerr<<"tempCopyTargetC ="<<tempCopyTargetC<<endl;
        //        std::cerr<<"tempCopyTargetR ="<<tempCopyTargetR<<endl;
        //        std::cerr<<"dAmplitudeValue_ ="<<dAmplitudeValue_<<endl;
        //        std::cerr<<"dRotationAngle_ ="<<dRotationAngle_<<endl;
        //        continue;
        //double goalHeight = crane_parameter_->crane_arm_to_ground_height_ - g_dHookHeight;  //吊钩到臂尖的高度 g_dHookHeight
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
        if(_crane_gear_switch_flag)
        {
            if(fabs(tempCopyTargetC - dRotationAngle_) < 10)//单位度
            {
                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 1);
            }
            else
            {
                //0高速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 0);
            }
            if(fabs(dAmplitudeValue_ - tempCopyTargetR) < 3)//单位m
            {
                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 1);
            }
            else
            {
                //0高速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 0);
            }
            _crane_gear_switch_flag = false;
        }

        //std::cerr<<"tempCopyTargetC - dRotationAngle_ ="<<tempCopyTargetC - dRotationAngle_<<std::endl;
        //std::cerr<<"dAmplitudeValue_ - tempCopyTargetR ="<<dAmplitudeValue_ - tempCopyTargetR<<std::endl;
        //回转方向指令更新

        if (!_bRotation_locate_finished /*&& _bone_enable_flag == false*/)
        {
            if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                rotationCompose = 4.5;
                _dPreControlRotationAngle = tempCopyTargetC + 10;
                qDebug()<<"-----------------右回转--------------------------";
            }
            else if((tempCopyTargetC - dRotationAngle_) > 0)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                rotationCompose = 2.0;
                _dPreControlRotationAngle = tempCopyTargetC - 10;
                qDebug()<<"-----------------左回转--------------------------";
            }
            //            _bone_enable_flag = true;
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
        }

        //变幅方向指令更新
        if(!_bLuff_locate_finished)
        {
            if((dAmplitudeValue_ - tempCopyTargetR) > 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅起方向
                //                qDebug()<<"-----------------变幅起--------------------------";
            }
            else if((dAmplitudeValue_ - tempCopyTargetR) < 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(-0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
                //                qDebug()<<"-----------------变幅落--------------------------";
            }
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
        }

        //        卷方向指令更新
        if(!_bWinch_locate_finished)
        {

            //            if(m_lift_work_mode_ == 0)
            //            {
            //                d_height = m_object_height;
            //            }
            //            else if(m_lift_work_mode_ == 1)
            //            {
            //                d_height = m_goods_height;
            //            }
            //if ((g_camerato_ground_height - 0.7) < (goalHeight))
            m_hookheight_mutex.lockForRead();


            if ((!_bRotation_locate_finished || !_bLuff_locate_finished) && !_bPreControlRotation)
            {

                if (dHookHeightCali_ <= 6.5)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
                else if (dHookHeightCali_  > 6.3 && dHookHeightCali_ < 6.5)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
                }
                else if(dHookHeightCali_ > 6.3)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            else if(_bRotation_locate_finished && _bLuff_locate_finished)
            {
                if(icrane_ok_once == false)
                {
                    sendIcraneStatusTo(0,1,"start");//开始调姿态
                    icrane_ok_once = true;
                }
                if (dHookHeightCali_ > 4.6 && icrane_ok)//icrane_ok 姿态over flag
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            m_hookheight_mutex.unlock();
        }


        //回转方向停止指令判断
        //x direction:target's distance is detected camera

        if (!_bRotation_locate_finished)
        {
            if(!_bPreControlRotation)
            {
                if(fabs(dRotationAngle_ - _dPreControlRotationAngle) <= 0.2)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0); //卷扬停
                    Send01D1DataCommand();

                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                    _bPreControlRotation = true;
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                }
            }
            else if(_bPreControlRotation == true && _bLuff_locate_finished == true)
            {
                //目标回转角度和当前回转角度比较  小于阈值发送刹车动作
                //if((m_isDetectTargetFlag && distance_x < 0.05) || (m_isDetectTargetFlag && (fabs(crane_parameter_->crane_rotation_Angle_ - m_tempCopyTargetC) < 0.3)))
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);	//回转
                if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                    //                    qDebug()<<"-----------------右回转--------------------------";
                }
                else if((tempCopyTargetC - dRotationAngle_) > 0)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                    //                    qDebug()<<"-----------------左回转--------------------------";
                }
                if(fabs(dRotationAngle_ - tempCopyTargetC) <= 0.2 /*&& isDetectTargetFlag*/)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                    //            usleep(100 * 1000);
                    Send01D1DataCommand();
                    _bRotation_locate_finished = true;		//回转定位完成标志
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                }
            }
        }
        //变幅方向停止指令判断
        //y direction:target's distance is detected camera
        if (/*(fabs(image_targetRelativeLen_y_) != 0) && */!_bLuff_locate_finished)
        {
            //变幅第一阶段完成操作

            if(fabs(tempCopyTargetR - dAmplitudeValue_) <= 0.1 /*&& isDetectTargetFlag)*/)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);    //变幅停

                usleep(50 * 1000);
                Send01D1DataCommand();
                _bLuff_locate_finished = true; //变幅方向停止指令判断
                std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                std::cerr<<"targetAmplitudeValue_"<<tempCopyTargetR<<std::endl;
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
            }
        }

        //                卷扬方向停止指令判断
        if (_bRotation_locate_finished && _bLuff_locate_finished)
        {
            m_hookheight_mutex.lockForRead();
            //            if (((m_lift_work_mode_ ==0) && ((fabs(_dHook_to_ground_height - m_object_height) < 0.9) || (fabs(dHookHeightCali_ - m_object_height) < 0.9))) ||
            //                    ((m_lift_work_mode_ ==1) && ((fabs(_dHook_to_ground_height - m_goods_height) < 0.9 || (fabs(dHookHeightCali_ - m_goods_height) < 0.9)))))
            if(dHookHeightCali_ < 4.6)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);	//卷扬停

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                Send01D0DataCommand();

                // precise positioning is already finished, send flag command
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                Send01D0DataCommand();

                //                if(!_bWinch_locate_finished)
                {
                    _bWinch_locate_finished = true;
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";

                    is_trajectory_track_slow_to_stop_complete_ = false;
                    reset_all_parameter_to_initial_status();
                    m_clsClientUsbToCan->clearCommand();

                    std::cerr<<"reset all command success"<<std::endl;
                }


                usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
            }
            m_hookheight_mutex.unlock();
        }

        if(!_bWinch_locate_finished)
            Send01D1DataCommand();
        usleep(200*100);

        //send target amplitude and current amplitude diff value to control system...
        //        //进行变幅速度的控制
        //        QString high_bit,low_bit;
        //        double distance_y = fabs(image_targetRelativeLen_y_ - 0.219*sin(dAngleValue_* M_PI/180) - hcfg_->camera_to_armcenter_x*cos(dAngleValue_* M_PI/180)) + crane_parameter_->laser_camera_to_ground_height_*tan(0.22*M_PI/180);
        //        if(m_isDetectTargetFlag)
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(distance_y) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }
        //        else
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(tempCopyTargetR - dAmplitudeValue_) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }

        //回转变幅都完成了，则完成精定位
        //        if(_bRotation_locate_finished && _bLuff_locate_finished)
        //        {
        //            //reset_all_parameter_to_initial_status();

        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();

        //            usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
        //            is_selectObjectFlag_ = false;


        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";

        //            reset_all_parameter_to_initial_status();
        //            m_clsClientUsbToCan->clearCommand();
        //        }
        //        if(!_bWinch_locate_finished)
        //            Send01D1DataCommand();
        //        usleep(200*100);
    }

    if(avoidcommandtask_.status == false)
    {
        if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == true)
        {
            std::cerr<<"yt4here"<<std::endl;
            sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
            resetavoidcommandtask_();//默认已完成，清空
            std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
        }
        else if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == false)
        {
            avoidcommandtask_.status1 = true;
            start_path_plan_Thread();
            std::cerr<<"avoidcommandtask_ start the number 2 avoid postrue!!!!!"<<std::endl;
        }
        else if(avoidcommandtask_.status1 == true && avoidcommandtask_.status2 == false)
        {
            std::cerr<<"yt4here"<<std::endl;
            sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
            resetavoidcommandtask_();//默认已完成，清空
            std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
        }
        else
        {
            std::cerr<<"avoidcommandtask_ status error!!!!!"<<std::endl;
        }
    }
    if(dispatchtask_.statusb == false)
    {
        //////////////////////播报
        is_start_play_voice_thread_ = true;
        play_voice_flag = 2;
        //////////////////////播报
        dispatchtask_.statusb = true;
        std::cerr<<"the dispatchtask_ building task is over,the start building tagget task"<<std::endl;
        start_path_plan_Thread();
    }
    else if(dispatchtask_.statusb == true && dispatchtask_.statust == false)
    {
        //play_voice(4);
        dispatchtask_.statust = true;
        std::cerr<<"the dispatchtask_ building targrt task is over,start the  safe  task"<<std::endl;
        start_path_plan_Thread();
    }
    else if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statusb == true && dispatchtask_.statuss == false)
    {
        //play_voice(6);
        dispatchtask_.statuss = true;
        std::cerr<<"the safe target is over"<<std::endl;
        std::cerr<<"the dispatch task is over"<<std::endl;
        sendTaskStatusTo(dispatchtask_.id,-1,2);
        resetdispatchtask_();
        //start_path_plan_Thread();

    }

    is_send_control_command_package_ = false;

}

void MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_direct()
{
    double _dHook_to_ground_height = 0.0;
    //sleep(3);
    bool _crane_gear_switch_flag = true;
    bool _bRotation_locate_finished = false;
    bool _bLuff_locate_finished = false;
    bool _bWinch_locate_finished = false;
    bool _bContinue_control_luff = false;
    bool icrane_ok_once = false;
    icrane_ok = false;
    bool _bPreControlRotation = false;
    double _dPreControlRotationAngle = 0.0;
    m_target_value_mutex.lockForRead();

    bool hook_up_over = false;

    double rotate_stop = 0.0;


    //算一次目标回转变幅
    vector<double> targetPoint;
    vector<double> targetRotAmp;

    //    current_goal_position.point_x =3120988.385;//3123589.042
    //    current_goal_position.point_y =382877.214;//396188.493

    //    current_goal_position.point_x =3120987.310;//3123589.042
    //    current_goal_position.point_y =382850.263;//396188.493

    //    current_goal_position.point_x =3120987.011;//3123589.042
    //    current_goal_position.point_y =382850.207;//396188.493


    targetPoint.push_back(current_goal_position.point_x);
    targetPoint.push_back(current_goal_position.point_y);
    std::cerr<<"current_goal_position"<<current_goal_position.point_x<<"    "<<current_goal_position.point_y<<std::endl;
    //rtk->process();//获取数据
    //targetRotAmp =rtk->getRotationAmp(targetPoint);
    //targetRotAmp =rtk->getRotationAmp2(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

    double tempCopyTargetC;
    double tempCopyTargetR;

    m_target_value_mutex.unlock();

    vector<double> currRotAmp;


    bool currCompuFlag =false;

    double rotationCompose =0.0;
    bool _bone_enable_flag = false;
    double d_height = 2.0;

    double predHookHeightCali_ = crane_parameter_->crane_hook_to_ground_height_ + 5;

    double targetRtkHeight =0.0;
    targetRtkHeight =48.36;
    dispatchtask_.zhuyuxing = 0;
    while(is_send_control_command_package_)
    {

        m_lift_work_status_ = 1;

        double dHookHeightCali_ =0.0;

        rtk->process();//获取数据

        if(rtk->hookData ==0.0)
        {
            std::cerr<<"the i_crane data is error"<<std::endl;
            continue;
        }
        dHookHeightCali_ =rtk->hookData -  targetRtkHeight;
        //crane_parameter_->crane_hook_to_ground_height_ = rtk->hookData - 49.52;

        if(currCompuFlag ==false)
            targetRotAmp =rtk->getCurrRotationAmp(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if(targetRotAmp.size()==2)
        {
            currCompuFlag =true;
            tempCopyTargetC = targetRotAmp[0];
            tempCopyTargetR = targetRotAmp[1]+0.1;
            std::cerr<<"tempCopyTargetR = "<< tempCopyTargetR << std::endl;
        }


        if(targetRotAmp.size() !=2)
        {
            std::cerr<<"the target rotationAmp is error"<<std::endl;
            continue;
        }

        //        m_param_mutex.lockForRead();

        if(rtk->armHeadData.size()!=2)
        {
            std::cerr<<"the armHead data is error!!!!!!!!!!"<<std::endl;
            std::cerr<<"the size is ="<<rtk->armHeadData.size()<<std::endl;
            continue;
        }

        currRotAmp =rtk->getCurrRotationAmp(rtk->armHeadData,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if(currRotAmp.size() !=2)
        {
            std::cerr<<"the curr rotAmp is error!!!!!!!!!!"<<std::endl;
            continue;
        }

        tempCopyTargetC = 360 - tempCopyTargetC;


        //ZAT1100's parameter is (width:0.8, height:1.05), the electric crane's parameter is  (width:0.35, height:0.5)
        //double dRotationAngle_ = currRotAmp[0] - atan(0.35/(crane_parameter_->crane_luff_length))*180/M_PI + 0.7;
        //double dAmplitudeValue_ = currRotAmp[1] - 0.5*sin(crane_parameter_->crane_luff_Angle_) - 0.15*cos(crane_parameter_->crane_luff_Angle_) + 0.6;

        //        std::cerr<<"atan(0.98/(crane_parameter_->crane_luff_length))*180/M_PI = "<<atan(0.98/(crane_parameter_->crane_luff_length - 0.3))*180/M_PI<<std::endl;
        //std::cerr<<"currRotAmp"<<currRotAmp[0]<<std::endl;


        double dAmplitudeValue_ = currRotAmp[1]  + 0.48* sin(crane_parameter_->crane_luff_Angle_*M_PI/180) + 0.15*cos(crane_parameter_->crane_luff_Angle_*M_PI/180);
        double dRotationAngle_ = 360 - currRotAmp[0] - atan(0.45/(dAmplitudeValue_ - 0.6))*180/M_PI;

        if(dRotationAngle_ > 360)
        {
            dRotationAngle_ = dRotationAngle_ -360;
        }
        if(dRotationAngle_ < 0)
        {
            dRotationAngle_ = dRotationAngle_ + 360;
        }
        //double dRotationAngle_ = 360 - crane_parameter_->crane_rotation_Angle_ + rotationCompose;

        //        double dAmplitudeValue_ = crane_parameter_->crane_luff_length;
        //        double dRotationAngle_ = crane_parameter_->crane_rotation_Angle_;

        //std::cerr<<"dRotationAngle_  dAmplitudeValue_ "<<dAmplitudeValue_<<"       "<<dRotationAngle_<<std::endl;
        if((tempCopyTargetC > 0) && (tempCopyTargetC <90))
        {
            if((dRotationAngle_ > 270) && (dRotationAngle_ <=360))
            {
                dRotationAngle_ = dRotationAngle_ - 360;
            }
        }
        if((dRotationAngle_ > 0) && (dRotationAngle_ <90))
        {
            if((tempCopyTargetC > 270) && (tempCopyTargetC <=360))
            {
                tempCopyTargetC = tempCopyTargetC - 360;
            }
        }

        //        std::cerr<<"tempCopyTargetCxxxxxxxxxxxxx ="<<tempCopyTargetC<<endl;
        //        std::cerr<<"tempCopyTargetRxxxxxxxxxxxxx ="<<tempCopyTargetR<<endl;
        //        std::cerr<<"dAmplitudeValue_xxxxxxxxxx ="<<dAmplitudeValue_<<endl;
        //        std::cerr<<"dRotationAngle_xxxxxxxxxxxx ="<<dRotationAngle_<<endl;
        //double goalHeight = crane_parameter_->crane_arm_to_ground_height_ - g_dHookHeight;  //吊钩到臂尖的高度 g_dHookHeight
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
        if(_crane_gear_switch_flag)
        {
            std::cerr<<"tempCopyTargetC ="<<tempCopyTargetC<<endl;
            std::cerr<<"tempCopyTargetR ="<<tempCopyTargetR<<endl;
            std::cerr<<"dAmplitudeValue_ ="<<dAmplitudeValue_<<endl;
            std::cerr<<"dRotationAngle_ ="<<dRotationAngle_<<endl;
            if(fabs(tempCopyTargetC - dRotationAngle_) < 10)//单位度
            {
                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 1);
            }
            else
            {
                //0高速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 0);
            }
            if(fabs(dAmplitudeValue_ - tempCopyTargetR) < 3)//单位m
            {
                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 1);
            }
            else
            {
                //0高速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 0);
            }
            _crane_gear_switch_flag = false;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////
        if(hook_up_over == false && dispatchtask_.statust == false && dispatchtask_.statusb == true)
        {
            if(dHookHeightCali_ < 7.0)//7.0
            {
                std::cerr<<"start up rope"<<std::endl;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                Send01D1DataCommand();
            }
            else
            {
                hook_up_over = true;
            }
        }
        else
        {
            hook_up_over = true;
        }
        if(hook_up_over == false)
        {
            continue;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////



        //std::cerr<<"tempCopyTargetC - dRotationAngle_ ="<<tempCopyTargetC - dRotationAngle_<<std::endl;
        //std::cerr<<"dAmplitudeValue_ - tempCopyTargetR ="<<dAmplitudeValue_ - tempCopyTargetR<<std::endl;
        //回转方向指令更新
        //        std::cerr<<"tempCopyTargetC ="<<tempCopyTargetC<<endl;
        //        std::cerr<<"tempCopyTargetR ="<<tempCopyTargetR<<endl;
        //        std::cerr<<"dAmplitudeValue_ ="<<dAmplitudeValue_<<endl;
        //        std::cerr<<"dRotationAngle_ ="<<dRotationAngle_<<endl;
        //        std::cerr<<"dHookHeightCali_"<<dHookHeightCali_<<std::endl;
        if (!_bRotation_locate_finished && dispatchtask_.zhuyuxing == 0/*&& _bone_enable_flag == false*/)
        {
            if(fabs(tempCopyTargetC - dRotationAngle_) < 5)//单位度
            {
                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 1);
                std::cerr<<"回转低速"<<std::endl;
            }
            if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                rotate_stop = 1.7;
                rotationCompose = 4.5;
                //_dPreControlRotationAngle = tempCopyTargetC + 10;
                //qDebug()<<"-----------------右回转--------------------------";
            }
            else if((tempCopyTargetC - dRotationAngle_) > 0)
            {
                rotate_stop = 0.5;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                rotationCompose = 2.0;
                //_dPreControlRotationAngle = tempCopyTargetC - 10;
                //qDebug()<<"-----------------左回转--------------------------";
            }
            //            _bone_enable_flag = true;
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
        }

        //变幅方向指令更新
        if(!_bLuff_locate_finished)
        {
            if((dAmplitudeValue_ - tempCopyTargetR) > 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅起方向
                //               qDebug()<<"-----------------变幅起--------------------------";
            }
            else if((dAmplitudeValue_ - tempCopyTargetR) < 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(-0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
                //qDebug()<<"-----------------变幅落--------------------------";
            }
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
        }

        //        卷方向指令更新
        if(!_bWinch_locate_finished)
        {

            m_hookheight_mutex.lockForRead();

            //tong   9   9.2    8.9

            if ((!_bRotation_locate_finished || !_bLuff_locate_finished))
            {

                if (dHookHeightCali_ <= 6.65)//7.0
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
                else if (dHookHeightCali_  > 6.65 && dHookHeightCali_ < 6.85)//7.0 7.2
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);//keep
                }
                else if(dHookHeightCali_ > 6.85)//7.2
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            else if(_bRotation_locate_finished && _bLuff_locate_finished)
            {

                if(icrane_ok_once == false)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);//keep
                    icrane_ok_once = true;
                    sendIcraneStatusTo(0,1,"start");//开始调姿态
                }
                if (dHookHeightCali_ > 5.4  && dispatchtask_.zhuyuxing == 2)//icrane_ok 姿态over flag//5.8
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            m_hookheight_mutex.unlock();
        }


        //回转方向停止指令判断
        //x direction:target's distance is detected camera

        if (!_bRotation_locate_finished)
        {
            if(fabs(dRotationAngle_ - tempCopyTargetC) <= rotate_stop /*&& isDetectTargetFlag*/)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                //            usleep(100 * 1000);

                //                reset_all_parameter_to_initial_status();
                //                emit eraseRect();
                //                m_clsClientUsbToCan->clearCommand();
                //                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                //                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                ///////////////////


                Send01D1DataCommand();
                _bRotation_locate_finished = true;		//回转定位完成标志
                std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
            }
        }
        //变幅方向停止指令判断
        //y direction:target's distance is detected camera
        if (/*(fabs(image_targetRelativeLen_y_) != 0) && */!_bLuff_locate_finished)
        {
            //变幅第一阶段完成操作

            if(fabs(tempCopyTargetR - dAmplitudeValue_) <= 0.1 /*&& isDetectTargetFlag)*/)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);    //变幅停

                usleep(50 * 1000);
                Send01D1DataCommand();
                _bLuff_locate_finished = true; //变幅方向停止指令判断
                std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                std::cerr<<"targetAmplitudeValue_"<<tempCopyTargetR<<std::endl;
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
            }
        }

        //                卷扬方向停止指令判断
        //dHookHeightCali_ = 4.5;
        if (_bRotation_locate_finished && _bLuff_locate_finished)
        {
            m_hookheight_mutex.lockForRead();
            //            if (((m_lift_work_mode_ ==0) && ((fabs(_dHook_to_ground_height - m_object_height) < 0.9) || (fabs(dHookHeightCali_ - m_object_height) < 0.9))) ||
            //                    ((m_lift_work_mode_ ==1) && ((fabs(_dHook_to_ground_height - m_goods_height) < 0.9 || (fabs(dHookHeightCali_ - m_goods_height) < 0.9)))))
            if(dHookHeightCali_ < 5.4)//5.8
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);	//卷扬停

                // precise positioning is already finished, send flag command
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                Send01D0DataCommand();

                std::cerr<<"waitting  break hook or on a hook"<<std::endl;
                if(dispatchtask_.statusb == false && dispatchtask_.hook_down_up == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,4);
                }
                if(dispatchtask_.statusb == true && dispatchtask_.statust == false && dispatchtask_.hook_down_up == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,10);
                }
                if(!_bWinch_locate_finished  && dispatchtask_.hook_down_up == true)////写挂钩
                {
                    dispatchtask_.hook_down_up = false;//下次还是挂钩为完成
                    _bWinch_locate_finished = true;
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;


                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";

                    is_trajectory_track_slow_to_stop_complete_ = false;
                    reset_all_parameter_to_initial_status();
                    m_clsClientUsbToCan->clearCommand();
                    dispatchtask_.hook_down_up = false;//写挂钩
                    std::cerr<<"reset all command success"<<std::endl;
                }
                usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
            }
            m_hookheight_mutex.unlock();
        }

        if(!_bWinch_locate_finished)
            Send01D1DataCommand();
        usleep(200*100);


        //send target amplitude and current amplitude diff value to control system...
        //        //进行变幅速度的控制
        //        QString high_bit,low_bit;
        //        double distance_y = fabs(image_targetRelativeLen_y_ - 0.219*sin(dAngleValue_* M_PI/180) - hcfg_->camera_to_armcenter_x*cos(dAngleValue_* M_PI/180)) + crane_parameter_->laser_camera_to_ground_height_*tan(0.22*M_PI/180);
        //        if(m_isDetectTargetFlag)
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(distance_y) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }
        //        else
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(tempCopyTargetR - dAmplitudeValue_) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }

        //回转变幅都完成了，则完成精定位
        //        if(_bRotation_locate_finished && _bLuff_locate_finished)
        //        {
        //            //reset_all_parameter_to_initial_status();

        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();

        //            usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
        //            is_selectObjectFlag_ = false;


        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";

        //            reset_all_parameter_to_initial_status();
        //            m_clsClientUsbToCan->clearCommand();
        //        }
        //        if(!_bWinch_locate_finished)
        //            Send01D1DataCommand();
        //        usleep(200*100);
    }
    is_send_control_command_package_ = false;
    send_control_command_package_.detach();
    if(avoidcommandtask_.status == false)
    {
        if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == true)
        {
            std::cerr<<"yt4here"<<std::endl;
            sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
            resetavoidcommandtask_();//默认已完成，清空
            std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
        }
        else if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == false)
        {
            avoidcommandtask_.status1 = true;
            start_path_plan_Thread();
            std::cerr<<"avoidcommandtask_ start the number 2 avoid postrue!!!!!"<<std::endl;
        }
        else if(avoidcommandtask_.status1 == true && avoidcommandtask_.status2 == false)
        {
            std::cerr<<"yt4here"<<std::endl;
            sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
            resetavoidcommandtask_();//默认已完成，清空
            std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
        }
        else
        {
            std::cerr<<"avoidcommandtask_ status error!!!!!"<<std::endl;
        }
    }
    if(dispatchtask_.statusb == false)
    {
        //////////////////////播报
        is_start_play_voice_thread_ = true;
        play_voice_flag = 2;
        //////////////////////播报
        dispatchtask_.statusb = true;
        if(send_control_command_package_autodrving.joinable())
        {
            send_control_command_package_autodrving.join();
        }
        is_send_control_command_package_autodrving = true;
        send_control_command_package_autodrving = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread, this);
        std::cerr<<"the dispatchtask_ building task is over,the start building tagget task"<<std::endl;
        if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statuss == true)
        {
            sendTaskStatusTo(dispatchtask_.id,-1,12);
            std::cerr<<"dispatch task is ok "<<std::endl;
        }

    }
    else if(dispatchtask_.statusb == true && dispatchtask_.statust == false)
    {
        //////////////////////播报
        is_start_play_voice_thread_ = true;
        play_voice_flag = 4;
        //////////////////////播报
        dispatchtask_.statust = true;
        if(send_control_command_package_autodrving.joinable())
        {
            send_control_command_package_autodrving.join();
        }
        is_send_control_command_package_autodrving = true;
        send_control_command_package_autodrving = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread, this);
        std::cerr<<"the dispatchtask_ building targrt task is over,start the  safe  task"<<std::endl;
        if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statuss == true)
        {
            sendTaskStatusTo(dispatchtask_.id,-1,12);
            std::cerr<<"dispatch task is ok "<<std::endl;
        }
    }
    else if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statuss == false)
    {
        //////////////////////播报
        is_start_play_voice_thread_ = true;
        play_voice_flag = 6;
        //////////////////////播报
        dispatchtask_.statuss = true;
        std::cerr<<"the safe target is over"<<std::endl;
        sendTaskStatusTo(dispatchtask_.id,-1,12);
        resetdispatchtask_();

    }
}


void MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo()
{
    qiangban_hook_down = false;
    bool hook_up_over = false;
    double _dHook_to_ground_height = 0.0;
    //sleep(3);
    bool _crane_gear_switch_flag = true;
    bool _bRotation_locate_finished = false;
    bool _bLuff_locate_finished = false;
    bool _bWinch_locate_finished = false;
    bool _bContinue_control_luff = false;
    bool icrane_ok_once = false;
    icrane_ok = false;
    bool _bPreControlRotation = false;
    double _dPreControlRotationAngle = 0.0;
    m_target_value_mutex.lockForRead();

    //算一次目标回转变幅
    vector<double> targetPoint;
    vector<double> targetRotAmp;

    //    current_goal_position.point_x =3120988.385;//3123589.042
    //    current_goal_position.point_y =382877.214;//396188.493

//        current_goal_position.point_x = 3121024.158;    //3121024.158
//        current_goal_position.point_y = 382852.287;     //382852.287    北方板子的点

    targetPoint.push_back(current_goal_position.point_x);
    targetPoint.push_back(current_goal_position.point_y);
    std::cerr<<"111111111111111111111current_goal_position"<<current_goal_position.point_x<<"    "<<current_goal_position.point_y<<std::endl;
    //rtk->process();//获取数据
    //targetRotAmp =rtk->getRotationAmp(targetPoint);
    //targetRotAmp =rtk->getRotationAmp2(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

    double tempCopyTargetC;
    double tempCopyTargetR;

    m_target_value_mutex.unlock();

    vector<double> currRotAmp;


    bool currCompuFlag =false;
    bool targetCompuFlagSecond =false;

    double rotationCompose =0.0;
    bool _bone_enable_flag = false;
    double d_height = 2.0;

    double predHookHeightCali_ = crane_parameter_->crane_hook_to_ground_height_ + 4;

    double targetRtkHeight =0.0;
    targetRtkHeight =48.36;
    dispatchtask_.zhuyuxing = 0;
    while(is_send_control_command_package_)
    {

        m_lift_work_status_ = 1;

        double dHookHeightCali_ =0.0;

        rtk->process();//获取数据

        if(rtk->hookData ==0.0)
        {
            std::cerr<<"the i_crane data is error"<<std::endl;
            continue;
        }
        dHookHeightCali_ =rtk->hookData -  targetRtkHeight;
        crane_parameter_->crane_hook_to_ground_height_ = dHookHeightCali_;

        //std::cerr<<"dHookHeightCali_"<<dHookHeightCali_<<std::endl;
        if(currCompuFlag ==false)
            targetRotAmp =rtk->getCurrRotationAmp(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if((targetRotAmp.size()==2))
        {
            currCompuFlag =true;
            tempCopyTargetC = targetRotAmp[0] + 6;
            tempCopyTargetR = targetRotAmp[1] + 0.4;
            m_target_value_mutex.lockForRead();
            m_tempCopyTargetC_ = tempCopyTargetC;
            m_tempCopyTargetR_ = tempCopyTargetR;
            m_target_value_mutex.unlock();

        }

        if(targetRotAmp.size() !=2)
        {
            std::cerr<<"the target rotationAmp is error"<<std::endl;
            continue;
        }

        //        m_param_mutex.lockForRead();

        if(rtk->armHeadData.size()!=2)
        {
            std::cerr<<"the armHead data is error!!!!!!!!!!"<<std::endl;
            std::cerr<<"the size is ="<<rtk->armHeadData.size()<<std::endl;
            continue;
        }

        currRotAmp =rtk->getCurrRotationAmp(rtk->armHeadData,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if(currRotAmp.size() !=2)
        {
            std::cerr<<"the curr rotAmp is error!!!!!!!!!!"<<std::endl;
            continue;
        }

        tempCopyTargetC = 360 - tempCopyTargetC;


        //ZAT1100's parameter is (width:0.8, height:1.05), the electric crane's parameter is  (width:0.35, height:0.5)
        //double dRotationAngle_ = currRotAmp[0] - atan(0.35/(crane_parameter_->crane_luff_length))*180/M_PI + 0.7;
        //double dAmplitudeValue_ = currRotAmp[1] - 0.5*sin(crane_parameter_->crane_luff_Angle_) - 0.15*cos(crane_parameter_->crane_luff_Angle_) + 0.6;

        //        std::cerr<<"atan(0.98/(crane_parameter_->crane_luff_length))*180/M_PI = "<<atan(0.98/(crane_parameter_->crane_luff_length - 0.3))*180/M_PI<<std::endl;
        //std::cerr<<"currRotAmp"<<currRotAmp[0]<<std::endl;


        //std::cerr<<"  bianfu  "<<currRotAmp[1]<<"  huizhuan  "<<360 - currRotAmp[0];
        //west  + 0.45     -2.5
        //south  +0.45         -5.5
        //north
        //        double dAmplitudeValue_ = currRotAmp[1] +  0.55;/*0.5* sin(crane_parameter_->crane_luff_Angle_*M_PI/180) + 0.15*cos(crane_parameter_->crane_luff_Angle_*M_PI/180)*/;
        //        double dRotationAngle_ = 360 - currRotAmp[0] - 2.5; /*+ atan(0.98/(dAmplitudeValue_ - 0.6))*180/M_PI;*/

        double dAmplitudeValue_ = currRotAmp[1] +  0.5* sin(crane_parameter_->crane_luff_Angle_*M_PI/180) + 0.15*cos(crane_parameter_->crane_luff_Angle_*M_PI/180);
        double dRotationAngle_ = 360 - currRotAmp[0] - 10;
//                std::cerr<<"targetCopyTargetCxxxxxxxxxxxxx ="<<tempCopyTargetC<<endl;
//                std::cerr<<"targetCopyTargetRxxxxxxxxxxxxx ="<<tempCopyTargetR<<endl;
//                std::cerr<<"dAmplitudeValue_xxxxxxxxxx ="<<dAmplitudeValue_<<endl;
//                std::cerr<<"dRotationAngle_xxxxxxxxxxxx ="<<dRotationAngle_<<endl;

        if(dRotationAngle_ > 360)
        {
            dRotationAngle_ = dRotationAngle_ -360;
        }
        if(dRotationAngle_ < 0)
        {
            dRotationAngle_ = dRotationAngle_ + 360;
        }
        if(dRotationAngle_ > 360)
        {
            dRotationAngle_ = dRotationAngle_ -360;
        }
        //double dRotationAngle_ = 360 - crane_parameter_->crane_rotation_Angle_ + rotationCompose;

        //        double dAmplitudeValue_ = crane_parameter_->crane_luff_length;
        //        double dRotationAngle_ = crane_parameter_->crane_rotation_Angle_;

        //std::cerr<<"dRotationAngle_  dAmplitudeValue_ "<<dAmplitudeValue_<<"       "<<dRotationAngle_<<std::endl;
        if((tempCopyTargetC > 0) && (tempCopyTargetC <90))
        {
            if((dRotationAngle_ > 270) && (dRotationAngle_ <=360))
            {
                dRotationAngle_ = dRotationAngle_ - 360;
            }
        }
        if((dRotationAngle_ > 0) && (dRotationAngle_ <90))
        {
            if((tempCopyTargetC > 270) && (tempCopyTargetC <=360))
            {
                tempCopyTargetC = tempCopyTargetC - 360;
            }
        }

        //        std::cerr<<"tempCopyTargetCxxxxxxxxxxxxx ="<<tempCopyTargetC<<endl;
        //        std::cerr<<"tempCopyTargetRxxxxxxxxxxxxx ="<<tempCopyTargetR<<endl;
        //        std::cerr<<"dAmplitudeValue_xxxxxxxxxx ="<<dAmplitudeValue_<<endl;
        //        std::cerr<<"dRotationAngle_xxxxxxxxxxxx ="<<dRotationAngle_<<endl;
        //double goalHeight = crane_parameter_->crane_arm_to_ground_height_ - g_dHookHeight;  //吊钩到臂尖的高度 g_dHookHeight
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
        if(_crane_gear_switch_flag)
        {
            //            std::cerr<<"tempCopyTargetC ="<<tempCopyTargetC<<endl;
            //            std::cerr<<"tempCopyTargetR ="<<tempCopyTargetR<<endl;
            //            std::cerr<<"dAmplitudeValue_ ="<<dAmplitudeValue_<<endl;
            //            std::cerr<<"dRotationAngle_ ="<<dRotationAngle_<<endl;
//            if(fabs(tempCopyTargetC - dRotationAngle_) < 10)//单位度
//            {
//                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);
//            }
//            else
//            {
//                //0高速档位
//                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 0);
//            }
//            if(fabs(dAmplitudeValue_ - tempCopyTargetR) < 3)//单位m
//            {
//                //1低速档位
//                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 1);
//            }
//            else
//            {
//                //0高速档位
//                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 0);
//            }
            _crane_gear_switch_flag = false;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////
        if(hook_up_over == false /*&& dispatchtask_.statust == false && dispatchtask_.statusb == true*/)
        {
            if(dHookHeightCali_ < 8.50)//7.0
            {

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                Send01D1DataCommand();
            }
            else
            {
                std::cerr<<"第一阶段 tempCopyTargetC ="<<360 - tempCopyTargetC<<std::endl;
                std::cerr<<"第一阶段 tempCopyTargetR ="<<tempCopyTargetR<<std::endl;
                std::cerr<<"start up rope  over"<<dHookHeightCali_<<std::endl;
                hook_up_over = true;
            }
        }
        else
        {
            hook_up_over = true;
        }
        if(hook_up_over == false)
        {
            continue;
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////
        //std::cerr<<"tempCopyTargetC - dRotationAngle_ ="<<tempCopyTargetC - dRotationAngle_<<std::endl;
        //std::cerr<<"dAmplitudeValue_ - tempCopyTargetR ="<<dAmplitudeValue_ - tempCopyTargetR<<std::endl;
        //回转方向指令更新
        //        std::cerr<<"tempCopyTargetC ="<<tempCopyTargetC<<endl;
        //        std::cerr<<"tempCopyTargetR ="<<tempCopyTargetR<<endl;
        //        std::cerr<<"dAmplitudeValue_ ="<<dAmplitudeValue_<<endl;
        //        std::cerr<<"dRotationAngle_ ="<<dRotationAngle_<<endl;
        //        std::cerr<<"dHookHeightCali_"<<dHookHeightCali_<<std::endl;
        if (!_bRotation_locate_finished && dispatchtask_.zhuyuxing == 0 && _bLuff_locate_finished == true/*&& _bone_enable_flag == false*/)
        {
//            if(fabs(tempCopyTargetC - dRotationAngle_) < 5)//单位度
//            {
//                //1低速档位
//                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 6, 1);
//                //std::cerr<<"回转低速"<<std::endl;
//            }
            if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                rotationCompose = 4.5;
                _dPreControlRotationAngle = tempCopyTargetC + 10;
                // qDebug()<<"-----------------右回转--------------------------";
            }
            else if((tempCopyTargetC - dRotationAngle_) > 0)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                rotationCompose = 2.0;
                _dPreControlRotationAngle = tempCopyTargetC - 10;
                //qDebug()<<"-----------------左回转--------------------------";
            }
            //            _bone_enable_flag = true;
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
        }

        //变幅方向指令更新
        if(!_bLuff_locate_finished)
        {
            if((dAmplitudeValue_ - tempCopyTargetR) > 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅起方向
                //                qDebug()<<"-----------------变幅起--------------------------";
            }
            else if((dAmplitudeValue_ - tempCopyTargetR) < 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(-0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
                //                qDebug()<<"-----------------变幅落--------------------------";
            }
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
        }

        //        卷方向指令更新
        if(!_bWinch_locate_finished)
        {

            //            if(m_lift_work_mode_ == 0)
            //            {
            //                d_height = m_object_height;
            //            }
            //            else if(m_lift_work_mode_ == 1)
            //            {
            //                d_height = m_goods_height;
            //            }
            //if ((g_camerato_ground_height - 0.7) < (goalHeight))
            m_hookheight_mutex.lockForRead();


            if ((!_bRotation_locate_finished || !_bLuff_locate_finished) && !_bPreControlRotation)
            {

                if (dHookHeightCali_ <= 8.30)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
                else if (dHookHeightCali_  > 8.30 && dHookHeightCali_ < 8.40)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
                }
                else if(dHookHeightCali_ > 8.40)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            else if(_bRotation_locate_finished && _bLuff_locate_finished)
            {
                if (dHookHeightCali_ > 8.25  && qiangban_hook_down == true)//icrane_ok 姿态over flag
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            m_hookheight_mutex.unlock();
        }


        //回转方向停止指令判断
        //x direction:target's distance is detected camera

        if (!_bRotation_locate_finished)
        {
            if(!_bPreControlRotation )
            {
                if(((fabs(dRotationAngle_ - _dPreControlRotationAngle) <= 1.5) || north_board == true) && _bLuff_locate_finished == true)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0); //卷扬停
                    Send01D1DataCommand();

                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                    _bPreControlRotation = true;


                    dispatchtask_.zhuyuxing = 1;
                    sendIcraneStatusTo(0,1,"start");//开始调姿态
                    north_board = false;

                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";

                    if(targetCompuFlagSecond ==false)
                        targetRotAmp =rtk->getCurrRotationAmp(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

                    if((targetRotAmp.size()==2))
                    {
                        targetCompuFlagSecond =true;
                        tempCopyTargetC = targetRotAmp[0] + 6;
                        tempCopyTargetR = targetRotAmp[1] +0.1;
                        m_target_value_mutex.lockForRead();
                        m_tempCopyTargetC_ = tempCopyTargetC;
                        m_tempCopyTargetR_ = tempCopyTargetR;
                        m_target_value_mutex.unlock();

                    }

                    std::cerr<<"开始调姿"<<std::endl;
                    std::cerr<<"开始调姿"<<std::endl;
                    std::cerr<<"开始调姿"<<std::endl;
                    is_start_play_voice_thread_ = true;
                    play_voice_flag = 2;
                    std::cerr<<"第二阶段 tempCopyTargetC ="<<360 - tempCopyTargetC<<std::endl;
                    std::cerr<<"第二阶段 tempCopyTargetR ="<<tempCopyTargetR<<std::endl;
                }
            }
            //else if(_bPreControlRotation == true && _bLuff_locate_finished == true && icrane_ok == true)
            else if(_bPreControlRotation == true && dispatchtask_.zhuyuxing == 2)
            {
                //目标回转角度和当前回转角度比较  小于阈值发送刹车动作
                //if((m_isDetectTargetFlag && distance_x < 0.05) || (m_isDetectTargetFlag && (fabs(crane_parameter_->crane_rotation_Angle_ - m_tempCopyTargetC) < 0.3)))
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);	//回转
                if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                    // qDebug()<<"-----------------2右回转--------------------------";
                }
                else if((tempCopyTargetC - dRotationAngle_) > 0)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                    //qDebug()<<"-----------------2左回转--------------------------";
                }
                if(fabs(dRotationAngle_ - tempCopyTargetC) <= 1 /*&& isDetectTargetFlag*/)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                    //            usleep(100 * 1000);
                    Send01D1DataCommand();
                    _bRotation_locate_finished = true;		//回转定位完成标志
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                }
            }
        }
        //变幅方向停止指令判断
        //y direction:target's distance is detected camera
        if (/*(fabs(image_targetRelativeLen_y_) != 0) && */!_bLuff_locate_finished)
        {
            //变幅第一阶段完成操作

            if(fabs(tempCopyTargetR - dAmplitudeValue_) <= 0.1 /*&& isDetectTargetFlag)*/)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);    //变幅停

                usleep(50 * 1000);
                Send01D1DataCommand();
                _bLuff_locate_finished = true; //变幅方向停止指令判断
                std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                std::cerr<<"targetAmplitudeValue_"<<tempCopyTargetR<<std::endl;
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
            }
        }

        //                卷扬方向停止指令判断
        //dHookHeightCali_ = 4.5;
        if (_bRotation_locate_finished && _bLuff_locate_finished)
        {
            m_hookheight_mutex.lockForRead();
            //            if (((m_lift_work_mode_ ==0) && ((fabs(_dHook_to_ground_height - m_object_height) < 0.9) || (fabs(dHookHeightCali_ - m_object_height) < 0.9))) ||
            //                    ((m_lift_work_mode_ ==1) && ((fabs(_dHook_to_ground_height - m_goods_height) < 0.9 || (fabs(dHookHeightCali_ - m_goods_height) < 0.9)))))
            if(dHookHeightCali_ < 8.25)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);	//卷扬停

                // precise positioning is already finished, send flag command
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                Send01D0DataCommand();

                std::cerr<<"waitting  break hook or on a hook"<<std::endl;
                if(dispatchtask_.statusb == false && dispatchtask_.hook_down_up == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,4);
                }
                if(dispatchtask_.statusb == true && dispatchtask_.statust == false && dispatchtask_.hook_down_up == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,10);
                }
                if(!_bWinch_locate_finished  && dispatchtask_.hook_down_up == true)////写挂钩
                {
                    dispatchtask_.hook_down_up = false;//下次还是挂钩为完成
                    _bWinch_locate_finished = true;
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";

                    is_trajectory_track_slow_to_stop_complete_ = false;
                    reset_all_parameter_to_initial_status();
                    m_clsClientUsbToCan->clearCommand();
                    dispatchtask_.hook_down_up = false;//写挂钩
                    std::cerr<<"reset all command success"<<std::endl;
                }
                usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
            }
            m_hookheight_mutex.unlock();
        }

        if(!_bWinch_locate_finished)
            Send01D1DataCommand();
        usleep(200*100);


        //send target amplitude and current amplitude diff value to control system...
        //        //进行变幅速度的控制
        //        QString high_bit,low_bit;
        //        double distance_y = fabs(image_targetRelativeLen_y_ - 0.219*sin(dAngleValue_* M_PI/180) - hcfg_->camera_to_armcenter_x*cos(dAngleValue_* M_PI/180)) + crane_parameter_->laser_camera_to_ground_height_*tan(0.22*M_PI/180);
        //        if(m_isDetectTargetFlag)
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(distance_y) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }
        //        else
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(tempCopyTargetR - dAmplitudeValue_) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }

        //回转变幅都完成了，则完成精定位
        //        if(_bRotation_locate_finished && _bLuff_locate_finished)
        //        {
        //            //reset_all_parameter_to_initial_status();

        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();

        //            usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
        //            is_selectObjectFlag_ = false;


        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";

        //            reset_all_parameter_to_initial_status();
        //            m_clsClientUsbToCan->clearCommand();
        //        }
        //        if(!_bWinch_locate_finished)
        //            Send01D1DataCommand();
        //        usleep(200*100);
    }
    is_send_control_command_package_ = false;
    send_control_command_package_.detach();
}



void MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test()
{

    qiangban_hook_down = false;
    bool hook_up_over = false;
    double _dHook_to_ground_height = 0.0;
    //sleep(3);
    bool _crane_gear_switch_flag = true;
    bool _bRotation_locate_finished = false;
    bool _bLuff_locate_finished = false;
    bool _bWinch_locate_finished = false;
    bool _bContinue_control_luff = false;
    bool icrane_ok_once = false;
    icrane_ok = false;
    bool _bPreControlRotation = false;
    double _dPreControlRotationAngle = 0.0;
    m_target_value_mutex.lockForRead();

    //算一次目标回转变幅
    vector<double> targetPoint;
    vector<double> targetRotAmp;

    //    current_goal_position.point_x =3120988.385;//3123589.042
    //    current_goal_position.point_y =382877.214;//396188.493

//        current_goal_position.point_x = 3121024.158;    //3121024.158
//        current_goal_position.point_y = 382852.287;     //382852.287    北方板子的点

    targetPoint.push_back(current_goal_position.point_x);
    targetPoint.push_back(current_goal_position.point_y);
    std::cerr<<"111111111111111111111current_goal_position"<<current_goal_position.point_x<<"    "<<current_goal_position.point_y<<std::endl;


    double tempCopyTargetC;
    double tempCopyTargetR;

    m_target_value_mutex.unlock();

    vector<double> currRotAmp;


    bool currCompuFlag =false;
    bool targetCompuFlagSecond =false;

    double rotationCompose =0.0;
    bool _bone_enable_flag = false;
    double d_height = 2.0;

    double predHookHeightCali_ = crane_parameter_->crane_hook_to_ground_height_ + 4;

    double targetRtkHeight =0.0;
    targetRtkHeight =48.36;
    dispatchtask_.zhuyuxing = 0;
    while(is_send_control_command_package_)
    {

        m_lift_work_status_ = 1;

        double dHookHeightCali_ =0.0;

        rtk->process();//获取数据

        if(rtk->hookData ==0.0)
        {
            std::cerr<<"the i_crane data is error"<<std::endl;
            continue;
        }
        dHookHeightCali_ =rtk->hookData -  targetRtkHeight;
        crane_parameter_->crane_hook_to_ground_height_ = dHookHeightCali_;

        //std::cerr<<"dHookHeightCali_"<<dHookHeightCali_<<std::endl;
        if(currCompuFlag ==false)
            targetRotAmp =rtk->getCurrRotationAmp(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if((targetRotAmp.size()==2))
        {
            currCompuFlag =true;
            tempCopyTargetC = targetRotAmp[0] + 6.5;
            tempCopyTargetR = targetRotAmp[1] + 0.1;
            m_target_value_mutex.lockForRead();
            m_tempCopyTargetC_ = tempCopyTargetC;
            m_tempCopyTargetR_ = tempCopyTargetR;
            m_target_value_mutex.unlock();

        }

        if(targetRotAmp.size() !=2)
        {
            std::cerr<<"the target rotationAmp is error"<<std::endl;
            continue;
        }

        //        m_param_mutex.lockForRead();

        if(rtk->armHeadData.size()!=2)
        {
            std::cerr<<"the armHead data is error!!!!!!!!!!"<<std::endl;
            std::cerr<<"the size is ="<<rtk->armHeadData.size()<<std::endl;
            continue;
        }

        currRotAmp =rtk->getCurrRotationAmp(rtk->armHeadData,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

        if(currRotAmp.size() !=2)
        {
            std::cerr<<"the curr rotAmp is error!!!!!!!!!!"<<std::endl;
            continue;
        }

        tempCopyTargetC = 360 - tempCopyTargetC;
        
        double dAmplitudeValue_ = currRotAmp[1] +  0.5* sin(crane_parameter_->crane_luff_Angle_*M_PI/180) + 0.15*cos(crane_parameter_->crane_luff_Angle_*M_PI/180);
        double dRotationAngle_ = 360 - currRotAmp[0] - 10;
        
        if(dRotationAngle_ > 360)
        {
            dRotationAngle_ = dRotationAngle_ -360;
        }
        if(dRotationAngle_ < 0)
        {
            dRotationAngle_ = dRotationAngle_ + 360;
        }
        if(dRotationAngle_ > 360)
        {
            dRotationAngle_ = dRotationAngle_ -360;
        }

        if((tempCopyTargetC > 0) && (tempCopyTargetC <90))
        {
            if((dRotationAngle_ > 270) && (dRotationAngle_ <=360))
            {
                dRotationAngle_ = dRotationAngle_ - 360;
            }
        }
        if((dRotationAngle_ > 0) && (dRotationAngle_ <90))
        {
            if((tempCopyTargetC > 270) && (tempCopyTargetC <=360))
            {
                tempCopyTargetC = tempCopyTargetC - 360;
            }
        }

        //double goalHeight = crane_parameter_->crane_arm_to_ground_height_ - g_dHookHeight;  //吊钩到臂尖的高度 g_dHookHeight
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
        if(_crane_gear_switch_flag)
        {
            std::cerr<<"tempCopyTargetC ="<<tempCopyTargetC<<endl;
            std::cerr<<"tempCopyTargetR ="<<tempCopyTargetR<<endl;
            std::cerr<<"dAmplitudeValue_ ="<<dAmplitudeValue_<<endl;
            std::cerr<<"dRotationAngle_ ="<<dRotationAngle_<<endl;
            if(fabs(tempCopyTargetC - dRotationAngle_) < 15)//单位度
            {
                //1低速档位std::cerr<<"tempCopyTargetR = "<< tempCopyTargetR << std::endl;
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);
            }
            else
            {
                //0高速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 0);
            }
//            if(fabs(dAmplitudeValue_ - tempCopyTargetR) < 3)//单位m
//            {
//                //1低速档位
//                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 1);
//            }
//            else
//            {
//                //0高速档位
//                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 0);
//            }
            _crane_gear_switch_flag = false;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////
        if(hook_up_over == false /*&& dispatchtask_.statust == false && dispatchtask_.statusb == true*/)
        {
            if(dHookHeightCali_ < 8.50)//7.0
            {

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                Send01D1DataCommand();
            }
            else
            {
                std::cerr<<"第一阶段 tempCopyTargetC ="<<360 - tempCopyTargetC<<std::endl;
                std::cerr<<"第一阶段 tempCopyTargetR ="<<tempCopyTargetR<<std::endl;
                std::cerr<<"start up rope  over"<<dHookHeightCali_<<std::endl;
                hook_up_over = true;
            }
        }
        else
        {
            hook_up_over = true;
        }
        if(hook_up_over == false)
        {
            continue;
        }

//        if (!_bRotation_locate_finished && dispatchtask_.zhuyuxing == 0/* && _bLuff_locate_finished == true/*&& _bone_enable_flag == false*/)
        if (!_bRotation_locate_finished && dispatchtask_.zhuyuxing == 0 && _bLuff_locate_finished == true/*&& _bone_enable_flag == false*/)
        {
            if(fabs(tempCopyTargetC - dRotationAngle_) < 15)//单位度
            {
                //1低速档位
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);
                //std::cerr<<"回转低速"<<std::endl;
            }
            if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                rotationCompose = 4.5;
                _dPreControlRotationAngle = tempCopyTargetC + 10;
                // qDebug()<<"-----------------右回转--------------------------";
            }
            else if((tempCopyTargetC - dRotationAngle_) > 0)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                rotationCompose = 2.0;
                _dPreControlRotationAngle = tempCopyTargetC - 10;
                //qDebug()<<"-----------------左回转--------------------------";
            }
            //            _bone_enable_flag = true;
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
        }

        //变幅方向指令更新
        if(!_bLuff_locate_finished)
        {
            if((dAmplitudeValue_ - tempCopyTargetR) > 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅起方向
                //                qDebug()<<"-----------------变幅起--------------------------";
            }
            else if((dAmplitudeValue_ - tempCopyTargetR) < 0)
            {
                QString high_bit;
                QString low_bit;
                QString send_string_3d0;
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(0 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 = low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(3.14 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(-0.002 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SplitCanDataByte(std::floor(1 * 100 * 180 / M_PI), high_bit, low_bit);
                send_string_3d0 += low_bit + high_bit;
                high_bit.clear();
                low_bit.clear();
                m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);

                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
                //                qDebug()<<"-----------------变幅落--------------------------";
            }
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
        }

        //        卷方向指令更新
        if(!_bWinch_locate_finished)
        {
            m_hookheight_mutex.lockForRead();

            if ((!_bRotation_locate_finished || !_bLuff_locate_finished) && !_bPreControlRotation)
            {

                if (dHookHeightCali_ <= 8.75)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
                else if (dHookHeightCali_  > 8.75 && dHookHeightCali_ < 8.85)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
                }
                else if(dHookHeightCali_ > 8.85)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            else if(_bRotation_locate_finished && _bLuff_locate_finished)
            {
                if (dHookHeightCali_ > 8.55  && qiangban_hook_down == true)//icrane_ok 姿态over flag
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
                }
            }
            m_hookheight_mutex.unlock();
        }


        //回转方向停止指令判断
        //x direction:target's distance is detected camera

        if (!_bRotation_locate_finished)
        {
            if(!_bPreControlRotation )
            {
                if(((fabs(dRotationAngle_ - _dPreControlRotationAngle) <= 1.5) || north_board == true) && _bLuff_locate_finished == true)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0); //卷扬停
                    Send01D1DataCommand();

                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                    _bPreControlRotation = true;


                    dispatchtask_.zhuyuxing = 1;
                    sendIcraneStatusTo(0,1,"start");//开始调姿态
                    north_board = false;

                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转第一阶段已完成！----------------------------------------------";

                    if(targetCompuFlagSecond ==false)
                        targetRotAmp =rtk->getCurrRotationAmp(targetPoint,crane_parameter_->crane_rotation_Angle_);//求臂尖回转变幅

                    if((targetRotAmp.size()==2))
                    {
                        targetCompuFlagSecond =true;
                        tempCopyTargetC = targetRotAmp[0] + 6;
                        tempCopyTargetR = targetRotAmp[1] +0.1;
                        m_target_value_mutex.lockForRead();
                        m_tempCopyTargetC_ = tempCopyTargetC;
                        m_tempCopyTargetR_ = tempCopyTargetR;
                        m_target_value_mutex.unlock();

                    }

                    std::cerr<<"开始调姿"<<std::endl;
                    std::cerr<<"开始调姿"<<std::endl;
                    std::cerr<<"开始调姿"<<std::endl;
                    is_start_play_voice_thread_ = true;
                    play_voice_flag = 2;
                    std::cerr<<"第二阶段 tempCopyTargetC ="<<360 - tempCopyTargetC<<std::endl;
                    std::cerr<<"第二阶段 tempCopyTargetR ="<<tempCopyTargetR<<std::endl;
                }
            }
            //else if(_bPreControlRotation == true && _bLuff_locate_finished == true && icrane_ok == true)
            else if(_bPreControlRotation == true && dispatchtask_.zhuyuxing == 2)
            {
                //目标回转角度和当前回转角度比较  小于阈值发送刹车动作
                //if((m_isDetectTargetFlag && distance_x < 0.05) || (m_isDetectTargetFlag && (fabs(crane_parameter_->crane_rotation_Angle_ - m_tempCopyTargetC) < 0.3)))
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);	//回转
                if ((tempCopyTargetC - dRotationAngle_) < 0)        //right rotation
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
                    // qDebug()<<"-----------------2右回转--------------------------";
                }
                else if((tempCopyTargetC - dRotationAngle_) > 0)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
                    //qDebug()<<"-----------------2左回转--------------------------";
                }
                if(fabs(dRotationAngle_ - tempCopyTargetC) <= 1 /*&& isDetectTargetFlag*/)
                {
                    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);	//回转停
                    //            usleep(100 * 1000);
                    Send01D1DataCommand();
                    _bRotation_locate_finished = true;		//回转定位完成标志
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"targetRotationAngle_"<<tempCopyTargetC<<std::endl;

                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！-------------------------------------";
                    qDebug()<<"--------------------------------------------回转已完成！----------------------------------------------";
                }
            }
        }
        //变幅方向停止指令判断
        //y direction:target's distance is detected camera
        if (/*(fabs(image_targetRelativeLen_y_) != 0) && */!_bLuff_locate_finished)
        {
            //变幅第一阶段完成操作

            if(fabs(tempCopyTargetR - dAmplitudeValue_) <= 0.1 /*&& isDetectTargetFlag)*/)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);    //变幅停

                usleep(50 * 1000);
                Send01D1DataCommand();
                _bLuff_locate_finished = true; //变幅方向停止指令判断
                std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                std::cerr<<"targetAmplitudeValue_"<<tempCopyTargetR<<std::endl;
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
                qDebug()<<"-----------------变幅已完成！--------------------------";
            }
        }

        //                卷扬方向停止指令判断
        //dHookHeightCali_ = 4.5;
        if (_bRotation_locate_finished && _bLuff_locate_finished)
        {
            m_hookheight_mutex.lockForRead();

            if(dHookHeightCali_ < 8.3)
            {
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);	//卷扬停

                // precise positioning is already finished, send flag command
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
                m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
                Send01D0DataCommand();

                std::cerr<<"waitting  break hook or on a hook"<<std::endl;
                if(dispatchtask_.statusb == false && dispatchtask_.hook_down_up == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,4);
                }
                if(dispatchtask_.statusb == true && dispatchtask_.statust == false && dispatchtask_.hook_down_up == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,10);
                }
                //if(!_bWinch_locate_finished  && dispatchtask_.hook_down_up == true)////写挂钩
                if(!_bWinch_locate_finished)//写挂钩
                {
                    dispatchtask_.hook_down_up = false;//下次还是挂钩为完成
                    _bWinch_locate_finished = true;
                    std::cerr<<"cudRotationAngle_"<<dRotationAngle_<<std::endl;
                    std::cerr<<"cudAmplitudeValue_"<<dAmplitudeValue_<<std::endl;
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------卷杨已完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";
                    qDebug()<<"-----------------精准定位完成！--------------------------";

                    is_trajectory_track_slow_to_stop_complete_ = false;
                    reset_all_parameter_to_initial_status();
                    m_clsClientUsbToCan->clearCommand();
                    dispatchtask_.hook_down_up = false;//写挂钩
                    std::cerr<<"reset all command success"<<std::endl;
                }
                usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
            }
            m_hookheight_mutex.unlock();
        }

        if(!_bWinch_locate_finished)
            Send01D1DataCommand();
        usleep(200*100);

        
        //send target amplitude and current amplitude diff value to control system...
        //        //进行变幅速度的控制
        //        QString high_bit,low_bit;
        //        double distance_y = fabs(image_targetRelativeLen_y_ - 0.219*sin(dAngleValue_* M_PI/180) - hcfg_->camera_to_armcenter_x*cos(dAngleValue_* M_PI/180)) + crane_parameter_->laser_camera_to_ground_height_*tan(0.22*M_PI/180);
        //        if(m_isDetectTargetFlag)
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(distance_y) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }
        //        else
        //        {
        //            m_clsClientUsbToCan->SplitCanDataByte(std::round(fabs(tempCopyTargetR - dAmplitudeValue_) * 10), high_bit, low_bit);
        //            m_clsClientUsbToCan->Data01D1[2] = low_bit;
        //        }

        //回转变幅都完成了，则完成精定位
        //        if(_bRotation_locate_finished && _bLuff_locate_finished)
        //        {
        //            //reset_all_parameter_to_initial_status();

        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();

        //            usleep(2000 * 1000);        //internal 2s,clear command and exit all threads
        //            is_selectObjectFlag_ = false;


        //            // precise positioning is already finished, send flag command
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
        //            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
        //            Send01D0DataCommand();
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";
        //            qDebug()<<"-----------------精准定位完成！--------------------------";

        //            reset_all_parameter_to_initial_status();
        //            m_clsClientUsbToCan->clearCommand();
        //        }
        //        if(!_bWinch_locate_finished)
        //            Send01D1DataCommand();
        //        usleep(200*100);
    }
    is_send_control_command_package_ = false;
    send_control_command_package_.detach();
//    if(avoidcommandtask_.status == false)
//    {
//        if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == true)
//        {
//            std::cerr<<"yt4here"<<std::endl;
//            sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
//            resetavoidcommandtask_();//默认已完成，清空
//            std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
//        }
//        else if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == false)
//        {
//            avoidcommandtask_.status1 = true;
//            start_path_plan_Thread();
//            std::cerr<<"avoidcommandtask_ start the number 2 avoid postrue!!!!!"<<std::endl;
//        }
//        else if(avoidcommandtask_.status1 == true && avoidcommandtask_.status2 == false)
//        {
//            std::cerr<<"yt4here"<<std::endl;
//            sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
//            resetavoidcommandtask_();//默认已完成，清空
//            std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
//        }
//        else
//        {
//            std::cerr<<"avoidcommandtask_ status error!!!!!"<<std::endl;
//        }
//    }
//    if(dispatchtask_.statusb == false)
//    {
//        //play_voice(2);
//        dispatchtask_.statusb = true;
//        if(send_control_command_package_autodrving.joinable())
//        {
//            send_control_command_package_autodrving.join();
//        }
//        is_send_control_command_package_autodrving = true;
//        send_control_command_package_autodrving = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread, this);
//        std::cerr<<"the dispatchtask_ building task is over,the start building tagget task"<<std::endl;
//        if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statuss == true)
//        {
//            sendTaskStatusTo(dispatchtask_.id,-1,12);
//            std::cerr<<"dispatch task is ok "<<std::endl;
//        }

//    }
//    else if(dispatchtask_.statusb == true && dispatchtask_.statust == false)
//    {
//        ///play_voice(4);
//        dispatchtask_.statust = true;
//        if(send_control_command_package_autodrving.joinable())
//        {
//            send_control_command_package_autodrving.join();
//        }
//        is_send_control_command_package_autodrving = true;
//        send_control_command_package_autodrving = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread, this);
//        std::cerr<<"the dispatchtask_ building targrt task is over,start the  safe  task"<<std::endl;
//        if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statuss == true)
//        {
//            sendTaskStatusTo(dispatchtask_.id,-1,12);
//            std::cerr<<"dispatch task is ok .reset auto drving state"<<std::endl;
//        }
//    }
//    else if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statuss == false)
//    {
//        //play_voice(6);
//        dispatchtask_.statuss = true;
//        std::cerr<<"the safe target is over"<<std::endl;
//        sendTaskStatusTo(dispatchtask_.id,-1,12);
//        resetdispatchtask_();
//        //start_path_plan_Thread();

//    }

}

void MainWindow::clear_all_can_send_data()
{
    for(int i = 0; i < 4; i++)
    {
        m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
        m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    }

    for (int i = 0; i < 8; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data02D0[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data02D1[i], j, 0);

            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data03D0[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data04D0[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01A2[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data02A2[i], j, 0);
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data03A2[i], j, 0);

        }
    }

    Send01D0DataCommand();
    Send01D1DataCommand();
    Send02D0DataCommand();
    Send02D1DataCommand();
}

void MainWindow::reset_all_parameter_to_initial_status()
{
    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");

    //planning_scene_visualization_timer_->stop();
    is_directly_visual_location_ = false;
    is_start_object_position_compute_ = false;
    is_send_control_command_package_ = false;
    is_detect_Chessboard_success = false;
    is_send_control_command_package_autodrving = false;
    //    g_system_running = false;
    //    g_obstacle_detect_enable = false;
    //    g_bCalibing = false;

    is_selectObjectFlag_ = false;
    is_start_crane_trajectory_track_ = false;
    //initial the position value;
    m_isDetectTargetFlag = false;

    crane_parameter_->laser_camera_to_ground_height_ = 0;
    sleep(2);
}


void MainWindow::Function_RTK_data_receive_Thread()
{
    while(true)
    {
        rtk->process();
//        crane_parameter_->crane_hook_to_ground_height_ = rtk->hookData - 48.36;// + 2.3;//2.3  hookuo and rtk distance
//        if((crane_parameter_->crane_rotation_Angle_ <= 1) || (crane_parameter_->crane_rotation_Angle_ >= 359))
//        {
//            heading_angle = rtk->carHeadingAngel - 180;
//            //std::cerr<<"heading_angle"<<heading_angle<<std::endl;
//        }
//        g_craneposition = rtk->rtkCenter();

        usleep(100*100);
    }
    usleep(100);
}
//////////////////////////////////////////////////////////////////////////////////
/// USB-CAN 接收CAN总线数据
/// //////////////////////////////////////////////////////////////////////////////
void MainWindow::Function_pcan_data_receive_Thread()
{
    bool bRecordStretchWay = false;
    int heart_num = 0;
    int num = 0;
    int currenttsakid = 0;
    double heading_angle = 0.0;
    //机群注册设备 开始
    rtk->process();
    g_craneposition = rtk->rtkCenter();
    int huaile = 5;
    while(huaile)
    {
        if(!g_craneposition[0] && !g_craneposition[1] && !g_craneposition[2])
        {
            rtk->process();
            g_craneposition.clear();
            g_craneposition = rtk->rtkCenter();
        }
        else
        {
            break;
        }
        huaile--;
    }
    sendRegeisterTo(g_craneposition[0],g_craneposition[1],g_craneposition[2]);
    g_craneposition.clear();
    //机群设备注册结束
    while (is_start_pcan_data_receive_thread_)
    {

        num++;
        heart_num++;
        if(heart_num == 65530)
        {
            heart_num = 0;
        }
        QString high_bit,low_bit,heart_send_data;
        m_clsClientUsbToCan->SplitCanDataByte(heart_num % 2, high_bit, low_bit);
        heart_send_data = low_bit + "0000000000000";
        m_clsClientUsbToCan->SendCanData("02D1",heart_send_data);

        m_param_mutex.lockForWrite();

        m_clsClientUsbToCan->ReceiveCanData();
        //获取臂尖角度传感器的值，并设置云台
        crane_parameter_->crane_mainarm_length_ = (m_clsClientUsbToCan->Data048F[0] + m_clsClientUsbToCan->Data048F[1] * 256) / 100.0;		//主臂长
        crane_parameter_->crane_luff_Angle_ = (m_clsClientUsbToCan->Data048F[2] + m_clsClientUsbToCan->Data048F[3] * 256) / 10.0;		//臂架顶端倾斜角度，即变幅角
        //crane_parameter_->crane_luff_Angle_ = (m_clsClientUsbToCan->Data02B7[6] + m_clsClientUsbToCan->Data02B7[7] * 256) / 10.0;		//臂尖倾斜角度
        crane_parameter_->crane_luff_length = (m_clsClientUsbToCan->Data048F[4] + m_clsClientUsbToCan->Data048F[5] * 256) / 100.0;	//变幅长
        crane_parameter_->crane_arm_to_ground_height_ = (m_clsClientUsbToCan->Data048F[6] + m_clsClientUsbToCan->Data048F[7] * 256) / 100.0;			//臂尖对地高度
        crane_parameter_->crane_rotation_Angle_ =(m_clsClientUsbToCan->Data03B7[4] + m_clsClientUsbToCan->Data03B7[5] * 256) / 100.0;		//回转角度
        //g_dHookHeight = (m_clsClientUsbToCan->Data02B7[2] + m_clsClientUsbToCan->Data02B7[3] * 256) / 100.0;		//吊钩对地高度
        if(crane_parameter_->crane_hook_to_ground_height_ == 0)
        {
            crane_parameter_->crane_hook_to_ground_height_ = (m_clsClientUsbToCan->Data02B7[2] + m_clsClientUsbToCan->Data02B7[3] * 256) / 100.0 - 2;		//吊钩对地高度
        }

        crane_parameter_->crane_rope_ratio_ = m_clsClientUsbToCan->Data02B7[6];
        crane_parameter_->crane_legState_ = m_clsClientUsbToCan->Data04B7[1]/16;          //支腿方式-0全伸 1半伸
        crane_parameter_->crane_Bob_Weight_ = m_clsClientUsbToCan->Data04B7[1]%16;          //配重0-56t;1-48t;2-36t;3-24t;4-12t;5-0t
        crane_parameter_->crane_arm_status_ = ((m_clsClientUsbToCan->Data04B7[2]/16)*10+m_clsClientUsbToCan->Data04B7[2]%16);    //臂架状态


        //std::cerr<<crane_parameter_->crane_rotation_Angle_<<"-------"<<crane_parameter_->crane_luff_Angle_<<"-----------"<<crane_parameter_->crane_hook_to_ground_height_<<std::endl;
        //获取当前臂架的伸缩方式
        if((m_clsClientUsbToCan->Data01B7[1] != 0) && (bRecordStretchWay == false))
        {
            crane_parameter_->crane_arm_stretch_State_ = QString::number(m_clsClientUsbToCan->Data01B7[1])+QString::number(m_clsClientUsbToCan->Data01B7[2])+QString::number(m_clsClientUsbToCan->Data01B7[3])
                    +QString::number(m_clsClientUsbToCan->Data01B7[4])+QString::number(m_clsClientUsbToCan->Data01B7[5])+QString::number(m_clsClientUsbToCan->Data01B7[6]);//臂架组合形式
            crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[1]);     //the first arm
            crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[2]);
            crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[3]);
            crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[4]);
            crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[5]);
            crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[6]);
            //for zat2200h763
            //crane_parameter_->crane_vec_stretch_way_.push_back(m_clsClientUsbToCan->Data01B7[7]);
            //m_vec_stretchWay.push_back(m_clsClientUsbToCan->Data01B7[7]);
            bRecordStretchWay = true;
        }

        if(crane_parameter_->crane_hook_to_ground_height_ > 16 && m_xubaike_pathplan == true)
        {
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 0);  //cancle 精准定位
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 0);

            Send01D1DataCommand();
            start_path_plan_Thread();
        }
//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
        bool f_test = true; //测试flag
        
        // 加锁更新数据
        std::unique_lock<std::mutex> lock(collisionWorker_->ctx->dataMutex_);
        
        if(!collisionWorker_->ctx->busData_.b_updated_){
            if (f_test) {
                collisionWorker_->ctx->busData_.rope_len_ = 10;
                collisionWorker_->ctx->busData_.load_exist_flag_ = false;
                collisionWorker_->ctx->busData_.vx_ = 0;
                collisionWorker_->ctx->busData_.vy_ = 0;
                collisionWorker_->ctx->busData_.vz_ = 0;
            } else {
                // 计算绳长：主臂高度 - 吊钩高度
                collisionWorker_->ctx->busData_.rope_len_ = crane_parameter_->crane_arm_to_ground_height_-crane_parameter_->crane_hook_to_ground_height_;
                collisionWorker_->ctx->busData_.load_exist_flag_ = false;
                collisionWorker_->ctx->busData_.vx_ = 0;
                collisionWorker_->ctx->busData_.vy_ = 0;
                collisionWorker_->ctx->busData_.vz_ = 0;
            }
            collisionWorker_->ctx->busData_.b_updated_= true; //待完善can掉线重联逻辑
        }
//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
        m_param_mutex.unlock();


//        joint2spaceCoordinate(crane_parameter_->crane_rotation_Angle_ ,crane_parameter_->crane_luff_length);

//        targetsInfors_x =3.5,targetsInfors_y =8;
//        space2jointCoordinate(targetsInfors_x,targetsInfors_y);
        //currSpaceAndJointCompute();
        //currSpaceAndJointCompute_luntai();


        if(num >= 300)
        {
            //g_dRotationAngle = 180;
            g_cranepostrue.append(crane_parameter_->crane_rotation_Angle_ * M_PI / 180);
            g_cranepostrue.append(crane_parameter_->crane_luff_Angle_ * M_PI / 180);
            //if(!is_start_crane_trajectory_track_ && !is_send_control_command_package_ && !on_path_planing && !is_send_control_command_package_autodrving)
            {
                //rtk->process();
                crane_parameter_->crane_hook_to_ground_height_ = rtk->hookData - 48.36;// + 2.3;//2.3  hookuo and rtk distance
                if((crane_parameter_->crane_rotation_Angle_ <= 1) || (crane_parameter_->crane_rotation_Angle_ >= 359))
                {
                    heading_angle = rtk->carHeadingAngel - 180;
                    //std::cerr<<"heading_angle"<<heading_angle<<std::endl;
                }
                g_craneposition = rtk->rtkCenter();

            }
            g_cranepostrue.append(crane_parameter_->crane_arm_to_ground_height_ - (crane_parameter_->crane_hook_to_ground_height_ + 2.3));
            //      g_craneposition.append(1.0);
            //      g_craneposition.append(2.0);
            //      g_craneposition.append(4.0);
            //	workstatus:	,//整型，枚举值，0 关机，1 前往堆放点，2 挂钩等待，3 前往安装点，
            //4 卸钩等待，5 回位,6 空闲，7 展臂（汽车起重机），8 收臂（汽车起重机），9 行驶（汽车起重机）。
            if(dispatchtask_.completeall == false && avoidcommandtask_.status == true
                    && commandtask_.status == true)
            {
                currenttsakid = dispatchtask_.id;
                if(dispatchtask_.statusb == false)
                {
                    workstate = 2;
                }
                if(dispatchtask_.statusb == true && dispatchtask_.statust == false)
                {
                    workstate = 7;
                }
                if(dispatchtask_.statusb == true && dispatchtask_.statust == true &&dispatchtask_.statuss == false)
                {
                    workstate = 12;
                }

            }
            else if(avoidcommandtask_.status == false && commandtask_.status == true)
            {
                workstate = 16;
            }
            else if(commandtask_.status == false)
            {
                workstate == 19;
            }
            else
            {
                workstate = 0;
            }
            if(dispatchtask_.completeall == true)
            {
                currenttsakid = -1;
            }
            if(gcranepositionL.size() == 0)
            {
                gcranepositionL = g_craneposition;
            }
            sendRobotStatusTo(g_cranepostrue, g_craneposition,workstate,crane_parameter_->crane_mainarm_length_,currenttsakid,heading_angle,gcranepositionL);
//            std::cerr.setf(ios::fixed);
//            std::cerr.precision(3);
//            std::cerr<<g_craneposition[0]<<"    "<<g_craneposition[1]<<"    "<<g_craneposition[2]<<std::endl;
            //std::cerr<<"sendRobotStatusTo chuang"<<std::endl;
            num = 0;
            g_cranepostrue.clear();
            g_craneposition.clear();
        }
        if(dispatchtask_.ifget == true && avoidcommandtask_.status == true
                && commandtask_.status == true)
        {
            //启动路径规划
            dispatchtask_.ifget = false;
            //std::cerr<<"get dispatch stak ,start building task"<<std::endl;

            if(dispatchtask_.autodrvingb == true)
            {
                if(dispatchtask_.statusb == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,3);
                    qDebug("去吊东西，吊起他————————————————————————————————————————————————.");
                    current_goal_position.point_x = dispatchtask_.taskinfor.buildinglocation.point_x;
                    current_goal_position.point_y = dispatchtask_.taskinfor.buildinglocation.point_y;
                    current_goal_position.point_z = dispatchtask_.taskinfor.buildinglocation.point_z;
                    dispatchtask_.autodrvingb = false;
                    if(send_control_command_package_.joinable())
                    {
                        send_control_command_package_.join();
                    }
                    is_send_control_command_package_ = true;
                    send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                }

            }
            else if(dispatchtask_.autodrvingt == true)
            {
                if(dispatchtask_.statusb = true && dispatchtask_.statust == false && gaizi_bool == false)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,9);
                    qDebug("去送东西--------墙板，放下他————————————————————————————————————————————————.");
                    current_goal_position.point_x = dispatchtask_.taskinfor.buildingtargetlocation.point_x;
                    current_goal_position.point_y = dispatchtask_.taskinfor.buildingtargetlocation.point_y;
                    current_goal_position.point_z = dispatchtask_.taskinfor.buildingtargetlocation.point_z;
                    dispatchtask_.autodrvingt = false;
                    if(send_control_command_package_.joinable())
                    {
                        send_control_command_package_.join();
                    }
                    is_send_control_command_package_ = true;
                    send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    //              if(zhuzi_bool == true)
                    //              {
                    //                  zhuzi_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_direct, this);
                    //              }
                    //             // send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    //              if(zuozi_bool == true)
                    //              {
                    //                  zhuzi_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo, this);
                    //              }
                    //              if(gaizi_bool == true)
                    //              {
                    //                  gaizi_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo, this);
                    //              }
                    //              if(qiangban_bool == true)
                    //              {
                    //                  qiangban_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    //              }
                }
                else if(dispatchtask_.statusb = true && dispatchtask_.statust == false && gaizi_bool == true)
                {
                    sendTaskStatusTo(dispatchtask_.id,-1,9);
                    qDebug("去送东西-------盖子，放下他————————————————————————————————————————————————.");
                    current_goal_position.point_x = dispatchtask_.taskinfor.buildingtargetlocation.point_x;
                    current_goal_position.point_y = dispatchtask_.taskinfor.buildingtargetlocation.point_y;
                    current_goal_position.point_z = dispatchtask_.taskinfor.buildingtargetlocation.point_z;
                    dispatchtask_.autodrvingt = false;
                    if(send_control_command_package_.joinable())
                    {
                        send_control_command_package_.join();
                    }
                    gaizi_bool = false;
                    is_send_control_command_package_ = true;
                    send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo, this);
                    //              if(zhuzi_bool == true)
                    //              {
                    //                  zhuzi_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_direct, this);
                    //              }
                    //             // send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    //              if(zuozi_bool == true)
                    //              {
                    //                  zhuzi_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo, this);
                    //              }
                    //              if(gaizi_bool == true)
                    //              {
                    //                  gaizi_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo, this);
                    //              }
                    //              if(qiangban_bool == true)
                    //              {
                    //                  qiangban_bool = false;
                    //                  send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    //              }
                }
            }
            else
            {
                //qDebug("有任务，但是无人驾驶没有到位————————————————————————————————————————————————.");
                dispatchtask_.ifget = true;
            }
            //start_path_plan_Thread();
            //for display 20220708
            // on_Accurate_positioning_triggered();
            //
        }
        if(avoidcommandtask_.ifget == true && commandtask_.status == true)
        {
            avoidcommandtask_.ifget = false;
            sendAvoidTaskStatusTo(avoidcommandtask_.id,0); //avoidcommandtask  status 0-get
            //启动路径规划
            start_path_plan_Thread();
            qDebug("收到机群协同避让命令————————————————————————————————————————————————.");
        }
        if(commandtask_.ifget == true)
        {
            commandtask_.ifget == false;
            sendCommandTaskStatusTo(commandtask_.id ,0,"truck crane get");
            if(commandtask_.taskinfor == 1)//1 stop;2  start
            {
                if(commandtask_.status == false)
                {
                    on_stop_crane_control();
                }
                if(avoidcommandtask_.status == false)
                {
                    resetavoidcommandtask_();
                    qDebug("避让任务急停清除————————————————————————————————————————————————.");
                }
                //////////////////////播报
                is_start_play_voice_thread_ = true;
                play_voice_flag = 11;
                //////////////////////播报
                qDebug("收到机群协同控制单元停止命令，进行轨迹跟踪停止————————————————————————————————————————————————.");
                on_stop_crane_control();
                resetcommandtask_();
            }
            if(commandtask_.taskinfor == 2)
            {
                //////////////////////播报
                is_start_play_voice_thread_ = true;
                play_voice_flag = 10;
                //////////////////////播报
                qDebug("收到机群协同控制单元启动命令，继续执行任务————————————————————————————————————————————————.");
                resetcommandtask_();
                if(dispatchtask_.completeall == false && (is_stop_send_trajectory_.load() == false && is_send_control_command_package_ == false))
                {
                    sendCommandTaskStatusTo(commandtask_.id,0,"success");
                    //start_path_plan_Thread();
                    if(dispatchtask_.statusb == false)
                    {
                        current_goal_position.point_x = dispatchtask_.taskinfor.buildinglocation.point_x;
                        current_goal_position.point_y = dispatchtask_.taskinfor.buildinglocation.point_y;
                        current_goal_position.point_z = dispatchtask_.taskinfor.buildinglocation.point_z;
                        is_send_control_command_package_ = true;
                        send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    }
                    if(dispatchtask_.statusb == true && dispatchtask_.statust == false)
                    {
                        current_goal_position.point_x = dispatchtask_.taskinfor.buildingtargetlocation.point_x;
                        current_goal_position.point_y = dispatchtask_.taskinfor.buildingtargetlocation.point_y;
                        current_goal_position.point_z = dispatchtask_.taskinfor.buildingtargetlocation.point_z;
                        is_send_control_command_package_ = true;
                        send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
                    }
                }
                else if(dispatchtask_.completeall == false && (is_stop_send_trajectory_.load() == true || is_send_control_command_package_ == true))
                {
                    sendCommandTaskStatusTo(commandtask_.id,3,"task is running!!!");
                }
                else
                {
                    sendCommandTaskStatusTo(commandtask_.id,3,"no task need to continue!!!");
                }
            }
        }


        usleep(10 * 100);
    }

    is_start_pcan_data_receive_thread_ = false;
}

void MainWindow::Function_play_voice_Thread()
{
    while(is_start_play_voice_thread_)
    {
        switch (play_voice_flag)
        {
        case 0:play_voice(0);break;
        case 1:play_voice(1);break;
        case 2:play_voice(2);break;
        case 3:play_voice(3);break;
        case 4:play_voice(4);break;
        case 5:play_voice(5);break;
        case 6:play_voice(6);break;
        case 7:play_voice(7);break;
        case 8:play_voice(8);break;
        case 9:play_voice(9);break;
        case 10:play_voice(10);break;
        case 11:play_voice(11);break;
        case 12:play_voice(12);break;
        case 13:play_voice(13);break;
        case 14:play_voice(14);break;
        case 15:play_voice(15);break;
        case 16:play_voice(16);break;
        case 17:play_voice(17);break;
        case 18:play_voice(18);break;
        case 19:play_voice(19);break;
        case 20:play_voice(20);break;
        case 21:play_voice(21);break;
        case 22:play_voice(22);break;
        default:
            break;
        }
        play_voice_flag = 100;
        usleep(1000);
    }
    is_start_play_voice_thread_ =false;
}

void MainWindow::leftRotation()
{
    std::cerr<<"rotate_left"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 0);

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);   
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
    Send01D1DataCommand();
}
void MainWindow::rightRotation()
{
    std::cerr<<"rotate_right"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
    Send01D1DataCommand();
}
void MainWindow::downAmp()
{
    std::cerr<<"luff_down"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flagstatust
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
    Send01D1DataCommand();
}
void MainWindow::upAmp()
{
    std::cerr<<"luff_up"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅落方向
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
    Send01D1DataCommand();
}
void MainWindow::downLuff()
{
    std::cerr<<"hook down"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
    Send01D1DataCommand();
}
void MainWindow::upLuff()
{
    std::cerr<<"hook  up"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
    Send01D1DataCommand();
}
void MainWindow::stopRotion()
{
    std::cerr<<"rotate_stop"<<std::endl;
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);//stop wind
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);//stop rotation
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);//stop luff
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);//wind up
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);

//    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    Send01D1DataCommand();
}
void MainWindow::stopAmp()
{
    std::cerr<<"luff_stop"<<std::endl;
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);//stop wind
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);//stop rotation
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);//stop luff
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);//wind up
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);

//    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    Send01D1DataCommand();
}
void MainWindow::stopLuff()
{
    std::cerr<<"hook  stop"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);//stop wind
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);//stop rotation
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);//stop luff
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);//wind up
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
//    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
//    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    Send01D1DataCommand();
}

void MainWindow::Function_voice_control_Thread()
{
    double target_rotate_angel =-1.0;
    double target_luff_angel =-1.0;
    double target_wind_height = -1.0;

    std::cerr<<"启动语音调度线程 启动语音调度线程is_start_voice_control_thread_ ="<<is_start_voice_control_thread_<<std::endl;
    while(is_start_voice_control_thread_)
    {
        //右回转  回转为逆时针变化
        if(rotate_up_right_x !=0.0)
        {
            std::cerr<<"语音调度上车右回转rotate_up_right_x ="<<rotate_up_right_x<<std::endl;
            std::cerr<<"语音调度上车右回转rotate_up_right_x ="<<rotate_up_right_x<<std::endl;
            target_rotate_angel =crane_parameter_->crane_rotation_Angle_ - rotate_up_right_x;

            if(target_rotate_angel < 0)
            {
                target_rotate_angel = target_rotate_angel + 360;
            }

            std::cerr<<"语音调度上车右回转目标回转target_rotate_angel ="<<target_rotate_angel<<std::endl;

            stopRotion();
            usleep(20*1000);
            rightRotation();

            rotate_up_right_x =0.0;
        }

        //左回转  回转为逆时针变化
        if(rotate_up_left_x !=0.0)
        {
            std::cerr<<"语音调度上车左回转rotate_up_left_x ="<<rotate_up_left_x<<std::endl;
            std::cerr<<"语音调度上车左回转rotate_up_left_x ="<<rotate_up_left_x<<std::endl;
            target_rotate_angel =crane_parameter_->crane_rotation_Angle_ + rotate_up_left_x;

            if(target_rotate_angel > 360)
            {
                target_rotate_angel = target_rotate_angel -360;
            }

            std::cerr<<"语音调度上车左回转目标回转target_rotate_angel ="<<target_rotate_angel<<std::endl;

            stopRotion();
            usleep(20*1000);
            leftRotation();

            rotate_up_left_x =0.0;
        }

        if((fabs(crane_parameter_->crane_rotation_Angle_ - target_rotate_angel) <1.0) && (target_rotate_angel != -1.0))
        {
            stopRotion();
            std::cerr<<"语音调度回转停止！！！"<<std::endl;
            std::cerr<<"语音调度回转停止！！！"<<std::endl;
            target_rotate_angel =-1.0;
        }



        //起变幅
        if(luff_up_x !=0.0)
        {
            std::cerr<<"语音调度起变幅luff_up_x ="<<luff_up_x<<std::endl;
            std::cerr<<"语音调度起变幅luff_up_x ="<<luff_up_x<<std::endl;
            target_luff_angel =crane_parameter_->crane_luff_Angle_ + luff_up_x;

            std::cerr<<"语音调度起变幅目标变幅target_luff_angel ="<<target_luff_angel<<std::endl;

            if(target_luff_angel > 75.0)
            {
                std::cerr<<"语音调度起变幅过载 过载！！！！！ ="<<std::endl;
                play_voice_flag =21;
            }
            else
            {
                stopAmp();
                usleep(20*1000);
                upAmp();
            }

            luff_up_x =0.0;
        }

        //落变幅
        if(luff_down_x !=0.0)
        {
            std::cerr<<"语音调度落变幅luff_down_x ="<<luff_down_x<<std::endl;
            std::cerr<<"语音调度落变幅luff_down_x ="<<luff_down_x<<std::endl;
            target_luff_angel =crane_parameter_->crane_luff_Angle_ - luff_down_x;

            std::cerr<<"语音调度落变幅目标变幅target_luff_angel ="<<target_luff_angel<<std::endl;

            if(target_luff_angel < 46.0)
            {
                std::cerr<<"语音调度起变幅过载 过载！！！！！ ="<<std::endl;
                play_voice_flag =21;
            }
            else
            {
                stopAmp();
                usleep(20*1000);
                downAmp();
            }

            luff_down_x =0.0;
        }

        if((fabs(crane_parameter_->crane_luff_Angle_ - target_luff_angel) <1.0) && (target_luff_angel != -1.0))
        {
            stopAmp();
            std::cerr<<"语音调度变幅停止！！！"<<std::endl;
            std::cerr<<"语音调度变幅停止！！！"<<std::endl;
            target_luff_angel =-1.0;
        }

        //起卷扬
        if(wind_up_x !=0.0)
        {
            std::cerr<<"语音调度起卷扬wind_up_x ="<<wind_up_x<<std::endl;
            std::cerr<<"语音调度起卷扬wind_up_x ="<<wind_up_x<<std::endl;
            target_wind_height =crane_parameter_->crane_hook_to_ground_height_ + wind_up_x;

            std::cerr<<"语音调度起卷扬目标卷杨target_wind_height ="<<target_wind_height<<std::endl;

            if(target_wind_height > 11.0)
            {
                std::cerr<<"语音调度卷杨 卷杨过载 过载！！！！！ ="<<std::endl;
                play_voice_flag =22;
            }
            else
            {
                stopLuff();
                usleep(20*1000);
                upLuff();
            }

            wind_up_x =0.0;
        }

        //落卷杨
        if(wind_down_x !=0.0)
        {
            std::cerr<<"语音调度落卷扬wind_down_x ="<<wind_down_x<<std::endl;
            std::cerr<<"语音调度落卷扬wind_down_x="<<wind_down_x<<std::endl;
            target_wind_height =crane_parameter_->crane_hook_to_ground_height_ - wind_down_x;

            std::cerr<<"语音调度落卷扬目标卷杨target_wind_height ="<<target_wind_height<<std::endl;

            if(target_wind_height < 1)
            {
                std::cerr<<"目标卷杨过低 过低！！！！！ ="<<std::endl;
                play_voice_flag =22;
            }
            else
            {
                stopLuff();
                usleep(20*1000);
                downLuff();
            }

            wind_down_x =0.0;
        }

        if((fabs(crane_parameter_->crane_hook_to_ground_height_ - target_wind_height) <0.1) && (target_wind_height != -1.0))
        {
            stopLuff();
            std::cerr<<"语音调度卷杨停止！！！"<<std::endl;
            std::cerr<<"语音调度卷杨停止！！！"<<std::endl;
            target_wind_height =-1.0;
        }

        //停止
        if(emergency_stop ==1)
        {
            std::cerr<<"语音调度停止！！！"<<std::endl;
            std::cerr<<"语音调度停止！！！"<<std::endl;
            stopRotion();
            stopAmp();
            stopLuff();
            emergency_stop =0;
        }

        usleep(200*1000);

    }
}


void MainWindow::Function_cramera_real_ptz_control_Thread()
{
    double nTempHeight = 0;

    QString _strZoomValue = "";
    int current_focusIndex = 0;
    int heart_num = 0;
    double _cameraHeight = 0.0;

    //is_start_cramera_real_ptz_control_thread_ = true;
    while (is_start_cramera_real_ptz_control_thread_)
    {
        //qDebug()<<heart_send_data;
        //std::cerr<<"heart_send_data" <<heart_num<<std::endl;
        //    _cameraHeight = crane_parameter_->crane_arm_to_ground_height_;
        //    if ((nTempHeight != _cameraHeight) && fabs(_cameraHeight - nTempHeight) > 0.2)
        //    {
        //      _strZoomValue = "10";  //10 the first focus of camera
        //      m_focusIndex = 0;
        //      if (_cameraHeight > 0 && _cameraHeight <= 30)
        //      {
        //        _strZoomValue = "10";  //10
        //        m_focusIndex = 0;
        //      }
        //      else if (_cameraHeight > 30 && _cameraHeight <= 40)
        //      {
        //        _strZoomValue = "12";  //12
        //        m_focusIndex = 1;
        //      }
        //      else if (_cameraHeight > 40 && _cameraHeight <= 60)
        //      {
        //        _strZoomValue = "20";
        //        m_focusIndex = 3;
        //      }
        //      else if (_cameraHeight > 60 && _cameraHeight <= 70)
        //      {
        //        _strZoomValue = "35";
        //        m_focusIndex = 6;
        //      }
        //      else if (_cameraHeight > 70 && _cameraHeight <= 80)
        //      {
        //        _strZoomValue = "30";
        //        m_focusIndex = 5;

        //      }
        //      else if (_cameraHeight > 80 && _cameraHeight <= 90)
        //      {
        //        _strZoomValue = "35";
        //        m_focusIndex = 6;

        //      }
        //      else
        //      {
        //        _strZoomValue = "12";
        //        m_focusIndex = 1;
        //      }
        //    }
        //    nTempHeight = _cameraHeight;

        //    if(current_focusIndex != m_focusIndex)
        //    {
        //      Camera_->setZoomValue(_strZoomValue);
        //    }
        //    current_focusIndex = m_focusIndex;
        usleep(1000 * 200);
    }
    is_start_cramera_real_ptz_control_thread_ = false;
    qDebug("DoCameraRealPTZControl thread exited.");
}

void MainWindow::Function_data_update_display_Thread()
{
    bool m_obstacle_light_flag = false;
    double _dpre_hook_height = 0.0;
    bool m_enable_light_flag = false;
    bool _tempColor = false;

    while(is_start_data_update_display_thread_)
    {
        m_param_mutex.lockForRead();
        double _dArmHeight = crane_parameter_->crane_arm_to_ground_height_;
        double _dMainArmLen = crane_parameter_->crane_mainarm_length_;
        double _dAngleValue = crane_parameter_->crane_luff_Angle_;
        double _dAmplitudeValue = crane_parameter_->crane_luff_length;
        double _dRotationAngle = crane_parameter_->crane_rotation_Angle_;
        m_param_mutex.unlock();

        m_target_value_mutex.lockForRead();
        double tempCopyTargetC = m_tempCopyTargetC_;
        double tempCopyTargetR = m_tempCopyTargetR_;
        m_target_value_mutex.unlock();

        m_hookheight_mutex.lockForRead();
        double _hook_height = /*crane_parameter_->crane_hook_to_ground_height_*/0.0;
        if(crane_parameter_->laser_hook_to_ground_height == 0)
        {
            _hook_height =  crane_parameter_->crane_hook_to_ground_height_;
        }
        _dpre_hook_height = crane_parameter_->laser_hook_to_ground_height;
        double _ground_height = crane_parameter_->laser_camera_to_ground_height_;
        m_hookheight_mutex.unlock();

        //起重机当前姿态参数显示
        QString str;
        str = QString::number(_dArmHeight, 'f', 2) + "m";
        ui.edit_arm_to_ground_height->setText(str);

        str = QString::number(_dMainArmLen, 'f', 2) + "m";
        ui.edit_current_mainArmLen->setText(str);

        str = QString::number(_dAngleValue, 'f', 2) + "°";
        ui.editAmpAngle->setText(str);

        str = QString::number(_dAmplitudeValue, 'f', 2) + "m";
        ui.edit_current_AmpLen->setText(str);

        str = QString::number(_dRotationAngle, 'f', 2) + "°";
        ui.editRoAngle->setText(str);

        str = QString::number(_dpre_hook_height - m_object_height, 'f', 2) + "°";
        ui.edit_current_hook_to_target_height->setText(str);

        ui.edit_device_State->setText(QString::fromLocal8Bit("正常"));

        //目标偏移量及运动控制目标参数
        double distance_x = 0.0;
        double distance_y = 0.0;
        if(is_start_object_position_compute_ == true)
        {
            distance_x = image_targetRelativeLen_x_ - hcfg_->camera_to_armcenter_x - _ground_height*tan(hcfg_->camera_xdire_angle*M_PI/180);
            distance_y = image_targetRelativeLen_y_ - hcfg_->camera_to_armcenter_y*cos(_dAngleValue* M_PI/180) + 0.131*sin(_dAngleValue* M_PI/180) + _ground_height*tan(hcfg_->camera_ydire_angle*M_PI/180);
        }

        ui.edit_target_hook_to_target_height->setText(QString::number(_hook_height /*- m_object_height*/, 'f', 2) + "m");
        ui.edit_target_RoAngle->setText(QString::number(tempCopyTargetC, 'f', 2) + "°");
        ui.edit_target_luffLen->setText(QString::number(tempCopyTargetR, 'f', 2) + "m");

        if(dispatchtask_.ifget == true && avoidcommandtask_.status == true && commandtask_.status == true && _tempColor == false)
        {
            _tempColor = true;
            ui.labDispatchTask->setStyleSheet("QLabel{background-color:rgb(0, 255, 0);}");
        }
        if(((m_clsClientUsbToCan->Data01B7[0] & 0x80) == 0x80) && (m_enable_light_flag == false))
        {
            m_enable_light_flag = true;
            SetStatusBarState(QString::fromLocal8Bit("提示：自动操控模式！"),  false);
        }
        else if(((m_clsClientUsbToCan->Data01B7[0] & 0x80) == 0x00) && (m_enable_light_flag == true))
        {
            m_enable_light_flag = false;
            SetStatusBarState(QString::fromLocal8Bit("提示：人工操控模式！"),  false);
        }

        //智能自动吊装状态参数显示,状态变化参数根据运动过程中而定
        if(is_stop_send_trajectory_.load() == true || is_start_crane_trajectory_track_ == true)
        {
            ui.edit_trajectory_State->setText(QString::fromLocal8Bit("进行中..."));
        }
        else if(is_predicted_collision_.load() == true && m_obstacle_light_flag == false)
        {
            m_obstacle_light_flag = true;
            ui.edit_trajectory_State->setText("避障中...");
            m_label_paint_mutex.lock();
            ui.labObsStateLight->setPixmap(QPixmap("Yellow.bmp"));
            m_label_paint_mutex.unlock();
        }
        else
        {
            ui.edit_trajectory_State->setText(QString::fromLocal8Bit("就绪"));
        }

        if(m_lift_work_status_ == 0)
        {
            ui.edit_accurate_Lift_State->setText(QString::fromLocal8Bit("就绪"));
        }
        else if(m_lift_work_status_ == 1)
        {
            ui.edit_accurate_Lift_State->setText(QString::fromLocal8Bit("进行中..."));
        }
        else if(m_lift_work_status_ == 2)
        {
            ui.edit_accurate_Lift_State->setText(QString::fromLocal8Bit("完成"));
        }

        usleep(5 * 1000);
    }

    is_start_data_update_display_thread_ = false;
    qDebug("DoThreadUpdateUiData thread exited.");
}
/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::Start_LabelVedio_Point_Slot(QPoint point)
{
    is_selectObjectFlag_ = false;
    is_start_object_position_compute_ = false;
    start_label_vedio_point_ = point;

    //获取开始坐标槽函数
    double label_size_w = label_vedio_->size().width();
    double label_size_h = label_vedio_->size().height();

    //计算比例 得出原图的坐标  (因为图片显示在label上大小变化了)
    m_vedioRoi_x = (point.x() / label_size_w) * m_image_width_;
    m_vedioRoi_y = (point.y() / label_size_h) * m_image_height_;
}

//cuxuan
void MainWindow::Start_Labelimg_Point_Slot(QPoint point)
{/*
    start_label_vedio_point_ = point;
    double label_size_w = label_vedio_->size().width();
    double label_size_h = label_vedio_->size().height();
    if(depth_img.empty())
    {
        depth_img = me->load_xml("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/depth.xml");
    }

    //计算比例 得出原图的坐标  (因为图片显示在label上大小变化了)
    m_vedioRoi_x = (point.x() / label_size_w) * depth_img.cols;// * (depth_img.cols / 2048);
    m_vedioRoi_y = (point.y() / label_size_h) * depth_img.rows;//* (depth_img.rows / 1536);
    int u = m_vedioRoi_x;
    int v = m_vedioRoi_y;
    int threshold1 = 20;
    std::cerr<<"click point is "<<u<<"    "<<v<<std::endl;
    if((depth_img.at<Vec3s>(v,u)[0]) == 0)
    {
        for(int i = 1;i <= threshold1;i++)
        {
            if((depth_img.at<Vec3s>(v + i,u)[0]) != 0)
            {
                v = v + i;
                break;
            }
            if((depth_img.at<Vec3s>(v,u + i)[0]) != 0)
            {
                u = u + i;
                break;
            }
            if((depth_img.at<Vec3s>(v + i,u + i)[0]) != 0)
            {
                u = u + i;
                v = v + i;
                break;
            }
            if((depth_img.at<Vec3s>(v - i,u)[0]) != 0)
            {
                v = v - i;
                break;
            }
            if((depth_img.at<Vec3s>(v,u - i)[0]) != 0)
            {
                u = u - i;
                break;
            }
            if((depth_img.at<Vec3s>(v + i,u - i)[0]) != 0)
            {
                u = u - i;
                v = v - i;
                break;
            }
        }
    }
    if((depth_img.at<Vec3s>(v,u)[0]) == 0)
    {
        std::cerr<<"error ,the target point Coordinate z is zero!!!!"<<std::endl;
        std::cerr<<crane_parameter_->crane_arm_to_ground_height_<<std::endl;
        std::cerr<<(depth_img.cols/2.0 - m_vedioRoi_x)*crane_parameter_->crane_arm_to_ground_height_/baseFocus[m_focusIndex]<<std::endl;
        std::cerr<<(depth_img.rows/2.0 - m_vedioRoi_y)*crane_parameter_->crane_arm_to_ground_height_/baseFocus[m_focusIndex]<<std::endl;
    }
    else
    {
        std::cerr<<depth_img.at<Vec3s>(v,u)[0]/100.0<<endl;
        std::cerr<<depth_img.at<Vec3s>(v,u)[1]/100.0<<endl;
        std::cerr<<depth_img.at<Vec3s>(v,u)[2]/100.0<<endl;
    }
    */
}

//精选
void MainWindow::Stop_LabelVedio_Point_Slot(QPoint point)
{
    //获取结束坐标槽函数
    if(!is_start_crane_trajectory_track_)
    {
        if(is_directly_visual_location_ == true)
        {
            if(QMessageBox::question(nullptr, "精准定位确认", "请确认当前框选是否正确?\n\n点击“精准定位”按钮进行空钩精定位控制?","是","否",0) == 1)
            {
                return;
            }
        }
        else
        {
            if(QMessageBox::question(nullptr, "粗定位确认", "请确认当前框选是否正确?\n\n待相机回正后,点击“路径规划”按钮执行路径规划操作","是","否",0) == 1)
            {
                return;
            }
        }
    }
    else
    {
        if(QMessageBox::question(nullptr, "精准定位确认", "请确认当前框选是否正确?确认后系统将自动进行精准定位操作","是","否",0) == 1)
        {
            return;
        }
    }
    std::cerr <<"---------------StopPointSlot:目标框选-----------------"<<std::endl;

    double label_size_w = label_vedio_->size().width();
    double label_size_h = label_vedio_->size().height();

    double point_End_x = (point.x() / label_size_w) * m_image_width_;
    double point_End_y = (point.y() / label_size_h) * m_image_height_;

    m_vedioRoi_x = m_vedioRoi_x > 1 ? m_vedioRoi_x : 1;
    m_vedioRoi_y = m_vedioRoi_y > 1 ? m_vedioRoi_y : 1;

    point_End_x = point_End_x < m_image_width_ ? point_End_x : m_image_width_ - 1;
    point_End_y = point_End_y < m_image_height_ ? point_End_y : m_image_height_ - 1;

    m_vedioRoi_w = point_End_x - m_vedioRoi_x;
    m_vedioRoi_h = point_End_y - m_vedioRoi_y;
    std::cerr << "m_vedioRoi_w is : " << m_vedioRoi_w << std::endl;
    std::cerr << "m_vedioRoi_h is : " << m_vedioRoi_h << std::endl;

    if(m_vedioRoi_h > 1 && m_vedioRoi_w > 1)
    {
        //calculate the center x and y coordination of the selected box
        int pix_center_point_x = (m_vedioRoi_x + point_End_x) / 2;
        int pix_center_point_y = (m_vedioRoi_y + point_End_y) / 2;

        //根据图像像素值计算物体相机坐标系值
        get_object_cameraCoordinate_based_cameraPixs(pix_center_point_x,pix_center_point_y);
        std::cerr<<"img center point coordinate value (x,y): (" << std::to_string(pix_center_point_x) << "," << std::to_string(pix_center_point_y) << ")"<<std::endl;

        is_selectObjectFlag_ = true;

        if(is_start_crane_trajectory_track_)
        {
            std::cerr<<"-----------------StopPointSlot: g_start_intel_crane = 1 :开启孪生网络跟踪线程------------------"<<std::endl;
            //                StartComputeAndPosition();
            emit eraseRect();
        }
        else
        {
            std::cerr<<"------------------------StopPointSlot:相机回归初始位置------------------------"<<std::endl;
            //                laserVerticalFlag  = true;              //雷达回归初始位置,startting get laser height
            m_clsClientToYunTai->recovery_camera_to_init_position(hcfg_->ptz_str_pitch_initial_angle, hcfg_->ptz_str_rotate_initial_angle);
            emit eraseRect();
            //多次框选初始化云台参数 杨涛add  for 云台复位后仍旧粗定位是对的
            camera_pitch_angle_ = 0;
            camera_rotate_angle_ = 0;
        }
        m_lift_work_status_ = 0;
    }
    else
    {
        std::cerr << "the select operation is not correct !" << std::endl;
    }
}


void MainWindow::read_lift_weight_form_data()
{
    QString config = "";
    double main_link_length = 0.0;
    QString main_link_trigger_type = "";
    if(crane_parameter_->crane_legState_ == 0)
    {
        config = QString::fromLocal8Bit("_total_outtrigger_");
    }
    else if(crane_parameter_->crane_legState_ == 1)
    {
        config = QString::fromLocal8Bit("_half_outtrigger_");
    }
    else
    {
        std::cerr<<"read liftweight but g_nLegState read error"<<std::endl;
    }

    if(crane_parameter_->crane_Bob_Weight_ == 0)
    {
        config += QString::fromLocal8Bit("68T");
    }
    else if(crane_parameter_->crane_Bob_Weight_ == 1)
    {
        config += QString::fromLocal8Bit("76T");

    }
    else if(crane_parameter_->crane_Bob_Weight_ == 2)
    {
        config += QString::fromLocal8Bit("48T");

    }
    else if(crane_parameter_->crane_Bob_Weight_ == 3)
    {
        config += QString::fromLocal8Bit("36T");

    }
    else if(crane_parameter_->crane_Bob_Weight_ == 4)
    {
        config += QString::fromLocal8Bit("12T");

    }
    else if(crane_parameter_->crane_Bob_Weight_ == 5)
    {
        config += QString::fromLocal8Bit("0T");

    }
    else
    {
        std::cerr << "read liftweight but read g_nBobWeight error" << std::endl;
    }

    QString strArmCombination = "";
    for(int i = 0; i < 6; i++)
    {
        strArmCombination += QString::number(1);
    }

    main_link_length = crane_parameter_->crane_mainarm_length_;
    main_link_trigger_type = strArmCombination;
    auto_crane_manipulation_->update_robot_work_config(config.toStdString(),main_link_length, main_link_trigger_type.toStdString());
}


//过滤障碍物
void MainWindow::filter_collision_objects(const std::vector<ObstacleInforStr>& detected_obstacles,
                                          std::vector<ObstacleInforStr>& filtered_obstacles)
{
    auto confirm_collsion_is_roi_inline = [](const PointType& pt,const PointType& roi_tl_pt,const PointType& roi_br_pt)
    {
        if((pt.x>roi_tl_pt.x && pt.x<roi_br_pt.x)
                && (pt.y<roi_tl_pt.y && pt.y>roi_br_pt.y))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    };

    auto calculate_position_distance = [](const ObstacleInforStr& obj_1, const ObstacleInforStr& obj_2)
    {
        return sqrt((obj_1.centerPt.x - obj_2.centerPt.x)*(obj_1.centerPt.x - obj_2.centerPt.x) +
                    (obj_1.centerPt.y - obj_2.centerPt.y)*(obj_1.centerPt.y - obj_2.centerPt.y) +
                    (obj_1.centerPt.z - obj_2.centerPt.z)*(obj_1.centerPt.z - obj_2.centerPt.z));
    };

    auto calculate_dimension_distance = [](const ObstacleInforStr& obj_1, const ObstacleInforStr& obj_2)
    {
        return fabs(obj_1.box3d.depth_z  - obj_2.box3d.depth_z)*
                fabs(obj_1.box3d.height_y - obj_2.box3d.height_y)*
                fabs(obj_1.box3d.width_x  - obj_2.box3d.width_x);
    };

    filtered_obstacles.clear();

    PointType tl_pt = {-5,5,0};
    PointType br_pt = {10,-5,0};

    //std::cerr << "*****************filter_collision_objects input size: " << detected_obstacles.size() << " *****************" <<std::endl;
    for (const auto& detected_obs: detected_obstacles)
    {
        //std::cerr<<"obstacle filter before*****************"<<std::endl;
        if(confirm_collsion_is_roi_inline(detected_obs.centerPt,tl_pt,br_pt))
        {
            //qDebug("The detected collision object is out of roi range.");
            continue;
        }
        bool is_use_last_one = false;
        for (const auto& last_added_object: last_added_collision_objects_)
        {
            if (calculate_position_distance(last_added_object, detected_obs)  < 2 &&
                    calculate_dimension_distance(last_added_object, detected_obs) < 2)
            {
                filtered_obstacles.push_back(last_added_object);
                is_use_last_one = true;
                break;
            }
        }
        if (!is_use_last_one)
        {
            //过滤掉紧贴地面以及虚警信息
            if(detected_obs.centerPt.z <0.5)
            {
                continue;
            }
            filtered_obstacles.push_back(detected_obs);
        }
    }
}


void MainWindow::visualize_Crane_Robot()
{
    auto current_joint_state = current_joint_state_;
    glWidget_->setRobotGLList(auto_crane_manipulation_->visualize_robot(current_joint_state.positions[0],
                              current_joint_state.positions[1],
            current_joint_state.positions[2]));
    glWidget_->setEnvironmentGLList(auto_crane_manipulation_->get_rendered_environment());

}

void MainWindow::Timer_pathplan_scence_Update()
{
    //visualize the collision detection results
    //remove the previous added collision objects detected from the scene monitor
    //auto_crane_manipulation_->remove_collision_object("test_replan_cube");
    std::vector<ObstacleInforStr> collision_objects;
    for (unsigned int j = 0; j < last_added_collision_objects_.size(); j++)
    {
        auto_crane_manipulation_->remove_collision_object("temp_ob_" + std::to_string(j));
    }

    //added the collision objects detected from the scene monitor
    m_obstacle_detect_class->GetObstaclesValue(collision_objects);
    Eigen::Isometry3f main_link_end_to_world_tf;
    if (auto_crane_manipulation_->get_main_link_end_to_world_tf(crane_parameter_->crane_rotation_Angle_*M_PI/180,
                                                                crane_parameter_->crane_luff_Angle_*M_PI/180,
                                                                crane_parameter_->crane_arm_to_ground_height_,
                                                                main_link_end_to_world_tf))
    {
        //transform the collision objects to world frame
        for (unsigned int i = 0; i < collision_objects.size(); i++)
        {
            Eigen::Isometry3f object_pose = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
                                                              *Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
            object_pose.translation() = Eigen::Vector3f(collision_objects[i].centerPt.x,
                                                        collision_objects[i].centerPt.y,
                                                        collision_objects[i].centerPt.z);
            object_pose = main_link_end_to_world_tf * object_pose;
            collision_objects[i].centerPt.x = object_pose.translation()[0];
            collision_objects[i].centerPt.y = object_pose.translation()[1];
            collision_objects[i].centerPt.z = object_pose.translation()[2];
        }

        //filter collision objects
        std::vector<ObstacleInforStr> filtered_collision_objects;
        filter_collision_objects(collision_objects, filtered_collision_objects);

        //add the filtered collision objects
        std::vector<std::string> allowed_collision_names;
        for (unsigned int j = 0; j < filtered_collision_objects.size(); j++)
        {
            std::string name = "temp_ob_" + std::to_string(j);
            auto_crane_manipulation_->add_cube_collision_object(name,
                                                                filtered_collision_objects[j].box3d.width_x,
                                                                filtered_collision_objects[j].box3d.height_y,
                                                                filtered_collision_objects[j].box3d.depth_z,
                                                                filtered_collision_objects[j].centerPt.x,
                                                                filtered_collision_objects[j].centerPt.y,
                                                                filtered_collision_objects[j].centerPt.z, 0, 0, 0);
            allowed_collision_names.push_back(name);
            //std::cerr << "add " << name << " into env" << std::endl;
        }

        //update the last_added_collision_objects_
        last_added_collision_objects_ = filtered_collision_objects;
        auto_crane_manipulation_->set_allowed_collision(allowed_collision_names);
    }
    else
    {
        std::cerr << "Failed to get_main_link_end_to_world_tf" << std::endl;
    }

    double _fobstacle_avoid_distance = 0.5;
    if (fabs(_fobstacle_avoid_distance) > 1e-6 && !planning_scene_visualization_timer_->isActive())
    {
        on_path_plan_triggered();
    }

    //    if (g_nPickPlaceMode == zl_common::app::zl::PickPlaceMode::REPEAT && !bThreadAntiSwing)
    //    {
    //        on_path_plan_triggered();
    //        on_Trajectory_tracking_triggered();
    //    }
}

void MainWindow::Timer_visualize_lift_pathplan_scence()
{
    std::vector<ObjResult> laser_obs;
    std::vector<ObstacleInforStr> arm_head_obs;
    ros_thread->getObjectsResult(laser_obs);
    if(laser_obs.size() > 0)
    {
        arm_head_obs.resize(laser_obs.size());
        for(int i = 0;i < laser_obs.size();i++)
        {
            PointType radarPos;
            PointType endPos;
            //滤掉钢丝绳
            if(laser_obs[i].c_x >= -3 && laser_obs[i].c_x <=3 && laser_obs[i].c_y >= -3 && laser_obs[i].c_y <=3)
            {
                continue;
            }
            radarPos.x = laser_obs[i].c_x;
            radarPos.y = laser_obs[i].c_y;
            radarPos.z = laser_obs[i].c_z;
            //radarPos.z = 70;
            m_obstacle_detect_class->PositionTranslate(radarPos,crane_parameter_->crane_luff_Angle_*M_PI/180,endPos);
            //滤掉 臂节
            if(endPos.x >= -3 && endPos.x <=3 && endPos.y >= -3 && endPos.y <=3)
            {
                continue;
            }
            arm_head_obs[i].centerPt.x = endPos.x;
            arm_head_obs[i].centerPt.y = endPos.y;
            arm_head_obs[i].centerPt.z = endPos.z;
            arm_head_obs[i].box3d.width_x = laser_obs[i].width_x;
            arm_head_obs[i].box3d.height_y = laser_obs[i].height_y;
            arm_head_obs[i].box3d.depth_z = laser_obs[i].depth_z;
            arm_head_obs[i].yaw = atan(2*(laser_obs[i].qw*laser_obs[i].qx+laser_obs[i].qy*laser_obs[i].qz)/(1-2*pow(laser_obs[i].qx,2)+2*pow(laser_obs[i].qy,2)))*180/M_PI;
            arm_head_obs[i].pitch = asin(2*(laser_obs[i].qw*laser_obs[i].qy-laser_obs[i].qz*laser_obs[i].qx))*180/M_PI;
            arm_head_obs[i].roll = atan(2*(laser_obs[i].qw*laser_obs[i].qz+laser_obs[i].qy*laser_obs[i].qx)/(1-2*pow(laser_obs[i].qy,2)+2*pow(laser_obs[i].qz,2)))*180/M_PI;
        }


        ofstream outfile("/home/zoomlion/3d_write_angle.txt");
        outfile<<"------------------------------------------------"<<std::endl;
        outfile<<"------------------------------------------------"<<std::endl;
        for(int i = 0;i < laser_obs.size();i++)
        {
            outfile<<arm_head_obs[i].yaw<<" "<<arm_head_obs[i].pitch<<" "<<arm_head_obs[i].roll<<std::endl;
        }
        outfile.close();


        Eigen::Isometry3f main_link_end_to_world_tf;
        std::vector<std::string> allowed_collision_names;
        if (auto_crane_manipulation_->get_main_link_end_to_world_tf(crane_parameter_->crane_rotation_Angle_*M_PI/180,
                                                                    crane_parameter_->crane_luff_Angle_*M_PI/180,
                                                                    crane_parameter_->crane_arm_to_ground_height_,
                                                                    main_link_end_to_world_tf))
            //    if (auto_crane_manipulation_->get_main_link_end_to_world_tf(current_joint_state_.positions[0],
            //                                                                current_joint_state_.positions[1],
            //                                                                current_joint_state_.positions[2],
            //                                                                main_link_end_to_world_tf))
        {
            //transform the collision objects to world frame
            for (unsigned int i = 0; i < arm_head_obs.size(); i++)
            {
                Eigen::Isometry3f object_pose = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
                                                                  *Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
                object_pose.translation() = Eigen::Vector3f(arm_head_obs[i].centerPt.x,
                                                            arm_head_obs[i].centerPt.y,
                                                            arm_head_obs[i].centerPt.z);
                object_pose = main_link_end_to_world_tf * object_pose;
                arm_head_obs[i].centerPt.x = object_pose.translation()[0];
                arm_head_obs[i].centerPt.y = object_pose.translation()[1];
                arm_head_obs[i].centerPt.z = object_pose.translation()[2];
            }

            //filter collision objects
            std::vector<ObstacleInforStr> filtered_collision_objects;
            filter_collision_objects(arm_head_obs, filtered_collision_objects);
            //std::cerr<<"path plan the filter_collision_objects is : "<<filtered_collision_objects.size()<<std::endl;

            //add the filtered collision objects
            for (unsigned int j = 0; j < filtered_collision_objects.size(); j++)
            {
                if(filtered_collision_objects[j].centerPt.z < 1 || filtered_collision_objects[j].centerPt.x > 50 || filtered_collision_objects[j].centerPt.y >50)
                {
                    continue;
                }
                std::string name = "temp_ob_" + std::to_string(j);
                auto_crane_manipulation_->add_cube_collision_object(name,
                                                                    filtered_collision_objects[j].box3d.height_y,
                                                                    filtered_collision_objects[j].box3d.width_x,
                                                                    filtered_collision_objects[j].box3d.depth_z,
                                                                    filtered_collision_objects[j].centerPt.x,
                                                                    filtered_collision_objects[j].centerPt.y,
                                                                    filtered_collision_objects[j].centerPt.z, 0, 0, (crane_parameter_->crane_rotation_Angle_ - filtered_collision_objects[j].roll)*M_PI/180);
                allowed_collision_names.push_back(name);

            }

            //update the last_added_collision_objects_
            last_added_collision_objects_ = filtered_collision_objects;
            auto_crane_manipulation_->set_allowed_collision(allowed_collision_names);
        }
        else
        {
            std::cerr << "Failed to get_main_link_end_to_world_tf" << std::endl;
        }
    }
    else if(laser_obs.size() == 0)
    {
        std::ifstream  fin("/home/zoomlion/filter_collision_objects_file.txt", std::ios::in);
        char  line[1024]={0};
        std::vector<ObstacleInforStr> get_3d_collision_objects_from_txt;
        ObstacleInforStr obj_;
        while(fin.getline(line, sizeof(line)))
        {
            std::stringstream  word(line);
            word >> obj_.centerPt.x;
            word >> obj_.centerPt.y;
            word >> obj_.centerPt.z;
            word >> obj_.box3d.width_x;
            word >> obj_.box3d.height_y;
            word >> obj_.box3d.depth_z;
            word >> obj_.roll;
            get_3d_collision_objects_from_txt.push_back(obj_);
        }
        fin.clear();
        fin.close();
        std::vector<ObstacleInforStr> filtered_collision_objects_txt;
        std::vector<std::string> allowed_collision_names;
        filter_collision_objects(get_3d_collision_objects_from_txt, filtered_collision_objects_txt);
        //add the filtered collision objects
        for (unsigned int j = 0; j < filtered_collision_objects_txt.size(); j++)
        {
            if(filtered_collision_objects_txt[j].centerPt.z < 1 || filtered_collision_objects_txt[j].centerPt.x > 50 || filtered_collision_objects_txt[j].centerPt.y >50)
            {
                continue;
            }
            std::string name = "temp_ob_" + std::to_string(j);
            auto_crane_manipulation_->add_cube_collision_object(name,
                                                                filtered_collision_objects_txt[j].box3d.height_y,
                                                                filtered_collision_objects_txt[j].box3d.width_x,
                                                                filtered_collision_objects_txt[j].box3d.depth_z,
                                                                filtered_collision_objects_txt[j].centerPt.x,
                                                                filtered_collision_objects_txt[j].centerPt.y,
                                                                filtered_collision_objects_txt[j].centerPt.z, 0, 0, (crane_parameter_->crane_rotation_Angle_ - filtered_collision_objects_txt[j].roll)*M_PI/180);
            allowed_collision_names.push_back(name);
        }
        last_added_collision_objects_ = filtered_collision_objects_txt;
        auto_crane_manipulation_->set_allowed_collision(allowed_collision_names);
    }
    else
    {
        std::cerr<<"3d restruction is error!!"<<std::endl;
        return;
    }

    if (m_clsClientUsbToCan->Data0383[6] & 0x10)//pitch up
    {
        glWidget_->pitch(-0.3);
    }
    else if (m_clsClientUsbToCan->Data0383[6] & 0x20)//pitch down
    {
        glWidget_->pitch(0.3);
    }
    else if (m_clsClientUsbToCan->Data0383[6] & 0x40)//rotate left
    {
        glWidget_->roll(0.3);
    }
    else if (m_clsClientUsbToCan->Data0383[6] & 0x80)//rotate right
    {
        glWidget_->roll(-0.3);
    }

    //visualize the anit swing joint trajectory
    //  if (g_is_need_update_anti_swing_trajectory.load())
    //  {
    //    int gl_lists;
    //    if (auto_crane_manipulation_->visualize_cartesian_trajectory(g_anti_swing_trajectory, gl_lists))
    //    {
    //      glWidget_->setSolutionTrajectoryEETipPath(gl_lists);
    //      g_is_need_update_anti_swing_trajectory.store(false);
    //    }
    //  }

    //  crane_parameter_->crane_rotation_Angle_ = current_joint_state_.positions[0] * 180 / M_PI;
    //  crane_parameter_->crane_luff_Angle_ = current_joint_state_.positions[1] * 180 / M_PI;
    double rope_length_offset = 0.0;
    glWidget_->setRobotGLList(auto_crane_manipulation_->visualize_robot(
                                  crane_parameter_->crane_rotation_Angle_*M_PI/180,
                                  crane_parameter_->crane_luff_Angle_*M_PI/180,
                                  crane_parameter_->crane_arm_to_ground_height_ -
                                  crane_parameter_->crane_hook_to_ground_height_ - 1.2 +
                                  rope_length_offset));
    glWidget_->setEnvironmentGLList(auto_crane_manipulation_->get_rendered_environment());

}


void MainWindow::sendJointTrajectory(std::promise<bool> *start_RTCC_promise)
{
    ompl::app::joint_trajectory_point_t point_now;
    auto get_current_dir_name = []()
    {
        std::string current_time;
        QDateTime datetime = QDateTime::currentDateTime();
        QString timestr = datetime.toString("yyyy-MM-dd hh:mm:ss");
        current_time = timestr.toStdString();
        return current_time;
    };

    std::string file_path = base_path + ".catkin_data/path_plan/joint_trajectory_control_data/" + get_current_dir_name() + ".txt";
    joint_trajectory_command_file_saver_ = std::ofstream(file_path);
    joint_trajectory_command_file_saver_ << "time"  << " "
                                         << "current pj0" << " " << "current pj1" << " " << "current pj2" << " "
                                         << "current vj0" << " " << "current vj1" << " " << "current vj2" << " "
                                         << "command pj0" << " " << "command pj1" << " " << "command pj2" << " "
                                         << "command vj0" << " " << "command vj1" << " " << "command vj2" << std::endl;
    current_joint_state_.positions[0] = crane_parameter_->crane_rotation_Angle_*M_PI/180;
    current_joint_state_.positions[1] = crane_parameter_->crane_luff_Angle_*M_PI/180;
    current_joint_state_.positions[2] = crane_parameter_->crane_arm_to_ground_height_ - crane_parameter_->crane_hook_to_ground_height_ - 1.2;

    previous_joint_state_ = current_joint_state_;

    m_clsClientUsbToCan->SendCanData("01D1", "0005000000000000");
    sleep(1);//sleep 1 second, wait the motor enabled

    is_stop_send_trajectory_.store(true);
    is_running_predict_collision_.store(true);//if set to true will trigger the collision detection thread, otherwise will not.
    sent_joint_trajectory_index_.store(0);
    int solution_trajectory_point_size = solution_trajectory_.points.size();

    last_sent_point_ = solution_trajectory_.points.front();

    ompl::app::joint_trajectory_t stitched_with_target_trajectory;
    stitched_with_target_trajectory.points.clear();
    ompl::app::joint_state_t stitch_threshold;
    stitch_threshold.positions.push_back(1000);//6 degree
    stitch_threshold.positions.push_back(1000);//6 degree
    stitch_threshold.positions.push_back(500);

    start_RTCC_promise->set_value(true);

    while (is_stop_send_trajectory_.load() && sent_joint_trajectory_index_.load() < solution_trajectory_point_size)
    {
        //    if(is_directly_visual_location_ && is_detect_Chessboard_success && ((point_now.velocities[0] != 0) || (point_now.velocities[1] != 0)))
        //    {
        //      int slow_stop_time = 4;
        //      ompl::app::joint_trajectory_point_t point;
        //      point.velocities.clear();
        //      point.positions.clear();
        //      double stop_r = (point_now.velocities[0]/(slow_stop_time*10));
        //      double stop_c = (point_now.velocities[1]/(slow_stop_time*10));
        //      double stop_h = (point_now.velocities[2]/(slow_stop_time*10));

        //      for(int i = 0;i < slow_stop_time*10;i++)
        //      {
        //        point.velocities.push_back(point_now.velocities[0] - stop_r*i);
        //        point.velocities.push_back(point_now.velocities[1] - stop_c*i);
        //        point.velocities.push_back(point_now.velocities[2] - stop_h*i);
        //        point.positions.push_back(point_now.positions[0] + (point.velocities[0]*i/10));
        //        point.positions.push_back(point_now.positions[1] + (point.velocities[1]*i/10));
        //        point.positions.push_back(point_now.positions[2] + (point.velocities[2]*i/10));

        //        sendJointTrajectoryPoint(point);
        //        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //        point.velocities.clear();
        //        point.positions.clear();
        //      }
        //      is_trajectory_track_slow_to_stop_complete_ = true;
        //      break;
        //    }

        m_pathplan_mutex.lock();
        int message = 0;
        if (!is_predicted_collision_.load())
        {
            if (true)//TODO check the controller state if send the next point
            {
                auto send_point = solution_trajectory_.points[sent_joint_trajectory_index_.load()];

                ompl::app::joint_state_t current_joint_state;

                current_joint_state.positions.push_back(crane_parameter_->crane_rotation_Angle_*M_PI/180);
                current_joint_state.positions.push_back(crane_parameter_->crane_luff_Angle_*M_PI/180);
                current_joint_state.positions.push_back(crane_parameter_->crane_arm_to_ground_height_);
                if ((fabs(current_joint_state.positions[0] - send_point.positions[0]) > stitch_threshold.positions[0] ||
                     fabs(current_joint_state.positions[1] - send_point.positions[1]) > stitch_threshold.positions[1])
                        && stitched_with_target_trajectory.points.empty())
                {
                    //stitch to target joint trajectory
                    auto_crane_manipulation_->stitch_to_target_trajectory(current_joint_state,
                                                                          solution_trajectory_,
                                                                          sent_joint_trajectory_index_.load(),
                                                                          stitched_with_target_trajectory, message);
                }

                if (stitched_with_target_trajectory.points.empty())
                {
                    //send joint trajectory
                    sendJointTrajectoryPoint(send_point);
                    point_now = send_point;

                    if(sent_joint_trajectory_index_.load() == 0)
                        sleep(3);//sleep 3s, when send the first joint trajectory point.

                    sent_joint_trajectory_index_++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                else
                {
                    //send the stitched joint trajectory
                    sendJointTrajectoryPoint(stitched_with_target_trajectory.points.front());
                    point_now = stitched_with_target_trajectory.points.front();
                    stitched_with_target_trajectory.points.erase(stitched_with_target_trajectory.points.begin());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        else
        {
            auto predicted_collision_index = predicted_collision_index_.load();
            auto sent_joint_trajectory_index = sent_joint_trajectory_index_.load();
            int current_to_predicted_collision = predicted_collision_index - sent_joint_trajectory_index;
            if (current_to_predicted_collision <= zl_common::app::zl::ReactionThreshold::E_STOP)
            {
                //TODO: send the emergence stop command to the controller
                qDebug("send trajectory thread: E_STOP, current_to_predicted_collision: %d", current_to_predicted_collision);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                if (!is_replanning_.load())
                {
                    solution_trajectory_ = auto_crane_manipulation_->get_solution_trajectory();
                    solution_trajectory_point_size = solution_trajectory_.points.size();
                    sent_joint_trajectory_index_.store(0);
                    predicted_collision_index_.store(0);
                    is_predicted_collision_.store(false);
                }
            }
            else if (sent_joint_trajectory_index < slow_down_end_index_.load())
            {
                if (!is_replanning_.load())
                {
                    if (sent_joint_trajectory_index_.load() == replan_start_index_.load())
                    {
                        auto replanned_solution_trajectory = auto_crane_manipulation_->get_solution_trajectory();
                        if (!replanned_solution_trajectory.points.empty())
                        {
                            solution_trajectory_ = replanned_solution_trajectory;
                            solution_trajectory_point_size = solution_trajectory_.points.size();
                            sent_joint_trajectory_index_.store(0);
                            predicted_collision_index_.store(0);
                            is_predicted_collision_.store(false);
                            continue;
                        }
                        else
                        {
                            qDebug("send trajectory thread: [sent_joint_trajectory_index < slow_down_end_index_], the replanned trajectroy is empty, it seems that replan failed.");
                            break;
                        }
                    }
                    else if (sent_joint_trajectory_index_.load() > replan_start_index_.load())
                    {
                        qDebug("send trajectory thread: the sent_joint_trajectory_index_ is greater than replan_start_index_.");
                        //TODO slow down the trajectory.
                    }
                    else
                    {
                        qDebug("send trajectory thread: waiting the robot moves to the replan start state.");
                    }
                }
                //TODO check the controller state
                if (true)
                {
                    //send joint trajectory
                    qDebug("send trajectory thread: sent_joint_trajectory_index is smaller than slow_down_end_index, so continue to send the trajectory, current_to_predicted_collision: %d", current_to_predicted_collision);
                    int time;
                    if (stitched_with_target_trajectory.points.empty())
                    {
                        //send joint trajectory
                        auto previous_point = solution_trajectory_.points[sent_joint_trajectory_index_.load() - 1];
                        auto send_point = solution_trajectory_.points[sent_joint_trajectory_index_.load()];

                        sendJointTrajectoryPoint(send_point);
                        point_now = send_point;
                        sent_joint_trajectory_index_++;

                        time = static_cast<int>((send_point.time_from_start - previous_point.time_from_start) * 1000);
                        std::this_thread::sleep_for(std::chrono::milliseconds(time));
                    }
                    else
                    {
                        //send the stitched joint trajectory
                        sendJointTrajectoryPoint(stitched_with_target_trajectory.points.front());
                        point_now = stitched_with_target_trajectory.points.front();
                        stitched_with_target_trajectory.points.erase(stitched_with_target_trajectory.points.begin());

                        time = 100;
                        std::this_thread::sleep_for(std::chrono::milliseconds(time));
                    }
                    qDebug("send trajectory thread: sleep time: %d", time );
                }
                else
                {
                    qWarning("send trajectory thread, FATAL_ERROR: the controller state is not ready, failed to send next command.");
                }
            }
            else if (sent_joint_trajectory_index == slow_down_end_index_.load())
            {
                //send the slow down end point
                //TODO: check the controller state
                if (true)
                {
                    //send joint trajectory
                    qDebug("send trajectory thread: continue to send the trajectory, current_to_predicted_collision: %d", current_to_predicted_collision);
                    auto previous_point = solution_trajectory_.points[sent_joint_trajectory_index_.load() - 1];
                    auto send_point = solution_trajectory_.points[sent_joint_trajectory_index_.load()];

                    sendJointTrajectoryPoint(send_point);
                    point_now = send_point;

                    int time = static_cast<int>((send_point.time_from_start - previous_point.time_from_start) * 1000);
                    std::cout << "sleep time: " << time << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(time));
                }
                else
                {
                    qFatal("send trajectory thread, FATAL_ERROR: the controller state is not ready, failed to send next command.");
                }

                //update the solution trajectory, if replanning was done
                if (!is_replanning_.load())
                {
                    if (sent_joint_trajectory_index <= replan_start_index_.load())
                    {
                        auto replanned_solution_trajectory = auto_crane_manipulation_->get_solution_trajectory();
                        if (!replanned_solution_trajectory.points.empty())
                        {
                            solution_trajectory_ = replanned_solution_trajectory;
                            solution_trajectory_point_size = solution_trajectory_.points.size();
                            sent_joint_trajectory_index_.store(0);
                            predicted_collision_index_.store(0);
                            is_predicted_collision_.store(false);
                            continue;
                        }
                        else
                        {
                            qDebug("send trajectory thread: [sent_joint_trajectory_index <= slow_down_end_index_], the replanned trajectroy is empty, it seems that replan failed.");
                            break;
                        }
                    }
                    else
                    {
                        qDebug("send trajectory thread: the sent_joint_trajectory_index_ is greater than replan_start_index_.");
                        //TODO slow down the trajectory.
                        auto replanned_solution_trajectory = auto_crane_manipulation_->get_solution_trajectory();
                        if (!replanned_solution_trajectory.points.empty())
                        {
                            solution_trajectory_ = replanned_solution_trajectory;
                            solution_trajectory_point_size = solution_trajectory_.points.size();
                            sent_joint_trajectory_index_.store(0);
                            predicted_collision_index_.store(0);
                            is_predicted_collision_.store(false);
                            continue;
                        }
                        else
                        {
                            qDebug("send trajectory thread: [sent_joint_trajectory_index > replan_start_index_], the replanned trajectroy is empty, it seems that replan failed.");
                            break;
                        }
                    }
                }
                else
                {
                    qDebug("send trajectory thread: slow down end trajectory point was sent, so waiting the replan to be finished.");
                }
            }
            else
            {
                qDebug("send trajectory thread: sent_joint_trajectory_index is greater than slow_down_end_index_, so send the Estop to the controller.");
                //TODO: send the emergence stop command to the controller

                qDebug("send trajectory thread: E_STOP, current_to_predicted_collision: %d", current_to_predicted_collision);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                if (!is_replanning_.load())
                {
                    solution_trajectory_ = auto_crane_manipulation_->get_solution_trajectory();
                    solution_trajectory_point_size = solution_trajectory_.points.size();
                    sent_joint_trajectory_index_.store(0);
                    predicted_collision_index_.store(0);
                    is_predicted_collision_.store(false);
                }
            }
        }
        m_pathplan_mutex.unlock();
    }

    if(avoidstopstart == true)//ytadd for jiting slow
    {
        std::cerr<<"statrt send stop point ////////////////////////////////////////////////// "<<std::endl;
        m_clsClientUsbToCan->SendCanData("01D1", "0005000000000000");//kaiqishineng
        std::cerr<<point_now.velocities[0]<<std::endl;
        std::cerr<<point_now.velocities[1]<<std::endl;
        std::cerr<<point_now.velocities[2]<<std::endl;
        int slow_stop_time = 2;
        ompl::app::joint_trajectory_point_t point;
        point.velocities.clear();
        point.positions.clear();
        double stop_r = (point_now.velocities[0]/(slow_stop_time*10));
        double stop_c = (point_now.velocities[1]/(slow_stop_time*10));
        double stop_h = (point_now.velocities[2]/(slow_stop_time*10));

        for(int i = 0;i < slow_stop_time*10;i++)
        {
            point.velocities.push_back(point_now.velocities[0] - stop_r*i);
            point.velocities.push_back(point_now.velocities[1] - stop_c*i);
            point.velocities.push_back(point_now.velocities[2] - stop_h*i);
            point.positions.push_back(point_now.positions[0] + (point.velocities[0]*i/10));
            point.positions.push_back(point_now.positions[1] + (point.velocities[1]*i/10));
            point.positions.push_back(point_now.positions[2] + (point.velocities[2]*i/10));
            m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);
            m_clsClientUsbToCan->SendCanData("01D1", "0005000000000000");//kaiqishineng
            sendJointTrajectoryPoint(point);
            std::cerr<<"send stop joint "<<i<<std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            point.velocities.clear();
            point.positions.clear();
        }
        std::cerr<<"statrt send stop point ////////////////////////////////////////////////// "<<std::endl;
        avoidstopstart = false;
        sleep(2);
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0);     //close path tracker flag
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 0);     //lifting mode flag
        sendCommandTaskStatusTo(commandtask_.id ,2,"success");
    }

    is_running_predict_collision_.store(false);
    joint_trajectory_command_file_saver_.close();
    //路径跟踪到位防止急停
    sleep(5);
    std::cerr << "hook height is " << crane_parameter_->crane_hook_to_ground_height_ << std::endl;
    //send disable tracking trajectory flag
    m_clsClientUsbToCan->SendCanData("01D1", "0006000000000000");//

    if (realtime_collision_check_thread_.joinable())
    {
        realtime_collision_check_thread_.join();
    }

    is_stop_send_trajectory_.store(false);
    is_replanning_.store(false);
    is_predicted_collision_.store(false);

    //  emit showMB();
    send_joint_trajectory_thread_.detach();

    if(pathend_but_no_jingzhundingwei == false && commandtask_.status == true)
    {
        std::cerr<<"yt1here"<<std::endl;
        if(((fabs(crane_parameter_->crane_rotation_Angle_ - current_goal.rotate) > 0.5 && fabs(crane_parameter_->crane_rotation_Angle_ - current_goal.rotate) < 30)
            || fabs(crane_parameter_->crane_luff_Angle_ - current_goal.amplitude) > 0.2) && avoidcommandtask_.status == true) //非避让任务      //任务编号为0：避让不执行精准定位
        {
            is_send_control_command_package_ = true;
            send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK, this);
        }
        else if(fabs(crane_parameter_->crane_rotation_Angle_ - current_goal.rotate) >= 30 && avoidcommandtask_.status == true)
        {
            std::cerr<<"yt2here"<<std::endl;
            m_set_new_pathplan_start_position = true;
            m_xubaike_pathplan = true;
        }
        else
        {
            std::cerr<<"yt3here"<<std::endl;
            if(avoidcommandtask_.status == false)
            {
                if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == true)
                {
                    //////////////////////播报
                    is_start_play_voice_thread_ = true;
                    play_voice_flag = 9;
                    //////////////////////播报
                    std::cerr<<"yt4here"<<std::endl;
                    sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
                    resetavoidcommandtask_();//默认已完成，清空
                    std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
                }
                else if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == false)
                {
                    avoidcommandtask_.status1 = true;
                    on_path_plan_triggered();
                    std::cerr<<"avoidcommandtask_ start the number 2 avoid postrue!!!!!"<<std::endl;
                }
                else if(avoidcommandtask_.status1 == true && avoidcommandtask_.status2 == false)
                {
                    //////////////////////播报
                    is_start_play_voice_thread_ = true;
                    play_voice_flag = 9;
                    //////////////////////播报
                    std::cerr<<"yt4here"<<std::endl;
                    sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
                    resetavoidcommandtask_();//默认已完成，清空
                    std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
                }
                else
                {
                    std::cerr<<"avoidcommandtask_ status error!!!!!"<<std::endl;
                }
                //qDebug()<<"-----------------pathend_but_no_jingzhundingwei  --------------------------";
                //2021.11.18 add by xbk 轨迹跟踪结束后，发送到达目标位置完成 end
            }
            else
            {
                if(dispatchtask_.statusb == false)
                {
                    dispatchtask_.statusb = true;
                    std::cerr<<"the dispatchtask_ building task is over,the start building tagget task"<<std::endl;
                    start_path_plan_Thread();
                }
                else if(dispatchtask_.statusb == true && dispatchtask_.statust == false)
                {
                    dispatchtask_.statust = true;
                    std::cerr<<"the dispatchtask_ building targrt task is over,start the  safe  task"<<std::endl;
                    start_path_plan_Thread();
                }
                else if(dispatchtask_.statusb == true && dispatchtask_.statust == true && dispatchtask_.statusb == true && dispatchtask_.statuss == false)
                {
                    dispatchtask_.statuss = true;
                    std::cerr<<"the safe target is over"<<std::endl;
                    std::cerr<<"the dispatch task is over"<<std::endl;
                    sendTaskStatusTo(dispatchtask_.id,-1,2);
                    resetdispatchtask_();
                    //start_path_plan_Thread();

                }
            }
        }
    }
    else
    {
        if(avoidcommandtask_.status == false)
        {
            if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == true)
            {
                std::cerr<<"yt4here"<<std::endl;
                sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
                resetavoidcommandtask_();//默认已完成，清空
                std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
            }
            else if(avoidcommandtask_.status1 == false && avoidcommandtask_.status2 == false)
            {
                avoidcommandtask_.status1 = true;
                on_path_plan_triggered();
                std::cerr<<"avoidcommandtask_ start the number 2 avoid postrue!!!!!"<<std::endl;
            }
            else if(avoidcommandtask_.status1 == true && avoidcommandtask_.status2 == false)
            {
                std::cerr<<"yt4here"<<std::endl;
                sendAvoidTaskStatusTo(avoidcommandtask_.id,2);//执行完成
                resetavoidcommandtask_();//默认已完成，清空
                std::cerr<<"avoidcommandtask_ complete!!!!!"<<std::endl;
            }
            else
            {
                std::cerr<<"avoidcommandtask_ status error!!!!!"<<std::endl;
            }
        }
    }

    //if(!server->m_task_num_flag)///yt  question ??????
    //    if(server->m_task_num_flag)//lnfa 2021.12.1
    //    {
    //        server->send_task_reply(3);         //发送定位完成标识状态
    //        qDebug()<<"-----------------m_task_num_flag已完成！send_task_reply(3)----";
    //    }
    //std::cerr<<"server->m_task_num_flag"<<server->m_task_num_flag<<std::endl;
    pathend_but_no_jingzhundingwei = false;
    qDebug("The send trajectory thread exited");
}

bool MainWindow::sendJointTrajectoryPoint(ompl::app::joint_trajectory_point_t point)
{
    auto check_velocity_reverse = [](const double& v1, const double& last_v2)->bool
    {
        if (fabs(last_v2) > 0.00001 && v1 * last_v2 < 0.00000001)
        {
            return true;
        }
        return false;
    };

    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);//lift mode ytadd for;because 20211126 路径跟踪不动，谭松涛说次标志位为0,因此加此语句一直开启此模式。
    qDebug("sending a joint trajectory point.");

    QString high_bit;
    QString low_bit;
    QString send_string_3d0;
    if (fabs(point.velocities[0]) < 0.00001 || check_velocity_reverse(point.velocities[0], last_sent_point_.velocities[0]))
        point.velocities[0] = 0.0;
    m_clsClientUsbToCan->SplitCanDataByte(std::floor(point.velocities[0] * 100 * 180 / M_PI), high_bit, low_bit);
    std::cerr << "point.velocities[0]: " << point.velocities[0] << std::endl;
    send_string_3d0 = low_bit + high_bit;
    high_bit.clear();
    low_bit.clear();
    double point_rotate_position = 0.0;
    if(point.positions[0] < 0)
    {
        point_rotate_position = point.positions[0] + 2 * M_PI;
    }
    else
    {
        point_rotate_position = point.positions[0];
    }
    //m_clsClientUsbToCan->SplitCanDataByte(std::floor(point.positions[0] * 100 * 180 / M_PI), high_bit, low_bit);
    m_clsClientUsbToCan->SplitCanDataByte(std::floor(point_rotate_position * 100 * 180 / M_PI), high_bit, low_bit);
    send_string_3d0 += low_bit + high_bit;
    std::cerr << "point.positions[0] " << point_rotate_position * 180 / M_PI<< std::endl;
    high_bit.clear();
    low_bit.clear();
    if (fabs(point.velocities[1]) < 0.00001 || check_velocity_reverse(point.velocities[1], last_sent_point_.velocities[1]))
        point.velocities[1] = 0.0;
    m_clsClientUsbToCan->SplitCanDataByte(std::floor(point.velocities[1] * 100 * 180 / M_PI), high_bit, low_bit);
    std::cerr << "point.velocities[1]: " << point.velocities[1] << std::endl;
    send_string_3d0 += low_bit + high_bit;
    high_bit.clear();
    low_bit.clear();
    m_clsClientUsbToCan->SplitCanDataByte(std::floor(point.positions[1] * 100 * 180 / M_PI), high_bit, low_bit);
    send_string_3d0 += low_bit + high_bit;

    high_bit.clear();
    low_bit.clear();
    QString send_string_4d0;
    if (fabs(point.velocities[2]) < 0.00001 || check_velocity_reverse(point.velocities[2], last_sent_point_.velocities[2]))
        point.velocities[2] = 0.0;
    m_clsClientUsbToCan->SplitCanDataByte(std::floor(point.velocities[2] * 100), high_bit, low_bit);
    std::cerr << "point.velocities[2]: " << point.velocities[2] << std::endl;
    send_string_4d0 = low_bit + high_bit;
    high_bit.clear();
    low_bit.clear();
    m_clsClientUsbToCan->SplitCanDataByte(std::floor(point.positions[2] * 100), high_bit, low_bit);
    std::cerr << "point.positions[2]: " << point.positions[2] << std::endl;
    send_string_4d0 += low_bit + high_bit + "00000000";

    last_sent_point_ = point;

    m_clsClientUsbToCan->SendCanData("03D0", send_string_3d0);
    m_clsClientUsbToCan->SendCanData("04D0", send_string_4d0);

    //save the joint trajectory velocity command into the file
    current_joint_state_.positions[0] = crane_parameter_->crane_rotation_Angle_*M_PI/180;
    current_joint_state_.positions[1] = crane_parameter_->crane_luff_Angle_*M_PI/180;
    current_joint_state_.positions[2] = crane_parameter_->crane_arm_to_ground_height_ - crane_parameter_->crane_hook_to_ground_height_ - 1.2;
    current_joint_state_.velocities[0] = (current_joint_state_.positions[0] - previous_joint_state_.positions[0]) / 0.1;
    current_joint_state_.velocities[1] = (current_joint_state_.positions[1] - previous_joint_state_.positions[1]) / 0.1;
    current_joint_state_.velocities[2] = (current_joint_state_.positions[2] - previous_joint_state_.positions[2]) / 0.1;
    joint_trajectory_command_file_saver_ << QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString() << " "
                                         << current_joint_state_.positions[0] << " " << current_joint_state_.positions[1] << " " << current_joint_state_.positions[2] << " "
                                         << current_joint_state_.velocities[0] << " " << current_joint_state_.velocities[1] << " " << current_joint_state_.velocities[2] << " "
                                         << point.positions[0] << " " << point.positions[1] << " " << point.positions[2] << " "
                                         << point.velocities[0] << " " << point.velocities[1] << " " << point.velocities[2] << std::endl;
    previous_joint_state_ = current_joint_state_;
}

//防碰撞检测
void MainWindow::predictCollision()
{
    std::size_t predict_start_index;
    std::size_t predict_end_index;
    if (!auto_crane_manipulation_->get_collision_check_start_index(predict_start_index))
    {
        qDebug("Failed to find the predict collision start index.");
        return;
    }
    if (!auto_crane_manipulation_->get_collision_check_end_index(predict_end_index))
    {
        qDebug("Failed to find the predict collision end index.");
        return;
    }

    auto current_index = sent_joint_trajectory_index_.load();
    while (current_index <= predict_start_index && is_running_predict_collision_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_index = sent_joint_trajectory_index_.load();
    }
    qDebug("++++++++++++++Start predict collision thread++++++++++++++++.");

    std::vector<ObstacleInforStr> collision_objects;
    while (is_running_predict_collision_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        //remove the previous added collision objects detected from the scene monitor
        //auto_crane_manipulation_->remove_collision_object("test_replan_cube");
        for (unsigned int j = 0; j < last_added_collision_objects_.size(); j++)
        {
            auto_crane_manipulation_->remove_collision_object("temp_ob_" + std::to_string(j));
        }

        //added the collision objects detected from the scene monitor
        //od.GetObstaclesValue(collision_objects);
        Eigen::Isometry3f main_link_end_to_world_tf;
        if (auto_crane_manipulation_->get_main_link_end_to_world_tf(crane_parameter_->crane_rotation_Angle_*M_PI/180,
                                                                    crane_parameter_->crane_luff_Angle_*M_PI/180,
                                                                    crane_parameter_->crane_arm_to_ground_height_,
                                                                    main_link_end_to_world_tf))
        {
            //transform the collision objects to world frame
            for (unsigned int i = 0; i < collision_objects.size(); i++)
            {
                Eigen::Isometry3f object_pose = Eigen::Isometry3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
                                                                  *Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));
                object_pose.translation() = Eigen::Vector3f(collision_objects[i].centerPt.x,
                                                            collision_objects[i].centerPt.y,
                                                            collision_objects[i].centerPt.z);
                object_pose = main_link_end_to_world_tf * object_pose;
                collision_objects[i].centerPt.x = object_pose.translation()[0];
                collision_objects[i].centerPt.y = object_pose.translation()[1];
                collision_objects[i].centerPt.z = object_pose.translation()[2];

                //计算悬空的障碍物xpp
                if((collision_objects[i].centerPt.z - collision_objects[i].box3d.depth_z/2) > 1)
                {
                    collision_objects[i].box3d.depth_z = collision_objects[i].box3d.depth_z + (collision_objects[i].centerPt.z - collision_objects[i].box3d.depth_z/2);
                    collision_objects[i].centerPt.z = collision_objects[i].box3d.depth_z/2;
                }
            }

            //filter collision objects
            std::vector<ObstacleInforStr> filtered_collision_objects;
            filter_collision_objects(collision_objects, filtered_collision_objects);

            //add the filtered collision objects
            //test_pathplan();//yt测试路径规划用
            std::vector<std::string> allowed_collision_names;
            for (unsigned int j = 0; j < filtered_collision_objects.size(); j++)
            {
                std::string name = "temp_ob_" + std::to_string(j);
                auto_crane_manipulation_->add_cube_collision_object(name,
                                                                    filtered_collision_objects[j].box3d.width_x,
                                                                    filtered_collision_objects[j].box3d.height_y,
                                                                    filtered_collision_objects[j].box3d.depth_z,
                                                                    filtered_collision_objects[j].centerPt.x,
                                                                    filtered_collision_objects[j].centerPt.y,
                                                                    filtered_collision_objects[j].centerPt.z, 0, 0, 0);
                allowed_collision_names.push_back(name);
                //allowed_collision_names.push_back("temp_ob2+");
            }

            //update the last_added_collision_objects_
            last_added_collision_objects_ = filtered_collision_objects;
            auto_crane_manipulation_->set_allowed_collision(allowed_collision_names);
        }
        //auto_crane_manipulation_->add_cube_collision_object("test_replan_cube", 3, 3, 10, 6, 13, 5, 0, 0, 0);

        while (predicted_collision_index_.load() && is_running_predict_collision_.load())
        {
            qDebug("predict collision thread: waiting the last replan be updated.");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!is_running_predict_collision_.load())
            break;

        size_t current_sent_point_index = sent_joint_trajectory_index_.load();
        std::cerr<<"current_sent_point_index is:   "<<current_sent_point_index<<std::endl;
        //if current sent point index is not within the start end and end index, no collision check needed.
        if (current_sent_point_index < predict_start_index || current_sent_point_index > predict_end_index)
            continue;
        size_t collision_check_range = 0;
        collision_check_range = solution_trajectory_.points.size()/10;

        int replan_message = 0;
        for (size_t i = 1; i <= collision_check_range; i++)
        {
            size_t next_point_index = current_sent_point_index + i;
            if (next_point_index < solution_trajectory_.points.size())
            {
                if (!auto_crane_manipulation_->check_collision(solution_trajectory_.points[next_point_index].positions[0],
                                                               solution_trajectory_.points[next_point_index].positions[1],
                                                               solution_trajectory_.points[next_point_index].positions[2]))
                {
                    predicted_collision_index_.store(next_point_index);
                    is_replanning_.store(true);
                    if (i <= collision_check_range - 3)
                    {
                        qDebug("predict collision thread: predicted collision in below E_STOP, the index: %zu", next_point_index);
                        ompl::app::joint_trajectory_t from_trajectory;
                        replan_start_index_.store(next_point_index - 1);
                        is_predicted_collision_.store(true);
                        from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - 1]);
                        from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - 2]);
                        if (!auto_crane_manipulation_->replan(ompl::app::ReplanCase::ESTOP, from_trajectory, replan_message))
                        {
                            qDebug("predict collision thread: failed to replan the path in estop case");
                        }
                        else
                        {
                            auto_crane_manipulation_->get_collision_check_end_index(predict_end_index);
                            //glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
                            qDebug("predict collision thread: replan the path success in estop case");
                        }
                    }
                    else if (i <= collision_check_range - 2)
                    {
                        //selected a point between the current_sent_point_index and collision index
                        qDebug("predict collision thread: predicted collision in below SLOW_DOWN, the index: %zu", next_point_index);

                        std::size_t stop_index;
                        update_solution_trajectory_mutex_.lock();
                        auto_crane_manipulation_->plan_slow_down(current_sent_point_index,
                                                                 next_point_index,
                                                                 stop_index,
                                                                 solution_trajectory_);
                        update_solution_trajectory_mutex_.unlock();
                        slow_down_end_index_.store(stop_index);
                        is_predicted_collision_.store(true);

                        ompl::app::joint_trajectory_t from_trajectory;
                        if (stop_index < next_point_index)
                        {
                            qDebug("predict collision thread: the stop index is smaller than the prediceted collision index.");
                            replan_start_index_.store(stop_index);
                            from_trajectory.points.push_back(solution_trajectory_.points[stop_index]);
                            from_trajectory.points.push_back(solution_trajectory_.points[stop_index - 1]);
                            if (!auto_crane_manipulation_->replan(ompl::app::ReplanCase::ESTOP, from_trajectory, replan_message))
                            {
                                qDebug("predict collision thread: failed to replan the path in the slow down case");
                            }
                            else
                            {
                                auto_crane_manipulation_->get_collision_check_end_index(predict_end_index);
                                //glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
                                qDebug("predict collision thread: replan the path success in the slow down case");
                            }
                        }
                        else
                        {
                            replan_start_index_.store(next_point_index - zl_common::app::zl::ReactionThreshold::E_STOP - 3);
                            from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - zl_common::app::zl::ReactionThreshold::E_STOP - 3]);
                            from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - zl_common::app::zl::ReactionThreshold::E_STOP]);
                            if (!auto_crane_manipulation_->replan(ompl::app::ReplanCase::SLOW_DOWN, from_trajectory, replan_message))
                            {
                                qDebug("predict collision thread: failed to replan the path in the slow down case");
                            }
                            else
                            {
                                //glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
                                qDebug("predict collision thread: replan the path success in the slow down case");
                            }
                        }
                    }
                    else if (i < collision_check_range - 1)
                    {
                        qDebug("predict collision thread: predicted collision in below NORMAL, the index: %zu", next_point_index);
                        std::size_t stop_index;
                        update_solution_trajectory_mutex_.lock();
                        auto_crane_manipulation_->plan_slow_down(current_sent_point_index,
                                                                 next_point_index,
                                                                 stop_index,
                                                                 solution_trajectory_);
                        update_solution_trajectory_mutex_.unlock();
                        slow_down_end_index_.store(stop_index);
                        is_predicted_collision_.store(true);

                        ompl::app::joint_trajectory_t from_trajectory;
                        if (stop_index < next_point_index)
                        {
                            qDebug("predict collision thread: the stop index is smaller than the prediceted collision index.");
                            replan_start_index_.store(stop_index);
                            from_trajectory.points.push_back(solution_trajectory_.points[stop_index]);
                            from_trajectory.points.push_back(solution_trajectory_.points[stop_index - 1]);
                            if (!auto_crane_manipulation_->replan(ompl::app::ReplanCase::ESTOP, from_trajectory, replan_message))
                            {
                                qDebug("predict collision thread: failed to replan the path");
                            }
                            else
                            {
                                auto_crane_manipulation_->get_collision_check_end_index(predict_end_index);
                                //glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
                                qDebug("predict collision thread: replan the path success");
                            }
                        }
                        else
                        {
                            replan_start_index_.store(next_point_index - zl_common::app::zl::ReactionThreshold::REPLAN);
                            from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - zl_common::app::zl::ReactionThreshold::REPLAN]);
                            from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - zl_common::app::zl::ReactionThreshold::REPLAN + 2]);
                            if (!auto_crane_manipulation_->replan(ompl::app::ReplanCase::NORMAL, from_trajectory, replan_message))
                            {
                                qDebug("predict collision thread: failed to replan the path");
                            }
                            else
                            {
                                auto_crane_manipulation_->get_collision_check_end_index(predict_end_index);
                                //glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
                                qDebug("predict collision thread: replan the path success");
                            }
                        }
                    }
                    else
                    {
                        qDebug("predict collision thread: predicted collision in NORMAL, the index: %zu", next_point_index);
                        slow_down_end_index_.store(solution_trajectory_.points.size());
                        is_predicted_collision_.store(true);
                        ompl::app::joint_trajectory_t from_trajectory;
                        replan_start_index_.store(next_point_index - 18);
                        from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - 12]);
                        from_trajectory.points.push_back(solution_trajectory_.points[next_point_index - 8]);
                        if (!auto_crane_manipulation_->replan(ompl::app::ReplanCase::NORMAL, from_trajectory, replan_message))
                        {
                            qDebug("predict collision thread: failed to replan the path in normal case, try to slow down.");
                            std::size_t stop_index;
                            update_solution_trajectory_mutex_.lock();
                            auto_crane_manipulation_->plan_slow_down(sent_joint_trajectory_index_.load(),
                                                                     next_point_index,
                                                                     stop_index,
                                                                     solution_trajectory_);
                            update_solution_trajectory_mutex_.unlock();
                            slow_down_end_index_.store(stop_index);
                        }
                        else
                        {
                            auto_crane_manipulation_->get_collision_check_end_index(predict_end_index);
                            //glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
                            qDebug("predict collision thread: goooooooooooooooood replan the path success");
                        }
                    }
                    is_replanning_.store(false);
                    break;
                }
            }
        }
    }
    //remove the previous added collision objects detected from the scene monitor
    //auto_crane_manipulation_->remove_collision_object("test_replan_cube");
    for (unsigned int j = 0; j < last_added_collision_objects_.size(); j++)
    {
        auto_crane_manipulation_->remove_collision_object("temp_ob_" + std::to_string(j));
    }

    //stop the realtime objects detection thread
    //  g_obstacle_detect_enable = false;
    realtime_collision_check_thread_.detach();
    qDebug("+++++++++++++++The predict collision thread exited+++++++++++++++++");
}


void MainWindow::on_Scene_reconstruction_visualization_triggered()
{
    if(start_scence_visualization_thread_.joinable())
    {
        start_scence_visualization_thread_.join();
    }
    is_start_scence_visualization_thread_ = true;
    start_scence_visualization_thread_ = std::thread(&MainWindow::Function_start_scence_visualization_Thread, this);
}


void MainWindow::Function_start_scence_visualization_Thread()
{
//     
//     cv::Mat bigrgbimage;
//     while(is_start_scence_visualization_thread_)
//     {
//         int ret =0;
//         QString rotateAngleStr;
//         int hundred_num,one_num,percent_num,roate_angle;
//         m_param_mutex.lockForRead();
//         ret =cv::imwrite("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + std::to_string(0) + ".jpg",m_gframe_);
//         ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + std::to_string(0) + ".pcd",*m_laser_cloud_frame_);
//         m_param_mutex.unlock();
//         m_clsClientToYunTai->sendByteDataToPTZ("5A 04 81 00 00 20 00 00 AA");
//         sleep(3);
//         m_param_mutex.lockForRead();
//         ret =cv::imwrite("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + std::to_string(1) + ".jpg",m_gframe_);
//         ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + std::to_string(1) + ".pcd",*m_laser_cloud_frame_);
//         m_param_mutex.unlock();
//         for(int i = 1;i <= 8;i++)
//         {
//             roate_angle = i * 40;
//             hundred_num = int(roate_angle / 100);
//             one_num = int(roate_angle - (100 * hundred_num));
//             percent_num = int((roate_angle - (100 * hundred_num) - one_num)*100);
//             rotateAngleStr = "5A 04 82 00 " + QString("%1").arg(hundred_num, 2, 10, QChar('0')) + " " +
//                     QString("%1").arg(one_num, 2, 10, QChar('0')) + " " + QString("%1").arg(percent_num, 2, 10, QChar('0'))
//                     + " 00 AA";
//             std::cerr<<rotateAngleStr.toStdString()<<std::endl;
//             m_clsClientToYunTai->sendByteDataToPTZ(rotateAngleStr);
//             sleep(3);
//             m_param_mutex.lockForRead();
//             ret =cv::imwrite("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + std::to_string(i+1) + ".jpg",m_gframe_);
//             ret = pcl::io::savePCDFileASCII("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + std::to_string(i+1) + ".pcd",*m_laser_cloud_frame_);
//             m_param_mutex.unlock();
//         }
//         m_clsClientToYunTai->recovery_camera_to_init_position(hcfg_->ptz_str_pitch_initial_angle, hcfg_->ptz_str_rotate_initial_angle);
//         //mosaic
//         string intrinsic_path = "/home/zoomlion/ytrest/parameters/intrinsic.txt";
//         string extrinsic_path = "/home/zoomlion/ytrest/parameters/extrinsic.txt";
//         std::cerr<<"yt1 success"<<std::endl;
//         me->set_factor(100);
//         std::cerr<<"yt2 success"<<std::endl;
//         std::vector<float> intrinsic;
//         me->getIntrinsic(intrinsic_path, intrinsic);
//         std::cerr<<"getIntrinsic success"<<std::endl;
//         std::vector<float> extrinsic;
//         me->getExtrinsic(extrinsic_path, extrinsic);
//         std::cerr<<"extrinsic success"<<std::endl;
//         //std::cerr<<intrinsic[0]<<std::endl;
//         vector<cv::Mat> rgb_image;
//         cv::Mat base_rgb_image = imread("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/0.jpg");
//         vector<cv::Mat> depth_image;
//         string rgb_name;
//         cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
//         cameraMatrix.at<double>(0, 0) = intrinsic[0];
//         cameraMatrix.at<double>(0, 2) = intrinsic[2];
//         cameraMatrix.at<double>(1, 1) = intrinsic[4];
//         cameraMatrix.at<double>(1, 2) = intrinsic[5];
//         //double distortion[5]={0.6427699020727743 ,-1.035415272524145 , 0.01865586672918605 , -0.0324618917359827 , 1.26936121570552};
//         double distortion[5]={0,0,0,0,0};
//         // set radial distortion and tangential distortion
//         cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
//         distCoeffs.at<double>(0, 0) = distortion[0];
//         distCoeffs.at<double>(1, 0) = distortion[1];
//         distCoeffs.at<double>(2, 0) = distortion[2];
//         distCoeffs.at<double>(3, 0) = distortion[3];
//         distCoeffs.at<double>(4, 0) = distortion[4];
//         for(int i = 1;i < 10;i++)
//         {
//             rgb_name = "/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + to_string(i) + ".jpg";
//             cv::Mat rgb_pic = imread(rgb_name);
//             //im.Distortion_correction(rgb_pic,2.0,"/home/zoomlion/ytrest/parameters/intrinsic.txt");
//             rgb_image.push_back(rgb_pic);

//         }
//         string pcd_name;
//         cv::Mat base_depth_img;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud0(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）
//         for(int i = 0;i < 10;i++)
//         {
//             pcd_name = "/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/" + to_string(i) + ".pcd";
//             Mat depth_img(1536,2048,CV_16SC3,cv::Scalar::all(0.0));
//             if( i != 0)
//             {
//                 if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name, *pointCloud0) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
//                 {
//                     PCL_ERROR("Couldn't read file.pcd \n"); //文件不存在时，返回错误，终止程序。
//                 }
//                 me->Transorm_pcd_to_depth(depth_img,intrinsic,extrinsic,pointCloud0,lc::factor,i);
//                 depth_image.push_back(depth_img);
//             }
//             else
//             {
//                 if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_name, *pointCloud0) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
//                 {
//                     PCL_ERROR("Couldn't read file.pcd \n"); //文件不存在时，返回错误，终止程序。
//                 }
//                 me->Transorm_pcd_to_depth(depth_img,intrinsic,extrinsic,pointCloud0,lc::factor,i);
//                 base_depth_img = depth_img;
//             }
//         }
//         bigrgbimage = im.mosaic_RGB_image(rgb_image,base_rgb_image,2);
//         bigrgbimage = im.mosaic_rgb_iamge_repair(bigrgbimage,8);
//         Mat bigdepthimage = im.mosaic_depth_image(depth_image,base_depth_img);
//         cv::imwrite("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/depth.png",bigdepthimage);
//         bigdepthimage = im.mosaic_depth_iamge_repair(bigdepthimage,8);
//         me->save_to_xml(bigdepthimage,"/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/depth.xml");
//         cv::imwrite("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/rgb.jpg",bigrgbimage);
//         //waitKey(0);
//         //for display
//         pcl::PointCloud<pcl::PointXYZ>::Ptr All_view_point(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::PointXYZ point;
//         for(int i = 0; i < bigdepthimage.rows; i ++)//v
//         {
//             for(int j = 0; j< bigdepthimage.cols; j ++)//u
//             {
//                 if(bigdepthimage.at<cv::Vec3s>(i,j)[0] == 0)
//                 {
//                     continue;
//                 }
//                 point.x = bigdepthimage.at<cv::Vec3s>(i,j)[0];
//                 point.y = bigdepthimage.at<cv::Vec3s>(i,j)[1];
//                 point.z = bigdepthimage.at<cv::Vec3s>(i,j)[2];
//                 All_view_point->points.push_back(point);
//             }
//         }
//         pcl::io::savePCDFileBinary("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/depth.pcd",*All_view_point);
//         is_start_scence_visualization_thread_ = false;
//     }
//     cv::resize(bigrgbimage,bigrgbimage,cv::Size(2048,1536));
//     QImage img_frame = QImage((const unsigned char*)(bigrgbimage.data), bigrgbimage.cols, bigrgbimage.rows, QImage::Format_RGB888);

//     m_label_paint_mutex.lock();
//     label_img_->setPixmap(QPixmap::fromImage(img_frame.scaled(label_vedio_->size())));
//     m_label_paint_mutex.unlock();
//     ui.tabWidget_display->setCurrentIndex(1);
    
}


void MainWindow::on_Accurate_positioning_triggered()
{

    //  if(QMessageBox::Yes != QMessageBox::information(nullptr, "提示", "目标是否已经进入视场内，可进行精准定位？"),
    //     QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes)
    {
        std::cerr<<"11111111111111111"<<std::endl;
        start_siamRPN_accurate_position();
    }
    //  else
    //  {
    //    return;
    //  }
}

void MainWindow::Function_Start_PathPlan_Thread()
{
    while(m_start_pathPlan_thread_flag)
    {
        //如果正执行精准定位或者路径规划，则等待执行完成后再进行路径规划动作
        if(is_send_control_command_package_ == true || on_path_planing == true) //|| is_stop_send_trajectory_.load() == true)
        {
            continue;
        }
        on_path_plan_triggered();
        m_start_pathPlan_thread_flag = false;
    }
}

void MainWindow::on_path_plan_triggered()
{
    auto_crane_manipulation_->set_auto_crane_link_length(crane_parameter_->crane_mainarm_length_);

    pathend_but_no_jingzhundingwei = false;
    on_path_planing = true;

    float itemweight = 0;   //杨涛定义重量信息20220419 jiqunxietong
    read_lift_weight_form_data();               //read liftweight form
    qDebug("g_dMainArmLen: %f", crane_parameter_->crane_mainarm_length_);

    //check if need to stop vision features detection
    if (is_start_object_position_compute_ == true)
    {
        is_start_object_position_compute_ = false;
    }

    //  //当目标处于回转角度为零界点（类似10度、350度间）的地方，防止路径规划绕大圈 start
    ompl::app::joint_state_t start_state;
    ompl::app::joint_state_t goal_state;

    if(object_init_world_x > 0 && object_init_world_y > 0)
    {
        if(crane_parameter_->crane_rotation_Angle_ > 270)
        {
            crane_parameter_->crane_rotation_Angle_ = crane_parameter_->crane_rotation_Angle_ - 360;
        }
    }
    //当目标处于回转角度为零界点（类似10度、350度间）的地方，防止路径规划绕大圈 end
    start_state.positions.push_back(crane_parameter_->crane_rotation_Angle_*M_PI/180);
    start_state.positions.push_back(crane_parameter_->crane_luff_Angle_*M_PI/180);

    double min_distance_hook_ground = 2.5;      //
    if (crane_parameter_->crane_hook_to_ground_height_ < min_distance_hook_ground)
    {
        start_state.positions.push_back(crane_parameter_->crane_arm_to_ground_height_ - min_distance_hook_ground);
    }
    else
    {
        start_state.positions.push_back(crane_parameter_->crane_arm_to_ground_height_ - crane_parameter_->crane_hook_to_ground_height_ );
    }

    //set goal postrue;
    //避让任务规划优先，其次为常规任务
    //常规任务中，执行顺序为被吊装物位-被吊物吊目标位置-吊装任务完成后安全位置（不一定有）；；对于无人驾驶目标点不作处理
    ompl::app::position_t goal_position;
    if(avoidcommandtask_.status == false)
    {
        //弧度
        if(avoidcommandtask_.status1 == false)
        {
            //play_voice(8);
            goal_state.positions.push_back(avoidcommandtask_.targetpostrue1.rotate);
            goal_state.positions.push_back(avoidcommandtask_.targetpostrue1.amplitude);
            goal_state.positions.push_back(avoidcommandtask_.targetpostrue1.roller);
        }
        else if(avoidcommandtask_.status2 == false && avoidcommandtask_.status1 == true)
        {
            //play_voice(8);
            goal_state.positions.push_back(avoidcommandtask_.targetpostrue2.rotate);
            goal_state.positions.push_back(avoidcommandtask_.targetpostrue2.amplitude);
            goal_state.positions.push_back(avoidcommandtask_.targetpostrue2.roller);
        }
        else
        {
            std::cerr<<"avoidcommandtask_ target error !!!!!!!!!!!!!"<<std::endl;
            return;
        }
        std::cerr<<goal_state.positions[0]<<"  "<<goal_state.positions[1]<<"  "<<goal_state.positions[2]<<std::endl;
        double task_ee_tip_x, task_ee_tip_y, task_ee_tip_z;
        if (!auto_crane_manipulation_->get_ee_tip_position(goal_state.positions[0],
                                                           goal_state.positions[1],
                                                           goal_state.positions[2],
                                                           task_ee_tip_x, task_ee_tip_y, task_ee_tip_z))
        {
            std::cerr << "Failed to calculate the ee_tip position." << std::endl;
            return;
        }
        goal_position.x = task_ee_tip_x;
        goal_position.y = task_ee_tip_y;
        goal_position.z = crane_parameter_->crane_arm_to_ground_height_ - goal_state.positions[2];
    }
    if(avoidcommandtask_.status == true && dispatchtask_.completeall == false)
    {
        if(dispatchtask_.statusb == false)
        {
            //play_voice(1);
            goal_position.x = dispatchtask_.taskinfor.buildinglocation.point_x;
            goal_position.y = dispatchtask_.taskinfor.buildinglocation.point_y;
            goal_position.z = dispatchtask_.taskinfor.buildinglocation.point_z;
        }
        if(dispatchtask_.statust == false && dispatchtask_.statusb == true)
        {
            //play_voice(3);
            goal_position.x = dispatchtask_.taskinfor.buildingtargetlocation.point_x;
            goal_position.y = dispatchtask_.taskinfor.buildingtargetlocation.point_y;
            goal_position.z = dispatchtask_.taskinfor.buildingtargetlocation.point_z;
            itemweight = dispatchtask_.taskinfor.weight;
        }
        if(dispatchtask_.statust == true && dispatchtask_.statusb == true && dispatchtask_.statuss == false)
        {
            //play_voice(5);
            goal_position.x = dispatchtask_.safelocation.point_x;
            goal_position.y = dispatchtask_.safelocation.point_y;
            goal_position.z = dispatchtask_.safelocation.point_z;
        }
    }
    std::cerr << "goal_position.x_earth = " << goal_position.x << std::endl;
    std::cerr << "goal_position_earth.y = " << goal_position.y << std::endl;
    std::cerr << "goal_position_earth.z = " << goal_position.z << std::endl;
    current_goal_position.point_x = goal_position.x;
    current_goal_position.point_y = goal_position.y;
    current_goal_position.point_z = goal_position.z;
    vector<double> v_goal_position;
    rtk->process();
    v_goal_position = rtk->rtkTocar(goal_position.x,goal_position.y,goal_position.z,crane_parameter_->crane_rotation_Angle_);
    while(!v_goal_position[0] && !v_goal_position[1] && v_goal_position[2])
    {
        rtk->process();
        v_goal_position = rtk->rtkTocar(goal_position.x,goal_position.y,goal_position.z,crane_parameter_->crane_rotation_Angle_);
    }
    goal_position.x = v_goal_position[1];
    goal_position.y = -v_goal_position[0];
    goal_position.z = v_goal_position[2];
    std::cerr << "goal_position.x = " << goal_position.x << std::endl;
    std::cerr << "goal_position.y = " << goal_position.y << std::endl;
    std::cerr << "goal_position.z = " << goal_position.z << std::endl;
    if(goal_position.y >= 0 && goal_position.x > 0)
    {
        if(crane_parameter_->crane_rotation_Angle_ > 270)
        {
            crane_parameter_->crane_rotation_Angle_ = crane_parameter_->crane_rotation_Angle_ - 360;
        }
    }

    //yt add for roller start'20220419 jiqunxietong
    //  zl_common::app::structPointXYZ position;
    //  position.point_x = goal_position.x;
    //  position.point_y = goal_position.y;
    //  position.point_z = goal_position.z;
    //  current_goal = position_to_postrue_transform(position);

    //  current_goal.amplitude = acos(current_goal.amplitude/crane_parameter_->crane_mainarm_length_)*180/M_PI;
    //  std::cerr << "current_goal.rotate = " << current_goal.rotate << std::endl;
    //  std::cerr << "current_goal.amplitude = " << current_goal.amplitude << std::endl;
    //  std::cerr << "current_goal.roller = " << current_goal.roller << std::endl;
    //yt add for roller end 20220419 jiqunxietong


    //    ompl::app::position_t goal_position;
    ompl::app::item_info_t item;
    if(m_lift_work_mode_ == 0)  //空钩模式
    {
        item.length = 0;
        item.thickness = 0;
        item.rope_length = 0;
        item.weight = HOOK_WEIGHT;      //默认钩子+rope重量是1t
    }
    else      //吊载模式
    {
        item.length = m_object_len > m_object_width ? m_object_len : m_object_width;
        item.thickness = m_object_height + m_goods_height;
        item.rope_length = m_rope_height;
        item.weight = m_object_weight + HOOK_WEIGHT;//默认钩子+rope重量是1t
    }

    double move_up_offset = 1; //the default lift up offset is 1 meter.
    if (crane_parameter_->laser_hook_to_ground_height <= 2.5)
        move_up_offset = 0.1;
    else if(crane_parameter_->laser_hook_to_ground_height > 6)
        move_up_offset = 0.1;
    std::cerr<<"crane_parameter_->crane_hook_to_ground_height_ is : "<<crane_parameter_->crane_hook_to_ground_height_<<std::endl;

    double move_down_offset = 0.2;
    double min_safety_proximity = 0.5;

    std::cerr << " crane_parameter_->crane_rope_ratio_: " << crane_parameter_->crane_rope_ratio_ << std::endl;
    auto_crane_manipulation_->update_rope_ratio(crane_parameter_->crane_rope_ratio_);

    std::cerr << "The goal position: x " << goal_position.x << " y " << goal_position.y << " z " << goal_position.z << std::endl;

    int plan_message = 0;
    //  if(m_lift_work_mode_ == 1)  //空钩模式
    //  {
    //    goal_position.x = 5;
    //    goal_position.y = 5;
    //    goal_position.z = 8 -1.7;
    //  }
    //  else
    //  {
    //      goal_position.x = 8;
    //      goal_position.y = -1;
    //      goal_position.z = 5 - 1.7;
    //  }
    size_t joint_trajectory_size = auto_crane_manipulation_->plan(start_state,
                                                                  goal_position,
                                                                  item,
                                                                  move_up_offset,
                                                                  move_down_offset,
                                                                  min_safety_proximity,
                                                                  plan_message);

    //2021.11.18 机群协同控制端，路径规划失败或成功时发送可执行消息 start
    if(plan_message != 0)//failed
    {
        if(avoidcommandtask_.status == false)
        {
            sendAvoidTaskStatusTo(avoidcommandtask_.id,3);//规划失败
            resetavoidcommandtask_();//默认已完成，清空
        }
        if(avoidcommandtask_.status == true && dispatchtask_.completeall == false)
        {

            sendTaskStatusTo(dispatchtask_.id,-1,3);//规划失败
            resetdispatchtask_();//默认已完成，清空
        }
    }
    else//success
    {
        if(avoidcommandtask_.status == false)
        {
            sendAvoidTaskStatusTo(avoidcommandtask_.id,1);//规划成功
        }
        if(avoidcommandtask_.status == true && dispatchtask_.completeall == false)
        {
            sendTaskStatusTo(dispatchtask_.id,-1,1);//规划成功
        }
    }

    solution_trajectory_.points.clear();
    m_trajectory_point_size = joint_trajectory_size;
    std::cerr<<"规划的路径点总数:    "<<joint_trajectory_size<<std::endl;
    m_pathplan_stop = true;
    on_path_planing = false;

    auto get_current_dir_name = []()
    {
        std::string current_time;
        QDateTime datetime = QDateTime::currentDateTime();
        QString timestr = datetime.toString("yyyy-MM-dd hh:mm:ss");
        current_time = timestr.toStdString();
        return current_time;
    };

    if (joint_trajectory_size > 1)
    {
        //2021.12.02 xbk 如出现刚发完命令，则执行完路径规划后，不执行轨迹跟踪控制
        //20220419   yt  question?????
        if(commandtask_.status == false && commandtask_.taskinfor == 1)
        {
            commandtask_.status = true;
            std::cerr<<"path plan is success ,but now need to stop because command is 1"<<std::endl;
            solution_trajectory_.points.clear();
            m_trajectory_point_size = 0;
            return;
        }

        solution_trajectory_ = auto_crane_manipulation_->get_solution_trajectory();

        //save the planned trajectory into file
        std::string file_path = base_path + ".catkin_data/path_plan/" + get_current_dir_name() + ".txt";
        std::ofstream f_path_plan_out(file_path);
        f_path_plan_out << "time from start"  << " "
                        << "position j0" << " " << "position j1" << " " << "position j2" << " "
                        << "velocity j0" << " " << "velocity j1" << " " << "velocity j2" << std::endl;
        for (const auto& point: solution_trajectory_.points)
        {
            f_path_plan_out << point.time_from_start << " " << point.positions[0] << " " << point.positions[1] << " " << point.positions[2] << " "
                            << point.velocities[0] << " " << point.velocities[1] << " " << point.velocities[2] << " " << std::endl;
        }
        f_path_plan_out.close();
        glWidget_->setSolutionTrajectoryEETipPath(auto_crane_manipulation_->get_vis_joint_trajectory());
        on_Trajectory_tracking_triggered();
    }
    else
    {
        std::cerr << "Failed to plan the motion." << std::endl;
        glWidget_->setSolutionTrajectoryEETipPath(0);
        return;
    }
}

void MainWindow::on_Trajectory_tracking_triggered()
{
    is_start_crane_trajectory_track_ = true;

    //  start_Obstacles_Detect_Thread();        //open the obstacles detect Thread
    //  qDebug("------------------------------on_IntelBtn_clicked:start autolift!----------------------------------/n");

    if (solution_trajectory_.points.empty())
    {
        qDebug("The solution trajectory is empty, seems the path planning was failed.");
        return;
    }
    //    solution_trajectory_.points[0].
    if (!solution_trajectory_.points.empty())
    {
        //start realtime collision object detection
        qDebug("----------------------------------------------------start realtime collision detection!-------------------------------------/n");

        //start the trajectory send thread
        std::promise<bool> start_realtime_check_collision_promise;
        std::future<bool> start_realtime_collision_check_future = start_realtime_check_collision_promise.get_future();
        if (send_joint_trajectory_thread_.joinable())
        {
            send_joint_trajectory_thread_.join();
        }

        connect( thread, SIGNAL(showMB()), this, SLOT(showMessageBox()), Qt::BlockingQueuedConnection ) ;
        send_joint_trajectory_thread_ = std::thread(&MainWindow::sendJointTrajectory, this, &start_realtime_check_collision_promise);

        //start realtime collision detection
        if (start_realtime_collision_check_future.get())
            realtime_collision_check_thread_ = std::thread(&MainWindow::predictCollision, this);
    }
}

void MainWindow::showMessageBox()
{
    QMessageBox Msgbox;
    Msgbox.setText("路径跟踪完成，请在右侧视频中框选目标进行精准定位!");
    Msgbox.exec();
}

void MainWindow::on_suspended_triggered()
{
    //  m_frame_mutex.lockForRead();
    //  Mat image = m_gframe_;
    //  m_frame_mutex.unlock();
    Mat image = imread("/home/zoomlion/PC_WORK/Project/ros/Catkin_ws_autoLift/rebuild3d/rgb.jpg");
    cv::resize(image,image,cv::Size(2048,1536));
    QImage img_frame = QImage((const unsigned char*)(image.data), image.cols, image.rows, QImage::Format_RGB888);

}

void MainWindow::on_attach_lift_param_setBtn_clicked()
{
    ui.tabWidget_param->setCurrentIndex(1);
}

void MainWindow::on_attach_clicked()
{
    if(ui.itemType->currentIndex() == 0)
    {
        m_object_len = ui.edit_goods_len->text().toFloat();
        m_object_width = ui.edit_goods_w->text().toFloat();
        m_object_weight = ui.edit_goods_h->text().toFloat();
    }
    else
    {
        m_object_len = ui.edit_circle_length->text().toFloat();
    }

    m_hook_height = ui.edit_hook_height->text().toFloat();
    m_rope_height = ui.edit_sling_height->text().toFloat();
    m_goods_height = ui.edit_target_height->text().toFloat();
    m_object_weight = ui.edit_goods_weight->text().toFloat();
    ui.tabWidget_param->setCurrentIndex(2);
}

void MainWindow::on_exit_btn_triggered()
{
    is_open_video_player_display_ = false;
    open_vedio_stream_thread_.join();

    is_start_object_position_compute_ = false;
    start_object_position_compute_thread_.join();

    is_send_control_command_package_ = false;
    send_control_command_package_.join();

    is_start_pcan_data_receive_thread_ = false;
    start_pcan_data_receive_thread_.join();

    is_start_cramera_real_ptz_control_thread_ = false;
    start_cramera_real_ptz_control_thread_.join();

    is_start_data_update_display_thread_ = false;
    start_data_update_display_thread_.join();

    realtime_collision_check_thread_.join();
    send_joint_trajectory_thread_.join();

    is_start_obstacles_detect_thread_ = false;
    start_obstacles_detect_thread_.join();

    is_start_scence_visualization_thread_ = false;
    start_scence_visualization_thread_.join();

    planning_scene_visualization_timer_->stop();
    app_update_timer_->stop();

//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
    collisionWorker_->stop();         // 告诉 worker 停止循环
    collisionWorkerThread_->quit();    // 告诉 Qt 事件循环退出
    collisionWorkerThread_->wait();    // 等待底层线程真正结束 (类似 join)

    delete glWidget_;
    delete auto_crane_manipulation_;
    delete m_clsClientToYunTai;
    delete m_clsClientUsbToCan;
    delete m_obstacle_detect_class;
    delete  crane_parameter_;

    reset_all_parameter_to_initial_status();
    sleep(1);
    close();
}

void MainWindow::start_path_plan_Thread()
{
    m_start_pathPlan_thread_flag = true;

    if(thread_start_pathPlan.joinable())
    {
        thread_start_pathPlan.join();
    }

    thread_start_pathPlan = std::thread(&MainWindow::Function_Start_PathPlan_Thread, this);
}

//jiqunxietong

void MainWindow::on_stop_crane_control()
{
    pathend_but_no_jingzhundingwei = true;
    if(is_stop_send_trajectory_.load() == true)
    {
        avoidstopstart = true;
        solution_trajectory_.points.clear();//yt add,exit send joint function
        is_stop_send_trajectory_.store(false);     //stop send trajectory thread

        //        std::cerr<<"statrt send stop point ////////////////////////////////////////////////// "<<std::endl;
        //        clsClientUsbToCan->SendCanData("01D1", "0005000000000000");//kaiqishineng
        //        std::cerr<<point_now.velocities[0]<<std::endl;
        //        std::cerr<<point_now.velocities[1]<<std::endl;
        //        std::cerr<<point_now.velocities[2]<<std::endl;
        //        int slow_stop_time = 20;
        //        ompl::app::joint_trajectory_point_t point;
        //        point.velocities.clear();
        //        point.positions.clear();
        //        double stop_r = (point_now.velocities[0]/(slow_stop_time*10));
        //        double stop_c = (point_now.velocities[1]/(slow_stop_time*10));
        //        double stop_h = (point_now.velocities[2]/(slow_stop_time*10));

        //        for(int i = 0;i < slow_stop_time*10;i++)
        //        {
        //            point.velocities.push_back(point_now.velocities[0] - stop_r*i);
        //            point.velocities.push_back(point_now.velocities[1] - stop_c*i);
        //            point.velocities.push_back(point_now.velocities[2] - stop_h*i);
        //            point.positions.push_back(point_now.positions[0] + (point.velocities[0]*i/10));
        //            point.positions.push_back(point_now.positions[1] + (point.velocities[1]*i/10));
        //            point.positions.push_back(point_now.positions[2] + (point.velocities[2]*i/10));
        //            clsClientUsbToCan->UpdateCanData(clsClientUsbToCan->Data01D1[1], 2, 1);
        //            clsClientUsbToCan->SendCanData("01D1", "0005000000000000");//kaiqishineng
        //            sendJointTrajectoryPoint(point);
        //            std::cerr<<"send stop joint "<<i<<std::endl;
        //            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //            point.velocities.clear();
        //            point.positions.clear();
        //}
        //std::cerr<<"end send stop point ////////////////////////////////////////////////// "<<std::endl;

        is_running_predict_collision_.store(false);
        //reset flags
        is_replanning_.store(false);
        is_predicted_collision_.store(false);

        //send_joint_trajectory_thread_.detach();

        //clsClientUsbToCan->SendCanData("01D1", "0006000000000000");//close path track flag
        //        //clsClientUsbToCan->UpdateCanData(clsClientUsbToCan->Data01D1[1], 0, 0);     //close path tracker flag
        //        //clsClientUsbToCan->UpdateCanData(clsClientUsbToCan->Data01D1[1], 2, 0);     //lifting mode flag
        //        sendCommandTaskStatusTo(commandtask_.id ,2,"success");
    }
    Send01D0DataCommand();

    if(is_send_control_command_package_ == true)
    {
        is_send_control_command_package_ = false;
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 0);  //cancle 精准定位
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 0);

        Send01D1DataCommand();
        pathend_but_no_jingzhundingwei = false;
        sendCommandTaskStatusTo(commandtask_.id ,2,"success");
        qDebug("on_stop_crane_control------bThreadSendPackage-------pathend_but_no_jingzhundingwei = false");
    }
}

void MainWindow::set_new_pathplan_start_positon()
{
    std::cerr<<"come_here"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    Send01D1DataCommand();
    if(crane_parameter_->crane_hook_to_ground_height_ - 16 <= 0)
    {
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
        m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
        Send01D1DataCommand();
    }
}

//jiqunxietong
void MainWindow::resolveMqttData(mqtt::const_message_ptr msg){
    std::cout << "Message arrived MainWindows" << std::endl;
    std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
    std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;

    std::cerr.setf(ios::fixed);
    std::cerr.precision(3);

    json msg_j = json::parse(msg->to_string());
    //    json j = {
    //      {"token","11"},	//口令信息，保留位
    //      {"GPSposition",	//吊物差分GPS坐标位置信息
    //      {
    //      "x",1,	//浮点型，坐标，下同
    //      "y",2,
    //      "z",3
    //    }},
    //    {"jointState",
    //          {
    //           1 //positionparam浮点型，回转角
    //          ,2 //浮点型，塔机：变幅长度（小车距离，单位米）；起重机：变幅角度（俯仰角）
    //          ,3 //浮点型，卷扬高度（臂架末端到吊钩末端的距离，米）。
    //          }},
    //  };
    //    cout<<"resolveMqttData:..topic:"<<msg->get_topic()<<"..."<<msg_j<<endl;
    // std::map<std::string, std::shared_ptr<ompl::app::Manipulation>>::iterator  iterd = Device_map.begin();
    try{
        //智能设备只订阅自身的avoidcommand topic task/crane/[id]
        if (msg->get_topic().find("command/avoid/truck_crane-01") != string::npos)
        {
            std::cerr<<"get avoidcommandtask_ truck_crane-01"<<std::endl;
            avoidcommandtask_.id = msg_j.at("task").at("taskid");
            avoidcommandtask_.type = msg_j.at("task").at("type");
            avoidcommandtask_.status = false;
            avoidcommandtask_.ifget = true;
            if(msg_j.at("task").contains("targetposition"))
            {
                std::cerr<<"get avoid targetposition"<<std::endl;
                avoidcommandtask_.targetpositon.point_x = msg_j.at("task").at("targetposition").at("x");
                avoidcommandtask_.targetpositon.point_y = msg_j.at("task").at("targetposition").at("y");
                avoidcommandtask_.targetpositon.point_z = msg_j.at("task").at("targetposition").at("z");
            }

            if(msg_j.at("task").contains("jointstateposition"))
            {
                std::cerr<<"get avoid jointstateposition1"<<std::endl;
                avoidcommandtask_.status1 = false;
                avoidcommandtask_.targetpostrue1.rotate = msg_j.at("task").at("jointstateposition")[0];
                avoidcommandtask_.targetpostrue1.amplitude = msg_j.at("task").at("jointstateposition")[1];
                avoidcommandtask_.targetpostrue1.roller = msg_j.at("task").at("jointstateposition")[2];
            }
            if(msg_j.at("task").contains("jointstateposition2"))
            {
                std::cerr<<"get avoid jointstateposition2"<<std::endl;
                avoidcommandtask_.status2 = false;
                avoidcommandtask_.targetpostrue2.rotate = msg_j.at("task").at("jointstateposition2")[0];
                avoidcommandtask_.targetpostrue2.amplitude = msg_j.at("task").at("jointstateposition2")[1];
                avoidcommandtask_.targetpostrue2.roller = msg_j.at("task").at("jointstateposition2")[2];
            }
        }
        //智能设备只订阅自身的task topic
        if (msg->get_topic().find("task/crane/truck_crane-01") != string::npos)
        {
#if 1
            //////////////////////播报
            is_start_play_voice_thread_ = true;
            play_voice_flag = 0;
            //////////////////////播报
            std::cerr<<"get dispatchtask_ truck_crane-01"<<std::endl;
            dispatchtask_.id = msg_j.at("task").at("taskid");
            dispatchtask_.type = msg_j.at("task").at("type");

            writetxt(dispatchtask_.id);

            if(msg_j.at("task").contains("targetposition"))
            {
                std::cerr<<"get targetposition"<<std::endl;
                dispatchtask_.statusa = false;
                dispatchtask_.auto_crane_location.point_x = msg_j.at("task").at("targetposition").at("x");
                dispatchtask_.auto_crane_location.point_y = msg_j.at("task").at("targetposition").at("y");
                dispatchtask_.auto_crane_location.point_z = msg_j.at("task").at("targetposition").at("z");
            }
            if(msg_j.at("task").contains("buildingunit"))
            {
                std::cerr<<"get buildingunit"<<std::endl;

                dispatchtask_.taskinfor.id = msg_j.at("task").at("buildingunit").at("unitid");
                dispatchtask_.taskinfor.type = msg_j.at("task").at("buildingunit").at("unittype");
                dispatchtask_.taskinfor.weight = msg_j.at("task").at("buildingunit").at("unitweight");
                dispatchtask_.taskinfor.orientation = msg_j.at("task").at("buildingunit").at("orientation");
                if(msg_j.at("task").at("buildingunit").contains("originposition"))
                {
                    std::cerr<<"get dispatch task of building location task"<<std::endl;
                    if(msg_j.at("task").at("buildingunit").at("originposition").at("x") > 0.0001)
                    {
                        dispatchtask_.statusb = false;
                    }
                    dispatchtask_.taskinfor.buildinglocation.point_x = msg_j.at("task").at("buildingunit").at("originposition").at("x");
                    dispatchtask_.taskinfor.buildinglocation.point_y = msg_j.at("task").at("buildingunit").at("originposition").at("y");
                    dispatchtask_.taskinfor.buildinglocation.point_z = msg_j.at("task").at("buildingunit").at("originposition").at("z");
                    std::cerr<<"dispatchtask_.taskinfor.buildinglocation.point_x"<<dispatchtask_.taskinfor.buildinglocation.point_x<<std::endl;
                    std::cerr<<"dispatchtask_.taskinfor.buildinglocation.point_y"<<dispatchtask_.taskinfor.buildinglocation.point_y<<std::endl;
                    std::cerr<<"dispatchtask_.taskinfor.buildinglocation.point_z"<<dispatchtask_.taskinfor.buildinglocation.point_z<<std::endl;
                }
                if(msg_j.at("task").at("buildingunit").contains("originposition"))
                {
                    std::cerr<<"get dispatch task of buildingtarget location task"<<std::endl;
                    if(msg_j.at("task").at("buildingunit").at("targetposition").at("x") > 0.0001)
                    {
                        dispatchtask_.statust = false;
                    }
                    //dispatchtask_.statust = false;
                    dispatchtask_.taskinfor.buildingtargetlocation.point_x = msg_j.at("task").at("buildingunit").at("targetposition").at("x");
                    dispatchtask_.taskinfor.buildingtargetlocation.point_y = msg_j.at("task").at("buildingunit").at("targetposition").at("y");
                    dispatchtask_.taskinfor.buildingtargetlocation.point_z = msg_j.at("task").at("buildingunit").at("targetposition").at("z");
                    std::cerr<<"dispatchtask_.taskinfor.buildingtargetlocation.point_x"<<dispatchtask_.taskinfor.buildingtargetlocation.point_x<<std::endl;
                    std::cerr<<"dispatchtask_.taskinfor.buildingtargetlocation.point_y"<<dispatchtask_.taskinfor.buildingtargetlocation.point_y<<std::endl;
                    std::cerr<<"dispatchtask_.taskinfor.buildingtargetlocation.point_z"<<dispatchtask_.taskinfor.buildingtargetlocation.point_z<<std::endl;
                }
            }
            //      if(msg_j.at("task").contains("safeposition"))
            //      {
            //        std::cerr<<"get safelocation"<<std::endl;
            //        dispatchtask_.statuss = false;
            //        dispatchtask_.safelocation.point_x = msg_j.at("task").at("safeposition").at("x");
            //        dispatchtask_.safelocation.point_y = msg_j.at("task").at("safeposition").at("y");
            //        dispatchtask_.safelocation.point_z = msg_j.at("task").at("safeposition").at("z");
            //      }
            if(dispatchtask_.statusb == false || dispatchtask_.statust == false || dispatchtask_.statuss == false)
            {
                dispatchtask_.ifget = true;
                dispatchtask_.completeall = false;
                std::cerr<<"get dispatchtask_ is valid"<<std::endl;
                sendTaskStatusTo(dispatchtask_.id,0,0); //dispatchtask_  status 0-get
                sendTaskStatusTo(dispatchtask_.id,-1,1);
                sendAutodrvingStatusTo(1,1);//wurenjiashi kais
                //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 0, 1);//无人驾驶状态标志位
                Send01D1DataCommand();
            }
            else
            {
                resetdispatchtask_();
                std::cerr<<"get dispatchtask_ but is invalid"<<std::endl;
            }
            #endif
        }

        //智能设备只订阅自身的command topic
        if (msg->get_topic().find("command/truck_crane-01") != string::npos)
        {
            //        //////////////////////播报
            //        is_start_play_voice_thread_ = true;
            //        play_voice_flag = 2;
            //        //////////////////////播报
            std::cerr<<"get commandtask_ truck_crane-01"<<std::endl;
            commandtask_.id = msg_j.at("commandid");
            commandtask_.status = false;
            commandtask_.ifget = true;
            commandtask_.taskinfor = msg_j.at("command");
        }
        //与自动调姿的交互逻辑
        if (msg->get_topic().find("command/ack1/icrane-02") != string::npos)
        {
            std::cerr<<"command/ack1/icrane-02"<<std::endl;
            std::cerr<<"acktype"<<msg_j.at("acktype")<<std::endl;
            //msg_j.at("commandid");
            if(msg_j.at("acktype") == 6)
            {
                dispatchtask_.zhuyuxing = 2;
            }
        }
        //与无人驾驶的的交互逻辑
        if (msg->get_topic().find("state/autodrving/truck_crane-01") != string::npos)
        {
            std::cerr<<"state/autodrving/truck_crane-01"<<std::endl;
            //std::cerr<<"acktype"<<msg_j.at("acktype")<<std::endl;
            //msg_j.at("commandid");
            if((msg_j.at("taskstate") == 2) && (msg_j.at("taskid") == 1))
            {
                sendTaskStatusTo(dispatchtask_.id,-1,2);
                dispatchtask_.autodrvingb = true;
                dispatchtask_.ifget = true;
            }
            if((msg_j.at("taskstate") == 2) && (msg_j.at("taskid") == 2))
            {
                sendTaskStatusTo(dispatchtask_.id,-1,8);
                dispatchtask_.autodrvingt = true;
                dispatchtask_.ifget = true;
            }
            if((msg_j.at("taskstate") == 1) && (msg_j.at("taskid") == 1))
            {
                //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 0, 1);//无人驾驶状态标志位
                Send01D1DataCommand();
                std::cerr<<"正在无人驾驶前往货物堆放点"<<std::endl;
            }
            if((msg_j.at("taskstate") == 1) && (msg_j.at("taskid") == 2))
            {
                //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 0, 1);//无人驾驶状态标志位
                Send01D1DataCommand();
                std::cerr<<"正在无人驾驶前往货物目标点"<<std::endl;
            }
        }
        //收到调度系统的打招呼语音
        if (msg->get_topic().find("broadcast/received") != string::npos)
        {
            is_start_play_voice_thread_ = true;
            play_voice_flag = 20;//待修改
        }

        //语音调度指令
        float forward_x =0.0;
        float backward_x =0.0;
        float rotate_down_left_x =0.0;
        float rotate_down_right_x =0.0;
        int say_hello =0;
        int emergency_stop =0;
        int nav_to_lifting_start =0;
        int nav_to_lifting_destination =0;
        int auto_lifting_hook_location =0;
        int auto_lifting_object_location =0;
        int pose_restore =0;

        if (msg->get_topic().find("task/crane/crane-01") != string::npos)
        {
            std::cerr<<"解析语音调度的topic"<<std::endl;
            if(msg_j.at("task").contains("basic"))
            {
                forward_x =msg_j.at("task").at("basic").at("forward_x");
                if (forward_x !=0.0)
                    std::cerr<<"无人驾驶前进 forward_x ="<<forward_x<<std::endl;
                backward_x =msg_j.at("task").at("basic").at("backward_x");
                if (backward_x !=0.0)
                    std::cerr<<"无人驾驶后退 backward_x ="<<backward_x<<std::endl;
                rotate_down_left_x =msg_j.at("task").at("basic").at("rotate_down_left_x");
                if (rotate_down_left_x !=0.0)
                    std::cerr<<"底盘左转rotate_down_left_x ="<<rotate_down_left_x<<std::endl;
                rotate_down_right_x =msg_j.at("task").at("basic").at("rotate_down_right_x");
                if (rotate_down_right_x !=0.0)
                    std::cerr<<"底盘左转rotate_down_right_x ="<<rotate_down_right_x<<std::endl;

                say_hello =msg_j.at("task").at("basic").at("say_hello");
                if(say_hello ==1)
                {
                    is_start_play_voice_thread_ = true;
                    play_voice_flag = 19;
                }

                emergency_stop =msg_j.at("task").at("basic").at("emergency_stop");
                if(emergency_stop ==1)
                    std::cerr<<"急停 急停 急停！！!"<<std::endl;
                rotate_up_left_x =msg_j.at("task").at("basic").at("rotate_up_left_x");
                rotate_up_right_x =msg_j.at("task").at("basic").at("rotate_up_right_x");
                luff_up_x =msg_j.at("task").at("basic").at("luff_up_x");
                luff_down_x =msg_j.at("task").at("basic").at("luff_down_x");
                wind_up_x =msg_j.at("task").at("basic").at("wind_up_x");
                wind_down_x =msg_j.at("task").at("basic").at("wind_down_x");

            }

            if(msg_j.at("task").contains("compound"))
            {
                nav_to_lifting_start =msg_j.at("task").at("compound").at("nav_to_lifting_start");
                if(nav_to_lifting_start ==1)
                {
                     std::cerr<<"前往吊装起点 起点！！!"<<std::endl;
                     std::cerr<<"前往吊装起点 起点！！!"<<std::endl;
                     stopRotion();
                     stopAmp();
                     stopLuff();
                }

                nav_to_lifting_destination =msg_j.at("task").at("compound").at("nav_to_lifting_destination");
                if(nav_to_lifting_destination ==1)
                {
                     std::cerr<<"前往吊装就位点 就位点！！!"<<std::endl;
                     std::cerr<<"前往吊装就位点 就位点！！!"<<std::endl;
                     stopRotion();
                     stopAmp();
                     stopLuff();
                }

                pose_restore =msg_j.at("task").at("compound").at("pose_restore");
                if(pose_restore ==1)
                {
                    std::cerr<<"恢复行驶姿态 恢复行驶姿态 恢复行驶姿态！！!"<<std::endl;
                    std::cerr<<"恢复行驶姿态 恢复行驶姿态 恢复行驶姿态！！!"<<std::endl;
                    stopRotion();
                    stopAmp();
                    stopLuff();
                }

                auto_lifting_hook_location =msg_j.at("task").at("compound").at("auto_lifting_hook_location");                                              
                if(auto_lifting_hook_location ==1)
                {
                    std::cerr<<"空钩自动对位 对位 对位！！!"<<std::endl;
                    std::cerr<<"空钩自动对位 对位 对位！！!"<<std::endl;
                    std::cerr<<"空钩自动对位 对位 对位！！!"<<std::endl;
                    stopRotion();
                    stopAmp();
                    stopLuff();
                }

                auto_lifting_object_location =msg_j.at("task").at("compound").at("auto_lifting_object_location");
                if(auto_lifting_object_location ==1)
                {
                    std::cerr<<"吊载自动对位 对位 对位！！!"<<std::endl;
                    std::cerr<<"吊载自动对位 对位 对位！！!"<<std::endl;
                    std::cerr<<"吊载自动对位 对位 对位！！!"<<std::endl;
                    stopRotion();
                    stopAmp();
                    stopLuff();
                }
            }
        }
    }
    catch(const json::exception& exc){
        std::cerr<<"JSON error..."<<exc.what()<<std::endl;
    }
}

void MainWindow::sendRobotStatusTo(QVector<float>  jointstates, vector<double>  GPSPosition,int workstatus,float g_ArmLen,int taskid,double headangle,vector<double>  GPSPositionL)
{
    json joint_status;
    string robotName = "truck_crane-01";
    joint_status["token"] = 1;
    std::string time;
    QDateTime datetime = QDateTime::currentDateTime();
    QString time2 = datetime.toString("yyyy-MM-dd hh:mm:ss");
    time = time2.toStdString();
    joint_status["time"] = time;
    //std::cerr<<"11111111111time ="<<time<<std::endl;
    //std::cerr<<joint_status["time"]<<std::endl;
    joint_status["id"] = robotName;
    joint_status["GPSposition"]["x"] = GPSPosition[1];
    joint_status["GPSposition"]["y"] = GPSPosition[0];
    joint_status["GPSposition"]["z"] = GPSPosition[2];
    //1111111111111
    joint_status["GPSpositionL"]["x"] = GPSPositionL[1];
    joint_status["GPSpositionL"]["y"] = GPSPositionL[0];
    joint_status["GPSpositionL"]["z"] = GPSPositionL[2];

    //joint_status["aa"]["x"] =5;
    //11111111111
    joint_status["jointState"] = jointstates;
    joint_status["CarHeadAngle"] = headangle;
    joint_status["workstatus"] = workstatus;
    joint_status["craneArmLen"] = g_ArmLen;
    joint_status["currenttaskid"] = taskid;
    string joint_status_s = joint_status.dump();
    string PUBTOPIC_test = "status/crane/";
    PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,0,joint_status_s);
    //std::cerr << "Sending joint_status To brain ..."<< PUBTOPIC_test << endl  ;
}
void MainWindow::sendTaskStatusTo(int taskid, int stage, int status)
{
    json task_status;
    string robotName = "truck_crane-01";
    task_status["taskid"] = taskid;
    task_status["status"] = stage;//整型，0 收到命令，1 规划成功，2 执行完成，3 规划失败
    task_status["stage"] = status;//整型，0 收到命令，1 规划成功，2 执行完成，3 规划失败
    task_status["errorcode"] = 4;
    string task_status_s = task_status.dump();
    string PUBTOPIC_test = "ack/";
    PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,0,task_status_s);
    std::cerr << "发送给调度系统的id 是"<< taskid<<"。。。。。消息内容是"<<status << std::endl  ;
    std::cerr << "topic ="<<PUBTOPIC_test<<std::endl;
}
void MainWindow::sendAvoidTaskStatusTo(int taskid, int status)
{
    json task_status;
    string robotName = "truck_crane-01";
    task_status["taskid"] = taskid;
    task_status["status"] = status;//整型，0 收到命令，1 规划成功，2 执行完成，3 规划失败
    task_status["errorcode"] = 4;
    string task_status_s = task_status.dump();
    string PUBTOPIC_test = "avoid/ack/";
    PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,1,task_status_s);
    std::cerr<< "sendAvoidTaskStatusTo To brain ...id "<<taskid<<"<-----"<<PUBTOPIC_test << std::endl  ;
}
void MainWindow::sendCommandTaskStatusTo(int commandid, int status,string message)
{
    json task_status;
    string robotName = "truck_crane-01";
    task_status["commandid"] = commandid;
    task_status["resultack"] = status;//整型，，1 规划成功，else失败
    task_status["errormessage"] = message;//errormessage   error massage
    string task_status_s = task_status.dump();
    string PUBTOPIC_test = "command/ack/";
    PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,1,task_status_s);
    std::cerr << "sendCommandTaskStatusTo To brain ..id ."<<commandid<< "<-----"<<PUBTOPIC_test << std::endl  ;
}
void MainWindow::sendIcraneStatusTo(int commandid, int status,string message)
{
    json task_status;
    string robotName = "truck_crane-01";
    task_status["commandid"] = "truck_crane-01";
    task_status["acktype"] = status;//整型，，1-start
    string task_status_s = task_status.dump();
    string PUBTOPIC_test = "command/iack/";
    PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,1,task_status_s);
    std::cerr << "sendIcraneStatusTo To brain ..id ."<<commandid<< "<-----"<<PUBTOPIC_test << std::endl  ;
}

void MainWindow::sendAutodrvingStatusTo(int taskid,int state)
{
    json task_status;
    string robotName = "truck_crane-01";
    task_status["taskid"] = taskid;
    task_status["taskstate"] = state;
    string task_status_s = task_status.dump();
    string PUBTOPIC_test = "state/autolifting/";
    PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,1,task_status_s);
    std::cerr << "sendIcraneStatusTo To brain ..id ."<<taskid<< "<-----"<<PUBTOPIC_test << std::endl  ;
}


void MainWindow::sendRegeisterTo(double x, double y,double z)
{
    std::cerr<<"注册x"<<x<<std::endl;
    std::cerr<<"注册y"<<y<<std::endl;
    std::cerr<<"注册z"<<z<<std::endl;
    json Regeister_status;
    string robotName = "truck_crane-01";
    Regeister_status["token"] = robotName;
    Regeister_status["uid"] = robotName;//2无人起重机
    Regeister_status["type"] = 2;//2无人起重机
    Regeister_status["GPSposition"]["x"] = x;
    Regeister_status["GPSposition"]["y"] = y;
    Regeister_status["GPSposition"]["z"] = z;
    string task_status_s = Regeister_status.dump();
    string PUBTOPIC_test = "manager/register";
    //PUBTOPIC_test += robotName;
    mqtt_client_->mqtt_pub(PUBTOPIC_test,1,task_status_s);
    std::cerr << "sendRegeisterTo  brain ..id .-----"<<PUBTOPIC_test << std::endl  ;
}

void MainWindow::resetavoidcommandtask_()
{
    avoidcommandtask_.id = 0;
    avoidcommandtask_.type = 0;
    avoidcommandtask_.status = true;
    avoidcommandtask_.ifget = false;
    avoidcommandtask_.status1 = true;
    avoidcommandtask_.status2 = true;
    avoidcommandtask_.targetpostrue1.amplitude = 0;
    avoidcommandtask_.targetpostrue1.roller = 0;
    avoidcommandtask_.targetpostrue1.rotate = 0;
    avoidcommandtask_.targetpostrue2.amplitude = 0;
    avoidcommandtask_.targetpostrue2.roller = 0;
    avoidcommandtask_.targetpostrue2.rotate = 0;
    avoidcommandtask_.targetpositon.point_x = 0;
    avoidcommandtask_.targetpositon.point_y = 0;
    avoidcommandtask_.targetpositon.point_z = 0;
}

void MainWindow::resetcommandtask_()
{
    commandtask_.id = 0;    current_goal_position.point_x =3121020.348;//3123589.042
    current_goal_position.point_y =382850.557;//396188.493
    commandtask_.status = true;
    commandtask_.taskinfor = 0;
    commandtask_.ifget = false;
}

void MainWindow::resetdispatchtask_()
{
    dispatchtask_.id = 0;
    dispatchtask_.type = 0;
    dispatchtask_.statusb = true;
    dispatchtask_.statust = true;
    dispatchtask_.statuss = true;
    dispatchtask_.statusa = true;
    dispatchtask_.ifget = false;
    dispatchtask_.completeall = true;


    dispatchtask_.autodrvingb = false;
    dispatchtask_.autodrvingt = false;
    dispatchtask_.hook_down_up = false;
    dispatchtask_.recoverautodving = false;
    dispatchtask_.zhuyuxing = 0;

    dispatchtask_.safelocation.point_x = 0;
    dispatchtask_.safelocation.point_y = 0;
    dispatchtask_.safelocation.point_z = 0;
    dispatchtask_.taskinfor.id = 0;
    dispatchtask_.taskinfor.orientation = 0;
    dispatchtask_.taskinfor.type = 0;
    dispatchtask_.taskinfor.weight = 0;
    dispatchtask_.taskinfor.buildinglocation.point_x = 0;
    dispatchtask_.taskinfor.buildinglocation.point_y = 0;
    dispatchtask_.taskinfor.buildinglocation.point_z = 0;
    dispatchtask_.taskinfor.buildingtargetlocation.point_x = 0;
    dispatchtask_.taskinfor.buildingtargetlocation.point_y = 0;
    dispatchtask_.taskinfor.buildingtargetlocation.point_z = 0;
    dispatchtask_.auto_crane_location.point_x = 0;
    dispatchtask_.auto_crane_location.point_y = 0;
    dispatchtask_.auto_crane_location.point_z = 0;

    //  dispatchtask_.id = 0;
    //  dispatchtask_.type = 0;
    //  dispatchtask_.statusb = false;
    //  dispatchtask_.statust = true;
    //  dispatchtask_.statuss = true;
    //  dispatchtask_.statusa = true;
    //  dispatchtask_.ifget = false;
    //  dispatchtask_.completeall = false;
    //  dispatchtask_.safelocation.point_x = 0;
    //  dispatchtask_.safelocation.point_y = 0;
    //  dispatchtask_.safelocation.point_z = 0;
    //  dispatchtask_.taskinfor.id = 0;
    //  dispatchtask_.taskinfor.orientation = 0;
    //  dispatchtask_.taskinfor.type = 0;
    //  dispatchtask_.taskinfor.weight = 0;
    //  dispatchtask_.taskinfor.buildinglocation.point_x = 5;
    //  dispatchtask_.taskinfor.buildinglocation.point_y = -5;
    //  dispatchtask_.taskinfor.buildinglocation.point_z = 5;
}
void MainWindow::play_voice(int index)
{
    string play_name;
    play_name = "play /home/zoomlion/mp3/"+to_string(index)+".mp3";
    system(play_name.c_str());
}

//jiqunxietong

//void MainWindow::get_env_scene()
//{

//    std::ifstream  fin("/home/zoomlion/filter_collision_objects_cube_file.txt", std::ios::in);
//    char  line[1024]={0};
//    std::vector<ObstacleInforStr> get_3d_collision_objects_from_txt;
//    ObstacleInforStr obj_;
//    while(fin.getline(line, sizeof(line)))
//    {
//        std::stringstream  word(line);
//        word >> obj_.centerPt.x;
//        word >> obj_.centerPt.y;
//        word >> obj_.centerPt.z;
//        word >> obj_.box3d.width_x;
//        word >> obj_.box3d.height_y;
//        word >> obj_.box3d.depth_z;
//        word >> obj_.roll;
//        get_3d_collision_objects_from_txt.push_back(obj_);
//    }
//    fin.clear();
//    fin.close();
//    std::vector<ObstacleInforStr> filtered_collision_objects_txt;
//    filter_collision_objects(get_3d_collision_objects_from_txt, filtered_collision_objects_txt);
//    //std::cerr<<"path plan the filter_collision_objects is : "<<filtered_collision_objects_txt.size()<<std::endl;
//    //add the filtered collision objects
//    for (unsigned int j = 0; j < filtered_collision_objects_txt.size(); j++)
//    {
//        if(filtered_collision_objects_txt[j].centerPt.z < 1 || filtered_collision_objects_txt[j].centerPt.x > 50 || filtered_collision_objects_txt[j].centerPt.y >50)
//        {
//            continue;
//        }
////            double siny_cosp = 2*(filtered_collision_objects_txt[j].qw*filtered_collision_objects_txt[j].qz+filtered_collision_objects_txt[j].qy*filtered_collision_objects_txt[j].qx);
////            double cosy_cosp = 1 - 2 * (filtered_collision_objects_txt[j].qy *filtered_collision_objects_txt[j].qy + filtered_collision_objects_txt[j].qz * filtered_collision_objects_txt[j].qz);
////            filtered_collision_objects_txt[j].roll =  atan2(siny_cosp,cosy_cosp) * 180 / M_PI;
//        std::string name = "temp_ob_" + std::to_string(j);
//        auto_crane_manipulation_.add_cube_collision_object(name,
//                                                           filtered_collision_objects_txt[j].box3d.height_y,
//                                                           filtered_collision_objects_txt[j].box3d.width_x,
//                                                           filtered_collision_objects_txt[j].box3d.depth_z,
//                                                           filtered_collision_objects_txt[j].centerPt.x,
//                                                           filtered_collision_objects_txt[j].centerPt.y,
//                                                           filtered_collision_objects_txt[j].centerPt.z, 0, 0,(g_dRotationAngle - filtered_collision_objects_txt[j].roll)*M_PI/180);
//    }

//    //add_cylinder_obs
//    std::ifstream  fincy("/home/zoomlion/filter_collision_objects_cylinder_file.txt", std::ios::in);
//    char  line1[1024]={0};
//    std::vector<ObstacleInforStr> get_3d_collision_cylinder_objects_from_txt;
//    ObstacleInforStr obj_1;
//    while(fincy.getline(line1, sizeof(line1)))
//    {
//        std::stringstream  word(line1);
//        word >> obj_1.centerPt.x;
//        word >> obj_1.centerPt.y;
//        word >> obj_1.centerPt.z;
//        word >> obj_1.box3d.width_x;
//        word >> obj_1.box3d.depth_z;
//        get_3d_collision_cylinder_objects_from_txt.push_back(obj_1);
//    }
//    fincy.clear();
//    fincy.close();
//    for (unsigned int j = 0; j < get_3d_collision_cylinder_objects_from_txt.size(); j++)
//    {
//        std::string name1 = "temp_cylinder_ob_" + std::to_string(j);
//        auto_crane_manipulation_.add_cylinder_collision_object(name1,
//                                                               get_3d_collision_cylinder_objects_from_txt[j].box3d.width_x,
//                                                               get_3d_collision_cylinder_objects_from_txt[j].box3d.depth_z,
//                                                               get_3d_collision_cylinder_objects_from_txt[j].centerPt.x,
//                                                               get_3d_collision_cylinder_objects_from_txt[j].centerPt.y,
//                                                               get_3d_collision_cylinder_objects_from_txt[j].centerPt.z, 0, 0, 0);
//    }

//}
zl_common::app::pointPostrue MainWindow::position_to_postrue_transform(zl_common::app::structPointXYZ position)
{
    zl_common::app::pointPostrue postrue;
    postrue.amplitude = sqrt(pow(position.point_x,2)+pow(position.point_y,2));
    if(position.point_x > 0 && position.point_y >= 0)
    {
        postrue.rotate = atan(position.point_y/position.point_x)*180/M_PI;
    }
    else if(position.point_x <= 0 && position.point_y > 0)
    {
        postrue.rotate = atan(fabs(position.point_x)/position.point_y)*180/M_PI + 90;
    }
    else if(position.point_x < 0 && position.point_y < 0)
    {
        postrue.rotate = 270 - atan(position.point_y/position.point_x)*180/M_PI;
    }
    else if(position.point_x > 0 && position.point_y < 0)
    {
        postrue.rotate = 360 - atan(fabs(position.point_y)/position.point_x)*180/M_PI;
    }
    else if(position.point_x < 0 && position.point_y == 0)
    {
        postrue.rotate = 180;
    }
    else if(position.point_x == 0 && position.point_y < 0)
    {
        postrue.rotate = 270;
    }
    else
    {
        std::cerr<<"the position of position_to_postrue_transform is error!!" <<std::endl;
        return postrue;
    }
    postrue.roller = crane_parameter_->crane_arm_to_ground_height_ - position.point_z;
    return postrue;
}

void MainWindow::on_auto_drive_finished_clicked()
{
    sendTaskStatusTo(dispatchtask_.id,-1,2);
    dispatchtask_.autodrvingb = true;
    dispatchtask_.ifget = true;

}

void MainWindow::on_hook_hang_or_off_finished_clicked()
{
//    if(dispatchtask_.statust == false)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,5);
//    }
//    else
    {
        sendTaskStatusTo(dispatchtask_.id,-1,11);
    }
    dispatchtask_.hook_down_up = true;
}

void MainWindow::on_posture_adjust_finished_clicked()
{
    dispatchtask_.zhuyuxing = 2;
    qDebug()<<"--------------------------------------------调姿完成----------------------------------------------";
}

void MainWindow::on_pushButton_one_clicked()
{    //current_goal_position.point_x =3121020.348;//3123589.042
    //current_goal_position.point_y =382850.557;//396188.493
    sendTaskStatusTo(dispatchtask_.id,-1,8);
    dispatchtask_.autodrvingt = true;
    dispatchtask_.ifget = true;
}

void MainWindow::on_pushButton_two_clicked()
{
    //sendTaskStatusTo(dispatchtask_.id,-1,10);
    //////////////////////播报
    std::cerr<<"voice test"<<std::endl;
    is_start_play_voice_thread_ = true;
    play_voice_flag = 1;
    //////////////////////播报
    sendAutodrvingStatusTo(1,1);
}
}  // namespace zl_Crane_AutomaticLift_trunk

void zl_Crane_AutomaticLift_trunk::MainWindow::on_beep_triggered()
{
    dispatchtask_.id = 6;
    dispatchtask_.type = 0;
    dispatchtask_.statusb = true;
    dispatchtask_.statust = false;
    dispatchtask_.statuss = true;
    dispatchtask_.statusa = true;
    dispatchtask_.ifget = true;
    dispatchtask_.completeall = false;


    dispatchtask_.autodrvingb = false;
    dispatchtask_.autodrvingt = false;
    dispatchtask_.hook_down_up = false;
    dispatchtask_.recoverautodving = false;
    dispatchtask_.zhuyuxing = 0;

    dispatchtask_.safelocation.point_x = 0;
    dispatchtask_.safelocation.point_y = 0;
    dispatchtask_.safelocation.point_z = 0;
    dispatchtask_.taskinfor.id = 0;
    dispatchtask_.taskinfor.orientation = 0;
    dispatchtask_.taskinfor.type = 0;
    dispatchtask_.taskinfor.weight = 0;
    dispatchtask_.taskinfor.buildinglocation.point_x = 3121011.972;
    dispatchtask_.taskinfor.buildinglocation.point_y = 382892.170;
    dispatchtask_.taskinfor.buildinglocation.point_z = 58.6;
    dispatchtask_.taskinfor.buildingtargetlocation.point_x = 3121011.972;
    dispatchtask_.taskinfor.buildingtargetlocation.point_y = 382892.170;
    dispatchtask_.taskinfor.buildingtargetlocation.point_z = 58.6;
    dispatchtask_.auto_crane_location.point_x = 0;
    dispatchtask_.auto_crane_location.point_y = 0;
    dispatchtask_.auto_crane_location.point_z = 0;
    sendAutodrvingStatusTo(1,1);//wurenjiashi kais
    //m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 0, 1);//无人驾驶状态标志位
    Send01D1DataCommand();

    //////////////////////播报
    is_start_play_voice_thread_ = true;
    play_voice_flag = 0;
    //////////////////////播报


    //sendTaskStatusTo(dispatchtask_.id,2,11);
    //dispatchtask_.autodrvingb = true;

    //    icrane_ok = true;
    //rtkflag1 =2;
    //m_lift_work_mode_++;
    //    return;
    //    sendIcraneStatusTo(0,1,"start");
    //    if(m_lift_work_mode_ < 11)
    //    play_voice(m_lift_work_mode_);
    //    if(m_lift_work_mode_ = 1)
    //    {
    //      dispatchtask_.id = 0;
    //      dispatchtask_.type = 0;
    //      dispatchtask_.statusb = false;
    //      dispatchtask_.statust = true;
    //      dispatchtask_.statuss = true;
    //      dispatchtask_.statusa = true;
    //      dispatchtask_.ifget = true;
    //      dispatchtask_.completeall = false;
    //      dispatchtask_.safelocation.point_x = 0;
    //      dispatchtask_.safelocation.point_y = 0;
    //      dispatchtask_.safelocation.point_z = 0;
    //      dispatchtask_.taskinfor.id = 0;
    //      dispatchtask_.taskinfor.orientation = 0;
    //      dispatchtask_.taskinfor.type = 0;
    //      dispatchtask_.taskinfor.weight = 0;
    //      dispatchtask_.taskinfor.buildinglocation.point_x = 5;
    //      dispatchtask_.taskinfor.buildinglocation.point_y = -5;
    //      dispatchtask_.taskinfor.buildinglocation.point_z = 5;
    //    }
    //    if(m_lift_work_mode_ = 2)
    //    {
    //      dispatchtask_.id = 0;
    //      dispatchtask_.type = 0;
    //      dispatchtask_.statusb = true;
    //      dispatchtask_.statust = true;
    //      dispatchtask_.statuss = false;
    //      dispatchtask_.statusa = true;
    //      dispatchtask_.ifget = true;
    //      dispatchtask_.completeall = false;
    //      dispatchtask_.safelocation.point_x = 1;
    //      dispatchtask_.safelocation.point_y = 1;
    //      dispatchtask_.safelocation.point_z = 1;
    //      dispatchtask_.taskinfor.id = 0;
    //      dispatchtask_.taskinfor.orientation = 0;
    //      dispatchtask_.taskinfor.type = 0;
    //      dispatchtask_.taskinfor.weight = 0;
    //      dispatchtask_.taskinfor.buildinglocation.point_x = 5;
    //      dispatchtask_.taskinfor.buildinglocation.point_y = -5;
    //      dispatchtask_.taskinfor.buildinglocation.point_z = 5;
    //    }


}



void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_clicked()
{
    is_send_control_command_package_autodrving = true;
    send_control_command_package_autodrving = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread, this);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_2_clicked()
{
    is_send_control_command_package_ = true;
    send_control_command_package_= std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_direct, this);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_3_clicked()
{
    is_send_control_command_package_ = true;
    send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_test, this);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_zhuzi1_clicked()
{
    zhuzi_bool = true;
    current_goal_position.point_x =3121024.112;//3123589.042
    current_goal_position.point_y =382850.427;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_zhuzi2_clicked()
{
    zhuzi_bool = true;
    current_goal_position.point_x =3120987.011;//3123589.042
    current_goal_position.point_y =382850.207;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_zhuzi3_clicked()
{
    zhuzi_bool = true;
    current_goal_position.point_x =3120987.011;//3123589.042
    current_goal_position.point_y =382850.207;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_zhuzi4_clicked()
{
    zhuzi_bool = true;
    current_goal_position.point_x =3121020.348;//3123589.042
    current_goal_position.point_y =382850.557;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_qiangban1_clicked()
{
    qiangban_bool = true;
    current_goal_position.point_x =3121022.337;//3123589.042
    current_goal_position.point_y =382854.154;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_qiangban2_clicked()
{
    qiangban_bool = true;
    current_goal_position.point_x =3121020.407;//
    current_goal_position.point_y =382852.404;//
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_qiangban3_clicked()
{
    qiangban_bool = true;
    current_goal_position.point_x =3121022.219;//3123589.042
    current_goal_position.point_y =382850.504;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_qiangban4_clicked()
{
    qiangban_bool = true;
    current_goal_position.point_x =3121024.158;//3123589.042
    current_goal_position.point_y =382852.287;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_6_clicked()
{
    m_lift_work_mode_ = 1;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_up_hook_clicked()
{
    std::cerr<<"hook  up"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 1);		//卷扬升
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_stop_hook_clicked()
{
    std::cerr<<"hook  stop"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);
    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_down_hook_clicked()
{
    std::cerr<<"hook)down"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 4, 0);		//卷扬落
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 1);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_7_clicked()
{
    current_goal_position.point_x =3121022.292;//3123589.042
    current_goal_position.point_y =382852.399;//396188.493
    is_send_control_command_package_ = true;
    send_control_command_package_ = std::thread(&MainWindow::Function_Send_Control_Command_Package_Thread_TestRTK_dizuo, this);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_5_clicked()
{
    qiangban_hook_down = true;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_rotate_left_clicked()
{
    std::cerr<<"rotate_left"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[2], 7, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 0);
    //qDebug()<<"-----------------左回转--------------------------";
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_rotate_stop_clicked()
{
    std::cerr<<"rotate_stop"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);

    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_rotate_right_clicked()
{
    std::cerr<<"rotate_right"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 2, 1);
    //qDebug()<<"-----------------左回转--------------------------";
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 1);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_luff_up_clicked()
{
    std::cerr<<"luff_up"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 1);	//变幅落方向
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_luff_stop_clicked()
{
    std::cerr<<"luff_stop"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 3, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 1, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 0);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 4, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D0[0], 7, 1);

    m_clsClientUsbToCan->SendCanData("01D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D1", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("04D0", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("01A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("02A2", "0000000000000000");
    m_clsClientUsbToCan->SendCanData("03A2", "0000000000000000");
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_luff_down_clicked()
{
    std::cerr<<"luff_down"<<std::endl;
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[3], 1, 1);//自动吊装与无人驾驶状态标志位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 0, 1);  //开启精准定位
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 0, 0); //close path tracker flagstatust
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 1, 1);
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 2, 1);  //lifting mode flag
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 6, 0);	//变幅落方向
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[0], 5, 1);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_zhuzi_clicked()
{
    zuozi_bool = true;
}


void zl_Crane_AutomaticLift_trunk::MainWindow::on_gaizi_clicked()
{
    gaizi_bool = true;
    current_goal_position.point_x =3121022.292;//3123589.042
    current_goal_position.point_y =382852.399;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_test_liang_clicked()
{
    readTxt();

    dispatchtask_.id =dispatchtask_id;

//    if(yt_test == 0)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,0);
//    }
//    else if(yt_test == 1)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,7);
//    }
//    else if(yt_test == 2)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,8);
//    }
//    else if(yt_test == 3)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,9);
//    }
//    if(yt_test == 4)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,10);
//    }
//    else if(yt_test == 5)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,11);
//    }
//    else if(yt_test == 6)
//    {
//        sendTaskStatusTo(dispatchtask_.id,-1,12);
//    }
//    else if(yt_test == 7)
//    {
//        sendTaskStatusTo(dispatchtask_.id,2,6);
//    }
//    yt_test++;
//    std::cerr<<"1111111111yt_test ="<<yt_test<<std::endl;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::writetxt(int id)
{
    ofstream outfile;
    string filePath ="id.txt";

    outfile.open(filePath);

    if(!outfile)
    {
        std::cerr<<"open idfile failed 22222222222"<<std::endl;
    }

    outfile << id <<endl;

    outfile.close();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::readTxt()
{
    char line[5];
    ifstream infile;

    infile.open("id.txt");
    if(!infile)
    {
        std::cerr<<"can not open file11111111111"<<std::endl;
        exit(1);
    }

    while(infile.getline(line,5,' '))
    {
        dispatchtask_id =stoi(line);
    }

    std::cerr<<"dispatchtask_id ="<<dispatchtask_id<<std::endl;
    infile.close();
}



void zl_Crane_AutomaticLift_trunk::MainWindow::on_start_target_voice_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 1;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_task_wancheng_voice_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 3;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_recome_safe_voice_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 5;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_8_clicked()
{
    north_board = true;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_9_clicked()
{
    gaizi_bool = true;
    current_goal_position.point_x =3121022.292;//3123589.042
    current_goal_position.point_y =382852.399;//396188.493
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_10_clicked()
{
    m_clsClientUsbToCan->UpdateCanData(m_clsClientUsbToCan->Data01D1[1], 6, 0);
    Send01D1DataCommand();
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_11_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 19;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_12_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 3;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_pushButton_13_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 4;
}


void zl_Crane_AutomaticLift_trunk::MainWindow::on_respose_tower_clicked()
{
    is_start_play_voice_thread_ = true;
    play_voice_flag = 7;
}

//void zl_Crane_AutomaticLift_trunk::MainWindow::on_board_qian_clicked()
//{
//    gcranepositionL.clear();
//    gcranepositionL.push_back(3121001.798 + num_qiangban * );
//    gcranepositionL.push_back(382847.384);
//    gcranepositionL.push_back(49.080);
//    std::cerr

//}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_gaizi_hou_clicked()
{
    
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_voice_control_clicked()
{
    //语音控制线程
    is_start_voice_control_thread_ = true;
    start_voice_control_thread_ = std::thread(&MainWindow::Function_voice_control_Thread, this);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_rotation_right_clicked()
{
    rotate_up_right_x =10;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_rotation_left_clicked()
{
    rotate_up_left_x =10;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_luff_up_2_clicked()
{
    luff_up_x =5;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_luff_down_2_clicked()
{
    luff_down_x =5;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_wind_up_clicked()
{
    wind_up_x =2;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::on_wind_down_clicked()
{
    wind_down_x =5;
}

void zl_Crane_AutomaticLift_trunk::MainWindow::space2jointCoordinate(double x,double y)
{
    //默认回转角车头为0度，逆时针增大
    double rotationAngel =0;
    double crane_luff_length =0;

    //打点坐标轴，沿车头为y轴正方向，x轴为垂直车头向右

    if(x >=0 && y >=0)
    {
        rotationAngel =360 - atan(x/y) * 180/M_PI;
    }
    else if(x >0 && y <=0 )
    {
        rotationAngel =270 - atan(fabs(y)/x) * 180/M_PI;
    }
    else if(x <=0 && y <0)
    {
        rotationAngel =180 - atan(fabs(x)/fabs(y)) * 180/M_PI;
    }
    else
    {
        rotationAngel = atan(fabs(x)/fabs(y)) * 180/M_PI;
    }
    crane_luff_length =sqrt(pow(x,2) + pow(y,2));

    jointDiff(rotationAngel,crane_luff_length);

}

void zl_Crane_AutomaticLift_trunk::MainWindow::jointDiff(double targetRotatonAngel,double targetLuffLength)
{
    double rotationAngelDiff_left =0;
    double rotationAngelDiff_right =0;
    double luffLengthDiff =0;
    double z_diff =0;

    rotationAngelDiff_left =fabs(targetRotatonAngel - (crane_parameter_->crane_rotation_Angle_));

    rotationAngelDiff_right =360 - rotationAngelDiff_left;

    luffLengthDiff =fabs(targetLuffLength) - crane_parameter_->crane_luff_length;

//    for(int i =0;i <targetsInfors_.size();i++)
//    {
//        for(int j=0;j <targetsInfors_[i].size();j++)
//        {
//            z_diff =  crane_parameter_->crane_hook_to_ground_height_ - targetsInfors_[i][j].center_point.z + targetsInfors_[i][j].height;
//        }
//    }

    z_diff =  crane_parameter_->crane_hook_to_ground_height_ - targetsInfors_z;

    QString str;
    if(targetRotatonAngel > crane_parameter_->crane_rotation_Angle_)
    {
        str ="逆"+ QString::number(rotationAngelDiff_left, 'f', 1) + "°" + "顺" + QString::number(rotationAngelDiff_right, 'f', 1) + "°";
    }
    else
    {
        str ="顺"+ QString::number(rotationAngelDiff_left, 'f', 1) + "°" + "逆" + QString::number(rotationAngelDiff_right, 'f', 1) + "°";
    }

    ui.lineEdit_R->setText(str);

    str = QString::number(luffLengthDiff, 'f', 2);
    ui.lineEdit_A->setText(str);

    str = QString::number(z_diff, 'f', 2);
    ui.lineEdit_L->setText(str);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::joint2spaceCoordinate(double rotationAngel,double crane_luff_length)
{
    //默认回转角车头为0度，逆时针增大

        //徐百科坐标系  x轴沿着车头向前，y轴沿着车头向左，
        //我的坐标系，y轴沿着车头向前，x轴沿着车头向右，

        //打点坐标轴，沿车头为y轴正方向，x轴为垂直车头向右
        double x =0;
        double y =0;
        if(rotationAngel >=0 && rotationAngel <90)
        {
            x =-crane_luff_length * sin(rotationAngel * M_PI/180);
            y =crane_luff_length * cos(rotationAngel * M_PI/180);
        }
        else if(rotationAngel >=90 && rotationAngel <180)
        {
            x = -crane_luff_length * sin((180 - rotationAngel) * M_PI/180);
            y = -crane_luff_length * cos((180 - rotationAngel) * M_PI/180);
        }
        else if(rotationAngel >=180 && rotationAngel <270)
        {
            x =crane_luff_length * sin((rotationAngel - 180) * M_PI/180);
            y = -crane_luff_length * cos((rotationAngel - 180) * M_PI/180);
        }
        else
        {
            x =crane_luff_length * cos((rotationAngel - 270) * M_PI/180);
            y =crane_luff_length * sin((rotationAngel - 270) * M_PI/180);
        }

        double x_diff =0;
        double y_diff =0;
        double z_diff =0;


        x_diff =  targetsInfors_x - x;
        y_diff =  targetsInfors_y - y;
        z_diff =  crane_parameter_->crane_hook_to_ground_height_ - targetsInfors_z ;

        QString str;
        str = QString::number(x_diff, 'f', 2);
        ui.lineEdit_X->setText(str);

        str = QString::number(y_diff, 'f', 2);
        ui.lineEdit_Y->setText(str);

        str = QString::number(z_diff, 'f', 2);
        ui.lineEdit_Z->setText(str);
}


void zl_Crane_AutomaticLift_trunk::MainWindow::currSpaceAndJointCompute()
{
    rtk->process();
    vector<double> currPoint;
    vector<double>  craneCenterPoint;
    double rotation,amplitude;


    //step1:计算当前的空间位置以及关节位置

    craneCenterPoint.push_back(rtk->carHeadData[1]);
    craneCenterPoint.push_back(rtk->carHeadData[2]);

    currPoint.clear();

    currPoint.push_back(rtk->armHeadData[0]);//正北
    currPoint.push_back(rtk->armHeadData[1]);//正东

    //rtk->getRotationAmp(currPoint,  craneCenterPoint, m_headAngle_);

    double center_N = craneCenterPoint[0];     //正北
    double center_E = craneCenterPoint[1];      //正东
    double headingAngle = rtk->carHeadingAngel - 180;       //航向角

    //正北 正东相对位置
    double Ndist =0;
    double Edist =0;

    //平移到参考系的坐标
    Ndist =currPoint[0] - center_N;
    Edist =currPoint[1] - center_E;

    //转到车身坐标系
    //旋转后,正北沿着车头向前,顺时针旋转,垂直车头向右
    double car_x =0;
    double car_y =0;

    //航向角是正北到正东的偏转
    car_y =Ndist*cos(headingAngle*M_PI/180) + Edist*sin(headingAngle*M_PI/180);//这是坐标系的旋转
    car_x =Edist*cos(headingAngle*M_PI/180) - Ndist*sin(headingAngle*M_PI/180);

    //平移到回转中心
    double translate_x =0;
    double translate_y =0;

    car_y =car_y - translate_y;

    //计算回转变幅长
//    double amplitude =0;
//    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    //y轴沿着车头向前，x轴沿着车头向右
    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/car_x)*180/M_PI;
        rotation =270 + rotation;
    }

    rotation =360 - rotation;

//step2 :计算目标的空间位置以及关节位置
    //默认回转角车头为0度，逆时针增大
    double targetRotationAngel =0;
    double targetCrane_luff_length =0;

    //打点坐标轴，沿车头为y轴正方向，x轴为垂直车头向右

    if(targetsInfors_x >=0 && targetsInfors_y >=0)
    {
        targetRotationAngel =360 - atan(targetsInfors_x/targetsInfors_y) * 180/M_PI;
    }
    else if(targetsInfors_x >0 && targetsInfors_y <=0 )
    {
        targetRotationAngel =270 - atan(fabs(targetsInfors_y)/targetsInfors_x) * 180/M_PI;
    }
    else if(targetsInfors_x <=0 && targetsInfors_y <0)
    {
        targetRotationAngel =180 - atan(fabs(targetsInfors_x)/fabs(targetsInfors_y)) * 180/M_PI;
    }
    else
    {
        targetRotationAngel = atan(fabs(targetsInfors_x)/fabs(targetsInfors_y)) * 180/M_PI;
    }
    targetCrane_luff_length =sqrt(pow(targetsInfors_x,2) + pow(targetsInfors_y,2));


    double rotationAngelDiff_left =0;
    double rotationAngelDiff_right =0;
    double luffLengthDiff =0;
    double z_diff =0;

    rotationAngelDiff_left =fabs(targetRotationAngel - rotation);

    rotationAngelDiff_right =360 - rotationAngelDiff_left;

    luffLengthDiff =fabs(targetCrane_luff_length) - amplitude;

    QString str;
    if(targetRotationAngel > rotation)
    {
        str ="逆"+ QString::number(rotationAngelDiff_left, 'f', 1) + "°" + "顺" + QString::number(rotationAngelDiff_right, 'f', 1) + "°";
    }
    else
    {
        str ="顺"+ QString::number(rotationAngelDiff_left, 'f', 1) + "°" + "逆" + QString::number(rotationAngelDiff_right, 'f', 1) + "°";
    }

    ui.lineEdit_R->setText(str);

    str = QString::number(luffLengthDiff, 'f', 2);
    ui.lineEdit_A->setText(str);

    str = QString::number(z_diff, 'f', 2);
    ui.lineEdit_L->setText(str);

    double x_diff =0;
    double y_diff =0;


    x_diff =  targetsInfors_x - car_x;
    y_diff =  targetsInfors_y - car_y;
    z_diff =  crane_parameter_->crane_hook_to_ground_height_ - targetsInfors_z ;

    str = QString::number(x_diff, 'f', 2);
    ui.lineEdit_X->setText(str);

    str = QString::number(y_diff, 'f', 2);
    ui.lineEdit_Y->setText(str);

    str = QString::number(z_diff, 'f', 2);
    ui.lineEdit_Z->setText(str);
}

void zl_Crane_AutomaticLift_trunk::MainWindow::currSpaceAndJointCompute_luntai()
{
    vector<double> currPoint;
    vector<double>  craneCenterPoint;
    double rotation,amplitude;


    //step1:计算当前的空间位置以及关节位置


    if (rtk->carHeadData.size() !=4)
    {
        cerr<<"the rtk position is error Please check rtk"<<endl;
        return;
    }

    craneCenterPoint.push_back(rtk->carHeadData[1]);
    craneCenterPoint.push_back(rtk->carHeadData[2]);

    currPoint.clear();

    if(rtk->armHeadData.size()!=2)
    {
        std::cerr<<"the armHead data is error!!!!!!!!!!"<<std::endl;
        std::cerr<<"the size is ="<<rtk->armHeadData.size()<<std::endl;
        return;
    }

    currPoint.push_back(rtk->armHeadData[0]);//正北
    currPoint.push_back(rtk->armHeadData[1]);//正东

    //rtk->getRotationAmp(currPoint,  craneCenterPoint, m_headAngle_);

    double center_N = craneCenterPoint[0];     //正北
    double center_E = craneCenterPoint[1];      //正东
    double headingAngle = rtk->carHeadingAngel - 180;       //航向角

    //正北 正东相对位置
    double Ndist =0;
    double Edist =0;

    //平移到参考系的坐标
    Ndist =currPoint[0] - center_N;
    Edist =currPoint[1] - center_E;

    //转到车身坐标系
    //旋转后,正北沿着车头向前,顺时针旋转,垂直车头向右
    double carRtk_x =0;
    double carRtk_y =0;

    //航向角是正北到正东的偏转,旋转到沿臂架
    carRtk_y =Ndist*cos(headingAngle*M_PI/180) + Edist*sin(headingAngle*M_PI/180);//这是坐标系的旋转
    carRtk_x =Edist*cos(headingAngle*M_PI/180) - Ndist*sin(headingAngle*M_PI/180);

    //平移到回转中心
    double translate_y =3.45;
    double translate_x =1.075;

    carRtk_y =carRtk_y - translate_y;
    carRtk_x -carRtk_x - translate_x;

    double car_y =0.0;
    double car_x =0.0;
    //臂在车身右边
    double boomToCarAngel =0;

    if (crane_parameter_->crane_rotation_Angle_ >180.0 && crane_parameter_->crane_rotation_Angle_ <=360.0)
    {
        boomToCarAngel =crane_parameter_->crane_rotation_Angle_ - 360.0;
    }
    else if(crane_parameter_->crane_rotation_Angle_ >=0.0 && crane_parameter_->crane_rotation_Angle_ <180.0)  //臂在车身左边
    {
        boomToCarAngel =crane_parameter_->crane_rotation_Angle_;
    }

    //旋转到沿车身
    car_y =carRtk_y*cos(boomToCarAngel*M_PI/180) + carRtk_x*sin(boomToCarAngel*M_PI/180);//这是坐标系的旋转
    car_x =carRtk_x*cos(boomToCarAngel*M_PI/180) - carRtk_y*sin(boomToCarAngel*M_PI/180);


    //旋转到沿车头向前

    //计算回转变幅长
//    double amplitude =0;
//    double rotation =0;

    amplitude =sqrt(pow(car_y ,2) + pow(car_x,2));
    //rotation =atan(car_x/car_y)*180/M_PI;

    //y轴沿着车头向前，x轴沿着车头向右
    if(car_x >0 && car_y >0)
    {
        rotation =atan(car_x/car_y)*180/M_PI;
    }
    else if (car_x >0 && car_y <0)
    {
        rotation =atan((fabs(car_y))/(fabs(car_x)))*180/M_PI;
        rotation =90 + rotation;
    }
    else if(car_x <0 && car_y <0)
    {
        rotation =atan(fabs(car_x)/(fabs(car_y)))*180/M_PI;
        rotation =180 + rotation;
    }
    else if(car_x <0 && car_y >0)
    {
        rotation =atan(fabs(car_y)/car_x)*180/M_PI;
        rotation =270 + rotation;
    }

    rotation =360 - rotation;

//step2 :计算目标的空间位置以及关节位置
    //默认回转角车头为0度，逆时针增大
    double targetRotationAngel =0;
    double targetCrane_luff_length =0;

    //打点坐标轴，沿车头为y轴正方向，x轴为垂直车头向右

    if(targetsInfors_x >=0 && targetsInfors_y >=0)
    {
        targetRotationAngel =360 - atan(targetsInfors_x/targetsInfors_y) * 180/M_PI;
    }
    else if(targetsInfors_x >0 && targetsInfors_y <=0 )
    {
        targetRotationAngel =270 - atan(fabs(targetsInfors_y)/targetsInfors_x) * 180/M_PI;
    }
    else if(targetsInfors_x <=0 && targetsInfors_y <0)
    {
        targetRotationAngel =180 - atan(fabs(targetsInfors_x)/fabs(targetsInfors_y)) * 180/M_PI;
    }
    else
    {
        targetRotationAngel = atan(fabs(targetsInfors_x)/fabs(targetsInfors_y)) * 180/M_PI;
    }
    targetCrane_luff_length =sqrt(pow(targetsInfors_x,2) + pow(targetsInfors_y,2));


    double rotationAngelDiff_left =0;
    double rotationAngelDiff_right =0;
    double luffLengthDiff =0;
    double z_diff =0;

    rotationAngelDiff_left =fabs(targetRotationAngel - rotation);

    rotationAngelDiff_right =360 - rotationAngelDiff_left;

    luffLengthDiff =fabs(targetCrane_luff_length) - amplitude;

    QString str;
    if(targetRotationAngel > rotation)
    {
        str ="逆"+ QString::number(rotationAngelDiff_left, 'f', 1) + "°" + "顺" + QString::number(rotationAngelDiff_right, 'f', 1) + "°";
    }
    else
    {
        str ="顺"+ QString::number(rotationAngelDiff_left, 'f', 1) + "°" + "逆" + QString::number(rotationAngelDiff_right, 'f', 1) + "°";
    }

    ui.lineEdit_R->setText(str);

    str = QString::number(luffLengthDiff, 'f', 2);
    ui.lineEdit_A->setText(str);

    str = QString::number(z_diff, 'f', 2);
    ui.lineEdit_L->setText(str);

    double x_diff =0;
    double y_diff =0;


    x_diff =  targetsInfors_x - car_x;
    y_diff =  targetsInfors_y - car_y;
    z_diff =  crane_parameter_->crane_hook_to_ground_height_ - targetsInfors_z ;

    str = QString::number(x_diff, 'f', 2);
    ui.lineEdit_X->setText(str);

    str = QString::number(y_diff, 'f', 2);
    ui.lineEdit_Y->setText(str);

    str = QString::number(z_diff, 'f', 2);
    ui.lineEdit_Z->setText(str);
}

//***************************************************************
//*********                 碰撞检测及点云可视化          ***********
//***************************************************************
void zl_Crane_AutomaticLift_trunk::MainWindow::initPCLViewer() {
    // 1. 使用 PCL 1.8 最基本的构造函数
    // 注意：只传 "pclviewer_" 和 false，这样 PCL 内部会创建自己的 RenderWindow
    pclviewer_.reset(new pcl::visualization::PCLVisualizer("pclviewer_", false));

    // 2. 获取 PCL 内部自动创建的那个窗口
    vtkRenderWindow* pcl_win = pclviewer_->getRenderWindow();

    // 3. 【关键步骤】将 PCL 的窗口“强行”交给 Qt 控件
    // 这样 QVTKWidget 就会显示 PCL 的内容，而不是自己再建一个窗口
    qvtkWidget_->SetRenderWindow(pcl_win);

    // 4. 关联交互器
    // 必须使用 qvtkWidget 提供的交互器，以确保 Qt 的鼠标事件能传给 PCL
    pclviewer_->setupInteractor(qvtkWidget_->GetInteractor(), qvtkWidget_->GetRenderWindow());

    // 5. 解决 Stack underflow (1284) 的补丁
    // 这个问题通常是因为多重采样 (MultiSampling) 在旧版 VTK 和 Qt5 之间冲突
    pcl_win->SetMultiSamples(0); 

    // 调节视角
    
    pclviewer_->resetCamera();
    pclviewer_->setCameraPosition(
        -100.0,   0.0,   -40.0,  // 相机位置 //
        63,    0.0,   0.0,       // 焦点位置 //collisionWorker_->ctx->busData_.rope_len_ 
        0.0,     0.0,   1.0        // Z 轴向上
    );

    pclviewer_->setBackgroundColor(0.1, 0.1, 0.1);
}
void zl_Crane_AutomaticLift_trunk::MainWindow::updateVisualization(CollisionResult result, 
                                        CloudVisVector visInfo) 
{
    // 渲染逻辑（在此处执行你原来的可视化代码）
    for (const auto& vis_ptr : visInfo) {
        const auto& vis = *vis_ptr;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(vis.cloud_, vis.r_, vis.g_, vis.b_);
        if (!pclviewer_->updatePointCloud(vis.cloud_, color, vis.cloudID_)) {
            pclviewer_->addPointCloud(vis.cloud_, color, vis.cloudID_);
        }
    }

    // 绘制碰撞线
    pcl::PointXYZI start = result.nearest_point_;
    pcl::PointXYZI end;
    end.x = start.x - result.dir_[0] * result.minDistance_;
    end.y = start.y - result.dir_[1] * result.minDistance_;
    end.z = start.z - result.dir_[2] * result.minDistance_;
    
    pclviewer_->removeShape("minDistance");
    pclviewer_->addLine(start, end, 1.0, 0.0, 0.0, "minDistance");

    // 刷新渲染
    qvtkWidget_->GetRenderWindow()->Render();
}
//***************************************************************
//*********                 碰撞检测及点云可视化 end     ***********
//***************************************************************