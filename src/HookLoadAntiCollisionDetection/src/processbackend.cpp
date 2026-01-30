#include <HookLoadAntiCollisionDetection/processbackend.h>
template <typename PointT>
ProcessBackEnd<PointT>::ProcessBackEnd(ros::NodeHandle& nh,std::shared_ptr<SharedSyncContext<PointT>> ctx)
:ctx_(ctx)
{

    //---------------params---------------------------------
    nh.param<int>("accum_frames",accum_frames_,5); //帧累计数
    nh.param<int>("frame_rate",frame_rate_,1); //循环频率
    nh.param<std::string>("input_topic",subscrib_topic_,"/converted_points"); //输入话题
    nh.param<std::string>("output_topic",advertise_topic_,"/22");   //输出话题



    pc_sub_ = nh.subscribe(subscrib_topic_, 10, &ProcessBackEnd::cloudCallBack,this); //消息队列长度：10
    // pub_ = nh.advertise<sensor_msgs::PointCloud2>(advertise_topic_,1); //点云发布

    preprocess_thread_ = std::thread(&ProcessBackEnd::processLoop,this);

    pipeline_ = std::make_shared<Pipeline<PointT>>(ctx_);
}

template <typename PointT>
ProcessBackEnd<PointT>::~ProcessBackEnd(){
    if(preprocess_thread_.joinable()){preprocess_thread_.join();}
}


#ifdef ROSBAG_TEST
    /**
    *@brief 点云回调函数，采集一帧点云后存入帧缓存池中，并通知处理线程使用
    */
    template <typename PointT>
    void ProcessBackEnd<PointT>::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msgs)
    {
        {
            //回调函数
            std::lock_guard<std::mutex> lock(pc_mtx_);
            if (pc_buf_.size() > 5) {
                pc_buf_.pop_front(); // 丢掉最老帧
                ROS_WARN("drop one stale pc frame");
            }
            pc_buf_.push_back(msgs);
        }
        pc_cv_.notify_one();
    }

    /**
    *@brief  处理线程：接收回调函数更新的数据，运行推理
    */
    template <typename PointT>
    void ProcessBackEnd<PointT>::processLoop(){
        ros::Rate rate(frame_rate_);
        while(ros::ok())
        {
            sensor_msgs::PointCloud2ConstPtr pc_msg;
            {
                std::unique_lock<std::mutex> lock(pc_mtx_);
                if ( !pc_cv_.wait_for(lock, std::chrono::milliseconds(100), [&](){ return !pc_buf_.empty(); })) {
                    ROS_WARN("wait for input pc msgs.");
                    pipeline_->modeReset();
                    continue;
                }
                pc_msg = pc_buf_.back();   // 只取最新
                pc_buf_.clear();           // 清空旧帧

            }
            // typename pcl::PointCloud<PointT>::Ptr pcl_pc_frame(new pcl::PointCloud<PointT>);
            // pcl::fromROSMsg(*pc_msg,*pcl_pc_frame);

            pcl::PCLPointCloud2 cloud2;
            pcl_conversions::toPCL(*pc_msg, cloud2);
            typename pcl::PointCloud<PointT>::Ptr pcl_pc_frame(new pcl::PointCloud<PointT>);
            pcl::fromPCLPointCloud2(cloud2, *pcl_pc_frame);

            // accumCloud(pcl_pc_frame);

            if(!pcl_pc_frame || pcl_pc_frame->points.size()==0){
                ROS_WARN("[=== PIPELINE ===] FAILED . input cloud is empty or nullptr ");
                rate.sleep();
                continue;
            }

            pipeline_->forward(pcl_pc_frame);//处理一帧点云

            auto t0 = ros::Time::now();
            rate.sleep();
            ros::Duration cycle = rate.cycleTime();
        }
    }
#else
    /**
    *@brief 点云回调函数，采集一帧点云后存入帧缓存池中，并通知处理线程使用
    */
    template <typename PointT>
    void ProcessBackEnd<PointT>::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msgs)
    {

        // 1. 将 ROS 消息转换为 PCL 点云
        typename pcl::PointCloud<PointT>::Ptr current_cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msgs, *current_cloud);
        {   
            std::lock_guard<std::mutex> lock(pc_mtx_);
            pc_buf_.push_back(current_cloud);
            // 限制 buffer 最大长度（例如15帧），防止内存无限增长
            if (pc_buf_.size() > 10) {
                pc_buf_.pop_front();
            }
            if (pc_buf_.size() >= 5) {
                pc_cv_.notify_one();
            }
        }
    }

    /**
    *@brief  处理线程：接收回调函数更新的数据，运行推理
    */
    template <typename PointT>
    void ProcessBackEnd<PointT>::processLoop(){
        ros::Rate rate(frame_rate_);
        while (ros::ok()) {
            if(!ctx_->busData_.b_updated_){
                ROS_WARN("[=== PIPELINE ===] FAILED . canData not updated! ");
                continue;
            }
            std::vector<typename pcl::PointCloud<PointT>::Ptr> frames_to_merge;

            {
                std::unique_lock<std::mutex> lock(pc_mtx_);
                pc_cv_.wait(lock, [this]() { return pc_buf_.size() >= 5; });
                std::cout << "Buffer backlog: " << pc_buf_.size() << " frames. Using latest 5." << std::endl;
                // 如果积压了，只取最后 5 帧
                while(pc_buf_.size() > 5){
                    pc_buf_.pop_front(); 
                }

                // 现在 buffer 里精准剩下最近的 5 帧，全部取出
                while(!pc_buf_.empty()){
                    frames_to_merge.push_back(pc_buf_.front());
                    pc_buf_.pop_front();
                }
            }

            // 3. 执行累积
            typename pcl::PointCloud<PointT>::Ptr final_cloud(new pcl::PointCloud<PointT>);
            for(auto& frame : frames_to_merge){
                *final_cloud += *frame; // 注意：若运动剧烈仍需坐标变换
            }

            if (final_cloud->empty()) continue;
            // std::cout<<"num points input: "<<final_cloud->size()<<std::endl;
            // 4. 立即进行碰撞检测
            pipeline_->forward(final_cloud); 
        }
    }
#endif


template class ProcessBackEnd<pcl::PointXYZ>;
template class ProcessBackEnd<pcl::PointXYZI>;