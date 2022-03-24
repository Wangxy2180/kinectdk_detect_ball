/**
 * 
 * 功能：采集iaikinect2输出的彩色图和深度图数据，并进行网球的识别
 * 这个是我抄的
 */

#include <iostream>
#include <string>
// #include <stdlib.h>
#include <vector>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <cv_bridge/cv_bridge.h>
//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

//时间戳精准匹配模式
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// #include <obj_detected/ball_pose.h>
// #include <obj_detected/pred_draw.h>

// #include <time.h>
#include <Eigen/Core>
// #include <array>

using namespace cv;
using namespace std;
using namespace Eigen;

typedef struct
{
	int x;
	int y;
	int r;
} circle_xyr;

typedef struct
{
	float x;
	float y;
	float z;
} posXYZ;
// C++结构体可以舍弃typedef了

//定义hsv空间下的“绿色”的色彩范围
cv::Scalar green_lower = cv::Scalar(35, 83, 86);
cv::Scalar green_upper = cv::Scalar(77, 255, 255);

class Detector
{
private:
	const string topic_color_recv;
	const string topic_depth_recv;

	const string topic_ball_pose_pub;
	geometry_msgs::PointStamped point_in_plane;

	int detected_frame_count = 1;

	bool color_update_flag, depth_update_flag;
	bool you_can_detect_it;
	//这个参数没用上，后续有键盘响应时可能能用上
	bool isRunning;

	double elapsed = 0.0;

	ros::AsyncSpinner spinner;
	ros::NodeHandle nh;

	image_transport::ImageTransport it;

	image_transport::SubscriberFilter *subImageColor, *subImageDepth;
	message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

	// ros::Subscriber suber_color;
	// ros::Subscriber suber_depth;

	// 消息同步,严格时间戳对齐
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
	message_filters::Synchronizer<ExactSyncPolicy> *syncExact;

	std::mutex lock;

	cv::Mat color_frame, depth_frame;
	cv::Mat lookupX, lookupY;
	cv::Mat camera_Matrix_color, camera_Matrix_depth;

	ros::Publisher ball_pose_puber;

	std::chrono::time_point<std::chrono::high_resolution_clock> zero_time;
	std::chrono::time_point<std::chrono::high_resolution_clock> recved_color_time;
	ros::Time ros_zero_time;
	ros::Time ros_recved_color_time;

	Matrix4f hand_eye_calib;

	const size_t queueSize;

	bool get_camera_info;

	const float obj_pos_non_move_range;
	const float distance_to_wall;

	posXYZ last_pos;

public:
	Detector(const std::string &topic_color, const std::string &topic_depth, const std::string &topic_ball_pose_pub)
		: topic_color_recv(topic_color), topic_depth_recv(topic_depth), spinner(0), nh("~"),
		  color_update_flag(false), depth_update_flag(false), topic_ball_pose_pub(topic_ball_pose_pub), you_can_detect_it(false),
		  it(nh), queueSize(5), get_camera_info(true), obj_pos_non_move_range(0.01), distance_to_wall(3)
	{
		camera_Matrix_color = cv::Mat::zeros(3, 3, CV_64F);
		camera_Matrix_depth = cv::Mat::zeros(3, 3, CV_64F);
		last_pos.x = 0;
		last_pos.y = 0;
		last_pos.z = 0;
	}

	~Detector() {}

	void run()
	{
		start();
		stop();
	}

private:
	void start()
	{
		isRunning = true;

		ball_pose_puber = nh.advertise<geometry_msgs::PointStamped>(topic_ball_pose_pub, 10);

		std::string topicCameraInfoColor = topic_color_recv.substr(0, topic_color_recv.rfind('/')) + "/camera_info";
		std::string topicCameraInfoDepth = topic_depth_recv.substr(0, topic_depth_recv.rfind('/')) + "/camera_info";
		image_transport::TransportHints hints("compressed");
		//如果换成compressed会不会好一点，但是之前一直用的都是raw
		//这里为神码还要camerainfo呢
		subImageColor = new image_transport::SubscriberFilter(it, topic_color_recv, queueSize, hints);
		subImageDepth = new image_transport::SubscriberFilter(it, topic_depth_recv, queueSize, hints);
		subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
		subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

		// suber_color = nh.subscribe(topic_color_recv, 1, &Detector::callback_color, this);
		// suber_depth = nh.subscribe(topic_depth_recv, 1, &Detector::callback_depth, this);
		// ros::AsyncSpinner spinner(3); // Use 3 threads
		// spinner.start();

		//使用时间戳精准匹配模式
		syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
		syncExact->registerCallback(boost::bind(&Detector::callback_color_depth, this, _1, _2, _3, _4));

		spinner.start();

		std::chrono::milliseconds duration(1);
		while (!color_update_flag || !depth_update_flag)
		{
			if (!ros::ok())
				return;
			std::this_thread::sleep_for(duration);
		}
		//这里分别输入宽和高，即col和row
		createLookup(this->color_frame.cols, this->color_frame.rows);

		obj_detecting();
		ROS_INFO("obj_detecting stopped");
	}

	void stop()
	{

		ROS_INFO("stop...1");
		spinner.stop();
		isRunning = false;

		ROS_INFO("stop...2");
		delete subImageColor;
		delete subImageDepth;
		delete subCameraInfoColor;
		delete subCameraInfoDepth;

		ROS_INFO("stop...3");
		delete syncExact;
		ROS_INFO("stoped...");
	}

	void callback_color_depth(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
							  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
	{
		ROS_INFO("callback please check the timestamp");
		cv::Mat color, depth;
		// cout << "来了老弟～" << endl;
		//读取内参？
		//这两步没看懂，先不抄了，这个相机信息到这里就终结了，他获取了相机矩阵，然后lookup了
		//一会打印以下，看一看获取和不获取，信息是否一样
		//似乎depth的相机矩阵并没有用到，所以你出现的意义是什么
		//只在第一帧时获取一下，万一能节省点资源呢
		if (get_camera_info)
		{
			readCameraInfo(cameraInfoColor, camera_Matrix_color);
			readCameraInfo(cameraInfoDepth, camera_Matrix_depth);
			get_camera_info = false;
		}
		readImage(imageColor, color);
		readImage(imageDepth, depth);
		// cout << " type " << color.type() << endl;
		//他的type是16,但是没有找到16的define，懒得找了

		if (color.type() == CV_16U)
		{
			cv::Mat tmp;
			color.convertTo(tmp, CV_8U, 0.02);
			cv::cvtColor(tmp, color, CV_GRAY2BGR);
		}

		lock.lock();
		this->color_frame = color;
		this->depth_frame = depth;
		color_update_flag = true;
		depth_update_flag = true;
		// recved_color_time = std::chrono::high_resolution_clock::now();
		
		ros_recved_color_time=ros::Time::now();
		lock.unlock();
	}

	// 别问，问就是抄的
	void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
	{
		double *itC = cameraMatrix.ptr<double>(0, 0);
		for (size_t i = 0; i < 9; ++i, ++itC)
		{
			*itC = cameraInfo->K[i];
		}
	}

	void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
	{
		cv_bridge::CvImageConstPtr pCvImage;
		pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
		pCvImage->image.copyTo(image);
	}

	// //callback_color和callback_depth废弃了
	// void callback_color(const sensor_msgs::Image::ConstPtr image_data)
	// {
	// 	cout << "彩色" << endl;
	// 	// 抄的readImage里的
	// 	// 声明一个CvImage指针
	// 	cv::Mat color_frame_callback;
	// 	cv_bridge::CvImageConstPtr pCvImage;
	// 	//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
	// 	pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);
	// 	pCvImage->image.copyTo(color_frame_callback);
	// 	lock.lock();
	// 	this->color_frame = color_frame_callback.clone();
	// 	color_update_flag = true;
	// 	recved_color_time = std::chrono::high_resolution_clock::now();
	// 	lock.unlock();
	// 	// std::cout << "colorsrze" << (this->color_frame).rows << " " << (this->color_frame).cols << std::endl;
	// }

	// void callback_depth(const sensor_msgs::Image::ConstPtr image_data)
	// {
	// 	cv::Mat depth_frame_callback;
	// 	cv_bridge::CvImageConstPtr pCvImage;
	// 	pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);
	// 	pCvImage->image.copyTo(depth_frame_callback);
	// 	lock.lock();
	// 	this->depth_frame = depth_frame_callback;
	// 	depth_update_flag = true;
	// 	lock.unlock();
	// }

	bool obj_detecting()
	{

		hand_eye_calib = read_hand_eye_calib_file();

		//如果上边这个获取手眼标定文件的函数失效了，那就用这个比较朴素的方法
		// hand_eye_calib << -0.94731, 0.15769, 0.11761, -1134.9,
		// 	-0.098953, 0.13417, -0.97976, 1555.1,
		// 	-0.17015, -0.95443, -0.088936, 718.39,
		// 	0, 0, 0, 1;
		cv::Mat color_img, depth_img;
		while (ros::ok() && isRunning)
		{
			// posXYZ camera_coord;
			if (color_update_flag && depth_update_flag)
			{
				lock.lock();
				color_img = (this->color_frame).clone();
				depth_img = (this->depth_frame).clone();
				color_update_flag = false;
				depth_update_flag = false;
				you_can_detect_it = true;
				lock.unlock();

				//这里的if好像可以去掉了
				// you_can_detect_it = false;
				//得出网球的像素坐标的xy
				//这绝了，这第一次运行到这里，他会蹦出一个半径有400多万的，原点00的圆 4398358
				//这是一个弱智的错误
				circle_xyr obj_tennis_info = {0, 0, -1};

				obj_tennis_info = detect_tennis_by_color(color_img);
				// cout << "tennis info(x,y,r):" << obj_tennis_info.x << " " << obj_tennis_info.y << "  " << obj_tennis_info.r << endl;
				//这里应该有一步判断。如果半径大于1就识别到了,然后执行下列操作
				//其实这里不应该让他大于1,拿远一点试一试多少比较合适
				if (obj_tennis_info.r > 1 && obj_tennis_info.r < 400000)
				{
					if (detected_frame_count == 1)
					{
						cout << "firstle" << endl;
						// zero_time = std::chrono::high_resolution_clock::now();
						ros_zero_time = ros_recved_color_time;
						elapsed = 0;
					}
					else
					{
						// 下边两行暂时废弃
						// std::chrono::time_point<std::chrono::high_resolution_clock> now_time = std::chrono::high_resolution_clock::now();
						// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(recved_color_time - zero_time).count() / 1000.0;
						// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(recved_color_time - zero_time).count();
						// 先计算微秒，然后除以1000得出毫秒
						// elapsed = std::chrono::duration_cast<std::chrono::microseconds>(recved_color_time - zero_time).count() / 1000.0;
						elapsed=(ros::Time::now()-ros_zero_time).toSec();
					}
					cout << "elapsed is " << elapsed << "s" << endl;
					cout << "detected_frame_count: " << detected_frame_count++ << endl;

					posXYZ camera_coord;
					posXYZ world_coord;
					// 得出相机xyz坐标与世界xyz坐标
					camera_coord = get_depth_kinect(obj_tennis_info, depth_img);

					world_coord = coord_camera2world(camera_coord);

					// // 应该在这里把它发出去
					// obj_detected::ball_pose ball_pose_info;
					// //从相机坐标换到世界坐标了，单位米，先注释，不删吧
					// ball_pose_info.x = world_coord.x / 1000.0;
					// ball_pose_info.y = world_coord.y / 1000.0;
					// ball_pose_info.z = world_coord.z / 1000.0;
					// ball_pose_info.T = elapsed;
					// ball_pose_info.count = detected_frame_count;

					//判断pos的运动是否超出了范围
					bool isMoving = moving_or_stationary(world_coord);

					draw_obj_info(color_img, obj_tennis_info, camera_coord, world_coord, elapsed);
					if ((camera_coord.z > 0) && ((camera_coord.z / 1000.0) < distance_to_wall) && isMoving)
					{
						// 应该在这里把它发出去
						// obj_detected::ball_pose ball_pose_info;
						geometry_msgs::PointStamped ball_pose_info;
						//从相机坐标换到世界坐标了，单位米，先注释，不删吧
						ball_pose_info.point.x = world_coord.x / 1000.0;
						ball_pose_info.point.y = world_coord.y / 1000.0;
						ball_pose_info.point.z = world_coord.z / 1000.0;
						ball_pose_info.header.stamp = ros::Time().fromSec(elapsed);
						ball_pose_info.header.frame_id = detected_frame_count;
						ball_camera_pose_pub(ball_pose_info);
					}
					if (isMoving)
					{
						//在这画个图
					}
					cout << "--------------------------------------" << endl;
				}
				// else
				// 	vis = color_img;
				// imshow("visualization (Press Esc to exit)", color_img);
				// if (waitKey(1) == 27)
				// 	isRunning = false;
			}
		}
	}

	bool moving_or_stationary(const posXYZ &cur_pos)
	{
		bool ret = false;
		// if (detected_frame_count == 1)
		// {
		// 	last_pos = cur_pos;
		// 	ret = true;
		// }

		// 只要有一个方向上移动距离超出了范围，就认定他发生了移动
		// if (abs(last_pos.x - cur_pos.x) > (obj_pos_non_move_range * 1000) || abs(last_pos.y - cur_pos.y) > (obj_pos_non_move_range * 1000) || abs(last_pos.z - cur_pos.z) > (obj_pos_non_move_range * 1000))
		if (is_over_range(last_pos.x, cur_pos.x) || is_over_range(last_pos.y, cur_pos.y) || is_over_range(last_pos.z, cur_pos.z))
			ret = true;
		else
			ret = false;
		// 更新last值
		if (ret)
		{
			cout << "MOVING..." << ret << endl;
			last_pos = cur_pos;
		}
		return ret;
	}

	// 好像多此一举啊，并不是
	bool is_over_range(float last, float cur)
	{
		// 因为原始的世界坐标是毫米
		if (abs(cur - last) > obj_pos_non_move_range * 1000)
			return true;
		else
			return false;
	}

	circle_xyr detect_tennis_by_color(const cv::Mat &ColorImg)
	{
		clock_t start_time;
		start_time = clock();

		cv::Mat mask_tennis = pre_proc(ColorImg, green_lower, green_upper);

		//找轮廓
		vector<vector<cv::Point>> contours_green;
		cv::findContours(mask_tennis, contours_green, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		//找面积最大的边界，整个if0.001s，很神奇
		if (!contours_green.empty())
		{
			//此循环计算所有轮廓的面积
			vector<double> contours_green_area_vec = calc_area_by_contours(contours_green);
			cv::Point center;
			int radius = 0;

			find_max_area(&contours_green, &contours_green_area_vec, &center, &radius);
			cout << "center:" << center << " radius:" << radius << endl;

			cout << "detect once: " << (double)(clock() - start_time) / CLOCKS_PER_SEC << "s" << endl;
			return circle_xyr{center.x, center.y, radius};
		}
		else
			return circle_xyr{-1, -1, -1};
	}

	//抄来的函数
	void createLookup(size_t width, size_t height)
	{
		//内参矩阵这个样子
		// k << 263.525, 0, 240,
		// 	0, 263.425, 135,
		// // 	0, 0, 1;
		// const float fx = 1.0f / (263.525 * 2);
		// const float fy = 1.0f / (263.425 * 2);
		// const float cx = 480;
		// const float cy = 270;

		const float fx = 1.0f / camera_Matrix_color.at<double>(0, 0);
		const float fy = 1.0f / camera_Matrix_color.at<double>(1, 1);
		const float cx = camera_Matrix_color.at<double>(0, 2);
		const float cy = camera_Matrix_color.at<double>(1, 2);
		cout << "fx    is " << camera_Matrix_color.at<double>(0, 0) << endl;
		cout << "cx xy is " << cx << "  " << cy << endl;

		float *it;

		lookupY = cv::Mat(1, height, CV_32F);
		it = lookupY.ptr<float>();
		// 这里是在做长宽对应吗？
		//其中，r代表像素坐标,cxcy是图像中心点，
		for (size_t r = 0; r < height; ++r, ++it)
		{
			*it = (r - cy) * fy;
		}

		lookupX = cv::Mat(1, width, CV_32F);
		it = lookupX.ptr<float>();
		for (size_t c = 0; c < width; ++c, ++it)
		{
			*it = (c - cx) * fx;
		}
	}

	void draw_obj_info(cv::Mat &color_img, const circle_xyr &obj_tennis_info, const posXYZ &camera_coord, const posXYZ &world_coord, const double elapsed)
	{
		Point center(obj_tennis_info.x, obj_tennis_info.y);
		int radius = obj_tennis_info.r;
		if (radius > 1)
		{
			cv::circle(color_img, center, radius, cv::Scalar(0, 255, 255), 2);
			cv::circle(color_img, center, 5, cv::Scalar(0, 0, 255), -1);
		}
		else
			cout << "半径太小了" << endl;
		//显示彩色图中圆心坐标
		string text1 = "pixel (x,y),r (" + to_string(center.x) + "," + to_string(center.y) + ")" + "," + to_string(radius);
		cv::putText(color_img, text1, cv::Point(10, 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255));
		string text2 = "camera(x,y,z) (" + to_string(camera_coord.x / 1000.0) + "," + to_string(camera_coord.y / 1000.0) + "," + to_string(camera_coord.z / 1000.0) + ")";
		cv::putText(color_img, text2, cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255));
		string text3 = "world (x,y,z) (" + to_string(world_coord.x / 1000.0) + "," + to_string(world_coord.y / 1000.0) + "," + to_string(world_coord.z / 1000.0) + ")";
		cv::putText(color_img, text3, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255));
		string text4 = "time/s " + to_string(elapsed / 1000.0);
		cv::putText(color_img, text4, cv::Point(10, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 0, 255));
		// return color_img;
	}

	/*@ brief根据轮廓计算面积
	@param	contours 包含轮廓的vector
	@return area	 包含所有轮廓面积的vector
	*/
	vector<double> calc_area_by_contours(const vector<vector<cv::Point>> &contours)
	{
		if (!contours.empty())
		{
			vector<double> area;
			for (int i = 0; i < contours.size(); i++)
			{
				//这句可以优化成一句
				double temp_area;
				temp_area = cv::contourArea(contours[i]);
				area.push_back(temp_area);
			}
			return area;
		}
	}

	/*@brief 图像预处理
	@param	*frame		 等待预处理的帧，直接对其进行修改
	@param	color_lower  hsv的下界
	@param	color_upper	 hsv的上界
	@return mask		 二值图，在颜色范围内为1，否则0
	*/
	cv::Mat pre_proc(const cv::Mat &frame, const cv::Scalar &color_lower, const cv::Scalar &color_upper, bool isRed = false,
					 const cv::Scalar &red_lower = cv::Scalar(0, 0, 0), const cv::Scalar &red_upper = cv::Scalar(0, 0, 0))
	{
		cv::Mat blurred;
		cv::Mat hsv;
		cv::Mat mask;

		//把高斯核尺寸从11改为5，大概能节省0.004s，反正也能用。
		cv::GaussianBlur(frame, blurred, cv::Size(11, 11), 0);
		// cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
		cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
		//二值化，阈值内白色，其他黑色
		cv::inRange(hsv, color_lower, color_upper, mask);

		if (isRed)
		{ //有种奇技淫巧，把BGR当作RGB去处理，然后BR通道就互换，就可以用蓝色去识别
			cv::Mat mask_red_180;
			cv::inRange(hsv, red_lower, red_upper, mask_red_180);
			//下边这句好像有问题啊，神码情况，当时咋没发现，我服
			cv::addWeighted(mask, 1.0, mask, 1.0, 0.0, mask);
		}

		//经过试验，二者时间差不多，0.001s左右
		//事实上，不做这一步，结果好像也没啥差别，原文用的None，不知道为啥这样做
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		//cv::erode(mask, mask, element, cv::Point(-1, -1), 2);
		//cv::dilate(mask, mask, element, cv::Point(-1, -1), 2);
		cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element, cv::Point(-1, -1), 2);

		return mask;
	}

	/*@brief 找到最大面积，并从轮廓与面积两个vector中删除
	@param	*contours		待处理的边界
	@param	*contours_area  待处理的面积
	@param	*center			求得的当前最大面积最小包围圆的圆心
	@param	*radius			求得的当前最大面积最小包围圆的半径
	*/
	void find_max_area(vector<vector<cv::Point>> *contours, vector<double> *contours_area,
					   cv::Point *center, int *radius)
	{
		//总觉得这么写会出问题，但是，就这样吧
		if (!(*contours).empty())
		{
			cv::Point2f cen;
			float rad;
			//找面积最大的轮廓
			int max_index = max_element((*contours_area).begin(), (*contours_area).end()) - (*contours_area).begin();
			vector<cv::Point> max_contour = (*contours)[max_index];

			//阅后即焚，用完即删，现在没啥用
			(*contours_area).erase((*contours_area).begin() + max_index);
			(*contours).erase((*contours).begin() + max_index);

			//最小内接圆中心点和半径
			cv::minEnclosingCircle(max_contour, cen, rad);
			*center = cen;
			*radius = rad + 0.5;
		}
	}

	//获取识别到的圆心的深度信息
	posXYZ get_depth_kinect(const circle_xyr &obj_info, const cv::Mat &depth_img)
	{
		//单位毫米
		float depth_value = depth_img.at<uint16_t>(obj_info.y, obj_info.x);
		//将搜索的矩形扩大
		const int rect_radius = obj_info.r * 1;
		//找个范围内的×最小深度×值
		//被封印的少年
		//是不是应该右和下边界减1
		const int left = (obj_info.x - rect_radius) < 0 ? 0 : (obj_info.x - rect_radius);
		const int right = (obj_info.x + rect_radius) > depth_img.cols ? depth_img.cols - 1 : (obj_info.x + rect_radius);
		const int top = (obj_info.y - rect_radius) < 0 ? 0 : (obj_info.y - rect_radius);
		const int bottom = (obj_info.y + rect_radius) > depth_img.rows ? depth_img.rows - 1 : (obj_info.y + rect_radius);
		// cout << "it is" << left << " " << right << " " << top << " " << bottom << endl;
		int min_depth_value = depth_value;
		for (int col = left; col < right + 1; col++)
		{
			for (int row = top; row < bottom + 1; row++)
			{
				int temp_depth = depth_img.at<uint16_t>(row, col);
				// if ((depth_img.at<uint16_t>(row, col) < min_depth_value))
				if ((temp_depth < min_depth_value) && (temp_depth > 1))
				{
					min_depth_value = depth_img.at<uint16_t>(row, col);
				}
			}
		}

		cout << "center depthValue: " << depth_value << endl;
		cout << "min    depthValue: " << min_depth_value << endl;
		//××最小深度赋值××
		depth_value = min_depth_value;

		// 这一行深度数据是彩色数据的第几行
		const float y_pix = lookupY.at<float>(0, obj_info.y);
		//这个是为了便利每一行的内容
		const float x_pix = lookupX.at<float>(0, obj_info.x);

		posXYZ camera_coord;
		camera_coord.z = depth_value;
		camera_coord.x = x_pix * depth_value;
		camera_coord.y = y_pix * depth_value;
		// cout << "xyzis" << camera_coord.x
		// 	 << "  " << camera_coord.y
		// 	 << "  " << camera_coord.z << endl;
		return camera_coord;
	}

	posXYZ coord_camera2world(const posXYZ &camera_coord_xyz)
	{
		// cout << "123" << endl;

		MatrixXf camera_coord(4, 1);
		camera_coord << camera_coord_xyz.x, camera_coord_xyz.y, camera_coord_xyz.z, 1;

		cout << "camera_coord" << endl
			 << camera_coord << endl;
		//世界坐标计算，他应该是（4,1）的，但是如果不指定会怎样呢？
		MatrixXf world_coord(4, 1);
		world_coord = hand_eye_calib * camera_coord;
		// cout << "world_coord " << endl
		// 	 << world_coord << endl;

		posXYZ world_coord_xyz;
		world_coord_xyz.x = world_coord(0, 0);
		world_coord_xyz.y = world_coord(1, 0);
		world_coord_xyz.z = world_coord(2, 0);
		return world_coord_xyz;
	}

	Matrix4f read_hand_eye_calib_file()
	{
		ifstream read_file;
		string a;
		stringstream ss;
		//获取工作空间路径
		//绝了，这里getcwd直接就获取工作空间路径了？不是可执行文件路径？
		string cur_path;
		cur_path = getcwd(NULL, 0);
		// cout << "current path is " << cur_path << endl;
		// int prefix_pos = 0;
		// for (int i = 0; i < 3; i++)
		// {
		// 	prefix_pos = cur_path.find('/', prefix_pos + 1);
		// }
		// string wordspace_path;
		// wordspace_path = cur_path.substr(0, prefix_pos);
		// cout << "wordspace_path " << wordspace_path << endl;

		//手眼标定文件路径
		//我尼马！！！！！！！！！！！！！！服了
		// string hand_eye_calib_file_path = wordspace_path + "/src/obj_detect/hand_eye_calib.txt";
		string hand_eye_calib_file_path = cur_path + "/src/kinectdk_detected/hand_eye_calib.txt";
		cout << "手眼标定文件路径是：" << hand_eye_calib_file_path << endl;
		read_file.open(hand_eye_calib_file_path);
		if (!read_file)
		{
			cout << "ERROR：没有找到手眼标定文件" << endl;
			//第一次用上这个变量，没读到文件就直接停止
			isRunning = false;
			Matrix4f ERROR_RET;
			ERROR_RET << 0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0;
			return ERROR_RET;
		}

		string line;
		double data[16];
		int i = 0;
		//从文件中读取数据，并保存在data数组中
		while (getline(read_file, line))
		{
			stringstream ss;
			ss << line;
			while (!ss.eof())
			{
				ss >> data[i++];
			}
		}
		// for (int i = 0; i < 16; i++)
		// 	cout << data[i] << endl;
		read_file.close();
		//-_-为啥我要用矩阵，写他三个很长的式子不好吗，何必给自己找麻烦--__--
		//注意数据类型必须一致，否则会报错YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY

		Eigen::Matrix4f MF;
		//这种方式看起来好傻啊，不知道有没有其他办法
		MF << data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7],
			data[8], data[9], data[10], data[11],
			data[12], data[13], data[14], data[15];
		cout << "**check** hand eye calib is:" << endl
			 << MF << endl;
		return MF;
	}

	bool ball_camera_pose_pub(geometry_msgs::PointStamped pub_info)
	{
		ball_pose_puber.publish(pub_info);
	}

	// 		// Eigen::MatrixXd uv(3, 1);
	// 		// uv(0, 0) = z*center1.x;
	// 		// uv(1, 0) = z*center1.y;
	// 		// uv(2, 0) = z;

	// 		// Eigen::MatrixXd coord(3, 1);
	// 		// coord = k.inverse()*uv;
	// 		// cout << "相对于深度传感器(三个小红点的中心位置）的xyz（毫米为单位）"<<endl;

	// 		// cout << "整体时间为" << (double)(clock() - start_time) / CLOCKS_PER_SEC << "s" << endl;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obj_detected");
	if (!ros::ok())
		return 0;

	string topic_color = "/rgb/image_rect_color";
	string topic_depth = "/depth_to_rgb/image_raw";

	string topic_ball_pose_pub = "/obj_detect/tennis_pose";

	ROS_INFO("---recv topic color: %s ---\n", topic_color.c_str());
	ROS_INFO("---recv topic depth: %s ---\n", topic_depth.c_str());

	ROS_INFO("---publ topic world: %s ---\n", topic_ball_pose_pub.c_str());

	Detector detector(topic_color, topic_depth, topic_ball_pose_pub);

	ROS_INFO("starting...");

	detector.run();

	//这条waitForShutdown，似乎只有ctrl+c才能终结，不跑这句了
	// ros::waitForShutdown();
	ros::shutdown();
	return 0;
}
