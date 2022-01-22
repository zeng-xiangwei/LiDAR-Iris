#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "TicToc.h"
#include "LidarIris.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#define PINK "\033[35m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define BLUE "\033[34m"
#define END "\033[0m"

using namespace std;

// number of sequence
const int N = 2761;

// kitti sequence
const string seq = "05";

/*0 for kitti "00","05" only same direction loops;
1 for kitti "08" only reverse loops; 
2 for both same and reverse loops*/
const int loop_event = 0;
typedef pcl::PointXYZI pointT;

struct loopPair {
    int curID;
    int loopID;
    float fitscore;
    loopPair() : curID(-1), loopID(-1), fitscore(-1) {}
};

std::vector<vector<int>> getGTFromPose(const string& pose_path)
{
    std::ifstream pose_ifs(pose_path);
    std::string line;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    int index = 1;
    while(getline(pose_ifs, line)) 
    {
        if(line.empty()) break;
        stringstream ss(line);
        float r1,r2,r3,t1,r4,r5,r6,t2,r7,r8,r9,t3;
        ss >> r1 >> r2 >> r3 >> t1 >> r4 >> r5 >> r6 >> t2 >> r7 >> r8 >> r9 >> t3;
        pcl::PointXYZI p;
        p.x = t1;
        p.y = 0;
        p.z = t3;
        p.intensity = index++;
        cout << p << endl;
        cloud->push_back(p);
    }

    pcl::io::savePCDFileASCII(seq +".pcd", *cloud);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<vector<int>> res(5000);
    for(int i = 0; i < cloud->points.size(); i++)
    {
        float radius = 4;
        std::vector<int> ixi;
        std::vector<float> ixf;
        pcl::PointXYZI p = cloud->points[i];
        int cur = p.intensity;
        std::vector<int> nrs;
        kdtree.radiusSearch(p,radius,ixi,ixf);
        for(int j = 0; j < ixi.size(); j++)
        {
            if(cloud->points[ixi[j]].intensity == cur) continue;
            nrs.push_back(cloud->points[ixi[j]].intensity);
        }
        sort(nrs.begin(), nrs.end());
        res[cur] = nrs;
    }
    
    std::ofstream gt_ofs("../gt"+ seq + ".txt");

    for(int i =0; i < res.size(); i++)
    {
        gt_ofs << i << " ";
        for(int j = 0; j < res[i].size(); j++)
        {
            gt_ofs << res[i][j] << " ";
        }
        gt_ofs << endl;
    }
    return res;
}

void DemoTest() {
    LidarIris iris(4, 18, 1.6, 0.75, 50);
    std::vector<LidarIris::FeatureDesc> dataset(N);
     //kitti pose xx.txt
    auto gt = getGTFromPose("/media/yingwang/Document/" +seq + "/"+seq +".txt");
    std::ofstream ofs("../test_res" + seq+".txt");

    for(int i =0; i <=N-1 ;i++)
    {
        std::stringstream ss;
        ss << setw(6) << setfill('0') << i;
        cout << ss.str()+".bin" << std::endl;

        // kitti velodyne bins
        std::string filename = "/media/yingwang/Document/" + seq + "/velodyne/" + ss.str() + ".bin";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
        std::fstream input(filename, std::ios::in | std::ios::binary);
        input.seekg(0, std::ios::beg);
        for (int ii=0; input.good() && !input.eof(); ii++) {
            pcl::PointXYZ point;
            input.read((char *) &point.x, 3*sizeof(float));
            float intensity;
            input.read((char *) &intensity, sizeof(float));
            cloud0->push_back(point);
        }
        cv::Mat1b li1 = LidarIris::GetIris(*cloud0);
        LidarIris::FeatureDesc fd1 = iris.GetFeature(li1);
        dataset[i] = fd1;
        float mindis = 1000;
        int loop_id = -1;
        for(int j = 0; j <= i-300; j++)
        {
            LidarIris::FeatureDesc fd2 = dataset[j];

            int bias;
            auto dis = iris.Compare(fd1, fd2, &bias);
            if(dis < mindis)
            {
                mindis = dis;
                loop_id = j;
            }


        }

        if(loop_id == -1) continue;
        // if(std::find(gt[i+1].begin(),gt[i+1].end(),loop_id+1)!=gt[i+1].end())
        // {
        //     ofs << i+1 << " " << loop_id+1 << " " << mindis << " " << 1 << std::endl; 
        //     cout << i+1 << " " << loop_id+1 << " " << mindis << " " << 1 << std::endl; 
        // }
        // else 
        // {
        //     ofs << i+1 << " " << loop_id+1 << " " << mindis << " " << 0 << std::endl; 
        //     cout << i+1 << " " << loop_id+1 << " " << mindis << " " << 0 << std::endl; 

        // }


    }
}

void testWithBag() {
    LidarIris iris(4, 18, 1.6, 0.75, 50);

    std::string path = "/home/dut-zxw/zxw/data_bag/noRotate/yanjiaolou2017-04-05-10-12-07.bag";
    std::shared_ptr<rosbag::Bag> bag_;
	bag_.reset(new rosbag::Bag);
	bag_->open(path, rosbag::bagmode::Read);

	rosbag::View view;
	view.addQuery(*bag_);

    std::string lidar_topic = "/velodyne_points";
    int cur_id = 0;
    int skip_count = 0;
	for (rosbag::MessageInstance const m : view) {
		const std::string &topic = m.getTopic();

		if (lidar_topic == topic) {
            skip_count++;
            if (skip_count % 3 != 0) continue;
            
            TicToc tic_toc;
			pcl::PointCloud<pointT>::Ptr pointcloud = pcl::make_shared<pcl::PointCloud<pointT>>();

			if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
				sensor_msgs::PointCloud2::ConstPtr scan_msg =
						m.instantiate<sensor_msgs::PointCloud2>();
				pcl::fromROSMsg(*scan_msg, *pointcloud);
			}

            // irisMag.makeAndSaveIrisAndKey<pointT>(pointcloud);
            // std::cout << "makeAndSaveIrisAndKey time: " << tic_toc.Toc() << " ms\n";
            // int loopID = -1;
            // // irisMag.detectLoopClosureID(cur_id, loopID);
            // float mindis = 1000;
            // loopID = -1;
            // LidarIris::FeatureDesc &fd1 = irisMag.historyFeatureDesc[cur_id];
            // for(int j = 0; j < cur_id - config.ignoreFrame; j++) {
            //     LidarIris::FeatureDesc &fd2 = irisMag.historyFeatureDesc[j];
            //     int bias;
            //     auto dis = irisMag.iris->Compare(fd1, fd2, &bias);
            //     if(dis < config.threshold && dis < mindis) {
            //         mindis = dis;
            //         loopID = j;
            //     }
            // }

            cv::Mat1b li1 = LidarIris::GetIris(*pointcloud);
            float mindis = 1000;
            int loopID = -1;
            iris.UpdateFrame(li1, cur_id, &mindis, &loopID);

            if (loopID == -1) {
                std::cout << PINK << "not loop" << END << std::endl;
            } else {
                std::cout << GREEN << "loop ID: " << loopID << END << std::endl;
            }
            std::cout << "\033[34m curID: " << cur_id << "  loopID: " << loopID << "  min distance: " << mindis << "\033[0m" << std::endl;

            std::cout << "time: " << tic_toc.Toc() << " ms\n\n";

            cur_id++;
		}
	}
}

void testTwoPCD() {
    LidarIris iris(4, 18, 1.6, 0.75, 50);
    pcl::PCDReader pcd_reader;
    pcl::PointCloud<pointT> cloud0, cloud1;
    std::string pcd1, pcd2;
    pcd1 = "/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris-1-master/data/64line_1.pcd";
    pcd2 = "/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris-1-master/data/64line_2.pcd";
    pcd_reader.read<pointT>(pcd1, cloud0);
    pcd_reader.read<pointT>(pcd2, cloud1);

    TicToc tic_toc;
    cv::Mat1b li1 = LidarIris::GetIris<pointT>(cloud0);
    cv::Mat1b li2 = LidarIris::GetIris<pointT>(cloud1);

    LidarIris::FeatureDesc fd1 = iris.GetFeature(li1);
    LidarIris::FeatureDesc fd2 = iris.GetFeature(li2);

    int bias;
    auto dis = iris.Compare(fd1, fd2, &bias);

    cv::Mat1b img_iris, img_T, img_M;
    cv::Mat im_color;
    cv::vconcat(fd1.img, fd2.img, img_iris);
    cv::imshow("LiDAR Iris before transformation", img_iris);
    // cv::imwrite("../img/before.bmp", img_iris);
    cv::applyColorMap(img_iris, im_color, cv::COLORMAP_JET);
    cv::imshow("LiDAR Iris before transformation color", im_color);
    
    cv::Mat temp = LidarIris::circShift(fd1.img, 0, bias);
    cv::vconcat(temp, fd2.img, img_iris);
    cv::imshow("LiDAR Iris after transformation", img_iris);
    // cv::imwrite("../img/after.bmp", img_iris);
    cv::applyColorMap(img_iris, im_color, cv::COLORMAP_JET);
    cv::imshow("LiDAR Iris after transformation color", im_color);

    cv::hconcat(fd1.T, fd2.T, img_T);
    cv::imshow("LiDAR Iris Template", img_T);
    // cv::imwrite("../img/temp.bmp", img_T);
    cv::applyColorMap(img_T, im_color, cv::COLORMAP_JET);
    cv::imshow("LiDAR Iris Template color", im_color);

    cv::hconcat(fd1.M, fd2.M, img_M);
    cv::imshow("LiDAR Iris M", img_M);
    cv::applyColorMap(img_M, im_color, cv::COLORMAP_JET);
    cv::imshow("LiDAR Iris M color", im_color);

    cv::waitKey(0);
}

void testWithGroundTruth() {
    LidarIris iris(4, 18, 1.6, 0.75, 50);
    std::vector<Eigen::Vector3f> gtPoses;
    std::ifstream infile;
    std::ofstream outfile;
    outfile.open("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/shineifengbi/keyposesID.txt");
    infile.open("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/shineifengbi/keyposes.txt");
    std::string line;
    int count = 0;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f pose;
        ss >> pose.x() >> pose.y() >> pose.z();
        gtPoses.push_back(pose);
        outfile << pose.x() << " "<< pose.y() << " " << pose.z() << " " << count << std::endl;
        count++;
    }
    infile.close();
    outfile.close();
    std::cout << "gt poses size: " << gtPoses.size() << std::endl;

    int cur_id;
    std::string dir("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/shineifengbi/");
    for (int i = 1; i < 1150; i += 2) {
        std::string pcd_dir = dir + std::to_string(i) + ".pcd";
        pcl::PCDReader pcd_reader;
        pcl::PointCloud<pointT> pc;
        pcd_reader.read<pointT>(pcd_dir, pc);

        TicToc tic_toc;
        cv::Mat1b li1 = LidarIris::GetIris(pc);
        float mindis = 1000;
        int loopID = -1;
        cur_id = i;
        iris.UpdateFrame(li1, cur_id, &mindis, &loopID);

        if (cur_id == 981) {
            int loop_bias;
            int loop_tmp_id;
            for(int j = 0; j < iris.featureList.size() - 150; j++) {
                LidarIris::FeatureDesc &fd2 = iris.featureList[j];
                int bias;
                auto dis = iris.Compare(iris.featureList.back(), fd2, &bias);
                if(dis < mindis) {
                    mindis = dis;
                    loopID = iris.frameIndexList[j];
                    loop_bias = bias;
                    loop_tmp_id = j;
                }
            }

            bool findLoop = false;
            if (loopID == -1 || mindis >= 0.3) {
                std::cout << PINK << "not loop" << END << std::endl;
                findLoop = false;
            } else {
                std::cout << GREEN << "loop ID: " << loopID << END << std::endl;
                findLoop = true;
            }
            std::cout << "\033[34m curID: " << cur_id << "  loopID: " << loopID << "  min H distance: " << mindis << "\033[0m" << std::endl;
            
            Eigen::Vector3f gtpose_loop, gtpose_cur;
            int tmp = loopID == -1 ? 0 : loopID;
            gtpose_loop = gtPoses[tmp];
            gtpose_cur = gtPoses[cur_id];
            float gt_dis = (gtpose_cur - gtpose_loop).norm();
            std::cout << "cur pose: " << gtpose_cur.transpose() << "  loop pose: " << gtpose_loop.transpose() << "  distance: " << gt_dis << std::endl;
            if (gt_dis <= 3 && findLoop) {
                std::cout << GREEN << "true loop" << END << std::endl;
            } else if (gt_dis > 3 && findLoop) {
                std::cout << RED << "false loop" << END << std::endl;
            }

            std::cout << "time: " << tic_toc.Toc() << " ms\n\n";

            auto fd1 = iris.featureList.back();
            auto fd2 = iris.featureList[loop_tmp_id];
            cv::Mat1b img_iris, img_T, img_M;
            cv::Mat im_color;
            cv::vconcat(fd1.img, fd2.img, img_iris);
            cv::imshow("LiDAR Iris before transformation", img_iris);
            // cv::imwrite("../img/before.bmp", img_iris);
            cv::applyColorMap(img_iris, im_color, cv::COLORMAP_JET);
            cv::imshow("LiDAR Iris before transformation color", im_color);
            
            cv::Mat temp = LidarIris::circShift(fd1.img, 0, loop_bias);
            cv::vconcat(temp, fd2.img, img_iris);
            cv::imshow("LiDAR Iris after transformation", img_iris);
            // cv::imwrite("../img/after.bmp", img_iris);
            cv::applyColorMap(img_iris, im_color, cv::COLORMAP_JET);
            cv::imshow("LiDAR Iris after transformation color", im_color);

            cv::hconcat(fd1.T, fd2.T, img_T);
            cv::imshow("LiDAR Iris Template", img_T);
            // cv::imwrite("../img/temp.bmp", img_T);
            cv::applyColorMap(img_T, im_color, cv::COLORMAP_JET);
            cv::imshow("LiDAR Iris Template color", im_color);

            cv::hconcat(fd1.M, fd2.M, img_M);
            cv::imshow("LiDAR Iris M", img_M);
            cv::applyColorMap(img_M, im_color, cv::COLORMAP_JET);
            cv::imshow("LiDAR Iris M color", im_color);

            cv::waitKey(0);
        }
    }
}

void testIrisAndFLANNWithGroundTruth() {
    LidarIris iris(4, 18, 1.6, 0.75, 50);
    std::vector<Eigen::Vector3f> gtPoses;
    std::ifstream infile;
    std::ofstream outfile;
    outfile.open("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/guoyuan/keyposesID.txt");
    infile.open("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/guoyuan/keyposes.txt");
    std::string line;
    int count = 0;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f pose;
        ss >> pose.x() >> pose.y() >> pose.z();
        gtPoses.push_back(pose);
        outfile << pose.x() << " "<< pose.y() << " " << pose.z() << " " << count << std::endl;
        count++;
    }
    infile.close();
    outfile.close();
    std::cout << "gt poses size: " << gtPoses.size() << std::endl;

    int cur_id;
    std::string dir("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/guoyuan/");
    std::vector<loopPair> trueLoops, falseLoops;
    for (int i = 1; i < 1824; i += 2) {
        std::string pcd_dir = dir + std::to_string(i) + ".pcd";
        pcl::PCDReader pcd_reader;
        pcl::PointCloud<pointT> pc;
        pcd_reader.read<pointT>(pcd_dir, pc);

        TicToc tic_toc;
        cv::Mat1b li1 = LidarIris::GetIris(pc);
        float mindis = 1000;
        int loopID = -1;
        cur_id = i;
        iris.UpdateFrame(li1, cur_id, &mindis, &loopID);
        
        int loop_bias;

        bool findLoop = false;
        float hmThreshold = 0.38;
        if (loopID == -1 || mindis >= hmThreshold) {
            std::cout << PINK << "not loop" << END << std::endl;
            findLoop = false;
        } else {
            std::cout << GREEN << "loop ID: " << loopID << END << std::endl;
            findLoop = true;
        }
        std::cout << "\033[34m curID: " << cur_id << "  loopID: " << loopID << "  min H distance: " << mindis << "\033[0m" << std::endl;
        
        Eigen::Vector3f gtpose_loop, gtpose_cur;
        int tmp = loopID == -1 ? 0 : loopID;
        gtpose_loop = gtPoses[tmp];
        gtpose_cur = gtPoses[cur_id];
        float gt_dis = (gtpose_cur - gtpose_loop).norm();
        std::cout << "cur pose: " << gtpose_cur.transpose() << "  loop pose: " << gtpose_loop.transpose() << "  distance: " << gt_dis << std::endl;
        
        double disThreshold = 3;
        if (gt_dis <= disThreshold && findLoop) {
            std::cout << GREEN << "true loop" << END << std::endl;
            loopPair trLoop;
            trLoop.curID = cur_id;
            trLoop.loopID = loopID;
            trLoop.fitscore = mindis;
            trueLoops.push_back(trLoop);

        } else if (gt_dis > disThreshold && findLoop) {
            std::cout << RED << "false loop" << END << std::endl;
            loopPair falLoop;
            falLoop.curID = cur_id;
            falLoop.loopID = loopID;
            falLoop.fitscore = mindis;
            falseLoops.push_back(falLoop);
        }

        std::cout << "time: " << tic_toc.Toc() << " ms\n\n";

        // auto fd1 = iris.featureList.back();
        // auto fd2 = iris.featureList[loop_tmp_id];
        // cv::Mat1b img_iris, img_T, img_M;
        // cv::Mat im_color;
        // cv::vconcat(fd1.img, fd2.img, img_iris);
        // cv::imshow("LiDAR Iris before transformation", img_iris);
        // // cv::imwrite("../img/before.bmp", img_iris);
        // cv::applyColorMap(img_iris, im_color, cv::COLORMAP_JET);
        // cv::imshow("LiDAR Iris before transformation color", im_color);
        
        // cv::Mat temp = LidarIris::circShift(fd1.img, 0, loop_bias);
        // cv::vconcat(temp, fd2.img, img_iris);
        // cv::imshow("LiDAR Iris after transformation", img_iris);
        // // cv::imwrite("../img/after.bmp", img_iris);
        // cv::applyColorMap(img_iris, im_color, cv::COLORMAP_JET);
        // cv::imshow("LiDAR Iris after transformation color", im_color);

        // cv::hconcat(fd1.T, fd2.T, img_T);
        // cv::imshow("LiDAR Iris Template", img_T);
        // // cv::imwrite("../img/temp.bmp", img_T);
        // cv::applyColorMap(img_T, im_color, cv::COLORMAP_JET);
        // cv::imshow("LiDAR Iris Template color", im_color);

        // cv::hconcat(fd1.M, fd2.M, img_M);
        // cv::imshow("LiDAR Iris M", img_M);
        // cv::applyColorMap(img_M, im_color, cv::COLORMAP_JET);
        // cv::imshow("LiDAR Iris M color", im_color);

        // cv::waitKey(0);
    }

    std::cout << GREEN << "true loops: " << END << std::endl;
    for (auto tr : trueLoops) {
        std::cout << "cur_id: " << tr.curID << "  loop_id: " << tr.loopID << "  mindist: " << tr.fitscore << std::endl;
    }
    std::cout << RED << "false loops: " << END << std::endl;
    for (auto fa : falseLoops) {
        std::cout << "cur_id: " << fa.curID << "  loop_id: " << fa.loopID << "  mindist: " << fa.fitscore << std::endl;
    }
    
}

void testSeletedFrames() {
    LidarIris iris(4, 18, 1.6, 0.75, 50);
    std::vector<Eigen::Vector3f> gtPoses;
    std::ifstream infile;
    std::ofstream outfile;
    outfile.open("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/shineifengbi/keyposesID.txt");
    infile.open("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/shineifengbi/keyposes.txt");
    std::string line;
    int count = 0;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        Eigen::Vector3f pose;
        ss >> pose.x() >> pose.y() >> pose.z();
        gtPoses.push_back(pose);
        outfile << pose.x() << " "<< pose.y() << " " << pose.z() << " " << count << std::endl;
        count++;
    }
    infile.close();
    outfile.close();
    std::cout << "gt poses size: " << gtPoses.size() << std::endl;

    int cur_id;
    std::string dir("/home/dut-zxw/zxw/test_any/lidar-iris/LiDAR-Iris/pcd/shineifengbi/");
    std::vector<loopPair> trueLoops, falseLoops;

    pcl::PointCloud<pointT>::Ptr featurePoses(new pcl::PointCloud<pointT>());
    pcl::KdTreeFLANN<pointT>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<pointT>());

    for (int i = 1; i < 1150; i += 2) {
        std::string pcd_dir = dir + std::to_string(i) + ".pcd";
        pcl::PCDReader pcd_reader;
        pcl::PointCloud<pointT> pc;
        pcd_reader.read<pointT>(pcd_dir, pc);

        TicToc tic_toc;
        cv::Mat1b li1 = LidarIris::GetIris(pc);
        float mindis = 1000;
        int loopID = -1;
        cur_id = i;
        iris.UpdateFrame(li1, cur_id, &mindis, &loopID);
        pointT cur_point;
        cur_point.x = gtPoses[i].x();
        cur_point.y = gtPoses[i].y();
        cur_point.z = gtPoses[i].z();
        cur_point.intensity = featurePoses->points.size();
        featurePoses->points.push_back(cur_point);
        
        int loop_bias;

        pcl::PointCloud<pointT>::Ptr surroundingKeyPoses(new pcl::PointCloud<pointT>());
        pcl::PointCloud<pointT>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<pointT>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(featurePoses); // create kd-tree
        double surroundingKeyframeSearchRadius = 3;
        kdtreeSurroundingKeyPoses->radiusSearch(featurePoses->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int k = 0; k < (int)pointSearchInd.size(); ++k)
        {
            int id = pointSearchInd[k];
            if (cur_id - iris.frameIndexList[id] < 100) {
                continue;
            }

            LidarIris::FeatureDesc &fd2 = iris.featureList[id];
            int bias;
            auto dis = iris.Compare(iris.featureList.back(), fd2, &bias);
            if(dis < mindis) {
                mindis = dis;
                loopID = iris.frameIndexList[id];
                loop_bias = bias;
                // loopID = j;
            }
        }

        bool findLoop = false;
        float hmThreshold = 0.38;
        if (loopID == -1 || mindis >= hmThreshold) {
            std::cout << PINK << "not loop" << END << std::endl;
            findLoop = false;
        } else {
            std::cout << GREEN << "loop ID: " << loopID << END << std::endl;
            findLoop = true;
        }
        std::cout << "\033[34m curID: " << cur_id << "  loopID: " << loopID << "  min H distance: " << mindis << "\033[0m" << std::endl;
        
        Eigen::Vector3f gtpose_loop, gtpose_cur;
        int tmp = loopID == -1 ? 0 : loopID;
        gtpose_loop = gtPoses[tmp];
        gtpose_cur = gtPoses[cur_id];
        float gt_dis = (gtpose_cur - gtpose_loop).norm();
        std::cout << "cur pose: " << gtpose_cur.transpose() << "  loop pose: " << gtpose_loop.transpose() << "  distance: " << gt_dis << std::endl;
        
        double disThreshold = 3;
        if (gt_dis <= disThreshold && findLoop) {
            std::cout << GREEN << "true loop" << END << std::endl;
            loopPair trLoop;
            trLoop.curID = cur_id;
            trLoop.loopID = loopID;
            trLoop.fitscore = mindis;
            trueLoops.push_back(trLoop);

        } else if (gt_dis > disThreshold && findLoop) {
            std::cout << RED << "false loop" << END << std::endl;
            loopPair falLoop;
            falLoop.curID = cur_id;
            falLoop.loopID = loopID;
            falLoop.fitscore = mindis;
            falseLoops.push_back(falLoop);
        }

        std::cout << "time: " << tic_toc.Toc() << " ms\n\n";
    }

    std::cout << GREEN << "true loops: " << END << std::endl;
    for (auto tr : trueLoops) {
        std::cout << "cur_id: " << tr.curID << "  loop_id: " << tr.loopID << "  mindist: " << tr.fitscore << std::endl;
    }
    std::cout << RED << "false loops: " << END << std::endl;
    for (auto fa : falseLoops) {
        std::cout << "cur_id: " << fa.curID << "  loop_id: " << fa.loopID << "  mindist: " << fa.fitscore << std::endl;
    }
    
}

int main(int argc, char *argv[])
{
    // testIrisAndFLANNWithGroundTruth();
    testSeletedFrames();
    return 0;
}