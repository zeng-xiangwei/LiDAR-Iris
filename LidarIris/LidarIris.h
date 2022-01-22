#ifndef _LIDAR_IRIS_H_
#define _LIDAR_IRIS_H_

#include <vector>
#include <flann/flann.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <queue>

class LidarIris
{
public:
    struct FeatureDesc
    {
        cv::Mat1b img;
        cv::Mat1b T;
        cv::Mat1b M;
    };

    LidarIris(int nscale, int minWaveLength, float mult, float sigmaOnf, int matchNum) : _nscale(nscale),
                                                                                         _minWaveLength(minWaveLength),
                                                                                         _mult(mult),
                                                                                         _sigmaOnf(sigmaOnf),
                                                                                         _matchNum(matchNum),
                                                                                         vecList(flann::Index<flann::L2<float>>(flann::KDTreeIndexParams(4)))
                                                                                         // indicesBuffer(std::vector<int>(matchNum)),
                                                                                         // distsBuffer(std::vector<float>(matchNum)),
                                                                                        //  indices(flann::Matrix<int>(new int[matchNum], 1, matchNum)),
                                                                                        //  dists(flann::Matrix<float>(new float[matchNum], 1, matchNum))
    {
    }
    LidarIris(const LidarIris &) = delete;
    LidarIris &operator=(const LidarIris &) = delete;

    template<typename PointType>
    static cv::Mat1b GetIris(const pcl::PointCloud<PointType> &cloud) 
    {
        // float lengthResolution = 1;
        // float length = 80;
        // float z_max = 15, z_min = -2;

        float lengthResolution = 0.3;
        float length = 30;
        float z_max = 2, z_min = -2;

        int len = length / lengthResolution;
        cv::Mat1b IrisMap = cv::Mat1b::zeros(len, 360);

        for (const PointType &p : cloud.points)
        {
            float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
            float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
            int Q_dis = std::min(std::max((int)floor(dis / lengthResolution), 0), len - 1);

            float height = (p.z - z_min) / (z_max - z_min) * 8.0;
            int Q_arc = std::min(std::max((int)floor(height), 0), 7);
            int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), 359);
            IrisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
        }
        
        return IrisMap;
    }

    //
    void UpdateFrame(const cv::Mat1b &frame, int frameIndex, float *matchDistance, int *matchIndex);
    //
    float Compare(const FeatureDesc &img1, const FeatureDesc &img2, int *bias = nullptr);
    //
    FeatureDesc GetFeature(const cv::Mat1b &src);
    FeatureDesc GetFeature(const cv::Mat1b &src, std::vector<float> &vec);
    std::vector<cv::Mat2f> LogGaborFilter(const cv::Mat1f &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf);
    void GetHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias);
    //
    static inline cv::Mat circRowShift(const cv::Mat &src, int shift_m_rows);
    static inline cv::Mat circColShift(const cv::Mat &src, int shift_n_cols);
    static cv::Mat circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols);

public:
    void LoGFeatureEncode(const cv::Mat1b &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf, cv::Mat1b &T, cv::Mat1b &M);

    int _nscale;
    int _minWaveLength;
    float _mult;
    float _sigmaOnf;
    int _matchNum;

    flann::Index<flann::L2<float>> vecList;
    std::vector<FeatureDesc> featureList;
    std::vector<int> frameIndexList;
    // flann::Matrix<int> indices;
    // flann::Matrix<float> dists;
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    // std::vector<int> indicesBuffer;
    // std::vector<float> distsBuffer;
    std::queue<flann::Matrix<float>> queriesBuffer;
    bool built_flann = false;
};

#endif
