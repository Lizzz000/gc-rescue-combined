#ifndef OC_SORT_CPP_KALMANBOXTRACKER_HPP
#define OC_SORT_CPP_KALMANBOXTRACKER_HPP
////////////// KalmanBoxTracker /////////////
#include "KalmanFilter.hpp"
#include "Utilities.hpp"
#include "iostream"
#include <atomic>
#include <memory>
/*
This class represents the internal state of individual
tracked objects observed as bbox.
*/


    class KalmanBoxTracker {
    public:
        /*method*/
        KalmanBoxTracker() {};
        KalmanBoxTracker(Eigen::VectorXf bbox_, int cls_, int delta_t_ = 3);
        void update(Eigen::Matrix<float, 5, 1>* bbox_, int cls_);
        Eigen::RowVectorXf predict();
        Eigen::VectorXf get_state();

    public:
        /*variable*/
        /* Process-global ID source; a single atomic fetch-add hands every tracker its own ID. */
        static std::atomic<int> count;
        Eigen::VectorXf bbox;// [5,1]
        std::unique_ptr<KalmanFilterNew> kf;
        int time_since_update;
        int id;
        std::vector<Eigen::VectorXf> history;
        int hits;
        int hit_streak;
        int age = 0;
        float conf;
        int cls;
        Eigen::RowVectorXf last_observation = Eigen::RowVectorXf::Zero(5);
        std::unordered_map<int, Eigen::VectorXf> observations;
        Eigen::RowVectorXf velocity = Eigen::RowVectorXf::Zero(2);// [2,1]
        int delta_t;
    };


#endif//OC_SORT_CPP_KALMANBOXTRACKER_HPP
