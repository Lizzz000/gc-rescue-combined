#include "Association.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>


    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> speed_direction_batch(const Eigen::MatrixXf& dets,
        const Eigen::MatrixXf& tracks) {
        Eigen::VectorXf CX1 = (dets.col(0) + dets.col(2)) / 2.0;
        Eigen::VectorXf CY1 = (dets.col(1) + dets.col(3)) / 2.f;
        Eigen::MatrixXf CX2 = (tracks.col(0) + tracks.col(2)) / 2.f;
        Eigen::MatrixXf CY2 = (tracks.col(1) + tracks.col(3)) / 2.f;
        Eigen::MatrixXf dx = CX1.transpose().replicate(tracks.rows(), 1) - CX2.replicate(1, dets.rows());
        Eigen::MatrixXf dy = CY1.transpose().replicate(tracks.rows(), 1) - CY2.replicate(1, dets.rows());
        Eigen::MatrixXf norm = (dx.array().square() + dy.array().square()).sqrt() + 1e-6f;
        dx = dx.array() / norm.array();
        dy = dy.array() / norm.array();
        return std::make_tuple(dy, dx);
    }
    Eigen::MatrixXf iou_batch(const Eigen::MatrixXf& bboxes1, const Eigen::MatrixXf& bboxes2) {
        Eigen::Matrix<float, Eigen::Dynamic, 1> a = bboxes1.col(0);// bboxes1[..., 0] (n1,1)
        Eigen::Matrix<float, 1, Eigen::Dynamic> b = bboxes2.col(0);// bboxes2[..., 0] (1,n2)
        Eigen::MatrixXf xx1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
        a = bboxes1.col(1);// bboxes1[..., 1]
        b = bboxes2.col(1);// bboxes2[..., 1]
        Eigen::MatrixXf yy1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
        a = bboxes1.col(2);// bboxes1[..., 2]
        b = bboxes2.col(2);// bboxes1[..., 2]
        Eigen::MatrixXf xx2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
        a = bboxes1.col(3);// bboxes1[..., 3]
        b = bboxes2.col(3);// bboxes1[..., 3]
        Eigen::MatrixXf yy2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
        Eigen::MatrixXf w = (xx2 - xx1).cwiseMax(0);
        Eigen::MatrixXf h = (yy2 - yy1).cwiseMax(0);
        Eigen::MatrixXf wh = w.array() * h.array();
        a = (bboxes1.col(2) - bboxes1.col(0)).array() * (bboxes1.col(3) - bboxes1.col(1)).array();
        b = (bboxes2.col(2) - bboxes2.col(0)).array() * (bboxes2.col(3) - bboxes2.col(1)).array();
        Eigen::MatrixXf part1_ = a.replicate(1, b.cols());
        Eigen::MatrixXf part2_ = b.replicate(a.rows(), 1);
        Eigen::MatrixXf Sum = part1_ + part2_ - wh;
        return wh.cwiseQuotient(Sum);
    }


    Eigen::MatrixXf giou_batch(const Eigen::MatrixXf& bboxes1, const Eigen::MatrixXf& bboxes2) {
        Eigen::Matrix<float, Eigen::Dynamic, 1> a = bboxes1.col(0);// bboxes1[..., 0] (n1,1)
        Eigen::Matrix<float, 1, Eigen::Dynamic> b = bboxes2.col(0);// bboxes2[..., 0] (1,n2)
        Eigen::MatrixXf xx1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
        a = bboxes1.col(1);// bboxes1[..., 1]
        b = bboxes2.col(1);// bboxes2[..., 1]
        Eigen::MatrixXf yy1 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
        a = bboxes1.col(2);// bboxes1[..., 2]
        b = bboxes2.col(2);// bboxes1[..., 2]
        Eigen::MatrixXf xx2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
        a = bboxes1.col(3);// bboxes1[..., 3]
        b = bboxes2.col(3);// bboxes1[..., 3]
        Eigen::MatrixXf yy2 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
        Eigen::MatrixXf w = (xx2 - xx1).cwiseMax(0);
        Eigen::MatrixXf h = (yy2 - yy1).cwiseMax(0);
        Eigen::MatrixXf wh = w.array() * h.array();
        a = (bboxes1.col(2) - bboxes1.col(0)).array() * (bboxes1.col(3) - bboxes1.col(1)).array();
        b = (bboxes2.col(2) - bboxes2.col(0)).array() * (bboxes2.col(3) - bboxes2.col(1)).array();
        Eigen::MatrixXf part1_ = a.replicate(1, b.cols());
        Eigen::MatrixXf part2_ = b.replicate(a.rows(), 1);
        Eigen::MatrixXf Sum = part1_ + part2_ - wh;
        Eigen::MatrixXf iou = wh.cwiseQuotient(Sum);

        a = bboxes1.col(0);
        b = bboxes2.col(0);
        Eigen::MatrixXf xxc1 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
        a = bboxes1.col(1);// bboxes1[..., 1]
        b = bboxes2.col(1);// bboxes2[..., 1]
        Eigen::MatrixXf yyc1 = (a.replicate(1, b.cols())).cwiseMin(b.replicate(a.rows(), 1));
        a = bboxes1.col(2);// bboxes1[..., 2]
        b = bboxes2.col(2);// bboxes1[..., 2]
        Eigen::MatrixXf xxc2 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));
        a = bboxes1.col(3);// bboxes1[..., 3]
        b = bboxes2.col(3);// bboxes1[..., 3]
        Eigen::MatrixXf yyc2 = (a.replicate(1, b.cols())).cwiseMax(b.replicate(a.rows(), 1));

        Eigen::MatrixXf wc = xxc2 - xxc1;
        Eigen::MatrixXf hc = yyc2 - yyc1;
        Eigen::MatrixXf area_enclose = wc.array() * hc.array();
        Eigen::MatrixXf giou = iou.array() - (area_enclose.array() - Sum.array()) / area_enclose.array();
        giou = (giou.array() + 1) / 2.0;
        return giou;
    }

    std::tuple<std::vector<Eigen::Matrix<int, 1, 2>>, std::vector<int>, std::vector<int>> associate(const Eigen::MatrixXf& detections, const Eigen::MatrixXf& trackers, float iou_threshold, const Eigen::MatrixXf& velocities, const Eigen::MatrixXf& previous_obs_, float vdc_weight) {
        if (detections.rows() == 0) {
            std::vector<int> unmatched;
            for (int i = 0; i < trackers.rows(); ++i) unmatched.push_back(i);
            return {std::vector<Eigen::Matrix<int, 1, 2>>{}, std::vector<int>{}, unmatched};
        }
        if (trackers.rows() == 0) {
            std::vector<int> unmatched_dets;
            for (int i = 0; i < detections.rows(); i++) {
                unmatched_dets.push_back(i);
            }
            return std::make_tuple(std::vector<Eigen::Matrix<int, 1, 2>>(), unmatched_dets, std::vector<int>());
        }
        Eigen::MatrixXf iou_matrix = iou_batch(detections, trackers);
        std::vector<std::pair<int,int>> matched_indices_vec;
        if (std::min(iou_matrix.cols(), iou_matrix.rows()) > 0) {
            Eigen::MatrixXf a = (iou_matrix.array() > iou_threshold).cast<float>();
            float sum1 = (a.rowwise().sum()).maxCoeff();
            float sum0 = (a.colwise().sum()).maxCoeff();
            if ((fabs(sum1 - 1) < 1e-12) && (fabs(sum0 - 1) < 1e-12)) {
                for (int i = 0; i < a.rows(); i++) {
                    for (int j = 0; j < a.cols(); j++) {
                        if (a(i, j) > 0) {
                            matched_indices_vec.push_back({i, j});
                        }
                    }
                }
            }
            else {
                // Unique IoU pairs need no direction matrices or trigonometry.
                // Build that cost only for ambiguous assignment, and skip it
                // completely when motion inertia is disabled.
                Eigen::MatrixXf cost_matrix = iou_matrix;
                if (vdc_weight != 0.0f) {
                    const auto directions = speed_direction_batch(detections, previous_obs_);
                    const auto& Y = std::get<0>(directions);
                    const auto& X = std::get<1>(directions);
                    for (int track = 0; track < trackers.rows(); ++track) {
                        if (previous_obs_(track, 4) < 0) continue;
                        for (int det = 0; det < detections.rows(); ++det) {
                            const float cosine = std::clamp(
                                velocities(track, 1) * X(track, det) +
                                velocities(track, 0) * Y(track, det), -1.0f, 1.0f);
                            const float angle_cost = (pi / 2.0f - std::acos(cosine)) / pi;
                            cost_matrix(det, track) += angle_cost * vdc_weight * detections(det, 4);
                        }
                    }
                }
                std::vector<std::vector<float>> cost_iou_matrix(cost_matrix.rows(), std::vector<float>(cost_matrix.cols()));
                for (int i = 0; i < cost_matrix.rows(); i++) {
                    for (int j = 0; j < cost_matrix.cols(); j++) {
                        cost_iou_matrix[i][j] = -cost_matrix(i, j);
                    }
                }
                std::vector<int> rowsol, colsol;
                execLapjv(cost_iou_matrix, rowsol, colsol, true, 0.01, true);
                for (int i = 0; i < static_cast<int>(rowsol.size()); i++) {
                    if (rowsol.at(i) >= 0) {
                        matched_indices_vec.push_back({colsol.at(rowsol.at(i)), rowsol.at(i)});
                    }
                }
            }
        }
        std::set<int> matched_det_set, matched_trk_set;
        for (auto& p : matched_indices_vec) {
            matched_det_set.insert(p.first);
            matched_trk_set.insert(p.second);
        }
        std::vector<int> unmatched_detections;
        for (int i = 0; i < detections.rows(); i++) {
            if (matched_det_set.find(i) == matched_det_set.end()) {
                unmatched_detections.push_back(i);
            }
        }
        std::vector<int> unmatched_trackers;
        for (int i = 0; i < trackers.rows(); i++) {
            if (matched_trk_set.find(i) == matched_trk_set.end()) {
                unmatched_trackers.push_back(i);
            }
        }
        std::vector<Eigen::Matrix<int, 1, 2>> matches;
        for (auto& p : matched_indices_vec) {
            if (iou_matrix(p.first, p.second) < iou_threshold) {
                unmatched_detections.push_back(p.first);
                unmatched_trackers.push_back(p.second);
            }
            else {
                Eigen::Matrix<int, 1, 2> tmp;
                tmp << p.first, p.second;
                matches.push_back(tmp);
            }
        }
        return std::make_tuple(matches, unmatched_detections, unmatched_trackers);
    }
