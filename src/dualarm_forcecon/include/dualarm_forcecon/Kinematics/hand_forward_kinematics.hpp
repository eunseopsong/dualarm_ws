#ifndef DUALARM_FORCECON_KINEMATICS_HAND_FORWARD_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_HAND_FORWARD_KINEMATICS_HPP_

#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>

#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace dualarm_forcecon {

class HandForwardKinematics {
public:
    HandForwardKinematics() = default;

    // 기존 코드 호환용 생성자
    HandForwardKinematics(const std::string& urdf_path,
                          const std::string& base_name_unused,
                          const std::vector<std::string>& tips_unused)
    {
        (void)tips_unused;
        std::string side = InferSideFromBaseName_(base_name_unused);
        Init(urdf_path, side);
    }

    // 기존 코드 호환용 Init
    bool Init(const std::string& urdf_path,
              const std::string& base_name_unused,
              const std::vector<std::string>& tips_unused)
    {
        (void)tips_unused;
        std::string side = InferSideFromBaseName_(base_name_unused);
        return Init(urdf_path, side);
    }

    // 핵심 Init: side = "left" / "right"
    bool Init(const std::string& urdf_path, const std::string& side) {
        side_ = side;
        ok_   = false;

        try {
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
        } catch (const std::exception& e) {
            std::printf("[HandFK Error] buildModel failed: %s\n", e.what());
            return false;
        }

        // ---- base reference 강제: left_joint_6 / right_joint_6 ----
        if      (side_ == "left")  base_joint_name_ = "left_joint_6";
        else if (side_ == "right") base_joint_name_ = "right_joint_6";
        else {
            std::printf("[HandFK Error] side must be 'left' or 'right' (got '%s')\n", side_.c_str());
            return false;
        }

        ResolveBaseReference_();

        // ---- tip frame 강제: link4 5개 ----
        tip_names_.clear();
        tip_names_.reserve(5);

        if (side_ == "left") {
            tip_names_ = {
                "left_link4_thumb",
                "left_link4_index",
                "left_link4_middle",
                "left_link4_ring",
                "left_link4_baby"
            };
        } else {
            tip_names_ = {
                "right_link4_thumb",
                "right_link4_index",
                "right_link4_middle",
                "right_link4_ring",
                "right_link4_baby"
            };
        }

        tip_fids_.assign(5, InvalidFrame_());
        int tips_found = 0;
        for (size_t i = 0; i < tip_names_.size(); ++i) {
            const auto& nm = tip_names_[i];
            if (model_.existFrame(nm)) {
                tip_fids_[i] = model_.getFrameId(nm);
                tips_found++;
            }
        }

        // ---- v12: hand joint mapping 구성 (15DoF independent + mimic q4=q3) ----
        BuildHandJointMapping15_();

        // ---- Print summary ----
        std::printf("[HandFK Info] Pinocchio OK?=1 base=%s side=%s nframes=%zu\n",
                    base_ref_name_.c_str(), side_.c_str(), model_.frames.size());

        std::printf("[HandFK Info] Resolved tips (key -> frame):\n");
        for (size_t i = 0; i < tip_names_.size(); ++i) {
            if (tip_fids_[i] < model_.frames.size()) {
                std::printf("  key='%s' -> frame='%s' (fid=%u)\n",
                            tip_names_[i].c_str(), tip_names_[i].c_str(), (unsigned)tip_fids_[i]);
            } else {
                std::printf("  key='%s' -> NOT FOUND\n", tip_names_[i].c_str());
            }
        }

        std::printf("[HandFK Info] Independent hand joint mapping (15 dof) idx->jointName:\n");
        for (size_t i = 0; i < hand15_joint_names_.size(); ++i) {
            const int qidx = hand15_qidx_[i];
            std::printf("  [%zu] : %s (qidx=%d)\n", i, hand15_joint_names_[i].c_str(), qidx);
        }

        std::printf("[HandFK Info] Mimic mapping (joint4 <- joint3):\n");
        for (int f = 0; f < 5; ++f) {
            std::printf("  finger[%d] qidx_joint4=%d <- qidx_joint3=%d\n",
                        f, hand_qidx_joint4_[f], hand15_qidx_[f * 3 + 2]);
        }

        ok_ = (tips_found == 5);
        if (!ok_) {
            std::printf("[HandFK Warn] tip frames not fully found (%d/5). Check URDF frame names.\n", tips_found);
        }
        return ok_;
    }

    bool ok() const { return ok_; }
    const std::string& side() const { return side_; }
    const std::string& base_ref_name() const { return base_ref_name_; }

    // =========================================================================
    // computeFingertips
    // 입력 허용:
    //  - 15DoF: [thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]
    //  - 20DoF: [thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]
    //           -> v12에서는 FK 계산 시 joint4 입력값을 무시하고 joint3로 mimic 적용
    // 출력:
    //  - 5개 tip positions in base(left_joint_6/right_joint_6)
    //    order = thumb, index, middle, ring, baby
    // =========================================================================

    std::vector<Eigen::Vector3d> computeFingertips(const std::vector<double>& hand) {
        Eigen::VectorXd v((int)hand.size());
        for (int i = 0; i < (int)hand.size(); ++i) v[i] = hand[i];
        return computeFingertips(v);
    }

    template <size_t N>
    std::vector<Eigen::Vector3d> computeFingertips(const std::array<double, N>& handN) {
        Eigen::VectorXd v((int)N);
        for (int i = 0; i < (int)N; ++i) v[i] = handN[i];
        return computeFingertips(v);
    }

    std::vector<Eigen::Vector3d> computeFingertips(const Eigen::VectorXd& hand_vec) {
        std::vector<Eigen::Vector3d> out;
        out.reserve(5);

        if (!ok_) {
            out.assign(5, Eigen::Vector3d::Zero());
            return out;
        }

        // 입력을 15DoF independent로 정규화
        Eigen::VectorXd hand15 = NormalizeToHand15_(hand_vec);

        // q 전체는 neutral로 만들고, hand joints만 심는다.
        Eigen::VectorXd q = pinocchio::neutral(model_);

        // 1) independent joints (joint1~3 for each finger, total 15)
        for (int i = 0; i < 15; ++i) {
            const int qidx = hand15_qidx_[i];
            if (qidx >= 0 && qidx < q.size()) {
                q[qidx] = hand15[i];
            }
        }

        // 2) mimic joints: joint4 = joint3
        for (int f = 0; f < 5; ++f) {
            const int qidx4 = hand_qidx_joint4_[f];
            const int src15 = f * 3 + 2; // joint3 in independent vector
            if (qidx4 >= 0 && qidx4 < q.size()) {
                q[qidx4] = hand15[src15];
            }
        }

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3 oMb = GetWorldToBase_();

        for (size_t i = 0; i < 5; ++i) {
            if (tip_fids_[i] >= model_.frames.size()) {
                out.emplace_back(Eigen::Vector3d::Zero());
                continue;
            }
            const pinocchio::SE3& oMt = data_.oMf[tip_fids_[i]];
            const pinocchio::SE3 bMt  = oMb.actInv(oMt);
            out.emplace_back(bMt.translation());
        }

        return out;
    }

    // =========================================================================
    // computeTipRotationsBase   (NEW)
    // 입력 허용:
    //  - 15DoF / 20DoF (computeFingertips와 동일)
    // 출력:
    //  - 5개 tip rotation matrices (R_base_tip)
    //    order = thumb, index, middle, ring, baby
    //
    // 의미:
    //  - tip(frame)의 벡터를 hand base frame으로 변환할 때 사용
    //    v_base = R_base_tip * v_tip
    // =========================================================================

    std::vector<Eigen::Matrix3d> computeTipRotationsBase(const std::vector<double>& hand) {
        Eigen::VectorXd v((int)hand.size());
        for (int i = 0; i < (int)hand.size(); ++i) v[i] = hand[i];
        return computeTipRotationsBase(v);
    }

    template <size_t N>
    std::vector<Eigen::Matrix3d> computeTipRotationsBase(const std::array<double, N>& handN) {
        Eigen::VectorXd v((int)N);
        for (int i = 0; i < (int)N; ++i) v[i] = handN[i];
        return computeTipRotationsBase(v);
    }

    std::vector<Eigen::Matrix3d> computeTipRotationsBase(const Eigen::VectorXd& hand_vec) {
        std::vector<Eigen::Matrix3d> out;
        out.reserve(5);

        if (!ok_) {
            out.assign(5, Eigen::Matrix3d::Identity());
            return out;
        }

        // 입력을 15DoF independent로 정규화
        Eigen::VectorXd hand15 = NormalizeToHand15_(hand_vec);

        // q 전체는 neutral로 만들고, hand joints만 심는다.
        Eigen::VectorXd q = pinocchio::neutral(model_);

        // 1) independent joints (joint1~3 for each finger, total 15)
        for (int i = 0; i < 15; ++i) {
            const int qidx = hand15_qidx_[i];
            if (qidx >= 0 && qidx < q.size()) {
                q[qidx] = hand15[i];
            }
        }

        // 2) mimic joints: joint4 = joint3
        for (int f = 0; f < 5; ++f) {
            const int qidx4 = hand_qidx_joint4_[f];
            const int src15 = f * 3 + 2; // joint3 in independent vector
            if (qidx4 >= 0 && qidx4 < q.size()) {
                q[qidx4] = hand15[src15];
            }
        }

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3 oMb = GetWorldToBase_();

        for (size_t i = 0; i < 5; ++i) {
            if (tip_fids_[i] >= model_.frames.size()) {
                out.emplace_back(Eigen::Matrix3d::Identity());
                continue;
            }

            const pinocchio::SE3& oMt = data_.oMf[tip_fids_[i]];
            const pinocchio::SE3 bMt  = oMb.actInv(oMt);

            Eigen::Matrix3d R = bMt.rotation();
            out.emplace_back(R);
        }

        return out;
    }

private:
    static pinocchio::FrameIndex InvalidFrame_() {
        return std::numeric_limits<pinocchio::FrameIndex>::max();
    }

    std::string InferSideFromBaseName_(const std::string& base_name) const {
        if (base_name.find("left") != std::string::npos)  return "left";
        if (base_name.find("right") != std::string::npos) return "right";
        return "left"; // fallback
    }

    void ResolveBaseReference_() {
        base_is_joint_ = false;
        base_joint_id_ = 0;
        base_frame_id_ = InvalidFrame_();
        base_ref_name_.clear();

        // 1) joint 우선
        const pinocchio::JointIndex jid = model_.getJointId(base_joint_name_);
        if (jid > 0) {
            base_is_joint_ = true;
            base_joint_id_ = jid;
            base_ref_name_ = base_joint_name_;
            return;
        }

        // 2) fallback: link6 frame
        const std::string link6 = (side_ == "left") ? "left_link_6" : "right_link_6";
        if (model_.existFrame(link6)) {
            base_is_joint_ = false;
            base_frame_id_ = model_.getFrameId(link6);
            base_ref_name_ = link6;
            std::printf("[HandFK Warn] base joint '%s' not found -> fallback frame '%s'\n",
                        base_joint_name_.c_str(), link6.c_str());
            return;
        }

        // 3) 실패
        base_ref_name_ = base_joint_name_;
        std::printf("[HandFK Error] base reference not found: joint '%s' nor frame '%s'\n",
                    base_joint_name_.c_str(),
                    ((side_ == "left") ? "left_link_6" : "right_link_6"));
    }

    // v12: independent 15DoF mapping + mimic joint4 mapping
    void BuildHandJointMapping15_() {
        hand15_joint_names_.clear();
        hand15_joint_names_.reserve(15);

        hand15_qidx_.assign(15, -1);
        hand_qidx_joint4_.assign(5, -1);

        auto push3 = [&](const std::string& prefix) {
            hand15_joint_names_.push_back(prefix + "1");
            hand15_joint_names_.push_back(prefix + "2");
            hand15_joint_names_.push_back(prefix + "3");
        };

        if (side_ == "left") {
            push3("left_thumb_joint");
            push3("left_index_joint");
            push3("left_middle_joint");
            push3("left_ring_joint");
            push3("left_baby_joint");
        } else {
            push3("right_thumb_joint");
            push3("right_index_joint");
            push3("right_middle_joint");
            push3("right_ring_joint");
            push3("right_baby_joint");
        }

        // independent qidx (15)
        for (int i = 0; i < 15; ++i) {
            const std::string& jn = hand15_joint_names_[i];
            const pinocchio::JointIndex jid = model_.getJointId(jn);
            if (jid == 0) {
                hand15_qidx_[i] = -1;
                continue;
            }
            hand15_qidx_[i] = model_.joints[jid].idx_q();
        }

        // mimic qidx for joint4 (5)
        for (int f = 0; f < 5; ++f) {
            std::string j4;
            if (side_ == "left") {
                if      (f == 0) j4 = "left_thumb_joint4";
                else if (f == 1) j4 = "left_index_joint4";
                else if (f == 2) j4 = "left_middle_joint4";
                else if (f == 3) j4 = "left_ring_joint4";
                else             j4 = "left_baby_joint4";
            } else {
                if      (f == 0) j4 = "right_thumb_joint4";
                else if (f == 1) j4 = "right_index_joint4";
                else if (f == 2) j4 = "right_middle_joint4";
                else if (f == 3) j4 = "right_ring_joint4";
                else             j4 = "right_baby_joint4";
            }

            const pinocchio::JointIndex jid4 = model_.getJointId(j4);
            if (jid4 == 0) {
                hand_qidx_joint4_[f] = -1;
            } else {
                hand_qidx_joint4_[f] = model_.joints[jid4].idx_q();
            }
        }
    }

    // 입력 hand_vec를 independent 15DoF로 정규화
    // - size >= 20: [f1..4]*5 로 보고 각 finger의 1,2,3만 사용 (joint4는 무시)
    // - size >= 15: independent 15로 간주
    // - size < 15 : 가능한 만큼만 채우고 나머지는 0
    Eigen::VectorXd NormalizeToHand15_(const Eigen::VectorXd& hand_vec) const {
        Eigen::VectorXd h15(15);
        h15.setZero();

        const int n = static_cast<int>(hand_vec.size());

        if (n >= 20) {
            // legacy 20DoF input -> compress to independent 15DoF
            for (int f = 0; f < 5; ++f) {
                const int b20 = f * 4;
                const int b15 = f * 3;
                h15[b15 + 0] = hand_vec[b20 + 0];
                h15[b15 + 1] = hand_vec[b20 + 1];
                h15[b15 + 2] = hand_vec[b20 + 2];
                // hand_vec[b20+3] is ignored (mimic assumed)
            }
            return h15;
        }

        // assume 15DoF input (or partial)
        const int n_use = std::min(n, 15);
        for (int i = 0; i < n_use; ++i) h15[i] = hand_vec[i];
        return h15;
    }

    pinocchio::SE3 GetWorldToBase_() const {
        if (base_is_joint_) return data_.oMi[base_joint_id_];
        else                return data_.oMf[base_frame_id_];
    }

private:
    pinocchio::Model model_;
    pinocchio::Data  data_{model_};

    bool ok_ = false;
    std::string side_;

    // base reference
    std::string base_joint_name_;
    std::string base_ref_name_;
    bool base_is_joint_ = false;
    pinocchio::JointIndex base_joint_id_ = 0;
    pinocchio::FrameIndex base_frame_id_ = InvalidFrame_();

    // tips
    std::vector<std::string> tip_names_;          // size 5
    std::vector<pinocchio::FrameIndex> tip_fids_; // size 5

    // v12 hand mapping
    // independent 15DoF: [thumb1..3, index1..3, middle1..3, ring1..3, baby1..3]
    std::vector<std::string> hand15_joint_names_; // size 15
    std::vector<int> hand15_qidx_;                // size 15
    std::vector<int> hand_qidx_joint4_;           // size 5 (mimic joint4 qidx)
};

} // namespace dualarm_forcecon

#endif