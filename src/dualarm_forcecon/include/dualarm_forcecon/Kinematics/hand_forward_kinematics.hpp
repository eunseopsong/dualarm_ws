#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>

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

    // ✅ 기존 코드 호환용 생성자 (DualArmForceControl.cpp의 make_shared 호출을 그대로 살림)
    HandForwardKinematics(const std::string& urdf_path,
                          const std::string& base_name_unused,
                          const std::vector<std::string>& tips_unused) {
        (void)tips_unused;
        std::string side = InferSideFromBaseName_(base_name_unused);
        Init(urdf_path, side);
    }

    // 기존 코드가 Init(urdf, base, tips) 같은 걸로 부를 수도 있어서 오버로드 유지
    bool Init(const std::string& urdf_path,
              const std::string& base_name_unused,
              const std::vector<std::string>& tips_unused) {
        (void)tips_unused;
        std::string side = InferSideFromBaseName_(base_name_unused);
        return Init(urdf_path, side);
    }

    // 핵심 Init: side="left"/"right"
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

        // ---- hand joint mapping(20dof) 구성: (thumb 1~4, index 1~4, middle 1~4, ring 1~4, baby 1~4) ----
        BuildHandJointMapping_();

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

        // 기존 로그처럼 매핑도 보여주고 싶으면 유지
        std::printf("[HandFK Info] Hand joint mapping (20 dof) idx->jointName:\n");
        for (size_t i = 0; i < hand_joint_names_.size(); ++i) {
            const int qidx = hand_qidx_[i];
            std::printf("  [%zu] : %s (qidx=%d)\n", i, hand_joint_names_[i].c_str(), qidx);
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

    // ==============================
    // computeFingertips: 기존 코드 호환 핵심 API
    // 입력: hand 20dof (thumb1-4, index1-4, middle1-4, ring1-4, baby1-4)
    // 출력: 5개 tip position (thumb, index, middle, ring, baby) in base(left_joint_6/right_joint_6)
    // ==============================

    std::vector<Eigen::Vector3d> computeFingertips(const std::vector<double>& hand20) {
        Eigen::VectorXd v((int)hand20.size());
        for (int i = 0; i < (int)hand20.size(); ++i) v[i] = hand20[i];
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

        // q 전체는 neutral로 만들고, hand joint 20개만 심는다.
        Eigen::VectorXd q = pinocchio::neutral(model_);

        const int n_in = (int)hand_vec.size();
        const int n_use = std::min(n_in, 20);
        for (int i = 0; i < n_use; ++i) {
            const int qidx = hand_qidx_[i];
            if (qidx >= 0 && qidx < q.size()) {
                q[qidx] = hand_vec[i];
            }
        }

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3 oMb = GetWorldToBase_();

        // tip order: thumb,index,middle,ring,baby (tip_names_도 이 순서로 들어있음)
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

private:
    static pinocchio::FrameIndex InvalidFrame_() {
        return std::numeric_limits<pinocchio::FrameIndex>::max();
    }

    std::string InferSideFromBaseName_(const std::string& base_name) const {
        // 네 기존 호출이 "left_hand_base_link"/"right_hand_base_link"라서 여기서 side 추론
        if (base_name.find("left") != std::string::npos)  return "left";
        if (base_name.find("right") != std::string::npos) return "right";
        // 못 찾으면 안전하게 left로(원하면 right로 바꿔도 됨)
        return "left";
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

    void BuildHandJointMapping_() {
        hand_joint_names_.clear();
        hand_joint_names_.reserve(20);

        auto push4 = [&](const std::string& prefix) {
            hand_joint_names_.push_back(prefix + "1");
            hand_joint_names_.push_back(prefix + "2");
            hand_joint_names_.push_back(prefix + "3");
            hand_joint_names_.push_back(prefix + "4");
        };

        // ✅ 기존 로그 스타일(thumb block -> index -> middle -> ring -> baby)로 유지
        if (side_ == "left") {
            push4("left_thumb_joint");
            push4("left_index_joint");
            push4("left_middle_joint");
            push4("left_ring_joint");
            push4("left_baby_joint");
        } else {
            push4("right_thumb_joint");
            push4("right_index_joint");
            push4("right_middle_joint");
            push4("right_ring_joint");
            push4("right_baby_joint");
        }

        hand_qidx_.assign(20, -1);

        for (int i = 0; i < 20; ++i) {
            const std::string& jn = hand_joint_names_[i];
            const pinocchio::JointIndex jid = model_.getJointId(jn);
            if (jid == 0) {
                hand_qidx_[i] = -1;
                continue;
            }
            // revolute는 nq=1이라 idx_q로 바로 넣으면 됨
            hand_qidx_[i] = model_.joints[jid].idx_q();
        }
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

    // base reference (forced to *_joint_6, fallback to *_link_6 frame)
    std::string base_joint_name_;
    std::string base_ref_name_;
    bool base_is_joint_ = false;
    pinocchio::JointIndex base_joint_id_ = 0;
    pinocchio::FrameIndex base_frame_id_ = InvalidFrame_();

    // tips
    std::vector<std::string> tip_names_;          // size 5
    std::vector<pinocchio::FrameIndex> tip_fids_; // size 5

    // hand joint mapping
    std::vector<std::string> hand_joint_names_; // size 20
    std::vector<int> hand_qidx_;                // size 20 (q index)
};

} // namespace dualarm_forcecon