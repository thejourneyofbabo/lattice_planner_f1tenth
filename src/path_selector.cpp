#include "lattice_planner_pkg/path_selector.hpp"
#include <algorithm>
#include <cmath>

namespace lattice_planner_pkg {
namespace advanced {

PathSelector::PathSelector(const PathSelectionConfig& config) : config_(config) {}

CandidateResult* PathSelector::selectOptimalPath(
    std::vector<CandidateResult>& candidates,
    bool has_obstacles,
    double current_s,
    const rclcpp::Clock::SharedPtr& clock) {
    
    if (candidates.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("path_selector"), "[DEBUG] No candidates to select from");
        return nullptr;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[DEBUG] PathSelector: %zu candidates, has_obstacles=%s, current_s=%.2f", 
               candidates.size(), has_obstacles ? "true" : "false", current_s);
    
    // Primary path 선택 (우선순위 기반)
    CandidateResult* primary = selectPrimaryPath(candidates, has_obstacles);
    CandidateResult* reference_candidate = findReferenceCandidate(candidates);
    
    // Detour 상태 업데이트
    updateDetourState(reference_candidate);
    
    // 현재 커밋된 경로가 후보 리스트에 있는지 확인
    CandidateResult* committed_in_set = nullptr;
    if (commit_state_.has_commit) {
        for (auto& c : candidates) {
            if (std::abs(c.d_offset - commit_state_.committed_offset) < 1e-3) {
                committed_in_set = &c;
                break;
            }
        }
    }
    
    // 스위칭 허용 여부 판단
    bool allow_switch = shouldAllowSwitch(committed_in_set, primary, has_obstacles, current_s, clock);
    
    CandidateResult* chosen = nullptr;
    
    // 경로 길이 기반 커밋이 최우선
    if (config_.path_length_commit_mode && commit_state_.has_commit && committed_in_set && !allow_switch) {
        chosen = committed_in_set;
    } else if (detour_state_.detour_active && !canReturnFromDetour(reference_candidate)) {
        // Detour 유지 강제
        if (commit_state_.has_commit && committed_in_set && 
            !(committed_in_set->collided || committed_in_set->out_of_track)) {
            chosen = committed_in_set;
        } else {
            chosen = primary;
        }
    } else {
        // 정상 선택 로직
        if (!has_obstacles) {
            if (!detour_state_.detour_active) {
                // 원래부터 중앙선
                if (reference_candidate && !reference_candidate->collided && !reference_candidate->out_of_track) {
                    chosen = reference_candidate;
                }
            } else {
                // Detour에서 복귀 가능한 상황
                if (canReturnFromDetour(reference_candidate)) {
                    chosen = reference_candidate;
                    commit_state_.has_commit = false; // 복귀 시 커밋 초기화
                } else {
                    // 아직 복귀 조건 미충족
                    if (commit_state_.has_commit && committed_in_set && 
                        !(committed_in_set->collided || committed_in_set->out_of_track)) {
                        chosen = committed_in_set;
                    } else {
                        chosen = primary;
                    }
                }
            }
        } else if (commit_state_.has_commit && committed_in_set && !allow_switch) {
            chosen = committed_in_set; // 이전 커밋 유지
        } else {
            chosen = primary; // 새 선택
        }
    }
    
    if (chosen) {
        RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[DEBUG] PathSelector chosen path: cost=%.3f, d_offset=%.3f, collided=%s", 
                   chosen->cost, chosen->d_offset, chosen->collided ? "true" : "false");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("path_selector"), "[DEBUG] PathSelector could not choose any path");
    }
    
    return chosen;
}

void PathSelector::updateCommitState(
    const CandidateResult* chosen_path,
    double current_s,
    const rclcpp::Clock::SharedPtr& clock) {
    
    if (!chosen_path) return;
    
    // 선택된 경로가 커밋과 다르면 커밋 갱신
    if (!commit_state_.has_commit || 
        std::abs(chosen_path->d_offset - commit_state_.committed_offset) > 1e-3) {
        
        commit_state_.has_commit = true;
        commit_state_.committed_offset = chosen_path->d_offset;
        commit_state_.committed_merge_s = chosen_path->merge_s;
        commit_state_.commit_start_s = current_s;
        commit_state_.commit_start_time = clock->now();
        commit_state_.committed_cost = chosen_path->cost;
        commit_state_.committed_ref_alignment = chosen_path->ref_alignment;
        
        // 경로 길이 기반 커밋: 현재 위치에서 path_length만큼 앞의 끝 지점 계산
        commit_state_.committed_path_end_s = current_s + config_.path_length;
    }
}

CandidateResult* PathSelector::selectPrimaryPath(std::vector<CandidateResult>& candidates, bool has_obstacles) {
    if (candidates.empty()) return nullptr;
    
    RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[DEBUG] selectPrimaryPath: has_obstacles=%s", has_obstacles ? "true" : "false");
    
    // RACELINE RECOVERY: Find raceline candidate first
    CandidateResult* raceline_candidate = nullptr;
    CandidateResult* safe_raceline = nullptr;
    
    for (auto& c : candidates) {
        if (std::abs(c.d_offset) < 0.05) { // raceline candidate
            raceline_candidate = &c;
            if (!c.collided && !c.out_of_track) {
                safe_raceline = &c;
                break; // Found safe raceline, use it immediately
            }
        }
    }
    
    // Priority 1: Safe raceline
    if (safe_raceline) {
        RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[RACELINE RECOVERY] Using safe raceline, cost=%.3f", safe_raceline->cost);
        return safe_raceline;
    }
    
    // Priority 2: Force raceline even with collision (prevent cutting inside)
    if (raceline_candidate && has_obstacles) {
        RCLCPP_WARN(rclcpp::get_logger("path_selector"), "[RACELINE RECOVERY] Forcing collision raceline to prevent cutting inside, cost=%.3f", raceline_candidate->cost);
        return raceline_candidate;
    }
    
    if (has_obstacles) {
        // 장애물 있을 때: 충돌하지 않는 경로 중 최소 cost
        double best_cost = std::numeric_limits<double>::infinity();
        CandidateResult* primary = nullptr;
        
        for (auto& c : candidates) {
            if (c.collided || c.out_of_track) continue;
            if (c.cost < best_cost) {
                best_cost = c.cost;
                primary = &c;
            }
        }
        
        if (!primary) {
            // 모두 충돌 시 cost 최소 (충돌 경로 포함)
            auto it = std::min_element(candidates.begin(), candidates.end(),
                [](const auto& a, const auto& b) { return a.cost < b.cost; });
            primary = (it != candidates.end() ? &*it : nullptr);
        }
        
        return primary;
    } else {
        // 장애물이 없을 때: raceline 우선, 없으면 cost 기반 선택
        if (raceline_candidate) {
            RCLCPP_INFO(rclcpp::get_logger("path_selector"), "[RACELINE RECOVERY] No obstacles, using raceline, cost=%.3f", raceline_candidate->cost);
            return raceline_candidate;
        }
        
        auto it = std::min_element(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) {
                // out_of_track / collided 우선 제외
                bool a_bad = a.collided || a.out_of_track;
                bool b_bad = b.collided || b.out_of_track;
                if (a_bad != b_bad) return !a_bad; // 좋은 것이 우선
                return a.cost < b.cost; // cost 기반 선택
            });
        
        return (it != candidates.end() ? &*it : nullptr);
    }
}

CandidateResult* PathSelector::findReferenceCandidate(std::vector<CandidateResult>& candidates) {
    for (auto& c : candidates) {
        if (std::abs(c.d_offset - config_.reference_offset_target) < config_.reference_offset_tolerance) {
            return &c;
        }
    }
    return nullptr;
}

bool PathSelector::shouldAllowSwitch(
    const CandidateResult* committed_path,
    const CandidateResult* primary_path,
    bool has_obstacles,
    double current_s,
    const rclcpp::Clock::SharedPtr& clock) {
    
    if (!commit_state_.has_commit || !committed_path) return true;
    
    // 충돌/트랙 이탈 시 즉시 스위치 허용
    if (committed_path->collided || committed_path->out_of_track) {
        return true;
    }
    
    if (config_.path_length_commit_mode) {
        // 경로 길이 기반 커밋: 경로 끝에 도달했는지 확인
        return (current_s >= commit_state_.committed_path_end_s);
    } else {
        // 기존 시간/거리 기반 커밋 로직
        double progress = current_s - commit_state_.commit_start_s;
        double elapsed = (clock->now() - commit_state_.commit_start_time).seconds();
        
        double min_progress_threshold = has_obstacles ? 
            config_.commit_obstacle_min_progress : config_.commit_min_progress;
        double min_time_threshold = has_obstacles ? 
            config_.commit_obstacle_min_time_sec : config_.commit_min_time_sec;
        double improve_ratio_threshold = has_obstacles ? 
            config_.commit_obstacle_improve_ratio : config_.commit_cost_improve_ratio;
        
        bool min_hold = (progress < min_progress_threshold) || (elapsed < min_time_threshold);
        
        if (primary_path) {
            double current_metric = committed_path->cost;
            double primary_metric = primary_path->cost;
            double improvement = (current_metric - primary_metric) / std::max(1e-6, current_metric);
            bool big_lateral_change = std::abs(primary_path->d_offset - commit_state_.committed_offset) >= 
                config_.commit_lateral_change_min;
            bool significant_improve = improvement > improve_ratio_threshold;
            
            if (min_hold && !significant_improve) {
                return false;
            }
        } else if (min_hold) {
            return false;
        }
        
        return true;
    }
}

void PathSelector::updateDetourState(const CandidateResult* reference_candidate) {
    // Detour 상태 갱신: committed offset이 reference tolerance 밖이면 detour
    if (commit_state_.has_commit) {
        detour_state_.detour_active = std::abs(commit_state_.committed_offset - config_.reference_offset_target) > 
            config_.reference_offset_tolerance;
    }
    
    // Reference candidate가 깨끗한지 평가
    bool reference_clean = false;
    if (reference_candidate && !reference_candidate->collided && !reference_candidate->out_of_track) {
        reference_clean = true;
    }
    
    if (reference_clean) {
        detour_state_.detour_clear_frames++;
    } else {
        detour_state_.detour_clear_frames = 0;
    }
}

bool PathSelector::canReturnFromDetour(const CandidateResult* reference_candidate) const {
    return detour_state_.detour_active && 
           reference_candidate && 
           !reference_candidate->collided && 
           !reference_candidate->out_of_track &&
           detour_state_.detour_clear_frames >= config_.detour_return_clear_frames_threshold;
}

} // namespace advanced
} // namespace lattice_planner_pkg