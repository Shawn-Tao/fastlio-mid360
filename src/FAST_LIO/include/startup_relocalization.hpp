#pragma once
#include "bounded_relocalization.hpp"
#include <atomic>
#include <future>
#include <memory>

namespace fastlio_relocalization {
// ROS-independent startup gate. A background search never blocks sensor input;
// acceptance also requires fresh, stationary frames after the search completes.
class Startup {
public:
    enum class Phase { WaitingImu, Collecting, Matching, Confirming, Ready, Failed };
private:
    std::shared_ptr<const Matcher> matcher_;
    int accumulation_frames_, confirmation_frames_, collected_=0, confirmed_=0;
    std::vector<Point> accumulated_;
    std::future<Result> worker_;
    std::atomic<bool> cancelled_{false};
    Phase phase_=Phase::WaitingImu;
    Result result_;
    std::string detail_="waiting_for_imu";
    void fail(const std::string& reason) { phase_=Phase::Failed; detail_=reason; result_.accepted=false; }
public:
    Startup(std::shared_ptr<const Matcher> matcher,int accumulation_frames=15,int confirmation_frames=3):
        matcher_(std::move(matcher)),accumulation_frames_(accumulation_frames),confirmation_frames_(confirmation_frames) {
        if (accumulation_frames<1 || accumulation_frames>100 || confirmation_frames<1 || confirmation_frames>30)
            throw std::invalid_argument("Relocalization accumulation/confirmation frames out of range");
    }
    ~Startup() { cancelled_.store(true); if (worker_.valid()) worker_.wait(); }
    Startup(const Startup&)=delete;
    Startup& operator=(const Startup&)=delete;
    Phase phase() const { return phase_; }
    bool ready() const { return phase_==Phase::Ready; }
    const Result& result() const { return result_; }
    const std::string& detail() const { return detail_; }
    const char* state() const {
        switch(phase_) {
            case Phase::WaitingImu: return "waiting_for_imu";
            case Phase::Collecting: return "collecting";
            case Phase::Matching: return "matching";
            case Phase::Confirming: return "confirming";
            case Phase::Ready: return "ready";
            case Phase::Failed: return "failed";
        }
        return "failed";
    }
    bool reset() {
        if (worker_.valid()) {
            if (worker_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return false;
            try { worker_.get(); } catch (const std::exception&) { /* discard abandoned result */ }
        }
        cancelled_.store(false); accumulated_.clear(); collected_=confirmed_=0;
        result_=Result{}; phase_=Phase::WaitingImu; detail_="waiting_for_imu";
        return true;
    }
    // Returns true only for the transition to READY, never for a tentative match.
    bool update(const std::vector<Point>& frame,bool stationary,bool imu_initialized) {
        if (phase_==Phase::Failed || ready()) return false;
        if (!imu_initialized) { detail_="waiting_for_imu"; return false; }
        if (phase_==Phase::WaitingImu) phase_=Phase::Collecting;
        if (phase_==Phase::Matching) {
            if (!stationary) cancelled_.store(true);
            if (worker_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return false;
            try { result_=worker_.get(); }
            catch (const std::exception&) { fail("search_exception"); return false; }
            if (cancelled_.load()) { fail("motion_during_search"); return false; }
            if (!result_.accepted) { fail(result_.reason); return false; }
            phase_=Phase::Confirming; detail_="confirming_fresh_scans"; return false;
        }
        if (!stationary) {
            if (phase_==Phase::Confirming) { fail("motion_during_confirmation"); return false; }
            accumulated_.clear(); collected_=0; detail_="keep_stationary"; return false;
        }
        auto scan=matcher_->prepare_scan(frame);
        if (scan.empty()) { detail_="waiting_for_points"; return false; }
        if (phase_==Phase::Collecting) {
            detail_="collecting_stationary_scans";
            accumulated_.insert(accumulated_.end(),scan.begin(),scan.end());
            if (++collected_<accumulation_frames_) return false;
            auto snapshot=std::move(accumulated_);
            cancelled_.store(false);
            try {
                worker_=std::async(std::launch::async,[this,snapshot=std::move(snapshot)] {
                    return matcher_->search(snapshot,[this] { return cancelled_.load(); });
                });
            } catch (const std::exception&) { fail("worker_start_failed"); return false; }
            phase_=Phase::Matching; detail_="searching_position_and_heading";
        } else if (phase_==Phase::Confirming) {
            if (!matcher_->evaluate(frame,result_.pose).accepted) { fail("fresh_scan_confirmation_failed"); return false; }
            if (++confirmed_>=confirmation_frames_) { phase_=Phase::Ready; detail_="matched_and_confirmed"; return true; }
        }
        return false;
    }
};
} // namespace fastlio_relocalization
