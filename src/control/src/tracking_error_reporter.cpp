#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>  
#include <sys/types.h>
#include <vector>

class TrackingErrorReporter {
public:
    TrackingErrorReporter() : nh_("~"), saved_(false) {
        nh_.param<std::string>("heading_error_topic", heading_error_topic_, std::string("/heading_error"));
        nh_.param<std::string>("cross_track_error_topic", cross_track_error_topic_, std::string("/cross_track_error"));
        nh_.param<std::string>("report_output_dir", report_output_dir_, std::string("/tmp/MPC_reports"));
        nh_.param<std::string>("report_prefix", report_prefix_, std::string("MPC_reports"));

        heading_sub_ = nh_.subscribe(heading_error_topic_, 100, &TrackingErrorReporter::headingCb, this);
        cte_sub_ = nh_.subscribe(cross_track_error_topic_, 100, &TrackingErrorReporter::cteCb, this);

        ROS_INFO("tracking_error_reporter heading_error_topic: %s", heading_error_topic_.c_str());
        ROS_INFO("tracking_error_reporter cross_track_error_topic: %s", cross_track_error_topic_.c_str());
        ROS_INFO("tracking_error_reporter output dir: %s", report_output_dir_.c_str());
    }

    ~TrackingErrorReporter() {
        saveReports();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber heading_sub_;
    ros::Subscriber cte_sub_;

    std::string heading_error_topic_;
    std::string cross_track_error_topic_;
    std::string report_output_dir_;
    std::string report_prefix_;

    ros::Time start_time_;
    bool saved_;

    std::vector<double> heading_time_sec_;
    std::vector<double> heading_error_rad_;
    std::vector<double> cte_time_sec_;
    std::vector<double> cte_meter_;

    static bool ensureDirectory(const std::string& dir_path) {
        struct stat info;
        if (stat(dir_path.c_str(), &info) == 0) {
            return (info.st_mode & S_IFDIR) != 0;
        }
        return mkdir(dir_path.c_str(), 0755) == 0;
    }

    static bool writeLineSvg(const std::string& file_path,
                             const std::vector<double>& x,
                             const std::vector<double>& y,
                             const std::string& title,
                             const std::string& y_label) {
        if (x.size() < 2 || y.size() < 2 || x.size() != y.size()) {
            return false;
        }

        const int width = 1200;
        const int height = 680;
        const int left = 90;
        const int right = 40;
        const int top = 60;
        const int bottom = 80;

        const double x_min = x.front();
        const double x_max = x.back();

        const std::pair<std::vector<double>::const_iterator, std::vector<double>::const_iterator> y_minmax =
            std::minmax_element(y.begin(), y.end());
        const double raw_y_min = *(y_minmax.first);
        const double raw_y_max = *(y_minmax.second);

        double y_range = raw_y_max - raw_y_min;
        if (y_range < 1e-9) {
            y_range = 1.0;
        }

        // Nice tick step: 1/2/5 * 10^n
        const double rough_step = y_range / 8.0;
        const double step_base = std::pow(10.0, std::floor(std::log10(std::max(rough_step, 1e-9))));
        const double step_ratio = rough_step / step_base;
        double y_tick_step = step_base;
        if (step_ratio > 5.0) {
            y_tick_step = 10.0 * step_base;
        } else if (step_ratio > 2.0) {
            y_tick_step = 5.0 * step_base;
        } else if (step_ratio > 1.0) {
            y_tick_step = 2.0 * step_base;
        }

        const double y_min = std::floor(raw_y_min / y_tick_step) * y_tick_step;
        const double y_max = std::ceil(raw_y_max / y_tick_step) * y_tick_step;
        const int y_tick_count = std::max(2, static_cast<int>(std::round((y_max - y_min) / y_tick_step)));

        std::ofstream out(file_path);
        if (!out.is_open()) {
            return false;
        }

        out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height << "\">\n";
        out << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
        out << "<text x=\"" << width / 2 << "\" y=\"30\" text-anchor=\"middle\" font-size=\"24\" font-family=\"sans-serif\">"
            << title << "</text>\n";
        out << "<line x1=\"" << left << "\" y1=\"" << (height - bottom) << "\" x2=\"" << (width - right)
            << "\" y2=\"" << (height - bottom) << "\" stroke=\"black\"/>\n";
        out << "<line x1=\"" << left << "\" y1=\"" << top << "\" x2=\"" << left
            << "\" y2=\"" << (height - bottom) << "\" stroke=\"black\"/>\n";
        out << "<text x=\"" << width / 2 << "\" y=\"" << (height - 20)
            << "\" text-anchor=\"middle\" font-size=\"18\" font-family=\"sans-serif\">time [s]</text>\n";
        out << "<text x=\"25\" y=\"" << height / 2
            << "\" text-anchor=\"middle\" font-size=\"18\" font-family=\"sans-serif\" transform=\"rotate(-90 25,"
            << height / 2 << ")\">" << y_label << "</text>\n";

        out << "<polyline fill=\"none\" stroke=\"#0A66C2\" stroke-width=\"2\" points=\"";
        for (size_t i = 0; i < x.size(); ++i) {
            const double px = left + ((x[i] - x_min) / std::max(1e-9, x_max - x_min)) * (width - left - right);
            const double py = (height - bottom) - ((y[i] - y_min) / (y_max - y_min)) * (height - top - bottom);
            out << px << "," << py;
            if (i + 1 < x.size()) {
                out << " ";
            }
        }
        out << "\"/>\n";

        out << "<text x=\"" << left << "\" y=\"" << (height - bottom + 25)
            << "\" font-size=\"14\" font-family=\"sans-serif\">" << x_min << " s</text>\n";
        out << "<text x=\"" << (width - right) << "\" y=\"" << (height - bottom + 25)
            << "\" text-anchor=\"end\" font-size=\"14\" font-family=\"sans-serif\">" << x_max << " s</text>\n";

        const int y_decimals = std::max(0, static_cast<int>(std::ceil(-std::log10(std::max(y_tick_step, 1e-9)))));
        for (int i = 0; i <= y_tick_count; ++i) {
            const double tick = y_min + static_cast<double>(i) * y_tick_step;
            const double ty = (height - bottom) - ((tick - y_min) / std::max(1e-9, y_max - y_min)) * (height - top - bottom);
            out << "<line x1=\"" << (left - 5) << "\" y1=\"" << ty
                << "\" x2=\"" << left << "\" y2=\"" << ty << "\" stroke=\"black\"/>\n";
            out << "<line x1=\"" << left << "\" y1=\"" << ty
                << "\" x2=\"" << (width - right) << "\" y2=\"" << ty
                << "\" stroke=\"#D9D9D9\" stroke-width=\"1\"/>\n";
            std::ostringstream label;
            label << std::fixed << std::setprecision(std::min(4, y_decimals)) << tick;
            out << "<text x=\"" << (left - 10) << "\" y=\"" << (ty + 4)
                << "\" text-anchor=\"end\" font-size=\"12\" font-family=\"sans-serif\">" << label.str() << "</text>\n";
        }
        out << "</svg>\n";
        return true;
    }

    double elapsedSec(const ros::Time& now) {
        if (start_time_.isZero()) {
            start_time_ = now;
        }
        return (now - start_time_).toSec();
    }

    void headingCb(const std_msgs::Float64::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        heading_time_sec_.push_back(elapsedSec(now));
        heading_error_rad_.push_back(msg->data);
    }

    void cteCb(const std_msgs::Float64::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        cte_time_sec_.push_back(elapsedSec(now));
        cte_meter_.push_back(msg->data);
    }

    void saveReports() {
        if (saved_) {
            return;
        }
        saved_ = true;

        if (heading_time_sec_.size() < 2 && cte_time_sec_.size() < 2) {
            ROS_WARN("tracking_error_reporter: not enough samples to write report");
            return;
        }
        if (!ensureDirectory(report_output_dir_)) {
            ROS_WARN("tracking_error_reporter: failed to create directory: %s", report_output_dir_.c_str());
            return;
        }

        const std::time_t now = std::time(nullptr);
        char timestamp[32];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

        const std::string base = report_output_dir_ + "/" + report_prefix_ + "_" + timestamp;
        const std::string heading_svg_path = base + "_heading_error_deg.svg";
        const std::string cte_svg_path = base + "_cross_track_error_m.svg";

        bool heading_ok = false;
        bool cte_ok = false;

        if (heading_time_sec_.size() >= 2) {
            std::vector<double> heading_deg;
            heading_deg.reserve(heading_error_rad_.size());
            for (double rad : heading_error_rad_) {
                heading_deg.push_back(rad * 180.0 / M_PI);
            }
            heading_ok = writeLineSvg(
                heading_svg_path,
                heading_time_sec_,
                heading_deg,
                "Heading Error vs Time",
                "heading error [deg]");
        }

        if (cte_time_sec_.size() >= 2) {
            cte_ok = writeLineSvg(
                cte_svg_path,
                cte_time_sec_,
                cte_meter_,
                "Cross Track Error vs Time",
                "cross track error [m]");
        }

        ROS_INFO("tracking_error_reporter heading report: %s (%s)",
                 heading_svg_path.c_str(),
                 heading_ok ? "ok" : "skip");
        ROS_INFO("tracking_error_reporter cte report: %s (%s)",
                 cte_svg_path.c_str(),
                 cte_ok ? "ok" : "skip");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_error_reporter");
    TrackingErrorReporter reporter;
    ros::spin();
    return 0;
}
