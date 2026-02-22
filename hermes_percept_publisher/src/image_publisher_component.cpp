#include <stdexcept>
#include "hermes_percept_publisher/image_publisher_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hermes_percept {

ImagePublisherComponent::ImagePublisherComponent(const rclcpp::NodeOptions & options)
: Node("image_publisher", options)
{
    // Declare parameters with defaults.
    // "source" accepts a camera index (e.g. "0"), an image file path, or a video file path.
    declare_parameter("source", std::string("0"));
    declare_parameter("publish_rate", 15.0);
    declare_parameter("flip_horizontal", false);
    declare_parameter("flip_vertical", false);
    // retry_on_failure: if true, work() will call init() again when a frame
    // cannot be read (e.g. transient camera disconnect). Not reconfigurable.
    declare_parameter("retry_on_failure", false);

    // Read initial values
    source_           = get_parameter("source").as_string();
    publish_rate_     = get_parameter("publish_rate").as_double();
    flip_horizontal_  = get_parameter("flip_horizontal").as_bool();
    flip_vertical_    = get_parameter("flip_vertical").as_bool();
    retry_on_failure_ = get_parameter("retry_on_failure").as_bool();

    // Validate initial publish_rate (must be positive and at most 1000 Hz)
    if (publish_rate_ <= 0.0 || publish_rate_ > 1000.0) {
        throw std::invalid_argument(
            "Initial publish_rate must be in the range (0, 1000] Hz, got " +
            std::to_string(publish_rate_));
    }

    // Register parameter-change callback
    param_cb_ = add_on_set_parameters_callback(
        std::bind(&ImagePublisherComponent::on_params_changed, this, std::placeholders::_1));

    // Create image_transport publisher (advertises raw + compressed topics)
    publisher_ = image_transport::create_publisher(this, "camera/image_raw");

    // Open the capture source and pre-compute flip values
    init();

    // Create timer driven by publish_rate
    reset_timer();

    RCLCPP_INFO(get_logger(),
        "Image Publisher Component Active (source='%s', rate=%.1f Hz).",
        source_.c_str(), publish_rate_);
}

ImagePublisherComponent::~ImagePublisherComponent()
{
    cap_.release();
}

void ImagePublisherComponent::compute_flip_params()
{
    needs_flip_ = flip_horizontal_ || flip_vertical_;
    if (flip_horizontal_ && flip_vertical_) {
        flip_code_ = -1;  // both axes
    } else if (flip_horizontal_) {
        flip_code_ = 1;   // around y-axis (horizontal flip)
    } else {
        flip_code_ = 0;   // around x-axis (vertical flip)
    }
}

void ImagePublisherComponent::init()
{
    // Try to parse source_ as a camera index
    try {
        std::size_t pos = 0;
        int index = std::stoi(source_, &pos);
        if (pos == source_.size() && index >= 0) {
            cap_.open(index);
            if (cap_.isOpened()) {
                RCLCPP_INFO(get_logger(), "Opened camera index %d.", index);
                is_video_ = false;
                compute_flip_params();
                return;
            }
            RCLCPP_WARN(get_logger(),
                "Could not open camera index %d; retrying as filename.", index);
        }
    } catch (const std::exception & e) {
        // source_ is not a plain integer — treat it as a filename
        RCLCPP_DEBUG(get_logger(),
            "Source '%s' is not a camera index (%s); opening as file.",
            source_.c_str(), e.what());
    }

    // Open as an image or video file
    cap_.open(source_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open source '%s'.", source_.c_str());
        throw std::runtime_error("Could not open capture source: " + source_);
    }
    RCLCPP_INFO(get_logger(), "Opened file source '%s'.", source_.c_str());

    // Determine whether this is a multi-frame video file so that work() can
    // rewind the stream when it reaches the end.
    is_video_ = cap_.get(cv::CAP_PROP_FRAME_COUNT) > 1;

    compute_flip_params();
}

void ImagePublisherComponent::reset_timer()
{
    using namespace std::chrono;
    auto period = duration_cast<nanoseconds>(duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&ImagePublisherComponent::work, this));
}

void ImagePublisherComponent::work()
{
    cv::Mat frame;
    if (!cap_.read(frame)) {
        // If the source is a video file, rewind to the beginning and loop.
        if (is_video_) {
            if (!cap_.set(cv::CAP_PROP_POS_FRAMES, 0)) {
                RCLCPP_WARN(get_logger(),
                    "Failed to rewind video source '%s'.", source_.c_str());
            }
            return;
        }
        ++consecutive_failures_;
        RCLCPP_WARN(get_logger(),
            "Failed to read frame from source '%s' (failure #%d).",
            source_.c_str(), consecutive_failures_);
        if (retry_on_failure_) {
            // Only attempt re-init on the 1st failure and then every 30
            // subsequent consecutive failures to avoid flooding with retries.
            if (consecutive_failures_ == 1 || consecutive_failures_ % 30 == 0) {
                RCLCPP_INFO(get_logger(), "Retrying capture source init...");
                cap_.release();
                try {
                    init();
                    consecutive_failures_ = 0;
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(get_logger(), "Re-init failed: %s", e.what());
                }
            }
        }
        return;
    }
    consecutive_failures_ = 0;

    // Apply pre-computed flip (avoids recalculating flip_code_ every frame).
    if (needs_flip_) {
        cv::flip(frame, frame, flip_code_);
    }

    // Publish via image_transport (provides raw + compressed topics automatically).
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = now();
    msg->header.frame_id = "camera";
    publisher_.publish(*msg);
}

rcl_interfaces::msg::SetParametersResult ImagePublisherComponent::on_params_changed(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
        if (p.get_name() == "publish_rate") {
            double new_rate = p.as_double();
            if (new_rate <= 0.0 || new_rate > 1000.0) {
                result.successful = false;
                result.reason = "publish_rate must be in the range (0, 1000] Hz";
                return result;
            }
            publish_rate_ = new_rate;
            reset_timer();
            RCLCPP_INFO(get_logger(), "Updated publish_rate to %.1f Hz.", publish_rate_);
        } else if (p.get_name() == "flip_horizontal") {
            flip_horizontal_ = p.as_bool();
            compute_flip_params();
        } else if (p.get_name() == "flip_vertical") {
            flip_vertical_ = p.as_bool();
            compute_flip_params();
        } else if (p.get_name() == "source") {
            std::string old_source = source_;
            source_ = p.as_string();
            cap_.release();
            try {
                init();
                consecutive_failures_ = 0;
            } catch (const std::exception & e) {
                // Restore previous source so the component stays functional.
                source_ = old_source;
                try {
                    init();
                } catch (const std::exception & restore_err) {
                    RCLCPP_ERROR(get_logger(),
                        "Failed to restore source '%s': %s. Component may be non-functional.",
                        old_source.c_str(), restore_err.what());
                }
                result.successful = false;
                result.reason = e.what();
                return result;
            }
        }
    }

    return result;
}

}  // namespace hermes_percept

RCLCPP_COMPONENTS_REGISTER_NODE(hermes_percept::ImagePublisherComponent)
