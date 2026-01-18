#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

static constexpr double ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW = -0.60; // meters

class OccupancyGridInflation : public rclcpp::Node {
public:

    OccupancyGridInflation() : Node("occupancy_grid_inflation") {
        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occ_grid", 10, std::bind(&OccupancyGridInflation::mapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OccupancyGridInflation::odomCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_occupancy_grid", 10);

        latest_grid_ = nullptr;
    }


private:
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        nav_msgs::msg::OccupancyGrid inflated_grid = *msg;

        inflateObstacles(inflated_grid, 10, 20, 0.9); // 10

        inflated_grid.header.frame_id = "odom";
        if (odom.has_value()) {
            // Extract yaw from quaternion
            double qw = odom.value().pose.pose.orientation.w;
            double qz = odom.value().pose.pose.orientation.z;
            double robotYawRadians = 2.0 * std::atan2(qz, qw);

            // Convert robot yaw to grid yaw (rotate by -90 degrees)
            double gridYawRadians = robotYawRadians - M_PI / 2.0;

            // Grid origin in GRID frame:
            // X: centered (width/2)
            // Y: 0.60 ahead of robot
            double gridHalfWidth = (inflated_grid.info.width * inflated_grid.info.resolution) / 2.0;

            double localX_grid = -gridHalfWidth;  // Center side-to-side
            double localY_grid = 0.60;             // Grid is 0.60 ahead of robot

            // Transform from grid frame to world frame using grid's orientation
            double cosGridYaw = std::cos(gridYawRadians);
            double sinGridYaw = std::sin(gridYawRadians);

            inflated_grid.info.origin.position.set__x(
                odom.value().pose.pose.position.x + localX_grid * cosGridYaw - localY_grid * sinGridYaw
            );
            inflated_grid.info.origin.position.set__y(
                odom.value().pose.pose.position.y + localX_grid * sinGridYaw + localY_grid * cosGridYaw
            );

            // Set grid orientation (robot yaw - 90 degrees)
            inflated_grid.info.origin.orientation.set__z(std::sin(gridYawRadians / 2.0));
            inflated_grid.info.origin.orientation.set__w(std::cos(gridYawRadians / 2.0));
        } else {
            inflated_grid.info.origin.position.set__x(-ROBOT_FORWARDS_BACKWARDS_POSITION_RELATIVE_TO_BOTTOM_OF_CAMERA_VIEW );
            inflated_grid.info.origin.position.set__y(inflated_grid.info.width / 2* inflated_grid.info.resolution);
            // rotate to match +x forwards, +y left
            inflated_grid.info.origin.orientation.set__z(-0.7071068);
            inflated_grid.info.origin.orientation.set__w(0.7071068);
        }
    
        std::cout << "publishing..." << std::endl;

        pub_->publish(inflated_grid);
    }    

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odom = *msg;
    }
/** 
 *  @brief Takes an occupancy grid from CV and makes the space around the obstacles less driveable.
 *  @param grid The grid to be inflated
 *  @param radius The radius of inflation
 *  @param add_on Used to calculate the area check for inflation
 *  @param decrease_factor: how quickly the inflation decreases as it moves away from an obstacle.
 *  @returns Nothing
 */
    void inflateObstacles(nav_msgs::msg::OccupancyGrid &grid, int radius, int add_on, double decrease_factor) {
        int width = grid.info.width;
        int height = grid.info.height;
        std::vector<int8_t> new_data = grid.data;
            
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (grid.data[y * width + x] > 50) { 
                    for (int dy = -(radius+add_on); dy <= radius+add_on; dy++) {
                        for (int dx = -(radius+add_on); dx <= radius+add_on; dx++) {
                            int real_radius = std::sqrt(std::abs(dx) + std::abs(dy));
                            int new_value = int(grid.data[y * width + x])  * (std::pow(decrease_factor,real_radius)); 
                            int nx = x + dx;
                            int ny = y + dy;

                            if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
                                if( grid.data[y * width + x] == 100  && std::abs(dx) < radius && std::abs(dy)  < radius){
                                    new_data[ny * width + nx] = 100;
                                }
                                else{
                                    new_data[ny * width + nx] = std::max(new_data[ny * width + nx], int8_t(new_value));
                                }
                            }
                        }
                    }
                }
            }
        }
        grid.data = new_data;

    }

    std::optional<nav_msgs::msg::Odometry> odom;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridInflation>());
    rclcpp::shutdown();
    return 0;
}
    
