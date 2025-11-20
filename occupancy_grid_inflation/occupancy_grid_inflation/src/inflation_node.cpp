#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGridInflation : public rclcpp::Node {
public:

    OccupancyGridInflation() : Node("occupancy_grid_inflation") {
        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occupancy_grid", 10, std::bind(&OccupancyGridInflation::mapCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_occupancy_grid", 10);

        latest_grid_ = nullptr;
    }


private:
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        nav_msgs::msg::OccupancyGrid inflated_grid = *msg;

        inflateObstacles(inflated_grid, 10, 20, 0.9); // 10

        zoneWeighting(inflated_grid);

        inflated_grid.header.frame_id = "odom";
    
        std::cout << "publishing..." << std::endl;

        pub_->publish(inflated_grid);
        //look into publishing the waypoint and map as one object. See how cell coord and such do it.


    }    

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



    //this is just some made up numbers. I want to alter the stack to use the local goal before testing.
    void zoneWeighting(
        nav_msgs::msg::OccupancyGrid& grid,
        double quadratic_factor = 2,
        double linear_factor = 1,
        double linear_ratio = .75,
        int top_bar_size = 5,
        int top_bar_weight = 5
    ){
        int width = grid.info.width;
        int height = grid.info.height;
            
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                //weight the bottom in a linear gradient
                if (y <= (height * linear_ratio)){
                    grid.data[y * width + x] += ((height * linear_ratio) - y) *  linear_factor;
                }
                //weight the top bar a little
                if (y > (height - top_bar_size)){
                    grid.data[y * width + x] += top_bar_weight;
                }
                //quadratic rating on the center
                //change the weighting as needed
                grid.data[y * width + x] += quadratic_factor * std::pow((width/2 - x), 2);

            }
        }
    }


    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridInflation>());
    rclcpp::shutdown();
    return 0;
}
    
