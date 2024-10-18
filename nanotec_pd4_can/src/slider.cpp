#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

class SliderNode : public rclcpp::Node {
public:
    SliderNode() : Node("slider_node") {
        // Publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("slider_value", 10);

        // Initialize Qt
        app_ = std::make_shared<QApplication>(argc_, argv_);

        // Create the main window
        QWidget *window = new QWidget;
        QVBoxLayout *layout = new QVBoxLayout;

        // Create the slider
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setRange(0, 10000);  // Slider range from 0 to 10000
        slider->setSingleStep(1000); // Step size of 1000

        // Create the button
        QPushButton *button = new QPushButton("Publish Value");

        // Add slider and button to layout
        layout->addWidget(slider);
        layout->addWidget(button);

        // Connect the button's clicked signal to the callback
        QObject::connect(button, &QPushButton::clicked, [this, slider]() {
            int value = slider->value();
            publishValue(value);
        });

        // Set the layout and show the window
        window->setLayout(layout);
        window->show();
    }

    // Run the Qt application
    void run() {
        app_->exec();
    }

private:
    void publishValue(int value) {
        auto message = std_msgs::msg::Int32();
        message.data = value;
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", value);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    std::shared_ptr<QApplication> app_;
    int argc_ = 0;
    char **argv_ = nullptr;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create and run the node
    auto node = std::make_shared<SliderNode>();
    node->run();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
