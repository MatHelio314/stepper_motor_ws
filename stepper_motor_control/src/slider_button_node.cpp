#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QLabel>
#include <QHBoxLayout>
#include <QProcess>  // For launching external processes


class SliderNode : public rclcpp::Node {
public:
    SliderNode() : Node("slider_node") {
        // Publisher
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("slider_value", 10);

        // Initialize Qt
        app_ = std::make_shared<QApplication>(argc_, argv_);

        // Create the main window
        QWidget *window = new QWidget;
        window->setWindowTitle("Position Control");  // Set window title

        // Main vertical layout
        QVBoxLayout *mainLayout = new QVBoxLayout;

        // Create and style the title label
        QLabel *titleLabel = new QLabel("Position Control");
        titleLabel->setAlignment(Qt::AlignCenter);  // Center align
        QFont titleFont("Arial", 16, QFont::Bold);
        titleLabel->setFont(titleFont);

        // Create the slider
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setRange(0, 10000);  // Slider range from 0 to 10000
        slider->setSingleStep(1000); // Step size of 1000
        slider->setStyleSheet("QSlider::handle { background-color: #3498db; }"
                              "QSlider::groove { background-color: #95a5a6; height: 6px; }"
                              "QSlider::sub-page { background-color: #2ecc71; }");

        // Create the label to show the slider value
        QLabel *valueLabel = new QLabel("Current value: 0");
        valueLabel->setAlignment(Qt::AlignCenter);  // Center align the text
        valueLabel->setStyleSheet("font-size: 14px; color: #2c3e50;");

        // Create a horizontal layout for the slider and value label
        QHBoxLayout *sliderLayout = new QHBoxLayout;
        sliderLayout->addWidget(slider);
        sliderLayout->addWidget(valueLabel);

        // Create and style the "Publish Value" button
        QPushButton *publishButton = new QPushButton("Publish Value");
        publishButton->setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px;"
                                     "padding: 10px; border-radius: 5px;");
        publishButton->setFixedWidth(200);  // Set button width

        // Create and style the "Launch Position Tick Node" button
        QPushButton *launchButton = new QPushButton("Launch Position Tick Node");
        launchButton->setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold; font-size: 14px;"
                                    "padding: 10px; border-radius: 5px;");
        launchButton->setFixedWidth(300);  // Set button width

        // Align the buttons to the center
        QHBoxLayout *buttonLayout = new QHBoxLayout;
        buttonLayout->addWidget(publishButton, 0, Qt::AlignCenter);
        buttonLayout->addWidget(launchButton, 0, Qt::AlignCenter);

        // Add title, slider layout, and buttons to main layout
        mainLayout->addWidget(titleLabel);
        mainLayout->addLayout(sliderLayout);
        mainLayout->addLayout(buttonLayout);

        // Connect the slider's valueChanged signal to update the label
        QObject::connect(slider, &QSlider::valueChanged, [valueLabel](int value) {
            valueLabel->setText(QString("Current value: %1").arg(value));
        });

        // Connect the "Publish Value" button's clicked signal to the callback
        QObject::connect(publishButton, &QPushButton::clicked, [this, slider]() {
            int value = slider->value();
            publishValue(value);
        });

        // Connect the "Launch Position Tick Node" button's clicked signal to the callback
        QObject::connect(launchButton, &QPushButton::clicked, [this]() {
            launchPositionTickNode();
        });

        // Set the layout and show the window
        window->setLayout(mainLayout);
        window->setFixedSize(500, 250);  // Set a fixed size for the window
        window->show();

        // Timer to periodically check if ROS2 is still running
        check_ros_shutdown_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() {
                if (!rclcpp::ok()) {
                    app_->quit();  // Quit Qt when ROS2 shuts down
                }
            });
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

    void launchPositionTickNode() {
        // Launch the "position_tick_node" using QProcess
        QProcess *process = new QProcess(nullptr);  // Use nullptr if no parent is needed
        process->start("ros2", QStringList() << "run" << "stepper_motor_control" << "position_tick_client");
        if (!process->waitForStarted()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch position_tick_node");
        } else {
            RCLCPP_INFO(this->get_logger(), "Launched position_tick_node");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    std::shared_ptr<QApplication> app_;
    rclcpp::TimerBase::SharedPtr check_ros_shutdown_timer_;  // Timer to check ROS shutdown
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
