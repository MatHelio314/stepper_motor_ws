#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QLabel>
#include <QHBoxLayout>  // For horizontal alignment

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

        // Create and style the button
        QPushButton *button = new QPushButton("Publish Value");
        button->setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px;"
                              "padding: 10px; border-radius: 5px;");
        button->setFixedWidth(200);  // Set button width

        // Align the button to the center
        QHBoxLayout *buttonLayout = new QHBoxLayout;
        buttonLayout->addWidget(button, 0, Qt::AlignCenter);

        // Add title, slider layout, and button to main layout
        mainLayout->addWidget(titleLabel);
        mainLayout->addLayout(sliderLayout);
        mainLayout->addLayout(buttonLayout);

        // Connect the slider's valueChanged signal to update the label
        QObject::connect(slider, &QSlider::valueChanged, [valueLabel](int value) {
            valueLabel->setText(QString("Current value: %1").arg(value));
        });

        // Connect the button's clicked signal to the callback
        QObject::connect(button, &QPushButton::clicked, [this, slider]() {
            int value = slider->value();
            publishValue(value);
        });

        // Set the layout and show the window
        window->setLayout(mainLayout);
        window->setFixedSize(400, 200);  // Set a fixed size for the window
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
