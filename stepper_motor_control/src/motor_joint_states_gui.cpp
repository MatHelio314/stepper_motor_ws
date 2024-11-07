#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateDisplay : public QWidget {
    Q_OBJECT

public:
    JointStateDisplay(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr)
        : QWidget(parent), node_(node) {
        // Set up the layout and labels
        QVBoxLayout *layout = new QVBoxLayout(this);
        
        position_label_ = new QLabel("Position: --", this);
        velocity_label_ = new QLabel("Velocity: --", this);
        torque_label_ = new QLabel("Torque: --", this);
        
        layout->addWidget(position_label_);
        layout->addWidget(velocity_label_);
        layout->addWidget(torque_label_);
        
        setLayout(layout);

        // ROS 2 subscriber for the joint state
        joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/third_shaft_joint/joint_states", 10,
            std::bind(&JointStateDisplay::jointStateCallback, this, std::placeholders::_1)
        );

        // Timer to regularly update the GUI
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &JointStateDisplay::updateLabels);
        timer_->start(100);  // Update every 100 ms
    }

private slots:
    void updateLabels() {
        position_label_->setText(QString("Position: %1").arg(current_position_, 0, 'f', 2));
        velocity_label_->setText(QString("Velocity: %1").arg(current_velocity_, 0, 'f', 2));
        torque_label_->setText(QString("Torque: %1").arg(current_torque_, 0, 'f', 2));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) current_position_ = msg->position[0];
        if (!msg->velocity.empty()) current_velocity_ = msg->velocity[0];
        if (!msg->effort.empty()) current_torque_ = msg->effort[0];
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    QLabel *position_label_;
    QLabel *velocity_label_;
    QLabel *torque_label_;
    QTimer *timer_;

    double current_position_ = 0.0;
    double current_velocity_ = 0.0;
    double current_torque_ = 0.0;
};

int main(int argc, char *argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("motor_joint_states_gui");

    // Initialize Qt application
    QApplication app(argc, argv);

    // Create and show the GUI
    JointStateDisplay display(node);
    display.setWindowTitle("Motor Joint State Display");
    display.resize(300, 150);
    display.show();

    // Run the Qt and ROS event loops
    auto ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    ros_executor->add_node(node);
    QTimer::singleShot(0, [&]() {
        ros_executor->spin();
    });

    int result = app.exec();

    // Shutdown ROS 2 after the GUI closes
    rclcpp::shutdown();
    return result;
}

#include "motor_joint_states_gui.moc"
