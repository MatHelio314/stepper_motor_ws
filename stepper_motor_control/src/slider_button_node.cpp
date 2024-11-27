#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QComboBox>
#include <QTimer>
#include <QSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QProcess>  // For launching and terminating external processes
#include "sensor_msgs/msg/joint_state.hpp"  // To subscribe to joint_states

double unit_coefficient = 0.1;
double current_position_1, current_position_2, current_position_3, current_position_4;
double current_torque_1, current_torque_2, current_torque_3, current_torque_4;

class SliderNode : public rclcpp::Node {
    
public:
    SliderNode() : Node("slider_node") {


        // Publishers for X, Y, and Z positions
        publisher_x_ = this->create_publisher<std_msgs::msg::Int32>("slider_value_x", 10);
        publisher_y_ = this->create_publisher<std_msgs::msg::Int32>("slider_value_y", 10);
        publisher_z_ = this->create_publisher<std_msgs::msg::Int32>("slider_value_z", 10);

        publisher_stop_x = this->create_publisher<std_msgs::msg::Int32>("stop_motor_X", 10);
        publisher_stop_y = this->create_publisher<std_msgs::msg::Int32>("stop_motor_Y", 10);
        publisher_stop_z = this->create_publisher<std_msgs::msg::Int32>("stop_motor_Z", 10);

        // Subscriptions for joint states
        joint_state_sub_1_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/first_shaft_joint/joint_states", 10,
            std::bind(&SliderNode::joint_state_callback_1, this, std::placeholders::_1));
            
        joint_state_sub_2_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/second_shaft_joint/joint_states", 10,
            std::bind(&SliderNode::joint_state_callback_2, this, std::placeholders::_1));

        // Subscription to joint states
        joint_state_sub_3_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/third_shaft_joint/joint_states", 10,
            std::bind(&SliderNode::joint_state_callback_3, this, std::placeholders::_1));

        joint_state_sub_4_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/fourth_shaft_joint/joint_states", 10,
            std::bind(&SliderNode::joint_state_callback_4, this, std::placeholders::_1));


        // Initialize Qt
        app_ = std::make_shared<QApplication>(argc_, argv_);

        // Create the main window
        QWidget *window = new QWidget;
        window->setWindowTitle("Position and Torque Control");

        // Main horizontal layout to hold both position and torque control
        QHBoxLayout *mainHLayout = new QHBoxLayout;



                    // Position control layout //



        QVBoxLayout *mainLayout = new QVBoxLayout;

    

        // Create and style the title label
        QLabel *titleLabel = new QLabel("Position Control");
        titleLabel->setAlignment(Qt::AlignCenter);
        QFont titleFont("Arial", 16, QFont::Bold);
        titleLabel->setFont(titleFont);
        mainLayout->addWidget(titleLabel);

        // Function to create a slider layout for each axis with a spinbox
        auto createSliderLayout = [&](const QString &axis, QSlider *&slider, QSpinBox *&spinBox, QPushButton *&enterButton, int minValue, int maxValue) {
            slider = new QSlider(Qt::Horizontal);
            slider->setRange(minValue, maxValue);
            slider->setSingleStep(1000);
            slider->setStyleSheet("QSlider::groove:horizontal { "
                                "border: 1px solid #999999; "
                                "height: 20px; "
                                "background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4); "
                                "margin: 2px 0; "
                                "} "
                                "QSlider::handle:horizontal { "
                                "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f); "
                                "border: 1px solid #5c5c5c; "
                                "width: 30px; "
                                "margin: -2px 0px; "
                                "} ");

            // Spinbox for displaying and controlling the slider value
            spinBox = new QSpinBox;
            spinBox->setRange(minValue, maxValue);
            spinBox->setSingleStep(1);
            spinBox->setAlignment(Qt::AlignCenter);
            spinBox->setStyleSheet("font-size: 14px; color: #2c3e50;");

            QHBoxLayout *sliderLayout = new QHBoxLayout;
            sliderLayout->addWidget(slider);
            sliderLayout->addWidget(spinBox);

            enterButton = new QPushButton(QString("Enter %1").arg(axis));
            enterButton->setFixedWidth(200);
            enterButton->setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px;"
                                    "padding: 10px; border-radius: 5px;");
            QHBoxLayout *enterLayout = new QHBoxLayout;
            enterLayout->addWidget(enterButton, 0, Qt::AlignCenter);

            mainLayout->addLayout(sliderLayout);
            mainLayout->addLayout(enterLayout);

            // Connect the slider and spinbox to update each other
            QObject::connect(slider, &QSlider::valueChanged, spinBox, &QSpinBox::setValue);
            QObject::connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);
        };


        // Create sliders and enter buttons for X, Y, and Z axes
        QSlider *sliderX, *sliderY, *sliderZ;
        QPushButton *enterButtonX, *enterButtonY, *enterButtonZ;
        QSpinBox *SpinBoxX,*SpinBoxY, *SpinBoxZ; 
        createSliderLayout("X", sliderX, SpinBoxX,enterButtonX, 0, unit_coefficient*257040);
        createSliderLayout("Y", sliderY, SpinBoxY,enterButtonY, 0, unit_coefficient*135360);
        createSliderLayout("Z", sliderZ, SpinBoxZ,enterButtonZ, 0, unit_coefficient*551232);


        // Connect "Enter" buttons to their respective publishers
        QObject::connect(enterButtonX, &QPushButton::clicked, [this, sliderX]() {
            int value = sliderX->value()/unit_coefficient;
            publishValue(publisher_x_, value, "X");
        });
        QObject::connect(enterButtonY, &QPushButton::clicked, [this, sliderY]() {
            int value = sliderY->value()/unit_coefficient;
            publishValue(publisher_y_, value, "Y");
            
        });
        QObject::connect(enterButtonZ, &QPushButton::clicked, [this, sliderZ]() {
            int value = sliderZ->value()/unit_coefficient;
            publishValue(publisher_z_, value, "Z");
        });

        // Buttons to launch position tick motors
        QPushButton *launchXButton = new QPushButton("Launch X Position Control");
        QPushButton *launchYButton = new QPushButton("Launch Y Position Control");
        QPushButton *launchZButton = new QPushButton("Launch Z Position Control");

        // Create "Close" buttons for each motor
        QPushButton *closeXButton = new QPushButton("Close/Stop X Position Control");
        QPushButton *closeYButton = new QPushButton("Close/Stop Y Position Control");
        QPushButton *closeZButton = new QPushButton("Close/Stop Z Position Control");

        // Create "Reset" buttons for each motor
        QPushButton *resetXButton = new QPushButton("Init/Reset X Motor");
        QPushButton *resetYButton = new QPushButton("Init/Reset Y Motor");
        QPushButton *resetZButton = new QPushButton("Init/Reset Z Motor");



        auto setButtonStyle = [](QPushButton *button, const QString &color) {
            button->setStyleSheet(QString("background-color: %1; color: white; font-weight: bold; font-size: 14px;"
                                        "padding: 10px; border-radius: 5px;").arg(color));
            button->setFixedWidth(220);
        };

        setButtonStyle(launchXButton, "#2ecc71"); // Green
        setButtonStyle(launchYButton, "#2ecc71");
        setButtonStyle(launchZButton, "#2ecc71");

        setButtonStyle(closeXButton, "#e74c3c");  // Red color
        setButtonStyle(closeYButton, "#e74c3c");
        setButtonStyle(closeZButton, "#e74c3c");  

        setButtonStyle(resetXButton, "#3c80e7");  // Blue color
        setButtonStyle(resetYButton, "#3c80e7");
        setButtonStyle(resetZButton, "#3c80e7");  



        // Layout for launch and close buttons

        QHBoxLayout *launchCloseXLayout = new QHBoxLayout;
        launchCloseXLayout->addWidget(resetXButton);
        launchCloseXLayout->addWidget(resetYButton);
        launchCloseXLayout->addWidget(resetZButton);

        QHBoxLayout *launchCloseYLayout = new QHBoxLayout;
        
        launchCloseYLayout->addWidget(launchXButton);
        launchCloseYLayout->addWidget(launchYButton);
        launchCloseYLayout->addWidget(launchZButton);

        QHBoxLayout *launchCloseZLayout = new QHBoxLayout;
        launchCloseZLayout->addWidget(closeXButton);
        launchCloseZLayout->addWidget(closeYButton);
        launchCloseZLayout->addWidget(closeZButton);


        // Add the layouts to the main layout
        mainLayout->addLayout(launchCloseXLayout);
        mainLayout->addLayout(launchCloseYLayout);
        mainLayout->addLayout(launchCloseZLayout);




        // Connect launch buttons to their respective callbacks
        QObject::connect(launchXButton, &QPushButton::clicked, [this]() { launchPositionTickNode("position_tick_client_X"); });
        QObject::connect(launchYButton, &QPushButton::clicked, [this]() { launchPositionTickNode("position_tick_client_Y"); });
        QObject::connect(launchZButton, &QPushButton::clicked, [this]() { launchPositionTickNode("position_tick_client_Z"); });


        // Connect close buttons to terminate the processes
        QObject::connect(closeXButton, &QPushButton::clicked, [this]() { publishValue(publisher_stop_x, 1, "X"); haltMotor("first_shaft_joint"); haltMotor("second_shaft_joint"); terminatePositionTickNode(x_process_); });  
        QObject::connect(closeYButton, &QPushButton::clicked, [this]() { publishValue(publisher_stop_y, 1, "Y"); haltMotor("third_shaft_joint"); terminatePositionTickNode(y_process_);});
        QObject::connect(closeZButton, &QPushButton::clicked, [this]() { publishValue(publisher_stop_z, 1, "Z"); haltMotor("fourth_shaft_joint"); terminatePositionTickNode(z_process_); });
        

        // Connect Reset buttons
        QObject::connect(resetXButton, &QPushButton::clicked, [this]() { launch_node("Reset_Home_X");});
        QObject::connect(resetYButton, &QPushButton::clicked, [this]() { launch_node("Reset_Home_Y");});
        QObject::connect(resetZButton, &QPushButton::clicked, [this]() { launch_node("Reset_Home_Z");});




        //             // Options Layout 



        QVBoxLayout *optionsLayout = new QVBoxLayout;

        // Title for Options Layout
        QLabel *optionsTitleLabel = new QLabel("Options");
        optionsTitleLabel->setAlignment(Qt::AlignCenter);
        QFont optionsTitleFont("Arial", 14, QFont::Bold);
        optionsTitleLabel->setFont(optionsTitleFont);
        optionsLayout->addWidget(optionsTitleLabel);

        // Add a dropdown (ComboBox) to select unit
        QLabel *unitLabel = new QLabel("Select Unit:");
        QComboBox *unitComboBox = new QComboBox;
        unitComboBox->addItems({"Degrees", "Millimeters", "Revolutions"});
        unitComboBox->setStyleSheet("font-size: 14px; color: #34495e;");

        QVBoxLayout *unitLayout = new QVBoxLayout;
        unitLayout->addWidget(unitLabel);
        unitLayout->addWidget(unitComboBox);
        optionsLayout->addLayout(unitLayout);

        // Add a button to apply the selection
        QPushButton *applyButton = new QPushButton("Apply");
        applyButton->setStyleSheet("background-color: #3498db; color: white; font-weight: bold; font-size: 14px;"
                                   "padding: 10px; border-radius: 5px;");
        optionsLayout->addWidget(applyButton, 0, Qt::AlignCenter);

        // Add feedback
        QLabel *feedback_1 = new QLabel;
        QLabel *feedback_2 = new QLabel;
        QLabel *feedback_3 = new QLabel;
        QLabel *feedback_4 = new QLabel;

        feedback_1->setText(QString("Actual position motor X1: %1 degrees \n Torque X1 : %2 %")
                        .arg(0.1 * current_position_1).arg(current_torque_1/10));
        feedback_2->setText(QString("Actual position motor X2: %1 degrees \n Torque X2 : %2 %")
                        .arg(0.1 * current_position_2).arg(current_torque_2/10));
        feedback_3->setText(QString("Actual position motor Y: %1 degrees \n Torque Y : %2 %")
                        .arg(0.1 * current_position_3).arg(current_torque_3/10));
        feedback_4->setText(QString("Actual position motor Z: %1 degrees \n Torque Z : %2 %")
                        .arg(0.1 * current_position_4).arg(current_torque_4/10));

        feedback_1->setAlignment(Qt::AlignLeft);
        feedback_2->setAlignment(Qt::AlignLeft);
        feedback_3->setAlignment(Qt::AlignLeft);
        feedback_4->setAlignment(Qt::AlignLeft);

        // Add the QLabel to the layout
        optionsLayout->addWidget(feedback_1);
        optionsLayout->addWidget(feedback_2);
        optionsLayout->addWidget(feedback_3);
        optionsLayout->addWidget(feedback_4);

        // spin boxes

        QSpinBox *spinBoxVel = new QSpinBox;
        spinBoxVel->setRange(0, 200);
        spinBoxVel->setSingleStep(1);
        spinBoxVel->setAlignment(Qt::AlignCenter);

        QPushButton *enterVelButton = new QPushButton(QString("Enter Velocity"));
        enterVelButton->setFixedWidth(200);

        optionsLayout->addWidget(spinBoxVel);
        optionsLayout->addWidget(enterVelButton);

        // Add space between elements
        optionsLayout->addStretch();


        // Connect the "Apply" button to a callback
        QObject::connect(applyButton, &QPushButton::clicked, [unitComboBox, this, sliderX, SpinBoxX, sliderY, SpinBoxY, sliderZ, SpinBoxZ]() {
            QString selectedUnit = unitComboBox->currentText();
            RCLCPP_INFO(this->get_logger(), "Selected Unit: %s", selectedUnit.toStdString().c_str());

            // Update the unit coefficient
            if (selectedUnit == "Degrees") {
                unit_coefficient = 0.1;
            } else if (selectedUnit == "Millimeters") {
                unit_coefficient = 0.001388888888888888888888889;
            } else if (selectedUnit == "Revolutions") {
                unit_coefficient = 0.000277777777777777777777778;
            }

            // Update slider ranges dynamically
            updateSliderRanges(sliderX, SpinBoxX, sliderY, SpinBoxY, sliderZ, SpinBoxZ);
        });

        QTimer *timer = new QTimer;
        QObject::connect(timer, &QTimer::timeout, [feedback_1,feedback_2,feedback_3,feedback_4, this, unitComboBox]() {
            rclcpp::spin_some(this->get_node_base_interface());
            QString selectedUnit = unitComboBox->currentText();
            feedback_1->setText(QString("Actual position motor X1: %1 %2, \n Torque X1 : %3 %")
                        .arg(unit_coefficient * current_position_1)
                        .arg(selectedUnit)
                        .arg(current_torque_1/10));
            feedback_2->setText(QString("Actual position motor X2: %1 %2, \n Torque X1 : %3 %")
                        .arg(unit_coefficient * current_position_2)
                        .arg(selectedUnit)
                        .arg(current_torque_2/10));
            feedback_3->setText(QString("Actual position motor Y: %1 %2, \n Torque Y : %3 %")
                        .arg(unit_coefficient * current_position_3)
                        .arg(selectedUnit)
                        .arg(current_torque_3/10));
            feedback_4->setText(QString("Actual position motor Z: %1 %2, \n Torque Z : %3 %")
                        .arg(unit_coefficient * current_position_4)
                        .arg(selectedUnit)
                        .arg(current_torque_4/10));

        });
        timer->start(10); // Trigger every 1000ms (1 second)

        QObject::connect(enterVelButton, &QPushButton::clicked, [spinBoxVel, this]() {
            int vel_value = spinBoxVel->value();

            QProcess *process = new QProcess(nullptr);
            // Construct the command with the parameter
            QStringList arguments;
            arguments << "run" << "stepper_motor_control" << "changeVel"
                    << "--ros-args" << "-p" << QString("%1:=%2").arg("VelValue").arg(vel_value);
            
            process->start("ros2", arguments);
            RCLCPP_INFO(this->get_logger(), "Launched ChangeVel Node");
        });


        // Add both layouts to the main horizontal layout

        mainHLayout->addLayout(mainLayout);
        mainHLayout->addLayout(optionsLayout);

        // Set the layout and show the window
        window->setLayout(mainHLayout);
        // window->setFixedSize(1400, 650);  // Adjusted size for more widgets
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


    // Separate publish function for each axis
    void publishValue(rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher, int value, const std::string &axis) {
        auto message = std_msgs::msg::Int32();
        message.data = value;
        RCLCPP_INFO(this->get_logger(), "Publishing %s: %d", axis.c_str(), value);
        publisher->publish(message);
    }

    void updateSliderRanges(QSlider *sliderX, QSpinBox *spinBoxX, QSlider *sliderY, QSpinBox *spinBoxY, QSlider *sliderZ, QSpinBox *spinBoxZ) {
            sliderX->setRange(0, unit_coefficient * 257040);
            spinBoxX->setRange(0, unit_coefficient * 257040);

            sliderY->setRange(0, unit_coefficient * 135360);
            spinBoxY->setRange(0, unit_coefficient * 135360);

            sliderZ->setRange(0, unit_coefficient * 551232);
            spinBoxZ->setRange(0, unit_coefficient * 551232);
    }


    // Callback function for joint state subscription of Motor 1
    void joint_state_callback_1(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
            current_position_1 = msg->position[0];  // Store current position of Motor 1
            current_torque_1 = msg->effort[0];
        }
    }

    // Callback function for joint state subscription of Motor 2
    void joint_state_callback_2(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
            current_position_2 = msg->position[0];  // Store current position of Motor 2
            current_torque_2 = msg->effort[0];
        }
    }

    void joint_state_callback_3(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
            current_position_3 = msg->position[0];  // Store current position of Motor 3
            current_torque_3 = msg->effort[0];
        }
    }

    // Callback function for joint state subscription of Motor
    void joint_state_callback_4(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->position.empty()) {
            current_position_4 = msg->position[0];  // Store current position of Motor 4
            current_torque_4 = msg->effort[0];
        }
    }






    void launchPositionTickNode(const QString &nodeName) {
        // Check if the process for the corresponding node is already running
        if ((nodeName.contains("position_tick_client_X") && x_process_ && x_process_->state() != QProcess::NotRunning) ||
            (nodeName.contains("position_tick_client_Y") && y_process_ && y_process_->state() != QProcess::NotRunning) ||
            (nodeName.contains("position_tick_client_Z") && z_process_ && z_process_->state() != QProcess::NotRunning) ||
            (nodeName.contains("torque_tick_client_X") && x_torque_process_ && x_torque_process_->state() != QProcess::NotRunning) ||
            (nodeName.contains("torque_tick_client_Y") && y_torque_process_ && y_torque_process_->state() != QProcess::NotRunning) ||
            (nodeName.contains("torque_tick_client_Z") && z_torque_process_ && z_torque_process_->state() != QProcess::NotRunning)) 
        {
            RCLCPP_ERROR(this->get_logger(), "%s is already running, cannot launch a new one.", nodeName.toStdString().c_str());
            return;
        }

        // If not running, launch a new process
        QProcess *process = new QProcess(nullptr);
        process->start("ros2", QStringList() << "run" << "stepper_motor_control" << nodeName);
        if (!process->waitForStarted()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch %s", nodeName.toStdString().c_str());
            delete process;  // Clean up if process fails to start
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Launched %s", nodeName.toStdString().c_str());
        }

        // Keep track of the process for termination
        if (nodeName.contains("position_tick_client_X")) {
            x_process_ = process;
        } else if (nodeName.contains("position_tick_client_Y")) {
            y_process_ = process;
        } else if (nodeName.contains("position_tick_client_Z")) {
            z_process_ = process;
        } else if (nodeName.contains("torque_tick_client_X")) {
            x_torque_process_ = process;
        } else if (nodeName.contains("torque_tick_client_Y")) {
            y_torque_process_ = process;
        } else if (nodeName.contains("torque_tick_client_Z")) {
            z_torque_process_ = process;
        }
        
    }

    void launch_node(const QString &nodeName)
    {
        // If not running, launch a new process
        QProcess *process = new QProcess(nullptr);
        process->start("ros2", QStringList() << "run" << "stepper_motor_control" << nodeName);
        if (!process->waitForStarted()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch %s", nodeName.toStdString().c_str());
            delete process;  // Clean up if process fails to start
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Launched %s", nodeName.toStdString().c_str());
        }
    }




    void terminatePositionTickNode(QProcess *&process) {
        if (process && process->state() != QProcess::NotRunning) {
            process->terminate();
            if (!process->waitForFinished(3000)) {
                // If process doesn't terminate within 3 seconds, kill it
                process->kill();
                process->waitForFinished();
            }
            RCLCPP_INFO(this->get_logger(), "Node process terminated.");
            delete process;
            process = nullptr;
        } else {
            RCLCPP_INFO(this->get_logger(), "Node process is already terminated.");
        }
    }

    void haltMotor(std::string motor_name) {
        // Call halt service
        const std::string command = "ros2 service call /"+ motor_name +"/halt std_srvs/srv/Trigger";
        // Execute the command
        int result = std::system(command.c_str());
        // Check if the command succeeded
        if (result == 0) {
           std::cout << "Called halt service successfully." << std::endl;
        } else {
            std::cerr << "Halt service call failed with error code: " << result << std::endl;
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_x_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_stop_x;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_stop_y;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_stop_z;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_y_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_z_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_torque_x_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_torque_y_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_torque_z_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_1_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_2_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_3_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_4_;



    std::shared_ptr<QApplication> app_;
    
    rclcpp::TimerBase::SharedPtr check_ros_shutdown_timer_;
    int argc_ = 0;
    char **argv_ = nullptr;

    // Processes for the position tick motors
    QProcess *x_process_ = nullptr;
    QProcess *y_process_ = nullptr;
    QProcess *z_process_ = nullptr;
    QProcess *x_torque_process_ = nullptr;
    QProcess *y_torque_process_ = nullptr;
    QProcess *z_torque_process_ = nullptr;
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

#include "slider_button_node.moc"



// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <QApplication>
// #include <QWidget>
// #include <QPushButton>
// #include <QSlider>
// #include <QVBoxLayout>
// #include <QSpinBox>
// #include <QLabel>
// #include <QHBoxLayout>
// #include <QProcess>  // For launching and terminating external processes

// class SliderNode : public rclcpp::Node {
// public:
//     SliderNode() : Node("slider_node") {
//         // Publishers for X, Y, and Z positions
//         publisher_x_ = this->create_publisher<std_msgs::msg::Int32>("slider_value_x", 10);
//         publisher_y_ = this->create_publisher<std_msgs::msg::Int32>("slider_value_y", 10);
//         publisher_z_ = this->create_publisher<std_msgs::msg::Int32>("slider_value_z", 10);

//         publisher_stop_x = this->create_publisher<std_msgs::msg::Int32>("stop_motor_X", 10);
//         publisher_stop_y = this->create_publisher<std_msgs::msg::Int32>("stop_motor_Y", 10);
//         publisher_stop_z = this->create_publisher<std_msgs::msg::Int32>("stop_motor_Z", 10);

//         // Publishers for X, Y, and Z torques
//         publisher_torque_x_ = this->create_publisher<std_msgs::msg::Int32>("torque_value_x", 10);
//         publisher_torque_y_ = this->create_publisher<std_msgs::msg::Int32>("torque_value_y", 10);
//         publisher_torque_z_ = this->create_publisher<std_msgs::msg::Int32>("torque_value_z", 10);

//         // Initialize Qt
//         app_ = std::make_shared<QApplication>(argc_, argv_);

//         // Create the main window
//         QWidget *window = new QWidget;
//         window->setWindowTitle("Position and Torque Control");

//         // Main horizontal layout to hold both position and torque control
//         QHBoxLayout *mainHLayout = new QHBoxLayout;



//                     // Position control layout //



//         QVBoxLayout *mainLayout = new QVBoxLayout;

//         // Create and style the title label
//         QLabel *titleLabel = new QLabel("Position Control");
//         titleLabel->setAlignment(Qt::AlignCenter);
//         QFont titleFont("Arial", 16, QFont::Bold);
//         titleLabel->setFont(titleFont);
//         mainLayout->addWidget(titleLabel);

//         // Function to create a slider layout for each axis with a spinbox
//         auto createSliderLayout = [&](const QString &axis, QSlider *&slider, QSpinBox *&spinBox, QPushButton *&enterButton, int minValue, int maxValue) {
//             slider = new QSlider(Qt::Horizontal);
//             slider->setRange(minValue, maxValue);
//             slider->setSingleStep(1000);
//             slider->setStyleSheet("QSlider::groove:horizontal { "
//                                 "border: 1px solid #999999; "
//                                 "height: 20px; "
//                                 "background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4); "
//                                 "margin: 2px 0; "
//                                 "} "
//                                 "QSlider::handle:horizontal { "
//                                 "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f); "
//                                 "border: 1px solid #5c5c5c; "
//                                 "width: 30px; "
//                                 "margin: -2px 0px; "
//                                 "} ");

//             // Spinbox for displaying and controlling the slider value
//             spinBox = new QSpinBox;
//             spinBox->setRange(minValue, maxValue);
//             spinBox->setSingleStep(1);
//             spinBox->setAlignment(Qt::AlignCenter);
//             spinBox->setStyleSheet("font-size: 14px; color: #2c3e50;");

//             QHBoxLayout *sliderLayout = new QHBoxLayout;
//             sliderLayout->addWidget(slider);
//             sliderLayout->addWidget(spinBox);

//             enterButton = new QPushButton(QString("Enter %1").arg(axis));
//             enterButton->setFixedWidth(200);
//             enterButton->setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px;"
//                                     "padding: 10px; border-radius: 5px;");
//             QHBoxLayout *enterLayout = new QHBoxLayout;
//             enterLayout->addWidget(enterButton, 0, Qt::AlignCenter);

//             mainLayout->addLayout(sliderLayout);
//             mainLayout->addLayout(enterLayout);

//             // Connect the slider and spinbox to update each other
//             QObject::connect(slider, &QSlider::valueChanged, spinBox, &QSpinBox::setValue);
//             QObject::connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);
//         };


//         // Create sliders and enter buttons for X, Y, and Z axes
//         QSlider *sliderX, *sliderY, *sliderZ;
//         QPushButton *enterButtonX, *enterButtonY, *enterButtonZ;
//         QSpinBox *SpinBoxX,*SpinBoxY, *SpinBoxZ; 
//         createSliderLayout("X", sliderX, SpinBoxX,enterButtonX, 0, 408240);
//         createSliderLayout("Y", sliderY, SpinBoxY,enterButtonY, 0, 336240);
//         createSliderLayout("Z", sliderZ, SpinBoxZ,enterButtonZ, 0, 551232);

//         // Connect "Enter" buttons to their respective publishers
//         QObject::connect(enterButtonX, &QPushButton::clicked, [this, sliderX]() {
//             int value = sliderX->value();
//             publishValue(publisher_x_, value, "X");
//         });
//         QObject::connect(enterButtonY, &QPushButton::clicked, [this, sliderY]() {
//             int value = sliderY->value();
//             publishValue(publisher_y_, value, "Y");
//         });
//         QObject::connect(enterButtonZ, &QPushButton::clicked, [this, sliderZ]() {
//             int value = sliderZ->value();
//             publishValue(publisher_z_, value, "Z");
//         });

//         // Buttons to launch position tick motors
//         QPushButton *launchXButton = new QPushButton("Launch X Position Control");
//         QPushButton *launchYButton = new QPushButton("Launch Y Position Control");
//         QPushButton *launchZButton = new QPushButton("Launch Z Position Control");

//         auto setButtonStyle = [](QPushButton *button, const QString &color) {
//             button->setStyleSheet(QString("background-color: %1; color: white; font-weight: bold; font-size: 14px;"
//                                         "padding: 10px; border-radius: 5px;").arg(color));
//             button->setFixedWidth(220);
//         };

//         setButtonStyle(launchXButton, "#2ecc71");
//         setButtonStyle(launchYButton, "#2ecc71");
//         setButtonStyle(launchZButton, "#2ecc71");

//         // Create "Close" buttons for each motor
//         QPushButton *closeXButton = new QPushButton("Close/Stop X Position Control");
//         QPushButton *resetXButton = new QPushButton("Init/Reset X Motor");
//         QPushButton *closeYButton = new QPushButton("Close/Stop Y Position Control");
//         QPushButton *resetYButton = new QPushButton("Init/Reset Y Motor");
//         QPushButton *closeZButton = new QPushButton("Close/Stop Z Position Control");
//         QPushButton *resetZButton = new QPushButton("Init/Reset Z Motor");

//         setButtonStyle(closeXButton, "#e74c3c");  // Red color
//         setButtonStyle(closeYButton, "#e74c3c");
//         setButtonStyle(closeZButton, "#e74c3c");  

//         setButtonStyle(resetXButton, "#3c80e7");  // Blue color
//         setButtonStyle(resetYButton, "#3c80e7");
//         setButtonStyle(resetZButton, "#3c80e7");  

//         // Layout for launch and close buttons

//         QHBoxLayout *launchCloseXLayout = new QHBoxLayout;
//         launchCloseXLayout->addWidget(resetXButton);
//         launchCloseXLayout->addWidget(resetYButton);
//         launchCloseXLayout->addWidget(resetZButton);

//         QHBoxLayout *launchCloseYLayout = new QHBoxLayout;
        
//         launchCloseYLayout->addWidget(launchXButton);
//         launchCloseYLayout->addWidget(launchYButton);
//         launchCloseYLayout->addWidget(launchZButton);

//         QHBoxLayout *launchCloseZLayout = new QHBoxLayout;
//         launchCloseZLayout->addWidget(closeXButton);
//         launchCloseZLayout->addWidget(closeYButton);
//         launchCloseZLayout->addWidget(closeZButton);

//         // Add the layouts to the main layout
//         mainLayout->addLayout(launchCloseXLayout);
//         mainLayout->addLayout(launchCloseYLayout);
//         mainLayout->addLayout(launchCloseZLayout);

//         // Connect launch buttons to their respective callbacks
//         QObject::connect(launchXButton, &QPushButton::clicked, [this]() { launchPositionTickNode("position_tick_client_X"); });
//         QObject::connect(launchYButton, &QPushButton::clicked, [this]() { launchPositionTickNode("position_tick_client_Y"); });
//         QObject::connect(launchZButton, &QPushButton::clicked, [this]() { launchPositionTickNode("position_tick_client_Z"); });



//         // Connect close buttons to terminate the processes
//         QObject::connect(closeXButton, &QPushButton::clicked, [this]() { publishValue(publisher_stop_x, 1, "X"); haltMotor("first_shaft_joint"); haltMotor("second_shaft_joint"); terminatePositionTickNode(x_process_); });  
//         QObject::connect(resetXButton, &QPushButton::clicked, [this]() { launch_node("Reset_Home_X");});
//         QObject::connect(closeYButton, &QPushButton::clicked, [this]() { publishValue(publisher_stop_y, 1, "Y"); haltMotor("third_shaft_joint"); terminatePositionTickNode(y_process_);});
//         QObject::connect(resetYButton, &QPushButton::clicked, [this]() { launch_node("Reset_Home_Y");});
//         QObject::connect(closeZButton, &QPushButton::clicked, [this]() { publishValue(publisher_stop_z, 1, "Z"); haltMotor("fourth_shaft_joint"); terminatePositionTickNode(z_process_); });




//                     // Torque control layout //




//         QVBoxLayout *rightLayout = new QVBoxLayout;

//         // Create and style the title label
//         QLabel *torqueTitleLabel = new QLabel("Torque Control");
//         torqueTitleLabel->setAlignment(Qt::AlignCenter);
//         torqueTitleLabel->setFont(titleFont);
//         rightLayout->addWidget(torqueTitleLabel);

//         // Function to create a slider layout for each axis
//         auto createTorqueSliderLayout = [&](const QString &axis, QSlider *&slider, QPushButton *&enterButton) {
//             slider = new QSlider(Qt::Horizontal);
//             slider->setRange(-5000, 5000);
//             slider->setSingleStep(100);
//             slider->setValue(0);
//             slider->setStyleSheet("QSlider::handle { background-color: #3498db; }"
//                                   "QSlider::groove { background-color: #95a5a6; height: 6px; }"
//                                   "QSlider::sub-page { background-color: #2ecc71; }");

//             QLabel *valueLabel = new QLabel(QString("%1 torque: 0").arg(axis));
//             valueLabel->setAlignment(Qt::AlignCenter);
//             valueLabel->setStyleSheet("font-size: 14px; color: #2c3e50;");

//             QHBoxLayout *sliderLayout = new QHBoxLayout;
//             sliderLayout->addWidget(slider);
//             sliderLayout->addWidget(valueLabel);

//             enterButton = new QPushButton(QString("Enter %1").arg(axis));
//             enterButton->setFixedWidth(200);
//             enterButton->setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px;"
//                                        "padding: 10px; border-radius: 5px;");
//             QHBoxLayout *enterLayout = new QHBoxLayout;
//             enterLayout->addWidget(enterButton, 0, Qt::AlignCenter);

//             rightLayout->addLayout(sliderLayout);
//             rightLayout->addLayout(enterLayout);

//             // Connect the slider to update the label
//             QObject::connect(slider, &QSlider::valueChanged, [valueLabel, axis](int value) {
//                 valueLabel->setText(QString("%1 torque: %2").arg(axis).arg(value));
//             });
//         };

//         // Create sliders and enter buttons for X, Y, and Z axes
//         QSlider *torqueSliderX, *torqueSliderY, *torqueSliderZ;
//         QPushButton *enterButtonTorqueX, *enterButtonTorqueY, *enterButtonTorqueZ;
//         createTorqueSliderLayout("X", torqueSliderX, enterButtonTorqueX);
//         createTorqueSliderLayout("Y", torqueSliderY, enterButtonTorqueY);
//         createTorqueSliderLayout("Z", torqueSliderZ, enterButtonTorqueZ);

//         // Connect "Enter" buttons to their respective publishers
//         QObject::connect(enterButtonTorqueX, &QPushButton::clicked, [this, torqueSliderX]() {
//             int value = torqueSliderX->value();
//             publishValue(publisher_torque_x_, value, "X");
//         });
//         QObject::connect(enterButtonTorqueY, &QPushButton::clicked, [this, torqueSliderY]() {
//             int value = torqueSliderY->value();
//             publishValue(publisher_torque_y_, value, "Y");
//         });
//         QObject::connect(enterButtonTorqueZ, &QPushButton::clicked, [this, torqueSliderZ]() {
//             int value = torqueSliderZ->value();
//             publishValue(publisher_torque_z_, value, "Z");
//         });

//         // Buttons to launch position tick motors
//         QPushButton *launchXButtonTorque = new QPushButton("Launch X Torque Tick Motors");
//         QPushButton *launchYButtonTorque = new QPushButton("Launch Y Torque Tick Motor");
//         QPushButton *launchZButtonTorque = new QPushButton("Launch Z Torque Tick Motor");

//         setButtonStyle(launchXButtonTorque, "#2ecc71");
//         setButtonStyle(launchYButtonTorque, "#2ecc71");
//         setButtonStyle(launchZButtonTorque, "#2ecc71");

//         // Create "Close" buttons for each motor
//         QPushButton *closeXButtonTorque = new QPushButton("Close X Torque Tick Motors");
//         QPushButton *closeYButtonTorque = new QPushButton("Close Y Torque Tick Motor");
//         QPushButton *closeZButtonTorque = new QPushButton("Close Z Torque Tick Motor");

//         setButtonStyle(closeXButtonTorque, "#e74c3c");  // Red color
//         setButtonStyle(closeYButtonTorque, "#e74c3c");
//         setButtonStyle(closeZButtonTorque, "#e74c3c");    

//         // Layout for launch and close buttons

//         QHBoxLayout *launchCloseXTorqueLayout = new QHBoxLayout;
//         launchCloseXTorqueLayout->addWidget(launchXButtonTorque);
//         launchCloseXTorqueLayout->addWidget(closeXButtonTorque);

//         QHBoxLayout *launchCloseYTorqueLayout = new QHBoxLayout;
//         launchCloseYTorqueLayout->addWidget(launchYButtonTorque);
//         launchCloseYTorqueLayout->addWidget(closeYButtonTorque);

//         QHBoxLayout *launchCloseZTorqueLayout = new QHBoxLayout;
//         launchCloseZTorqueLayout->addWidget(launchZButtonTorque);
//         launchCloseZTorqueLayout->addWidget(closeZButtonTorque);

//         // Add the layouts to the main layout
//         rightLayout->addLayout(launchCloseXTorqueLayout);
//         rightLayout->addLayout(launchCloseYTorqueLayout);
//         rightLayout->addLayout(launchCloseZTorqueLayout);

//         // Connect launch buttons to their respective callbacks
//         QObject::connect(launchXButtonTorque, &QPushButton::clicked, [this]() { launchPositionTickNode("torque_tick_client_X"); });
//         QObject::connect(launchYButtonTorque, &QPushButton::clicked, [this]() { launchPositionTickNode("torque_tick_client_Y"); });
//         QObject::connect(launchZButtonTorque, &QPushButton::clicked, [this]() { launchPositionTickNode("torque_tick_client_Z"); });

//         // Connect close buttons to terminate the processes
//         QObject::connect(closeXButtonTorque, &QPushButton::clicked, [this]() { terminatePositionTickNode(x_torque_process_); });
//         QObject::connect(closeYButtonTorque, &QPushButton::clicked, [this]() { terminatePositionTickNode(y_torque_process_); });
//         QObject::connect(closeZButtonTorque, &QPushButton::clicked, [this]() { terminatePositionTickNode(z_torque_process_); });



//         // Add both layouts to the main horizontal layout
//         mainHLayout->addLayout(mainLayout);
//         // mainHLayout->addLayout(rightLayout);
//         // Set the layout and show the window
//         window->setLayout(mainHLayout);
//         // window->setFixedSize(1000, 650);  // Adjusted size for more widgets
//         window->show();

//         // Timer to periodically check if ROS2 is still running
//         check_ros_shutdown_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100), [this]() {
//                 if (!rclcpp::ok()) {
//                     app_->quit();  // Quit Qt when ROS2 shuts down
//                 }
//             });
//     }

//     // Run the Qt application
//     void run() {
//         app_->exec();
//     }

// private:
//     // Separate publish function for each axis
//     void publishValue(rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher, int value, const std::string &axis) {
//         auto message = std_msgs::msg::Int32();
//         message.data = value;
//         RCLCPP_INFO(this->get_logger(), "Publishing %s: %d", axis.c_str(), value);
//         publisher->publish(message);
//     }

//     void launchPositionTickNode(const QString &nodeName) {
//         // Check if the process for the corresponding node is already running
//         if ((nodeName.contains("position_tick_client_X") && x_process_ && x_process_->state() != QProcess::NotRunning) ||
//             (nodeName.contains("position_tick_client_Y") && y_process_ && y_process_->state() != QProcess::NotRunning) ||
//             (nodeName.contains("position_tick_client_Z") && z_process_ && z_process_->state() != QProcess::NotRunning) ||
//             (nodeName.contains("torque_tick_client_X") && x_torque_process_ && x_torque_process_->state() != QProcess::NotRunning) ||
//             (nodeName.contains("torque_tick_client_Y") && y_torque_process_ && y_torque_process_->state() != QProcess::NotRunning) ||
//             (nodeName.contains("torque_tick_client_Z") && z_torque_process_ && z_torque_process_->state() != QProcess::NotRunning)) 
//         {
//             RCLCPP_ERROR(this->get_logger(), "%s is already running, cannot launch a new one.", nodeName.toStdString().c_str());
//             return;
//         }

//         // If not running, launch a new process
//         QProcess *process = new QProcess(nullptr);
//         process->start("ros2", QStringList() << "run" << "stepper_motor_control" << nodeName);
//         if (!process->waitForStarted()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to launch %s", nodeName.toStdString().c_str());
//             delete process;  // Clean up if process fails to start
//             return;
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Launched %s", nodeName.toStdString().c_str());
//         }

//         // Keep track of the process for termination
//         if (nodeName.contains("position_tick_client_X")) {
//             x_process_ = process;
//         } else if (nodeName.contains("position_tick_client_Y")) {
//             y_process_ = process;
//         } else if (nodeName.contains("position_tick_client_Z")) {
//             z_process_ = process;
//         } else if (nodeName.contains("torque_tick_client_X")) {
//             x_torque_process_ = process;
//         } else if (nodeName.contains("torque_tick_client_Y")) {
//             y_torque_process_ = process;
//         } else if (nodeName.contains("torque_tick_client_Z")) {
//             z_torque_process_ = process;
//         }
        
//     }

//     void launch_node(const QString &nodeName)
//     {
//         // If not running, launch a new process
//         QProcess *process = new QProcess(nullptr);
//         process->start("ros2", QStringList() << "run" << "stepper_motor_control" << nodeName);
//         if (!process->waitForStarted()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to launch %s", nodeName.toStdString().c_str());
//             delete process;  // Clean up if process fails to start
//             return;
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Launched %s", nodeName.toStdString().c_str());
//         }
//     }


//     void terminatePositionTickNode(QProcess *&process) {
//         if (process && process->state() != QProcess::NotRunning) {
//             process->terminate();
//             if (!process->waitForFinished(3000)) {
//                 // If process doesn't terminate within 3 seconds, kill it
//                 process->kill();
//                 process->waitForFinished();
//             }
//             RCLCPP_INFO(this->get_logger(), "Node process terminated.");
//             delete process;
//             process = nullptr;
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Node process is already terminated.");
//         }
//     }

//     void haltMotor(std::string motor_name) {
//         // Call halt service
//         const std::string command = "ros2 service call /"+ motor_name +"/halt std_srvs/srv/Trigger";
//         // Execute the command
//         int result = std::system(command.c_str());
//         // Check if the command succeeded
//         if (result == 0) {
//            std::cout << "Called halt service successfully." << std::endl;
//         } else {
//             std::cerr << "Halt service call failed with error code: " << result << std::endl;
//         }
//     }

//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_x_;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_stop_x;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_stop_y;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_stop_z;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_y_;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_z_;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_torque_x_;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_torque_y_;
//     rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_torque_z_;
//     std::shared_ptr<QApplication> app_;
//     rclcpp::TimerBase::SharedPtr check_ros_shutdown_timer_;
//     int argc_ = 0;
//     char **argv_ = nullptr;

//     // Processes for the position tick motors
//     QProcess *x_process_ = nullptr;
//     QProcess *y_process_ = nullptr;
//     QProcess *z_process_ = nullptr;
//     QProcess *x_torque_process_ = nullptr;
//     QProcess *y_torque_process_ = nullptr;
//     QProcess *z_torque_process_ = nullptr;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     // Create and run the node
//     auto node = std::make_shared<SliderNode>();
//     node->run();

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
