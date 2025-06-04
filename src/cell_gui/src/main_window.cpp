#include "cell_gui/main_window.hpp"
#include "ui_main_window.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QTimer>
#include <QProcess>
#include <QDebug>
#include <fstream>

namespace cell_gui
{

MainWindow::MainWindow(QWidget* parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow),
  system_running_(false)
{
  // Setup UI
  ui->setupUi(this);
  
  // Setup initial button states
  ui->stopButton->setEnabled(false);
  
  // Initialize ROS node
  initROS();
  
  // Initialize processes
  simulation_process_ = new QProcess(this);
  demo_process_ = new QProcess(this);
  
  // Connect process signals
  connect(simulation_process_, &QProcess::readyReadStandardOutput, [this]() {
    QString output = QString::fromLocal8Bit(simulation_process_->readAllStandardOutput());
    qDebug() << "Simulation: " << output;
  });

  connect(simulation_process_, &QProcess::errorOccurred, [this](QProcess::ProcessError error) {
    processError("Simulation", error);
  });

  connect(simulation_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          [this](int exitCode, QProcess::ExitStatus status) {
            processFinished("Simulation", exitCode, status);
          });
  
  connect(demo_process_, &QProcess::readyReadStandardOutput, [this]() {
    QString output = QString::fromLocal8Bit(demo_process_->readAllStandardOutput());
    qDebug() << "Demo: " << output;
  });

  connect(demo_process_, &QProcess::errorOccurred, [this](QProcess::ProcessError error) {
    processError("Demo", error);
  });

  connect(demo_process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          [this](int exitCode, QProcess::ExitStatus status) {
            processFinished("Demo", exitCode, status);
          });
  
  // Setup timer for periodic status updates
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &MainWindow::timerCallback);
  status_timer_->start(1000);  // Update every second
  
  // Load previous settings
  loadSettings();
  
  qDebug() << "GUI initialized";
}

MainWindow::~MainWindow()
{
  // Stop any running processes
  stopAllProcesses();
  
  // Save settings
  saveSettings();
  
  // Clean up
  delete ui;
}

void MainWindow::initROS()
{
  // Initialize ROS node
  rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("cell_gui_node");
  
  // Create publisher for commands
  command_pub_ = node_->create_publisher<std_msgs::msg::String>("gui_commands", 10);
  
  // Create subscription for status updates
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "system_status", 10, 
    [this](const std_msgs::msg::String::SharedPtr msg) {
      // Process status message and update UI
      updateSystemStatus(QString::fromStdString(msg->data));
    });
  
  qDebug() << "ROS node initialized";
}

void MainWindow::timerCallback()
{
  // Execute ROS callbacks
  rclcpp::spin_some(node_);
  
  // Update GUI with latest stats (in a real system these would come from ROS topics)
  if (system_running_) {
    // In a real system, these values would be updated from ROS topics
    // For simulation, we'll use some random values as placeholders
    static double cycle_time = 5.0;
    static double success_rate = 90.0;
    static int cycle_count = 0;
    
    if (rand() % 10 == 0) {  // Occasionally update values
      cycle_time = 4.5 + (rand() % 10) / 10.0;
      success_rate = 85.0 + (rand() % 15);
      cycle_count++;
      
      updateCycleTime(cycle_time);
      updateSuccessRate(success_rate);
    }
  }
}

void MainWindow::on_browseURDFButton_clicked()
{
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Open URDF File"), "", tr("URDF Files (*.urdf *.xacro);;All Files (*)"));
    
  if (!fileName.isEmpty()) {
    ui->urdfPathLineEdit->setText(fileName);
  }
}

void MainWindow::on_browseModelButton_clicked()
{
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Open Model File"), "", tr("Model Files (*.onnx *.pt *.tflite);;All Files (*)"));
    
  if (!fileName.isEmpty()) {
    ui->modelPathLineEdit->setText(fileName);
  }
}

void MainWindow::on_applyConfigButton_clicked()
{
  // Save current configuration
  saveSettings();
  
  QMessageBox::information(this, tr("Configuration"),
                         tr("Configuration saved. To apply changes, restart the system."));
}

void MainWindow::on_applyVisionButton_clicked()
{
  // Get vision settings
  QString pipeline;
  if (ui->simpleColorRadioButton->isChecked()) {
    pipeline = "simple_color";
  } else if (ui->yoloRadioButton->isChecked()) {
    pipeline = "yolo";
  } else if (ui->segmentationRadioButton->isChecked()) {
    pipeline = "segmentation";
  }
  
  double confidence = ui->confidenceSpinBox->value();
  double depth_limit = ui->depthLimitSpinBox->value();
  
  // Publish vision settings
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "vision_settings:" + pipeline.toStdString() + 
             ":" + std::to_string(confidence) + 
             ":" + std::to_string(depth_limit);
  command_pub_->publish(std::move(msg));
  
  QMessageBox::information(this, tr("Vision Settings"),
                         tr("Vision settings applied."));
}

void MainWindow::on_startButton_clicked()
{
  if (!system_running_) {
    // Start the simulation and demo
    launchSimulation();
    launchPickPlaceDemo();
    
    system_running_ = true;
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(true);
    
    updateSystemStatus("System starting...");
  }
}

void MainWindow::on_stopButton_clicked()
{
  if (system_running_) {
    // Stop all processes
    stopAllProcesses();
    
    system_running_ = false;
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(false);
    
    updateSystemStatus("System stopped");
  }
}

void MainWindow::launchSimulation()
{
  // Get gripper type from UI
  QString gripper_type = ui->suctionRadioButton->isChecked() ? "suction" : "two_finger";
  
  // Get conveyor settings
  double belt_length = ui->beltLengthSpinBox->value();
  double belt_width = ui->beltWidthSpinBox->value();
  double belt_height = ui->beltHeightSpinBox->value();
  double belt_speed = ui->beltSpeedSpinBox->value();
  
  // Create launch command
  QStringList args;
  args << "launch" << "cell_description" << "gazebo_simulation.launch.py"
       << "gripper_type:=" + gripper_type
       << "belt_length:=" + QString::number(belt_length)
       << "belt_width:=" + QString::number(belt_width)
       << "belt_height:=" + QString::number(belt_height)
       << "belt_speed:=" + QString::number(belt_speed)
       << "use_sim_time:=true";
  
  qDebug() << "Launching simulation with args:" << args.join(" ");
  
  // Start the simulation process
  simulation_process_->start("ros2", args);
  
  // Wait a bit for Gazebo to start up
  QThread::sleep(5);
}

void MainWindow::launchPickPlaceDemo()
{
  // Get gripper type and vision pipeline
  QString gripper_type = ui->suctionRadioButton->isChecked() ? "suction" : "two_finger";
  
  QString vision_pipeline;
  if (ui->simpleColorRadioButton->isChecked()) {
    vision_pipeline = "simple_color";
  } else if (ui->yoloRadioButton->isChecked()) {
    vision_pipeline = "yolo";
  } else if (ui->segmentationRadioButton->isChecked()) {
    vision_pipeline = "segmentation";
  }
  
  // Get place pose settings
  double place_pose_x = ui->placePoseXSpinBox->value();
  double place_pose_y = ui->placePoseYSpinBox->value();
  double place_pose_z = ui->placePoseZSpinBox->value();
  
  // Create launch command
  QStringList args;
  args << "launch" << "pick_place_demo" << "pick_place_demo.launch.py"
       << "gripper_type:=" + gripper_type
       << "vision_pipeline:=" + vision_pipeline
       << "use_sim:=true"
       << "use_sim_time:=true";
  
  qDebug() << "Launching pick and place demo with args:" << args.join(" ");
  
  // Start the demo process
  demo_process_->start("ros2", args);
}

void MainWindow::stopAllProcesses()
{
  // Stop the pick and place demo
  if (demo_process_->state() == QProcess::Running) {
    demo_process_->terminate();
    if (!demo_process_->waitForFinished(3000)) {
      demo_process_->kill();
    }
  }
  
  // Stop the simulation
  if (simulation_process_->state() == QProcess::Running) {
    simulation_process_->terminate();
    if (!simulation_process_->waitForFinished(3000)) {
      simulation_process_->kill();
    }
  }
  
  qDebug() << "All processes stopped";
}

void MainWindow::updateSystemStatus(const QString& status)
{
  ui->systemStatusLabel->setText("System Status: " + status);
}

void MainWindow::updateCycleTime(double time)
{
  ui->cycleTimeLabel->setText(QString("Last Cycle Time: %1 sec").arg(time, 0, 'f', 2));
}

void MainWindow::updateSuccessRate(double rate)
{
  ui->successRateLabel->setText(QString("Success Rate: %1%").arg(rate, 0, 'f', 1));
}

void MainWindow::saveSettings()
{
  QSettings settings("PickPlaceDemo", "CellGUI");
  
  // Save robot settings
  settings.setValue("robot/type", ui->robotTypeComboBox->currentText());
  settings.setValue("robot/urdf_path", ui->urdfPathLineEdit->text());
  
  // Save gripper settings
  settings.setValue("gripper/suction", ui->suctionRadioButton->isChecked());
  
  // Save conveyor settings
  settings.setValue("conveyor/length", ui->beltLengthSpinBox->value());
  settings.setValue("conveyor/width", ui->beltWidthSpinBox->value());
  settings.setValue("conveyor/height", ui->beltHeightSpinBox->value());
  settings.setValue("conveyor/speed", ui->beltSpeedSpinBox->value());
  
  // Save vision settings
  settings.setValue("vision/simple_color", ui->simpleColorRadioButton->isChecked());
  settings.setValue("vision/yolo", ui->yoloRadioButton->isChecked());
  settings.setValue("vision/segmentation", ui->segmentationRadioButton->isChecked());
  settings.setValue("vision/model_path", ui->modelPathLineEdit->text());
  settings.setValue("vision/confidence", ui->confidenceSpinBox->value());
  settings.setValue("vision/depth_limit", ui->depthLimitSpinBox->value());
  
  // Save pick place settings
  settings.setValue("pick_place/pose_x", ui->placePoseXSpinBox->value());
  settings.setValue("pick_place/pose_y", ui->placePoseYSpinBox->value());
  settings.setValue("pick_place/pose_z", ui->placePoseZSpinBox->value());
}

void MainWindow::loadSettings()
{
  QSettings settings("PickPlaceDemo", "CellGUI");
  
  // Load robot settings
  int index = ui->robotTypeComboBox->findText(settings.value("robot/type").toString());
  if (index >= 0) {
    ui->robotTypeComboBox->setCurrentIndex(index);
  }
  ui->urdfPathLineEdit->setText(settings.value("robot/urdf_path").toString());
  
  // Load gripper settings
  bool suction = settings.value("gripper/suction", true).toBool();
  ui->suctionRadioButton->setChecked(suction);
  ui->twoFingerRadioButton->setChecked(!suction);
  
  // Load conveyor settings
  ui->beltLengthSpinBox->setValue(settings.value("conveyor/length", 2.0).toDouble());
  ui->beltWidthSpinBox->setValue(settings.value("conveyor/width", 0.5).toDouble());
  ui->beltHeightSpinBox->setValue(settings.value("conveyor/height", 0.6).toDouble());
  ui->beltSpeedSpinBox->setValue(settings.value("conveyor/speed", 0.2).toDouble());
  
  // Load vision settings
  bool simple_color = settings.value("vision/simple_color", true).toBool();
  bool yolo = settings.value("vision/yolo", false).toBool();
  bool segmentation = settings.value("vision/segmentation", false).toBool();
  
  ui->simpleColorRadioButton->setChecked(simple_color);
  ui->yoloRadioButton->setChecked(yolo);
  ui->segmentationRadioButton->setChecked(segmentation);
  
  ui->modelPathLineEdit->setText(settings.value("vision/model_path").toString());
  ui->confidenceSpinBox->setValue(settings.value("vision/confidence", 0.5).toDouble());
  ui->depthLimitSpinBox->setValue(settings.value("vision/depth_limit", 3.0).toDouble());
  
  // Load pick place settings
  ui->placePoseXSpinBox->setValue(settings.value("pick_place/pose_x", 0.4).toDouble());
  ui->placePoseYSpinBox->setValue(settings.value("pick_place/pose_y", -0.3).toDouble());
  ui->placePoseZSpinBox->setValue(settings.value("pick_place/pose_z", 0.85).toDouble());
}

void MainWindow::on_actionSave_Configuration_triggered()
{
  QString fileName = QFileDialog::getSaveFileName(this,
    tr("Save Configuration"), "", tr("Config Files (*.ini);;All Files (*)"));
    
  if (!fileName.isEmpty()) {
    // First save to internal settings
    saveSettings();
    
    // Then copy to the requested file
    QSettings currentSettings("PickPlaceDemo", "CellGUI");
    QSettings customSettings(fileName, QSettings::IniFormat);
    
    for (const QString& key : currentSettings.allKeys()) {
      customSettings.setValue(key, currentSettings.value(key));
    }
    
    QMessageBox::information(this, tr("Save Configuration"),
                           tr("Configuration saved to file."));
  }
}

void MainWindow::on_actionLoad_Configuration_triggered()
{
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Load Configuration"), "", tr("Config Files (*.ini);;All Files (*)"));
    
  if (!fileName.isEmpty()) {
    QSettings customSettings(fileName, QSettings::IniFormat);
    QSettings currentSettings("PickPlaceDemo", "CellGUI");
    
    // Clear current settings
    for (const QString& key : currentSettings.allKeys()) {
      currentSettings.remove(key);
    }
    
    // Copy from custom file
    for (const QString& key : customSettings.allKeys()) {
      currentSettings.setValue(key, customSettings.value(key));
    }
    
    // Reload settings to UI
    loadSettings();
    
    QMessageBox::information(this, tr("Load Configuration"),
                           tr("Configuration loaded from file."));
  }
}

void MainWindow::on_actionExit_triggered()
{
  close();
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About Pick and Place Cell GUI"),
    tr("<h2>Pick and Place Cell GUI</h2>"
       "<p>Version 1.0</p>"
       "<p>A configurable interface for a vision-guided pick-and-place cell "
       "using ROS 2 Humble on Ubuntu 22.04.</p>"
       "<p>Features:</p>"
       "<ul>"
       "<li>Robot, gripper, and conveyor configuration</li>"
       "<li>Vision pipeline selection</li>"
       "<li>Pick and place operation control</li>"
       "</ul>"));
}

void MainWindow::processFinished(const QString& name, int exitCode, QProcess::ExitStatus status)
{
  if (status != QProcess::NormalExit || exitCode != 0) {
    QString msg = QString("%1 process exited with code %2").arg(name).arg(exitCode);
    QMessageBox::warning(this, tr("Process Finished"), msg);
    qWarning() << msg;
  } else {
    qDebug() << name << "process finished.";
  }

  if (simulation_process_->state() != QProcess::Running &&
      demo_process_->state() != QProcess::Running) {
    system_running_ = false;
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(false);
    updateSystemStatus("System stopped");
  } else {
    updateSystemStatus(name + " finished");
  }
}

void MainWindow::processError(const QString& name, QProcess::ProcessError error)
{
  QString msg = QString("%1 process error: %2").arg(name).arg(static_cast<int>(error));
  QMessageBox::warning(this, tr("Process Error"), msg);
  qWarning() << msg;

  if (simulation_process_->state() != QProcess::Running &&
      demo_process_->state() != QProcess::Running) {
    system_running_ = false;
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(false);
  }

  updateSystemStatus(name + " error");
}

} // namespace cell_gui
