#ifndef CELL_GUI_MAIN_WINDOW_HPP
#define CELL_GUI_MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QProcess>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace Ui {
class MainWindow;
}

namespace cell_gui
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

private slots:
  void on_browseURDFButton_clicked();
  void on_browseModelButton_clicked();
  void on_applyConfigButton_clicked();
  void on_applyVisionButton_clicked();
  void on_startButton_clicked();
  void on_stopButton_clicked();
  void on_actionSave_Configuration_triggered();
  void on_actionLoad_Configuration_triggered();
  void on_actionExit_triggered();
  void on_actionAbout_triggered();
  
  void updateSystemStatus(const QString& status);
  void updateCycleTime(double time);
  void updateSuccessRate(double rate);
  
private:
  void initROS();
  void saveSettings();
  void loadSettings();
  void launchSimulation();
  void launchPickPlaceDemo();
  void stopAllProcesses();
  void timerCallback();
  
  Ui::MainWindow* ui;
  
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  
  QProcess* simulation_process_;
  QProcess* demo_process_;
  QTimer* status_timer_;
  
  std::string config_path_;
  bool system_running_;
};

} // namespace cell_gui

#endif // CELL_GUI_MAIN_WINDOW_HPP
