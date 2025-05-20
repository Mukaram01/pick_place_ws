#include "cell_gui/main_window.hpp"

#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[])
{
  // Initialize Qt
  QApplication app(argc, argv);
  QApplication::setApplicationName("Pick and Place Cell GUI");
  QApplication::setOrganizationName("PickPlaceDemo");
  
  // Create and show the main window
  cell_gui::MainWindow w;
  w.show();
  
  // Start the Qt event loop
  return app.exec();
}
