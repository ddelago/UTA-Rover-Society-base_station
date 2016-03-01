/************************************************************************************************************************
 * position_plugin.cpp
 * This plugin allows GPS tracking using a ROS NavSatFix message.
 * 
 * @TODO 
 * Connect entered coordinates and GPS Position with a line
 * Editing of entered coordinates
 **********************************************************************************************************************/

// QT Includes
#include <QApplication>
#include <QStandardItemModel>
#include <QValidator>

// Marble Includes
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/MapThemeManager.h>
#include <marble/MarbleModel.h>
#include <marble/MarbleWidget.h>

// Own Includes
#include "rqt_position_plugin/position_plugin.h"

// ROS Plugin Include
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

namespace rqt_position_plugin {

// Default constructor
PositionPlugin::PositionPlugin()
   : rqt_gui_cpp::Plugin(), widget_(0) {
  // name QObjects
  setObjectName("PositionPlugin");
}

/***********************************************************************************************************************
 * This function is called when the user clicks plot. Creates the
 * objects for the waypoint and adds to appropriate data structs.
 *
 * @AUTHOR Robert Hamilton
 **********************************************************************************************************************/
void PositionPlugin::addWaypoint() {
  Marble::GeoDataPlacemark *tmp_placemark = new Marble::GeoDataPlacemark;
  tmp_placemark->setName(ui_.lineEdit_name->text());

  float lon = ui_.lineEdit_lon->text().toFloat();
  float lat = ui_.lineEdit_lat->text().toFloat();

  tmp_placemark->setCoordinate(lat, lon, 0, Marble::GeoDataCoordinates::Degree);

  Marble::GeoDataDocument *tmp_doc = new Marble::GeoDataDocument;
  tmp_doc->append(tmp_placemark);
  ui_.MarbleWidget->model()->treeModel()->addDocument(tmp_doc);

  waypoints.push_back(tmp_placemark);
  plotWaypoint();
  addWaypointsToListWidget();
}

/***********************************************************************************************************************
 * Changes the current ROS GPS topic on when the user changes
 * the GPS topic combobox.
 *
 * @PARAM topic_name a pointer to a QString containing the new GPS topic name
 *
 * @AUTHOR Tobias Baer
 **********************************************************************************************************************/
void PositionPlugin::changeGPSTopic(const QString &topic_name) {
  current_pos_subscriber.shutdown();
  current_pos_subscriber = getNodeHandle().subscribe<sensor_msgs::NavSatFix>(
     topic_name.toStdString().c_str(), 1, &PositionPlugin::GPSCallback, this);

  int idx = ui_.comboBox_gps->findText(topic_name);
  if (idx != -1)
    ui_.comboBox_gps->setCurrentIndex(idx);
}

/***********************************************************************************************************************
 * Changes the current map theme when the user changes
 * the map theme combobox.
 *
 * @PARAM idx The index of the map theme combobox
 *
 * @AUTHOR Tobias Baer
 **********************************************************************************************************************/
void PositionPlugin::changeMarbleTheme(int idx) {
  QStandardItemModel *model = m_map_theme_manager.mapThemeModel();
  QModelIndex index = model->index(idx, 0);
  QString theme = model->data(index, Qt::UserRole + 1).toString();

  ui_.MarbleWidget->setMapThemeId(theme);
}

/***********************************************************************************************************************
 * Iterate through all ROS topics and add those matching NavSatFix to the GPS combobox.
 * 
 * @AUTHOR Tobias Baer
 * @AUTHOR Robert Hamilton
 **********************************************************************************************************************/
void PositionPlugin::findNavSatFixTopics() {
  using namespace ros::master;

  std::vector<TopicInfo> topic_infos;
  getTopics(topic_infos);

  // Update combobox with available GPS topics
  ui_.comboBox_gps->clear();
  for (std::vector<TopicInfo>::iterator it = topic_infos.begin(); it != topic_infos.end(); it++) {
    TopicInfo topic = (TopicInfo) (*it);

    if (topic.datatype.compare("sensor_msgs/NavSatFix") == 0) {
      QString lineEdit_string(topic.name.c_str());
      ui_.comboBox_gps->addItem(lineEdit_string);
    }
  }
}

/*********************************************************************************************************************** 
 * Callback for the ROS GPS publisher. 
 *
 * @PARAM gps_ptr a pointer to a ROS NavSatFix msg containing GPS coordinates
 *
 * @AUTHOR Tobias Baer
 * @AUTHOR Robert Hamilton
 **********************************************************************************************************************/
void PositionPlugin::GPSCallback(const sensor_msgs::NavSatFixConstPtr &gps_ptr) {
  Marble::GeoDataCoordinates coord(Marble::GeoDataCoordinates(gps_ptr->longitude, gps_ptr->latitude, gps_ptr->altitude,
                                                              Marble::GeoDataCoordinates::Degree));

  emit coordinatesChanged(coord);
}

/***********************************************************************************************************************
 * Initialize plugin
 *
 * @PARAM context pointer to the rqt plugin connect
 *
 * @AUTHOR Tobias Baer
 * @AUTHOR Robert Hamilton
 **********************************************************************************************************************/
void PositionPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  // Command Line Args
  QStringList argv = context.argv();

  // create QWidget
  widget_ = new QWidget();

  // add widget to UI
  ui_.setupUi(widget_);

  // Set up MarbleWidget
  ui_.MarbleWidget->setMapThemeId("earth/googlesat/googlesat.dgml");
  ui_.MarbleWidget->setProjection(Marble::Mercator);

  // Center map to given position
  Marble::GeoDataCoordinates hanksville(-110.791955, 38.406285, 0.0, Marble::GeoDataCoordinates::Degree);
  ui_.MarbleWidget->centerOn(hanksville);
  ui_.MarbleWidget->setDistance(0.001);

  context.addWidget(widget_);
  ui_.comboBox_theme->setModel(m_map_theme_manager.mapThemeModel());
  ui_.lineEdit_lon->setValidator(new QDoubleValidator());
  ui_.lineEdit_lat->setValidator(new QDoubleValidator());

  // Create tracker placemark
  m_tracker_placemark = new Marble::GeoDataPlacemark("Rover");
  m_document = new Marble::GeoDataDocument;
  m_document->append(m_tracker_placemark);
  ui_.MarbleWidget->model()->treeModel()->addDocument(m_document);

  // Initial rover placemark 
  setTrackerCoordinates(hanksville);

  // Update the position label when the mouse moves
  //connect( ui_.MarbleWidget, SIGNAL( mouseMoveGeoPosition( QString ) ),
  //  ui_.label_position, SLOT( setText( QString ) ) );

  // Update GPS Topic on user selection
  connect(ui_.comboBox_gps, SIGNAL(activated(
                                      const QString & )),
          this, SLOT(changeGPSTopic(
                        const QString & )));

  // Refresh GPS Topics on user push
  connect(ui_.pushButton_refresh, SIGNAL(clicked()),
          this, SLOT(findNavSatFixTopics()));

  // Update map theme on user selection
//  connect( ui_.comboBox_theme, SIGNAL( currentIndexChanged( int ) ),
//    this, SLOT( changeMarbleTheme( int ) ) );

  // Signal for ROS GPS publisher callback
  connect(this, SIGNAL(coordinatesChanged(Marble::GeoDataCoordinates)),
          this, SLOT(setTrackerCoordinates(Marble::GeoDataCoordinates)),
          Qt::BlockingQueuedConnection);

  // Add waypoint
  connect(ui_.pushButton_plot, SIGNAL(clicked()),
          this, SLOT(addWaypoint()));

  // Edit waypoint
  connect(ui_.pushButton_edit, SIGNAL(clicked()),
          this, SLOT(editWaypoint()));

}

/***********************************************************************************************************************
 * Update the location of all the waypoints
 *
 *
 * @AUTHOR Robert Hamilton
 **********************************************************************************************************************/
void PositionPlugin::plotWaypoint() {
  for (std::vector<Marble::GeoDataPlacemark *>::iterator it = waypoints.begin(); it != waypoints.end(); it++) {
    ui_.MarbleWidget->model()->treeModel()->updateFeature(*it);
  }

  ui_.MarbleWidget->update();
}

/***********************************************************************************************************************
 * Restore the settings specific to this plugin
 *
 * @PARAM plugin_settings a pointer to the plugin settings
 * @PARAM instance_settings a pointer to the instance settings
 *
 * @AUTHOR Tobias Baer
 **********************************************************************************************************************/
void PositionPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                     const qt_gui_cpp::Settings &instance_settings) {
  const QString topic = instance_settings.value("position_plugin_gps_topic").toString();
  changeGPSTopic(topic);

  ui_.MarbleWidget->setDistance(instance_settings.value("position_plugin_zoom", 0.05).toReal());
  ui_.comboBox_theme->setCurrentIndex(instance_settings.value("position_plugin_theme", 0).toInt());
}

/***********************************************************************************************************************
 * Save the settings specific to this plugin
 * 
 * @AUTHOR Tobias Baer
 **********************************************************************************************************************/
void PositionPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                  qt_gui_cpp::Settings &instance_settings) const {
  QString topic(current_pos_subscriber.getTopic().c_str());
  instance_settings.setValue("position_plugin_gps_topic", topic);

  instance_settings.setValue("position_plugin_zoom", ui_.MarbleWidget->distance());
  instance_settings.setValue("position_plugin_theme", ui_.comboBox_theme->currentIndex());
}

/***********************************************************************************************************************
 * Update the tracker placemark. Called by QT S/S.
 * 
 * @PARAM coord a pointer to Marble GeoDataCoordinates containing 
 * the updated coordinates
 *
 * @AUTHOR Robert Hamilton
 **********************************************************************************************************************/
void PositionPlugin::setTrackerCoordinates(const Marble::GeoDataCoordinates &coord) {
  m_tracker_placemark->setCoordinate(coord);
  ui_.MarbleWidget->model()->treeModel()->updateFeature(m_tracker_placemark);
}

/***********************************************************************************************************************
 * Called when the user closes the plugin. 
 **********************************************************************************************************************/
void PositionPlugin::shutdownPlugin() {
  // Unregister from publishers
  current_pos_subscriber.shutdown();
}

/*
 * These methods aren't used. Just ROS defaults. May use later.
bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void PositionPlugin::addWaypointsToListWidget() {
  ui_.listWidget_waypoints->clear();

  for (std::vector<Marble::GeoDataPlacemark *>::iterator it = waypoints.begin(); it != waypoints.end(); it++) {
    ui_.listWidget_waypoints->addItem((*it)->name());
  }
}

void PositionPlugin::editWaypoint() {
  int index = ui_.listWidget_waypoints->currentRow();
  Marble::GeoDataPlacemark *waypoint = waypoints[index];


}

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_position_plugin::PositionPlugin, rqt_gui_cpp::Plugin)
