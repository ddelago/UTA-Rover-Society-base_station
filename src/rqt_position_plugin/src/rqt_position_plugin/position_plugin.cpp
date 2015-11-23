/*
 * position_plugin.cpp
 *
 * Copyright (C) 2015 Robert Hamilton rghamilton3@gmail.com
 *
 * Distributed under terms of the BSD license.
 * 
 * @TODO 
 * Add comments
 * Allow entry of GPS coordinates
 * Connect entered coordinates and GPS Position with a line
 * Display list of entered coordinates
 * Editing of entered coordinates
 */

// QT Includes
#include <QApplication>
#include <QThread>
#include <QTimer>
#include <QHash>
#include <qmath.h>
#include <QDebug>
#include <QVBoxLayout>
#include <QLabel>
#include <QStandardItemModel>
#include <QModelIndex>

// Marble Includes
#include <marble/MarbleWidget.h>
#include <marble/MarbleGlobal.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/MarbleModel.h>
#include <marble/MapThemeManager.h>

// Own Includes
#include "rqt_position_plugin/position_plugin.h"

// ROS Plugin Includes
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

namespace rqt_position_plugin
{

PositionPlugin::PositionPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    // Constructor
    // name QObjects
    setObjectName("PositionPlugin");
}

void PositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // Command Line Args
    QStringList argv = context.argv();

    // create QWidget
    widget_ = new QWidget();

    // add widget to UI
    ui_.setupUi( widget_ );

    // Set up MarbleWidget
    ui_.MarbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    ui_.MarbleWidget->setProjection( Marble::Mercator );

    // Center map to given position
    Marble::GeoDataCoordinates hanksville(-110.7131, 38.3614, 0.0, Marble::GeoDataCoordinates::Degree);
    ui_.MarbleWidget->centerOn(hanksville);
    ui_.MarbleWidget->setDistance(0.5);

    context.addWidget(widget_);
    ui_.comboBox_theme->setModel( m_map_theme_manager.mapThemeModel() );

    // Create tracker placemark
    m_tracker_placemark = new Marble::GeoDataPlacemark("Rover");
    m_document = new Marble::GeoDataDocument;
    m_document->append(m_tracker_placemark);
    ui_.MarbleWidget->model()->treeModel()->addDocument(m_document);


    // Update the position label when the mouse moves
    connect( ui_.MarbleWidget, SIGNAL( mouseMoveGeoPosition( QString ) ),
             ui_.label_position, SLOT( setText( QString ) ) );

    // Update GPS Topic on user selection
    connect( ui_.comboBox_gps, SIGNAL( activated( const QString & ) ),
             this, SLOT( changeGPSTopic( const QString & ) ) );

    // Refresh GPS Topics on user push
    connect( ui_.pushButton_refresh, SIGNAL( clicked() ),
             this, SLOT( findNavSatFixTopics() ) );

    // Update map theme on user selection
    connect( ui_.comboBox_theme, SIGNAL( currentIndexChanged( int ) ),
             this, SLOT( changeMarbleTheme( int ) ) );

    connect(this, SIGNAL( coordinatesChanged( Marble::GeoDataCoordinates ) ),
            this, SLOT( setTrackerCoordinates( Marble::GeoDataCoordinates ) ),
            Qt::BlockingQueuedConnection);

    setTrackerCoordinates(hanksville);
}

void PositionPlugin::GPSCallback(const sensor_msgs::NavSatFixConstPtr& gps_ptr)
{
    /* 
     * Trying to directly set the placemark coordinates or calling the 
     * setTrackerCoordinates method results in a seg fault therefore
     * a QT S/S connection is REQUIRED
     *
     * param gps_ptr
     * a ROS NavSatFix msg containing GPS coordinates
     *
     * return nothing
     */
    Marble::GeoDataCoordinates coord(Marble::GeoDataCoordinates( gps_ptr->longitude, gps_ptr->latitude, gps_ptr->altitude, Marble::GeoDataCoordinates::Degree));

    emit coordinatesChanged(coord);
}

void PositionPlugin::setTrackerCoordinates(const Marble::GeoDataCoordinates& coord)
{
    m_tracker_placemark->setCoordinate(coord);
    ui_.MarbleWidget->model()->treeModel()->updateFeature(m_tracker_placemark);
}

void PositionPlugin::findNavSatFixTopics()
{
    using namespace ros::master;

    std::vector<TopicInfo> topic_infos;
    getTopics(topic_infos);

    // Update combobox with available GPS topics
    ui_.comboBox_gps->clear();
    for(std::vector<TopicInfo>::iterator it=topic_infos.begin(); it!=topic_infos.end(); it++)
    {
        TopicInfo topic = (TopicInfo)(*it);
        if(topic.datatype.compare("sensor_msgs/NavSatFix")==0)
        {
            QString lineEdit_string(topic.name.c_str());
            ui_.comboBox_gps->addItem(lineEdit_string);
        }
    }
}

void PositionPlugin::changeMarbleTheme(int idx )
{
    /*
     * This method changes the current map
     * theme to the user selected theme
     */
    QStandardItemModel* model = m_map_theme_manager.mapThemeModel();
    QModelIndex index = model->index( idx , 0 );
    QString theme = model->data( index , Qt::UserRole+1  ).toString();

    ui_.MarbleWidget->setMapThemeId( theme );
}

void PositionPlugin::changeGPSTopic(const QString &topic_name)
{
    current_pos_subscriber.shutdown();
    current_pos_subscriber = getNodeHandle().subscribe< sensor_msgs::NavSatFix >(
                    topic_name.toStdString().c_str() , 1 , &PositionPlugin::GPSCallback, this );

    int idx = ui_.comboBox_gps->findText( topic_name );
    if ( idx != -1 )
        ui_.comboBox_gps->setCurrentIndex( idx );
}

void PositionPlugin::shutdownPlugin()
{
    // Unregister from publishers
    current_pos_subscriber.shutdown();
}

void PositionPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    QString topic(current_pos_subscriber.getTopic().c_str());
    instance_settings.setValue("position_plugin_gps_topic", topic);

    instance_settings.setValue("position_plugin_zoom", ui_.MarbleWidget->distance());
    instance_settings.setValue("position_plugin_theme", ui_.comboBox_theme->currentIndex());
}

void PositionPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    const QString topic = instance_settings.value("position_plugin_gps_topic").toString();
    changeGPSTopic(topic);

    ui_.MarbleWidget->setDistance( instance_settings.value("position_plugin_zoom" , 0.05 ).toReal() );
    ui_.comboBox_theme->setCurrentIndex( instance_settings.value("position_plugin_theme" , 0 ).toInt() );
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_EXPORT_CLASS(rqt_position_plugin::PositionPlugin , rqt_gui_cpp::Plugin )
