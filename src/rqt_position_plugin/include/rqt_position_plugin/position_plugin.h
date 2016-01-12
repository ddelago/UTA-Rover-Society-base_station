/*
 * position_plugin.h
 * Copyright (C) 2015 Robert Hamilton rghamilton3@gmail.com
 *
 * Distributed under terms of the BSD license.
 */

#ifndef POSITION_PLUGIN_H
#define POSITION_PLUGIN_H

// ROS Plugin Includes
#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>

// Message Includes
#include <sensor_msgs/NavSatFix.h>

// Qt Includes
#include <QtCore/QObject>

// Marble Includes
#include <marble/MapThemeManager.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoPainter.h>

// Own Includes
#include <ui_position_plugin.h>
#include <vector>

namespace rqt_position_plugin
{

class PositionPlugin : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:

    // ROS Required
    PositionPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    // ROS Topic Subscriber Callback
    void GPSCallback(const sensor_msgs::NavSatFixConstPtr &gps_ptr);

		void drawLines();
		void plotWaypoint();
		void waypointsToList();

		virtual void paintEvent(QPaintEvent *e);

    // Comment in to signal that the plugin has a way to configure it
    //bool hasConfiguration() const;
    //void triggerConfiguration();

signals:
    void coordinatesChanged(Marble::GeoDataCoordinates coord);

public slots:
    void setTrackerCoordinates(const Marble::GeoDataCoordinates &coord);

private:

private Q_SLOTS:

    void addWaypoint();
    void changeGPSTopic(const QString &topic_name);
    void changeMarbleTheme(int idx);
    void findNavSatFixTopics();

private:

    Ui_Form ui_;

    QWidget* widget_;
    
    std::vector<Marble::GeoDataPlacemark*> waypoints;

    Marble::GeoDataPlacemark* m_tracker_placemark;
    Marble::GeoDataDocument* m_document;
		Marble::GeoPainter* m_painter;
    Marble::MapThemeManager m_map_theme_manager;

    ros::Subscriber current_pos_subscriber;
};

} // namespace

#endif /* !POSITION_PLUGIN_H */
