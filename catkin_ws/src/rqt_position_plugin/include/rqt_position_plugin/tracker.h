/*
 * tracker.h
 * Copyright (C) 2015 Robert Hamilton <rghamilton3@gmail.com>
 *
 * Distributed under terms of the BSD license.
 */

#ifndef TRACKER_H
#define TRACKER_H

// QT Includes
#include <QtGui/QApplication>
#include <QtCore/QTimer>

// Marble Includes
#include <marble/MarbleWidget.h>
#include <marble/GeoDataCoordinates.h>

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>

using namespace Marble;

namespace rqt_position_plugin {

class Tracker : public QObject
{
    
    Q_OBJECT

public:
    Tracker(const sensor_msgs::NavSatFixConstPtr& gps_ptr);
 
signals:
    void coordinatesChanged(GeoDataCoordinates coord);
 
public slots:
    void startWork();
    void finishWork();

private slots:
    void iterate();
 
private:
    sensor_msgs::NavSatFixConstPtr* gps_ptr;
};

}

#endif /* !TRACKER_H */
