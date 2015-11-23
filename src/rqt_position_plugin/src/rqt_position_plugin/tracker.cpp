/*
 * tracker.cpp
 * Copyright (C) 2015 Robert Hamilton <rghamilton3@gmail.com>
 *
 * Distributed under terms of the BSD license.
 */

// QT Includes
#include <QtGui/QApplication>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QHash>
#include <QtCore/qmath.h>
#include <QtCore/QDebug>
#include <QtGui/QVBoxLayout>
 
// Marble Includes
#include <marble/MarbleWidget.h>
#include <marble/MarbleGlobal.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/MarbleModel.h>

// Own Includes
#include "rqt_position_plugin/tracker.h"

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>

using namespace Marble;

namespace rqt_position_plugin {

Tracker::Tracker(const sensor_msgs::NavSatFixConstPtr& gps_ptr) :
    QObject(),
    gps_ptr(gps_ptr)
{}
 
void Tracker::startWork()
{
    //m_timer->setInterval(0);
    //connect(m_timer, SIGNAL(timeout()), this, SLOT(iterate()));
    //m_timer->start();
}
 
void Tracker::iterate();
{
    qreal lon = gps_ptr->longitude; 
    qreal lat = gps_ptr->latitude;
 
    GeoDataCoordinates coord(lon, lat, 0.0, GeoDataCoordinates::Degree);
    emit coordinatesChanged(coord);
}
 
void Tracker::finishWork()
{
    m_timer->stop();
}

} // namespace



