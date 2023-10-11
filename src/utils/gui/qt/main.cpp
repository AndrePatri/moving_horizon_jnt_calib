// Copyright (C) 2023  Andrea Patrizi (AndrePatri, andreapatrizi1b6e6@gmail.com)
// 
// This file is part of moving_horizon_jnt_calib and distributed under the General Public License version 2 license.
// 
// moving_horizon_jnt_calib is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
// 
// moving_horizon_jnt_calib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with moving_horizon_jnt_calib.  If not, see <http://www.gnu.org/licenses/>.
// 
#include "cmake_config.h"

#include <QApplication>
#include <QMainWindow>
#include <QFile>
#include <QStyleFactory>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <iostream>

#include "ui_mhe_gui.h"

#include <ros/ros.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>

#include <moving_horizon_jnt_calib/SetCalibParams.h>
#include <moving_horizon_jnt_calib/JntCalibStatus.h>

int main(int argc, char *argv[])
{
    std::cout << "Loading UI file @" << UI_FULL_PATH << std::endl;

    QApplication app(argc, argv);

    // Set the application style
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    // Load the main window UI file
    QMainWindow mainWindow;
    Ui::MainWindow ui;
    QFile uiFile(UI_FULL_PATH);
    if (uiFile.open(QFile::ReadOnly)) {
        ui.setupUi(&mainWindow);
        uiFile.close();
    } else {
        QMessageBox::critical(&mainWindow, QObject::tr("Error"), QObject::tr("Failed to open UI file."));
        return -1;
    }

    // Set the main window properties
    mainWindow.setWindowTitle(QObject::tr("Rotor Dynamics MHE GUI"));
    mainWindow.resize(QDesktopWidget().availableGeometry(&mainWindow).size() * 0.7);
    mainWindow.move(QGuiApplication::screens().at(0)->geometry().center() - mainWindow.frameGeometry().center());

    // Show the main window
    mainWindow.show();

    return app.exec();
}
