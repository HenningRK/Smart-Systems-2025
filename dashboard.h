#pragma once
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>

namespace Ui {
class Dashboard;
}

class Dashboard : public QMainWindow {
    Q_OBJECT

public:
    explicit Dashboard(QWidget *parent = nullptr);
    ~Dashboard();

private:
    void readSensorData();

private:
    Ui::Dashboard *ui;
    QSerialPort *serial;
};
