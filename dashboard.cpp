#include "dashboard.h"
#include "ui_dashboard.h"
#include <QtDebug>

Dashboard::Dashboard(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Dashboard)
    , serial(new QSerialPort(this))
{
    ui->setupUi(this);
    setWindowTitle("Sensor Dashboard");

    // Configure serial port
    serial->setPortName("COM3");   // <-- change this to your serial port
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadOnly)) {
        qDebug() << "Serial port opened.";
        connect(serial, &QSerialPort::readyRead, this, &Dashboard::readSensorData);
    } else {
        qDebug() << "Failed to open serial port:" << serial->errorString();
    }
}


Dashboard::~Dashboard() {
    delete ui;
}

void Dashboard::readSensorData() {
    QByteArray data = serial->readAll();
    QString text = QString::fromUtf8(data).trimmed();

    // Example: Arduino sends "123" (distance in cm)
    bool ok;
    int distance = text.toInt(&ok);
    if (ok) {
        qDebug() << "Distance:" << distance << "cm";
        ui->labelSensor1->setText(QString("Distance: %1 cm").arg(distance));
        // assuming you added a QLabel called "label" in dashboard.ui
    }
}
