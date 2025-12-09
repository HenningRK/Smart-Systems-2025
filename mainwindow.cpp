#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSslSocket>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkProxyFactory>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , dashboard(nullptr)
    , networkManager(new QNetworkAccessManager(this))
{
    ui->setupUi(this);
    setWindowTitle("SmartSystems25");
}

MainWindow::~MainWindow() {
    delete ui;
    delete dashboard;
}

void MainWindow::on_button1_clicked() {

    if (!chatWindow) chatWindow = new ChatWindow(this);  // create once
    chatWindow->show();
    chatWindow->raise();
    chatWindow->activateWindow();

}

void MainWindow::onApiReply(QNetworkReply *reply) {
    if (reply->error() != QNetworkReply::NoError) {
        qDebug() << "Error:" << reply->errorString();
        reply->deleteLater();
        return;
    }

    QByteArray responseData = reply->readAll();
    QJsonDocument jsonDoc = QJsonDocument::fromJson(responseData);

    qDebug() << "API Response:" << jsonDoc;

    // Example: show response in QLabel
    if (jsonDoc.isObject()) {
        QJsonObject root = jsonDoc.object();
        if (root.contains("choices")) {
            QJsonArray choices = root["choices"].toArray();
            if (!choices.isEmpty()) {
                QString content = choices[0].toObject()["message"].toObject()["content"].toString();
                ui->label->setText(content);
            }
        }
    }

    reply->deleteLater();
}


void MainWindow::on_button2_clicked() {
    if (!dashboard) {
        dashboard = new Dashboard(this);  // create once
    }
    dashboard->show();  // open the dashboard window
    dashboard->raise();
    dashboard->activateWindow();
}
