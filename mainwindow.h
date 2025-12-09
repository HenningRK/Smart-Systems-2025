#pragma once
#include "chatwindow.h"
#include "dashboard.h"   // include the new widget
#include <QMainWindow>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_button1_clicked();
    void onApiReply(QNetworkReply *reply);
    void on_button2_clicked();

private:
    Ui::MainWindow *ui;
    Dashboard *dashboard;   // pointer to the dashboard

    QNetworkAccessManager *networkManager;
    ChatWindow *chatWindow = nullptr;
};



