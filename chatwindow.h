#pragma once
#include <QMainWindow>
#include <QTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QJsonArray>
#include <QFileDialog>
#include <QBuffer>
#include <QImage>
#include <QVector>
#include <QPoint>

//Multimedia
#include <QCamera>
#include <QMediaCaptureSession>
#include <QVideoSink>
#include <QVideoFrame>


class ChatWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit ChatWindow(QWidget *parent = nullptr);
    ~ChatWindow() = default;

private slots:
    void sendCurrentInput();
    void sendImage();       // Existing "send image from file"
    void onApiReply(QNetworkReply *reply);

    void startCamera();
    void stopCamera();
    void onNewVideoFrame(const QVideoFrame &frame);
    void captureAndSend();  // Capture current frame and send to AI

    void solveMazeFromFile(); // Solving mazes (drawn/pictures)
    void explainMazePath();
private:

    void postChat(const QString &userText);
    void postImage(const QString &prompt, const QString &dataUrl);
    void appendToHistory(const QString &speaker, const QString &text);

    // UI
    QTextEdit *history;
    QLineEdit *input;
    QPushButton *sendBtn;
    QPushButton *sendImageBtn;
    QPushButton *guideBtn;
    QVector<QPoint> lastGridPath; // <-- store BFS result here

    // Camera UI
    QLabel *preview;            // live preview
    QPushButton *startCamBtn;
    QPushButton *captureBtn;
    QPushButton *stopCamBtn;


    // Networking / chat
    QNetworkAccessManager *net;
    QJsonArray messages;        // running conversation
    QString apiKey;             // from env var
    QJsonArray conversationHistory; // memory AI chat

    // Multimedia
    QCamera *camera = nullptr;
    QMediaCaptureSession captureSession;
    QVideoSink *videoSink = nullptr;
    QImage lastFrame;           // Latest frame as QImage
};
