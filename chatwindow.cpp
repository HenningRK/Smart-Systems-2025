#include "chatwindow.h"

#include <QVBoxLayout>
#include <QWidget>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileDialog>
#include <QBuffer>
#include <QDateTime>
#include <QDebug>
#include <QPainter>
#include <queue>
#include <algorithm>
#include <cmath>
#include <QRegularExpression>

// Fallback parser: finds all [x,y] pairs in a string, even if JSON is truncated
static QVector<QPointF> parsePathFromLooseText(const QString &s)
{
    QVector<QPointF> pts;
    QRegularExpression re(R"(\[\s*([0-9]*\.?[0-9]+)\s*,\s*([0-9]*\.?[0-9]+)\s*\])");
    auto it = re.globalMatch(s);
    while (it.hasNext()) {
        auto m = it.next();
        double x = m.captured(1).toDouble();
        double y = m.captured(2).toDouble();
        if (x < 0.0) x = 0.0; if (x > 1.0) x = 1.0;
        if (y < 0.0) y = 0.0; if (y > 1.0) y = 1.0;
        pts.append(QPointF(x, y));
    }
    return pts;
}

static QString extractJsonObject(QString s)
{
    s = s.trimmed();

    // Strip ``` fences if present
    if (s.startsWith("```")) {
        int p = s.indexOf('\n');
        int q = s.lastIndexOf("```");
        if (p >= 0 && q > p)
            s = s.mid(p + 1, q - p - 1).trimmed();
    }

    // If it’s wrapped like “JSON: {...}”
    int firstBrace = s.indexOf('{');
    int lastBrace  = s.lastIndexOf('}');
    if (firstBrace >= 0 && lastBrace > firstBrace)
        s = s.mid(firstBrace, lastBrace - firstBrace + 1);

    return s;
}

static QRect findMazeBBox(const QImage &img, int wallLum = 200, int pad = 2)
{
    int minx = img.width(), miny = img.height(), maxx = -1, maxy = -1;
    for (int y = 0; y < img.height(); ++y) {
        const QRgb *row = reinterpret_cast<const QRgb*>(img.constScanLine(y));
        for (int x = 0; x < img.width(); ++x) {
            const QRgb c = row[x];
            int lum = int(0.2126*qRed(c) + 0.7152*qGreen(c) + 0.0722*qBlue(c));
            if (lum < wallLum) { // dark -> part of wall/frame
                if (x < minx) minx = x;
                if (y < miny) miny = y;
                if (x > maxx) maxx = x;
                if (y > maxy) maxy = y;
            }
        }
    }
    if (maxx < 0) return QRect(0,0,img.width(), img.height()); // fallback
    minx = qMax(0, minx - pad);
    miny = qMax(0, miny - pad);
    maxx = qMin(img.width()-1,  maxx + pad);
    maxy = qMin(img.height()-1, maxy + pad);
    return QRect(QPoint(minx,miny), QPoint(maxx,maxy));
}

// treat light pixels as free space
static inline bool isWhite(const QImage &img, int x, int y)
{
    if (x < 0 || y < 0 || x >= img.width() || y >= img.height()) return false;
    const QRgb c = img.pixel(x, y);
    const int lum = int(0.2126*qRed(c) + 0.7152*qGreen(c) + 0.0722*qBlue(c));
    return lum > 230; // white corridor (tweak threshold if needed)
}

// Build a coarse grid by sampling the maze image
static void buildGrid(const QImage &maze, int cellSize,
                      QVector<QVector<bool>> &grid)
{
    const int gw = (maze.width()  + cellSize - 1) / cellSize;
    const int gh = (maze.height() + cellSize - 1) / cellSize;

    grid.resize(gh);
    for (int gy = 0; gy < gh; ++gy) {
        grid[gy].resize(gw);
        for (int gx = 0; gx < gw; ++gx) {
            int x0 = gx * cellSize;
            int y0 = gy * cellSize;
            int x1 = qMin(x0 + cellSize, maze.width());
            int y1 = qMin(y0 + cellSize, maze.height());

            int whiteCount = 0;
            int total = 0;

            // sample every pixel in the cell
            for (int y = y0; y < y1; ++y) {
                for (int x = x0; x < x1; ++x) {
                    ++total;
                    if (isWhite(maze, x, y))
                        ++whiteCount;
        }
    }
            // require MOST of the cell to be white to be considered free
            double ratio = (total > 0) ? (double)whiteCount / (double)total : 0.0;
            grid[gy][gx] = (ratio > 0.7);  // >70% white → free cell
        }
    }
}

// Find two openings on the border of the grid
static bool findOpenings(const QVector<QVector<bool>> &grid,
                         QPoint &start, QPoint &goal)
{
    const int gh = grid.size();
    if (gh == 0) return false;
    const int gw = grid[0].size();

    QVector<QPoint> openings;

    // top and bottom
    for (int x = 0; x < gw; ++x) {
        if (grid[0][x])         openings.append(QPoint(x,0));
        if (grid[gh-1][x])      openings.append(QPoint(x,gh-1));
    }
    // left and right
    for (int y = 0; y < gh; ++y) {
        if (grid[y][0])         openings.append(QPoint(0,y));
        if (grid[y][gw-1])      openings.append(QPoint(gw-1,y));
    }

    if (openings.size() < 2) return false;
    start = openings.front();
    goal  = openings.back();
    return true;
}

// BFS on grid → list of grid cells from start to goal
static QVector<QPoint> bfsPath(const QVector<QVector<bool>> &grid,
                               const QPoint &start, const QPoint &goal)
{
    const int gh = grid.size();
    if (gh == 0) return {};
    const int gw = grid[0].size();

    auto idx = [gw](int x,int y){ return y*gw + x; };
    QVector<int> dist(gw*gh, -1);
    QVector<QPoint> parent(gw*gh, QPoint(-1,-1));

    std::queue<QPoint> q;
    q.push(start);
    dist[idx(start.x(), start.y())] = 0;

    const int dx[4] = {1,-1,0,0};
    const int dy[4] = {0,0,1,-1};

    while (!q.empty()) {
        QPoint u = q.front(); q.pop();
        if (u == goal) break;

        for (int k = 0; k < 4; ++k) {
            int nx = u.x() + dx[k];
            int ny = u.y() + dy[k];
            if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
            if (!grid[ny][nx]) continue;
            int id = idx(nx,ny);
            if (dist[id] != -1) continue;
            dist[id] = dist[idx(u.x(),u.y())] + 1;
            parent[id] = u;
            q.push(QPoint(nx,ny));
        }
    }

    if (dist[idx(goal.x(), goal.y())] == -1) {
        return {}; // no path
    }

    QVector<QPoint> path;
    for (QPoint v = goal; v != QPoint(-1,-1); v = parent[idx(v.x(),v.y())]) {
        path.append(v);
        if (v == start) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

static QString pathToMoves(const QVector<QPoint> &gridPath)
{
    if (gridPath.size() < 2)
        return "{}";

    struct Move { QString dir; int steps; };
    QVector<Move> moves;

    auto dirFromDelta = [](int dx, int dy)->QString {
        if (dx == 1 && dy == 0)  return "E";
        if (dx == -1 && dy == 0) return "W";
        if (dx == 0 && dy == 1)  return "S";
        if (dx == 0 && dy == -1) return "N";
        return "?";
    };

    QPoint prev = gridPath[0];
    QString curDir;
    int curSteps = 0;

    for (int i = 1; i < gridPath.size(); ++i) {
        QPoint p = gridPath[i];
        int dx = p.x() - prev.x();
        int dy = p.y() - prev.y();
        QString d = dirFromDelta(dx, dy);
        if (d == "?") {
            prev = p;
            continue; // skip weird jumps
        }
        if (curSteps == 0) {
            curDir = d;
            curSteps = 1;
        } else if (d == curDir) {
            ++curSteps;
        } else {
            moves.push_back({curDir, curSteps});
            curDir = d;
            curSteps = 1;
        }
        prev = p;
    }
    if (curSteps > 0)
        moves.push_back({curDir, curSteps});

    // Build a simple JSON-like string: [{"dir":"E","steps":5}, ...]
    QStringList parts;
    for (const auto &m : moves) {
        parts << QString("{\"dir\":\"%1\",\"steps\":%2}")
        .arg(m.dir)
            .arg(m.steps);
    }
    return "[" + parts.join(",") + "]";
}

ChatWindow::ChatWindow(QWidget *parent)
    : QMainWindow(parent),
    history(new QTextEdit(this)),
    input(new QLineEdit(this)),
    sendBtn(new QPushButton("Send", this)),
    sendImageBtn (new QPushButton("Send Image", this)),
    preview(new QLabel(this)),
    startCamBtn(new QPushButton("Start Camera", this)),
    captureBtn(new QPushButton("Capture & Send", this)),
    stopCamBtn(new QPushButton("Stop Camera", this)),
    guideBtn(new QPushButton("Explain Path", this)),
    net(new QNetworkAccessManager(this)),
    apiKey(QString::fromUtf8(qgetenv("OPENAI_API_KEY")))
{
    setWindowTitle("AI Chat");
    resize(720, 720);

    // central widget + layout
    auto *central = new QWidget(this);
    auto *layout  = new QVBoxLayout(central);
    setCentralWidget(central);

    history->setReadOnly(true);
    history->setPlaceholderText("Conversation will appear here...");
    input->setPlaceholderText("Type your message and press Enter…");

    // Row with input + buttons
    auto *inputRow = new QHBoxLayout();
    inputRow->addWidget(input, 1);
    inputRow->addWidget(sendBtn);
    inputRow->addWidget(sendImageBtn);
    inputRow->addWidget(guideBtn);

    // Solve Maze Button
    auto *solveBtn = new QPushButton("Solve Maze", this);
    inputRow->addWidget(solveBtn);
    connect(solveBtn, &QPushButton::clicked, this, &ChatWindow::solveMazeFromFile);

    // Camera preview + controls
    preview->setMinimumHeight(240);
    preview->setAlignment(Qt::AlignCenter);
    preview->setStyleSheet("border:1px solid #444; border-radius:8px;");

    auto *camRow = new QHBoxLayout();
    camRow->addWidget(startCamBtn);
    camRow->addWidget(captureBtn);
    camRow->addWidget(stopCamBtn);

    layout->addWidget(history);
    layout->addLayout(inputRow);
    layout->addWidget(preview);
    layout->addLayout(camRow);


    // Chat signals
    connect(sendBtn, &QPushButton::clicked, this, &ChatWindow::sendCurrentInput);
    connect(input, &QLineEdit::returnPressed, this, &ChatWindow::sendCurrentInput);
    connect(sendImageBtn, &QPushButton::clicked, this, &ChatWindow::sendImage);
    connect(guideBtn, &QPushButton::clicked, this, &ChatWindow::explainMazePath);

    // Camera signals
    connect(startCamBtn, &QPushButton::clicked, this, &ChatWindow::startCamera);
    connect(stopCamBtn,  &QPushButton::clicked, this, &ChatWindow::stopCamera);
    connect(captureBtn,  &QPushButton::clicked, this, &ChatWindow::captureAndSend);

    captureBtn->setEnabled(false);
    stopCamBtn->setEnabled(false);

    if (apiKey.isEmpty())
        appendToHistory("System", "OPENAI_API_KEY not set. Set it in your environment for development.");

    conversationHistory = QJsonArray{
        QJsonObject{{"role", "system"},
                {"content", "You are a helpful assistant inside a Qt chat window. "
                                "Remember the conversation history and respond naturally."}}};

    // Try loading memory from disk
    QFile memFile("memory.json");
    if (memFile.open(QIODevice::ReadOnly)) {
        QByteArray data = memFile.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);
        if (doc.isArray()) {
            conversationHistory = doc.array();
            appendToHistory("System", "(Loaded previous memory)");
        }
    }
}

void ChatWindow::appendToHistory(const QString &speaker, const QString &text) {
    history->append(QString("<b>%1:</b> %2").arg(speaker, text.toHtmlEscaped()));
}


/* ======== Camera control ======== */

void ChatWindow::startCamera() {
    if (camera) return;

    camera = new QCamera(this);
    videoSink = new QVideoSink(this);

    captureSession.setCamera(camera);
    captureSession.setVideoSink(videoSink);

    connect(videoSink, &QVideoSink::videoFrameChanged,
            this, &ChatWindow::onNewVideoFrame);

    camera->start();

    appendToHistory("System", "Camera started.");
    captureBtn->setEnabled(true);
    stopCamBtn->setEnabled(true);
    startCamBtn->setEnabled(false);
}

void ChatWindow::stopCamera() {
    if (!camera) return;
    camera->stop();
    camera->deleteLater();
    camera = nullptr;

    if (videoSink) {
        videoSink->deleteLater();
        videoSink = nullptr;
    }
    lastFrame = QImage();

    preview->clear();
    preview->setText("Camera stopped.");
    appendToHistory("System", "Camera stopped.");

    captureBtn->setEnabled(false);
    stopCamBtn->setEnabled(false);
    startCamBtn->setEnabled(true);
}

void ChatWindow::onNewVideoFrame(const QVideoFrame &frame) {
    if (!frame.isValid()) return;
    QVideoFrame f(frame);
    if (!f.map(QVideoFrame::ReadOnly)) return;
    QImage img = f.toImage();   // Qt converts to QImage
    f.unmap();

    if (!img.isNull()) {
        lastFrame = img;
        // show scaled preview
        preview->setPixmap(QPixmap::fromImage(img).scaled(
            preview->size()*devicePixelRatioF(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ChatWindow::captureAndSend() {
    if (lastFrame.isNull()) {
        appendToHistory("System", "No frame available. Is the camera running?");
        return;
    }

    // Resize to keep payload reasonable
    QImage scaled = lastFrame.scaled(1024, 1024, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    QByteArray bytes;
    QBuffer buf(&bytes);
    buf.open(QIODevice::WriteOnly);
    scaled.save(&buf, "JPEG", 85);
    QByteArray b64 = bytes.toBase64();
    QString dataUrl = "data:image/jpeg;base64," + QString::fromLatin1(b64);

    appendToHistory("You", "[captured a photo] Describe this scene.");
    postImage("Describe this scene.", dataUrl);
}

/* ======== Existing chat/text & vision methods ======== */

void ChatWindow::sendCurrentInput() {
    const QString userText = input->text().trimmed();
    if (userText.isEmpty() || apiKey.isEmpty())
        return;

    input->clear();
    postChat(userText);
}

void ChatWindow::postChat(const QString &userText) {
// Update UI
    appendToHistory("You", userText);
    input->setEnabled(false);
    sendBtn->setEnabled(false);

    // Add user turn to in-memory conversation
    conversationHistory.append(QJsonObject{
        {"role", "user"},
        {"content", userText}
    });

    // ---- build request ----
    QNetworkRequest req(QUrl("https://api.openai.com/v1/chat/completions"));
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    req.setRawHeader("Authorization", QString("Bearer %1").arg(apiKey).toUtf8());

    QJsonObject body;
    body["model"] = "gpt-4o-mini";
    body["messages"] = conversationHistory;

    auto *reply = net->post(req, QJsonDocument(body).toJson());

    connect(reply, &QNetworkReply::finished, this, [this, reply]() {

        QByteArray payload = reply->readAll();
        reply->deleteLater();

        QString replyText;

        // ---- Extract assistant text ----
        QJsonDocument doc = QJsonDocument::fromJson(payload);
        if (doc.isObject()) {
            QJsonArray choices = doc.object()["choices"].toArray();
            if (!choices.isEmpty()) {
                replyText = choices[0]
                        .toObject()["message"]
                        .toObject()["content"].toString();
            }
        }

        // Error fallback
        if (replyText.isEmpty()) {
            appendToHistory("Error", "Empty response.");
            input->setEnabled(true);
            sendBtn->setEnabled(true);
            return;
        }

        // Show in chat
        appendToHistory("AI", replyText);

        // Store in memory
        conversationHistory.append(QJsonObject{
            {"role", "assistant"},
            {"content", replyText}
        });

        // ---- Persist memory to disk ----
        QFile memFile("memory.json");
        if (memFile.open(QIODevice::WriteOnly)) {
            memFile.write(QJsonDocument(conversationHistory)
                          .toJson(QJsonDocument::Indented));
        }

        // Safety: keep last 20 messages to avoid token bloat
        if (conversationHistory.size() > 20) {
            conversationHistory.removeFirst();
        }

        // Re-enable UI
        input->setEnabled(true);
        sendBtn->setEnabled(true);
    });

}

void ChatWindow::onApiReply(QNetworkReply *reply) {
    QByteArray payload = reply->readAll();

    if (reply->error() != QNetworkReply::NoError) {
        appendToHistory("Error", reply->errorString());
        reply->deleteLater();
        input->setEnabled(true);
        sendBtn->setEnabled(true);
        return;
    }

    QJsonDocument doc = QJsonDocument::fromJson(payload);
    QString content;
    if (doc.isObject()) {
        auto root = doc.object();
        auto choices = root["choices"].toArray();
        if (!choices.isEmpty()) {
            content = choices[0].toObject()
            ["message"].toObject()
                ["content"].toString();
        }
    }
    if (content.isEmpty()) content = "(empty response)";

    // show + add assistant turn to conversation
    appendToHistory("AI", content);
    messages.append(QJsonObject{{"role","assistant"},{"content",content}});

    reply->deleteLater();
    input->setEnabled(true);
    sendBtn->setEnabled(true);
}

void ChatWindow::sendImage(){
    // Pick a local image
    QString path = QFileDialog::getOpenFileName(
        this,
        "Chose and image",
        QString(),
        "Images (*.png *.jpg *.jpeg *.bmp *.webp)"
        );
    if (path.isEmpty() || apiKey.isEmpty()) return;

    //load + (optionally) downscale to keep payload small
    QImage img(path);
    if (img.isNull()){
        appendToHistory("System", "Could not load image.");
        return;
    }

    // Resize to max 1024px (keeps quality, reduces size)
    QImage scaled = img.scaled(1024, 1024, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    // Encode as JPEG base64 (smaller than PNG for photos)
    QByteArray bytes;
    QBuffer buf(&bytes);
    buf.open(QIODevice::WriteOnly);
    scaled.save(&buf, "JPEG", 85); // 85% quality
    QByteArray b64 = bytes.toBase64();

    // Build data URL
    QString dataUrl = "data:image/jpeg;base64," + QString::fromLatin1(b64);

    //Optional: ask the model what to do
    QString prompt = "Describe this image in detail.";

    // Show in history and send
    appendToHistory("You", QString("[sent an image] %1").arg(prompt));
    postImage(prompt, dataUrl);

}

void ChatWindow::postImage(const QString &prompt, const QString &dataUrl){
    input->setEnabled(false);
    sendBtn->setEnabled(false);
    sendImageBtn->setEnabled(false);

    // Content parts for a single user message: text + image
    QJsonArray content;
    content.append(QJsonObject{{"type", "text"},{"text", prompt}});
    content.append(QJsonObject{
        {"type","image_url"},
        {"image_url", QJsonObject{{"url", dataUrl}}}
    });

    //Keep full conversation constext
    messages.append(QJsonObject{
        {"role","user"},
        {"content", content}
    });

    // POST /v1/chat/completions
    QNetworkRequest req(QUrl("https://api.openai.com/v1/chat/completions"));
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    req.setRawHeader("Authorization", QString("Bearer %1").arg(apiKey).toUtf8());

    QJsonObject body;
    body["model"] = "gpt-4o-mini";     // vision-capable
    body["messages"] = messages;

    QNetworkReply *reply = net->post(req, QJsonDocument(body).toJson());
    connect(reply, &QNetworkReply::finished, this, [this, reply]() { onApiReply(reply); });
}

static QString makeDataUrl(const QImage &img) {
    QByteArray bytes; QBuffer buf(&bytes); buf.open(QIODevice::WriteOnly);
    img.save(&buf, "JPEG", 90);
    return "data:image/jpeg;base64," + QString::fromLatin1(bytes.toBase64());
    // Optionally downscale to keep payload small
    QImage scaled = img.scaled(1024,1024, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}


void ChatWindow::solveMazeFromFile() {
    if (apiKey.isEmpty()) { appendToHistory("System","API key missing."); return; }

    const QString path = QFileDialog::getOpenFileName(this,"Pick maze image",{}, "Images (*.png *.jpg *.jpeg *.bmp *.webp)");
    if (path.isEmpty()) return;

    QImage img(path);
    if (img.isNull()) { appendToHistory("System","Could not load image."); return; }

    // crop to the maze frame
    QRect bbox = findMazeBBox(img);
    QImage mazeOnly = img.copy(bbox);

    // ---- build coarse grid and solve with BFS ----
    QVector<QVector<bool>> grid;
    int cellSize = 3; // 4px per cell; tweak as needed
    buildGrid(mazeOnly, cellSize, grid);

    QPoint start, goal;
    if (!findOpenings(grid, start, goal)) {
        appendToHistory("Error", "Could not find maze entrances.");
        return;
    }

    // New: block all border cells except start & goal
    const int gh = grid.size();
    const int gw = grid[0].size();

    auto same = [](const QPoint &a, const QPoint &b){
        return a.x() == b.x() && a.y() == b.y();
    };

    // top & bottom rows
    for (int x = 0; x < gw; ++x) {
        QPoint top(x, 0);
        QPoint bottom(x, gh - 1);
        if (!same(top, start) && !same(top, goal))       grid[0][x] = false;
        if (!same(bottom, start) && !same(bottom, goal)) grid[gh - 1][x] = false;
    }

    // left & right columns
    for (int y = 0; y < gh; ++y) {
        QPoint left(0, y);
        QPoint right(gw - 1, y);
        if (!same(left, start) && !same(left, goal))     grid[y][0] = false;
        if (!same(right, start) && !same(right, goal))   grid[y][gw - 1] = false;
    }

    QVector<QPoint> gridPath = bfsPath(grid, start, goal);
    if (gridPath.isEmpty()) {
        appendToHistory("Error", "No path found by BFS.");
        return;
    }
    lastGridPath = gridPath;
    QString movesJson = pathToMoves(gridPath);

    // Convert grid cells -> normalized [0,1] coordinates inside mazeOnly
    QVector<QPointF> points;
    points.reserve(gridPath.size());
    const double w = mazeOnly.width() - 1;
    const double h = mazeOnly.height() - 1;
    for (const QPoint &c : gridPath) {
        double cx = (c.x() + 0.5) * cellSize;
        double cy = (c.y() + 0.5) * cellSize;
        // clamp inside image
        cx = qBound(0.0, cx, w - 1.0);
        cy = qBound(0.0, cy, h - 1.0);
        double nx = cx / (w - 1.0);
        double ny = cy / (h - 1.0);
        points.append(QPointF(nx, ny));
    }

    // downscale this cropped maze for the API
    QImage scaled = mazeOnly.scaled(1024, 1024, Qt::KeepAspectRatio, Qt::SmoothTransformation);


    const int W = scaled.width();
    const int H = scaled.height();



    // Build a strict vision message: ask ONLY JSON back
    QJsonArray content;
    content.append(QJsonObject{{"type","text"},{"text",
    QString("You are a maze solver. White is free space. Black lines are walls.\n"
            "Return ONLY JSON (no markdown, no prose), exactly:\n"
            "Format: {\"path\":[[x0,y0],[x1,y1],...]}\n"
            "Coordinates are floats in [0,1] with origin at TOP-LEFT (y increases downward).\n"
            "Each step must move at most 0.02 in x or y (grid-like, no long jumps), "
            "and all points must stay in white corridors (never cross a black wall).\n"
            "Return between 120 and 200 points total."
            "White inside the outer black frame is free space. "
            "Black lines (including the outer rectangle) are walls. "
            "You MUST stay inside the outer rectangle and never cross outside it.\n"
            "Return ONLY JSON: {\"path\":[[x0,y0],[x1,y1],...]} ..."
            )}});
    content.append(QJsonObject{
        {"type","image_url"},
        {"image_url", QJsonObject{{"url", makeDataUrl(scaled)}}}
    });

    // We don’t mix this into the normal chat thread; send a one-off request
    /*QNetworkRequest req(QUrl("https://api.openai.com/v1/chat/completions"));
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    req.setRawHeader("Authorization", QString("Bearer %1").arg(apiKey).toUtf8());

    QJsonObject body;
    body["model"] = "gpt-4o-mini";
    body["messages"] = QJsonArray{
        QJsonObject{{"role","user"},{"content",content}}
    };

    body["temperature"] = 0.0;                         // deterministic
    body["max_tokens"] = 800;                          // enough for ~200 points
    body["response_format"] = QJsonObject{{"type","json_object"}}; // JSON-only if supported

    appendToHistory("You","[maze] Solve this maze and return JSON path.");

    auto *reply = net->post(req, QJsonDocument(body).toJson());
    connect(reply, &QNetworkReply::finished, this, [=]() {
        QByteArray payload = reply->readAll();
        reply->deleteLater();

        QString text;
        QJsonDocument doc = QJsonDocument::fromJson(payload);
        if (doc.isObject()) {
            QJsonArray choices = doc.object().value("choices").toArray();
            if (!choices.isEmpty()) {
                QJsonObject first   = choices.at(0).toObject();
                QJsonObject message = first.value("message").toObject();
                text = message.value("content").toString();
            }
        }

        if (text.isEmpty()) {
            appendToHistory("Error", "Empty response.");
            return;
        }

        2) Try to parse as proper JSON: {"path":[[x,y],...]}
        QString jsonText = extractJsonObject(text);   // if you already added this helper; if not, just use: jsonText = text;
        QJsonParseError perr{};
        QJsonDocument j = QJsonDocument::fromJson(jsonText.toUtf8(), &perr);

        QJsonArray pathArr;

        if (perr.error == QJsonParseError::NoError && j.isObject()) {
            pathArr = j.object().value("path").toArray();
        }

        // 3) If JSON failed OR pathArr empty, fall back to loose parser
        QVector<QPointF> points;

        if (!pathArr.isEmpty()) {
            // convert QJsonArray → QVector<QPointF>
            for (const QJsonValue &v : pathArr) {
                const QJsonArray a = v.toArray();
                if (a.size() != 2) continue;
                double x = a.at(0).toDouble();
                double y = a.at(1).toDouble();
                if (x < 0.0) x = 0.0; if (x > 1.0) x = 1.0;
                if (y < 0.0) y = 0.0; if (y > 1.0) y = 1.0;
                points.append(QPointF(x, y));
            }
        } else {
            appendToHistory("System", "JSON parse failed; using loose coordinate parser.");
            points = parsePathFromLooseText(text);
        }

        if (points.size() < 2) {
            appendToHistory("Error", "No usable path points found.");
            appendToHistory("AI (raw)", text.left(1000));
            return;
        }
        // ---- Add these helpers before drawing ----
        auto clamp01 = [](double v){ return v < 0 ? 0 : (v > 1 ? 1 : v); };

        auto countOutOfRange = [](const QJsonArray& path){
            int bad = 0;
            for (const QJsonValue &v : path) {
                const auto a = v.toArray();
                if (a.size()!=2) { ++bad; continue; }
                double x=a[0].toDouble(), y=a[1].toDouble();
                if (x<0 || x>1 || y<0 || y>1) ++bad;
            }
            return bad;
        };

        // Decide if we need to flip Y (origin mismatch)
        int bad = 0;
        for (const QPointF &p : points) {
            if (p.x() < 0.0 || p.x() > 1.0 || p.y() < 0.0 || p.y() > 1.0) ++bad;
        }
        bool flipY = (bad > points.size() / 10);
*/
        // Draw overlay
        QImage result = img.convertToFormat(QImage::Format_ARGB32);
        QPainter p(&result);
        QPen pen(Qt::red);
        pen.setWidth(qMax(3, result.width()/200));
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(pen);

        auto mapX = [&](double x){ return bbox.x() + qBound(0.0, x, 1.0) * w; };
        auto mapY = [&](double y){ return bbox.y() + qBound(0.0, y, 1.0) * h; };

        QPointF prev;
        bool havePrev = false;
        for (const QPointF &np : points) {
            QPointF cur(mapX(np.x()), mapY(np.y()));
            if (havePrev) p.drawLine(prev, cur);
            prev = cur;
            havePrev = true;
        }
        p.end();

        // Save & preview
        QString savePath = QDir::temp().filePath("maze_solved.png");
        result.save(savePath, "PNG");
        appendToHistory("System", "Maze solved locally (BFS). Saved to: " + savePath);
        preview->setPixmap(QPixmap::fromImage(result).scaled(
            preview->size()*devicePixelRatioF(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    //});
}
void ChatWindow::explainMazePath()
{
    if (lastGridPath.isEmpty()) {
        appendToHistory("System",
                        "No maze path available yet. "
                        "Press 'Solve Maze' first.");
        return;
    }

    QString movesJson = pathToMoves(lastGridPath);

    QString prompt =
        "You are a navigation assistant for a small robot car in a maze.\n"
        "The maze has already been solved by a BFS algorithm on a grid.\n"
        "The path is given as a sequence of moves of the form "
        "{\"dir\":\"E\",\"steps\":5} where dir is one of N,E,S,W and "
        "steps is the number of grid cells.\n"
        "Starting from the entrance and following the moves in order, "
        "give clear step-by-step instructions using ONLY these commands:\n"
        "- FORWARD <cells>\n"
        "- TURN LEFT\n"
        "- TURN RIGHT\n"
        "Be concise and numbered (Step 1, Step 2, ...).\n"
        "Here is the path:\n" + movesJson;

    // Send this as a normal user message to the chat
    postChat(prompt);
}


