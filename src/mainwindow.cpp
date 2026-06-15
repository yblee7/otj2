#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <fstream>

#include <QDateTime>
#include <QImageReader>
#include <QListWidget>
#include <QThread>
#include <QTimer>
#include <QSharedMemory>
#include <QSignalBlocker>

#include <sstream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->viewOriginal->setShowMagnifier(true);
    ui->viewOriginal->setDetectCorners(true);

    ui->viewRectified->setShowMagnifier(true);

    ui->radioButtonMagnifierLocationTopLeft->setChecked(true);

    connect(ui->viewOriginal, SIGNAL(infoMessageToMainWindow(QString *)), this, SLOT(printMessageOnStatusBar(QString *)));
    connect(ui->viewOriginal, SIGNAL(infoCornersUpdated()), this, SLOT(pointsUpdatedReceived()));

    connect(ui->viewRectified, SIGNAL(infoMessageToMainWindow(QString *)), this, SLOT(printMessageOnStatusBar(QString *)));
    connect(ui->viewRectified, SIGNAL(infoCornersUpdated()), this, SLOT(pointsUpdatedReceived()));

    connect(ui->viewOriginal, SIGNAL(keyPressed(QKeyEvent *)), this, SLOT(keyPressReceived(QKeyEvent *)));
    connect(ui->viewRectified, SIGNAL(keyPressed(QKeyEvent *)), this, SLOT(keyPressReceived(QKeyEvent *)));

    QCoreApplication::setOrganizationName("CLE");
    QCoreApplication::setApplicationName("OjtHomography");

    ui->lineEditDirectory->setReadOnly(true);
    ui->listViewFiles->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->listViewCornersOriginal->setEditTriggers(QAbstractItemView::NoEditTriggers);

    model = new QStringListModel(this);
    selection_model = nullptr;

    model_corners_original = new QStringListModel(this);
    selection_model_corners_original = nullptr;

    model_corners_rectified = new QStringListModel(this);
    selection_model_corners_rectified = nullptr;

    QSettings settings;
    last_directory = settings.value("last_directory").toString();

    if (!last_directory.isEmpty())
    {
        updateListView(last_directory);
        openFirstImageFileFromDirectory(last_directory);
    }

    homography = new Homography();
    connect(homography, SIGNAL(updateLogToMainWindow(std::stringstream &)), this, SLOT(logMessageReceived(std::stringstream &)));

    statusBar()->setStyleSheet("font: 11pt 'Lucida Console';");
}

MainWindow::~MainWindow()
{
    if (!this->isVisible())
        QApplication::quit();

    delete ui;
}

void MainWindow::printMessageOnStatusBar(QString *msg)
{
    statusBar()->showMessage(*msg);
}

void MainWindow::pointsUpdatedReceived()
{
    QObject *obj = sender();
    if (obj == nullptr)
        return;

    QList<QPointF> points;
    if(obj->objectName() == "viewOriginal")
    {
        points = ui->viewOriginal->getCorners();
    }
    else if(obj->objectName() == "viewRectified")
    {
        points = ui->viewRectified->getCorners();

        if (has_homography)
        {
            const QList<QPointF> original_points = transformPoints(points, H.inverse());
            if (original_points.size() == points.size())
            {
                const QSignalBlocker blocker(ui->viewOriginal);
                ui->viewOriginal->setCorners(original_points);
                updateCornerListView("viewOriginal", original_points);
            }
        }
    }
    else
    {
        return;
    }

    updateCornerListView(obj->objectName(), points);
}

QList<QPointF> MainWindow::transformPoints(const QList<QPointF> &points, const Eigen::Matrix3d &transform) const
{
    QList<QPointF> transformed_points;
    transformed_points.reserve(points.size());

    for (const QPointF &p : points)
    {
        Eigen::Vector3d q = transform * Eigen::Vector3d(p.x(), p.y(), 1.0);
        if (std::abs(q.z()) <= std::numeric_limits<double>::epsilon())
            continue;

        q /= q.z();
        transformed_points.push_back(QPointF(q.x(), q.y()));
    }

    return transformed_points;
}

void MainWindow::updateCornerListView(const QString &viewer_name, const QList<QPointF> &points)
{
    QStringList points_string_list;
    for (const auto &p : points)
    {
        QString point_string = QString("(%1, %2)").arg(p.x()).arg(p.y());
        points_string_list.push_back(point_string);
    }

    if(viewer_name == "viewOriginal")
    {
        model_corners_original->setStringList(points_string_list);
        ui->listViewCornersOriginal->setModel(model_corners_original);
    }
    else if(viewer_name == "viewRectified")
    {
        model_corners_rectified->setStringList(points_string_list);
        ui->listViewCornersRectified->setModel(model_corners_rectified);
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
    case Qt::Key_Escape:
        QMessageBox message_box;
        message_box.setWindowTitle(tr("OJT Image Viewer"));
        message_box.setText(tr("Exit program?"));
        message_box.setIcon(QMessageBox::Question);
        message_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        message_box.setDefaultButton(QMessageBox::No);
        message_box.setBaseSize(QSize(300, 150));
#ifdef __APPLE__
        QList<QAbstractButton *> bList = message_box.buttons();
        for (const auto &p : bList)
            p->setFocusPolicy(Qt::StrongFocus);
#endif
        int ret = message_box.exec();
        if (ret == QMessageBox::Yes)
        {
            QCoreApplication::quit();
        }
        break;
    }
}

void MainWindow::on_actionAbout_Program_triggered()
{
    QMessageBox::information(0,
                             tr("OJT Image Viewer"),
                             tr("OJT Image Viewer") +
                                 "\n" +
                                 tr("Version ") +
                                 QString::number(VERSION_MAJOR) + "." +
                                 QString::number(VERSION_MINOR) + "." +
                                 QString::number(VERSION_PATCH) + "." +
                                 QString::number(VERSION_BUILD_NUMBER) +
                                 "\n" +
                                 tr("@ 2022 CLE Inc.") +
                                 "\n" +
                                 tr("All rights reserved.") +
                                 "\n\n" +
                                 tr("Developers \n"
                                    "    Jin Han Lee (jhlee@cle.vision) \n"
                                    "    Durkhyun Cho (dcho@cle.vision) \n"
                                    "    Jamie Bak (jamiebak@cle.vision) \n"
                                    "    Kyoungmun Chang (kchang@cle.vision) \n"
                                    "    Sehoon Kang (shkang@cle.vision) \n"
                                    "    Jun Woo Choi (jwchoi@cle.vision) \n"
                                    "    Hyunjin Oh (jinoh@cle.vision) \n\n") +
                                 tr("Please contact CLE Inc. for more information"),
                             QMessageBox::Ok,
                             QMessageBox::Ok);
}

void MainWindow::on_actionOpen_triggered()
{
    this->on_btnDirectory_clicked();
}

void MainWindow::keyPressReceived(QKeyEvent *event)
{
    keyPressEvent(event);
}

void MainWindow::listViewFilesCurrentChanged(const QModelIndex &current, const QModelIndex &previous)
{
    if (fullpaths.size() < 1)
        return;

    QString file_path = fullpaths.at(current.row()).filePath();
    QImage image(file_path);

    if (!image.isNull())
    {
        current_filepath = file_path;
        has_homography = false;
        ui->viewOriginal->setImage(image);
        ui->tabWidget->setCurrentIndex(0);
    }
    else
    {
        qDebug() << "Image loading failed from " << fullpaths.first().filePath();
        qDebug() << QImageReader::supportedImageFormats();
    }
}

void MainWindow::updateListView(QDir dir)
{
    QDir directory(dir);

    filenames = directory.entryList(QStringList() << "*.jpg"
                                                  << "*.jpeg"
                                                  << "*.JPG"
                                                  << "*.JPEG"
                                                  << "*.png"
                                                  << "*.PNG"
                                                  << "*.bmp"
                                                  << "*.BMP",
                                    QDir::Files);

    model->setStringList(filenames);
    ui->listViewFiles->setModel(model);

    if (selection_model != nullptr)
    {
        disconnect(selection_model, &QItemSelectionModel::currentChanged, this, &MainWindow::listViewFilesCurrentChanged);
        selection_model = nullptr;
    }

    selection_model = ui->listViewFiles->selectionModel();
    connect(selection_model, &QItemSelectionModel::currentChanged, this, &MainWindow::listViewFilesCurrentChanged);
}

void MainWindow::openFirstImageFileFromDirectory(QDir dir)
{
    ui->viewOriginal->clearImage();
    ui->listViewFiles->setFocus();

    QDir directory(dir);
    fullpaths = directory.entryInfoList(QStringList() << "*.jpg"
                                                      << "*.jpeg"
                                                      << "*.JPG"
                                                      << "*.JPEG"
                                                      << "*.png"
                                                      << "*.PNG"
                                                      << "*.bmp"
                                                      << "*.BMP",
                                        QDir::AllEntries);

    if (fullpaths.size() < 1)
        return;

    QImage image(fullpaths.first().filePath());

    if (!image.isNull())
    {
        has_homography = false;
        ui->viewOriginal->setImage(image);
        ui->lineEditDirectory->setText(dir.path());
        selection_model->select(model->index(0, 0), QItemSelectionModel::Select);
        selection_model->setCurrentIndex(model->index(0, 0), QItemSelectionModel::Current);
    }
    else
    {
        qDebug() << "Image loading failed from " << fullpaths.first().filePath();
        qDebug() << QImageReader::supportedImageFormats();
    }
}

void MainWindow::on_btnDirectory_clicked()
{
    QDir directory = QFileDialog::getExistingDirectory(this, "Select a directory to open", last_directory.path());

    if (directory.isEmpty())
        return;

    QSettings settings;
    settings.setValue("last_directory", directory.path());
    last_directory = directory;

    updateListView(directory);
    openFirstImageFileFromDirectory(directory);

    ui->lineEditDirectory->setText(QDir::toNativeSeparators(directory.path()));
}

void MainWindow::on_btnSaveAs_clicked()
{
    if(ui->viewRectified->getImage().isNull())
    {
        int ret = QMessageBox::warning(this, tr("OJT Homography"),
                                       tr("No rectified image exists\n"
                                          "Aborting ..."),
                                       QMessageBox::Ok,
                                       QMessageBox::Ok);
        
        return;
    }

    QFileInfo fi(current_filepath);
    QString extension = "." + fi.suffix();
    QString new_filepath = current_filepath;
    new_filepath = new_filepath.replace(extension, "_rectified" + extension);
    new_filepath = QFileDialog::getSaveFileName(this, "Save As", new_filepath, "Images (*.png *.bmp *.jpg)");
    ui->viewRectified->getImage().save(new_filepath);
}

void MainWindow::on_radioButtonMagnifierLocationTopLeft_clicked()
{
    ui->viewOriginal->setMagnifierLocation(MagnifierLocation::TopLeft);
    ui->viewRectified->setMagnifierLocation(MagnifierLocation::TopLeft);
}

void MainWindow::on_radioButtonMagnifierLocationCursor_clicked()
{
    ui->viewOriginal->setMagnifierLocation(MagnifierLocation::Cursor);
    ui->viewRectified->setMagnifierLocation(MagnifierLocation::Cursor);
}

void MainWindow::on_radioButtonMagnifierLocationAdaptive_clicked()
{
    ui->viewOriginal->setMagnifierLocation(MagnifierLocation::Adaptive);
    ui->viewRectified->setMagnifierLocation(MagnifierLocation::Adaptive);
}

void MainWindow::on_btnRectify_clicked()
{
    QList<QPointF> points = ui->viewOriginal->getCorners();

    if (points.size() < 4)
        return;

    std::vector<Eigen::Vector2d> source_points = {Eigen::Vector2d(points[0].x(), points[0].y()),
                                                  Eigen::Vector2d(points[1].x(), points[1].y()),
                                                  Eigen::Vector2d(points[2].x(), points[2].y()),
                                                  Eigen::Vector2d(points[3].x(), points[3].y())};

    QImage image = ui->viewOriginal->getImage();

    homography->setImage(image);

    Eigen::Vector2d center((image.width() - 1) * 0.5, (image.height() - 1) * 0.5);
    Eigen::Vector3d euler_angles;
    double aspect_ratio = Homography::computeRealAspectRatio(center, source_points, euler_angles);

    std::stringstream msg;
    msg << "aspect_ratio: " << aspect_ratio;
    updateLog(msg);

    Eigen::Vector2d dp1 = Eigen::Vector2d(image.width() * 0.1, image.width() * 0.1);
    Eigen::Vector2d dp2 = Eigen::Vector2d(image.width() * 0.9, image.width() * 0.1);
    double new_width = (dp2 - dp1).norm();
    double new_height = new_width / aspect_ratio;
    Eigen::Vector2d dp3 = Eigen::Vector2d(image.width() * 0.9, image.width() * 0.1 + new_height);
    Eigen::Vector2d dp4 = Eigen::Vector2d(image.width() * 0.1, image.width() * 0.1 + new_height);
    std::vector<Eigen::Vector2d> destination_points = {dp1, dp2, dp3, dp4};

    H = homography->compute(source_points, destination_points);
    has_homography = true;

    const Eigen::IOFormat matrix_format(10, 0, " ", "\n", "", "", "", "");
    std::stringstream homography_msg;
    homography_msg << "homography matrix:\n" << H.format(matrix_format);
    updateLog(homography_msg);

    QImage image_transformed = homography->getTransformedImage();
    ui->viewRectified->setImage(image_transformed);

    QList<QPointF> destination_points_qt = {QPointF(destination_points[0](0), destination_points[0](1)),
                                            QPointF(destination_points[1](0), destination_points[1](1)),
                                            QPointF(destination_points[2](0), destination_points[2](1)),
                                            QPointF(destination_points[3](0), destination_points[3](1))};
    
    ui->viewRectified->setCorners(destination_points_qt);

    ui->tabWidget->setCurrentIndex(1);
}

void MainWindow::logMessageReceived(std::stringstream &msg)
{
    updateLog(msg);
    msg.str(""); // clear message for next time use
}

void MainWindow::updateLog(const QString &log)
{
    ui->plainTextEdit->moveCursor(QTextCursor::End);
    ui->plainTextEdit->insertPlainText(log + "\n");
    ui->plainTextEdit->moveCursor(QTextCursor::End);
    std::cerr << log.toStdString() << std::endl;
}

void MainWindow::updateLog(const std::string &log)
{
    updateLog(QString::fromStdString(log));
}

void MainWindow::updateLog(const char *log)
{
    updateLog(QString::fromUtf8(log));
}

void MainWindow::updateLog(std::stringstream &log)
{
    updateLog(log.str());
    log.str("");
}
