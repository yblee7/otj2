#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QCloseEvent>
#include <QStandardPaths>
#include <QStringListModel>
#include <QItemSelectionModel>

#include <string>

#include "image_viewer.h"
#include "homography.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QDir last_directory;
    QStringListModel *model;
    QItemSelectionModel *selection_model;

    QStringListModel *model_corners_original;
    QItemSelectionModel *selection_model_corners_original;

    QStringListModel *model_corners_rectified;
    QItemSelectionModel *selection_model_corners_rectified;

    Homography *homography;

    QStringList filenames;
    QFileInfoList fullpaths;
    QString current_filepath;

    Eigen::Matrix3d H;

    void updateListView(QDir dir);
    void openFirstImageFileFromDirectory(QDir dir);

    void updateLog(const QString& log);
    void updateLog(const std::string& log);
    void updateLog(const char* log);
    void updateLog(std::stringstream &log);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

public slots:
    void logMessageReceived(std::stringstream &msg);

private slots:    
    void keyPressReceived(QKeyEvent *ev);
    void printMessageOnStatusBar(QString *msg);

    void pointsUpdatedReceived();

    void listViewFilesCurrentChanged(const QModelIndex &current, const QModelIndex &previous);

    void on_btnDirectory_clicked();
    void on_btnRectify_clicked();
    void on_btnSaveAs_clicked();
    void on_actionOpen_triggered();
    void on_actionAbout_Program_triggered();

    void on_radioButtonMagnifierLocationTopLeft_clicked();
    void on_radioButtonMagnifierLocationCursor_clicked();
    void on_radioButtonMagnifierLocationAdaptive_clicked();

signals:
    void keyPressed(QKeyEvent *ev);
};
#endif // MAINWINDOW_H
