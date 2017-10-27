#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPixmap>
#include <QTimer>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QGraphicsView>

#include "my_tcpsocket.h"
#include "laser_machine.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>

using namespace cv;

class QTimer;
class QTime;
class QFile;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget * parent = 0);
    ~MainWindow();

public slots:
    void talking(const QString &);
    void talking2(const QString &);
    void tr_Take_Contour(vector<vector<cv::Point> >, double, double);

signals:
    void Quit();
    void NextFrame();
    void SetScale(int);
    void SetThVal(int);
    void SetCoeff(int = -1, int = -1, int = -1);
    void SetSendFlag();
    void setCalibrateFlag();
    void setShift(int);
    void Take_Contour(vector<vector<cv::Point> >, double, double);
    void SetCalibrationPoints(uint point, bool axis, bool incdec);
    void SetCalibrationPoints(vector<Point2f> );

private slots:
    void GetCalibrationPoints(vector<Point2f> );
    void on_pushButton_4_clicked();
    void on_pushButton_2_clicked();
    void imgShow(Mat const &, int);

    void on_pushButton_3_clicked();

    void on_verticalSlider_2_valueChanged(int value);

    void on_verticalSlider_3_valueChanged(int value);

    void on_pushButton_clicked();

    void on_pushButton_5_clicked();

    void on_lineEdit_2_editingFinished();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_16_clicked();

    void on_verticalSlider_sliderReleased();

    void on_verticalSlider_valueChanged(int value);

    void on_pushButton_17_clicked();

    void on_pushButton_18_clicked();

    void on_verticalSlider_4_valueChanged(int value);

    void on_pushButton_19_clicked();

    void on_pushButton_20_clicked();

    void on_pushButton_22_clicked();

    void on_pushButton_21_clicked();

    void on_pushButton_24_clicked();

    void on_verticalSlider_5_valueChanged(int value);

private:
    Ui::MainWindow * ui;

    My_TCPSocket * _socket;
    Laser_Machine * laserMachine_;

    vector<Point2f> * outputCorners_;

    bool _socketState = false;
    QGraphicsScene * scene;
    QGraphicsPixmapItem * itm;
    QImage * dst;
    QTimer * _timer;
    QTime * time;
    QFile * pointsFile;
};

#endif  // MAINWINDOW_H
