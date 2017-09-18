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
// #include "contour_analysing.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class QTimer;
class QTime;

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
    void SetCoeff(int = -1, int = -1, int = -1);
    void SetSendFlag();
    void Take_Contour(vector<vector<cv::Point> >, double, double);

private slots:
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

private:
    Ui::MainWindow * ui;

    My_TCPSocket * _socket;
    //    contour_analysing * _contour;

    bool _socketState = false;
    QGraphicsScene * scene;
    QGraphicsPixmapItem * itm;
    QImage * dst;
    //    QThread * th;
    QTimer * _timer;
    QTime * time;
};

#endif  // MAINWINDOW_H
