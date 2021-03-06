#include "mainwindow.h"
#include "contour_analysing.h"
#include "my_tcpsocket.h"
#include <QApplication>
#include <QThread>


Q_DECLARE_METATYPE(cv::Point)
Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(vector<vector<cv::Point> > )
Q_DECLARE_METATYPE(vector<Point2f> )

int main(int argc, char * argv[])
{
    QApplication a(argc, argv);

    qRegisterMetaType<cv::Mat>("Mat");
    qRegisterMetaType<cv::Point>("cv::Point");
    qRegisterMetaType<vector<vector<cv::Point> > >("vector<vector<cv::Point>>");
    qRegisterMetaType<vector<Point2f> >("vector<Point2f>");

    MainWindow w;
    contour_analysing ca;
    QThread th;

    //    ca.setParent(0);
    ca.moveToThread(&th);

    QObject::connect(&ca, SIGNAL(Take_Frame(Mat const&,int)), &w, SLOT(imgShow(Mat const&,int)));
    QObject::connect(&ca, SIGNAL(talking(const QString&)), &w, SLOT(talking(const QString&)));
    QObject::connect(&ca, SIGNAL(Take_Contour(vector<vector<cv::Point> >,double,double)), &w, SLOT(tr_Take_Contour(vector<vector<cv::Point> >,double,double)));
    QObject::connect(&w, SIGNAL(NextFrame()), &ca, SLOT(NextFrame()));
    QObject::connect(&w, SIGNAL(SetCoeff(int,int,int)), &ca, SLOT(UpdateCoeff(int,int,int)));
    QObject::connect(&w, SIGNAL(SetSendFlag()), &ca, SLOT(SetSendFlag()));
    QObject::connect(&w, SIGNAL(SetScale(int)), &ca, SLOT(SetScale(int)));
    QObject::connect(&w, SIGNAL(SetThVal(int)), &ca, SLOT(SetThVal(int)));
    QObject::connect(&w, SIGNAL(SetCalibrationPoints(uint,bool,bool)), &ca, SLOT(SetCalibrationPoints(uint,bool,bool)));
    QObject::connect(&w, SIGNAL(setCalibrateFlag()), &ca, SLOT(setCalibrateFlag()));
    QObject::connect(&ca, SIGNAL(GetCalibrationPoints(vector<Point2f> )), &w, SLOT(GetCalibrationPoints(vector<Point2f> )));
    QObject::connect(&w, SIGNAL(SetCalibrationPoints(vector<Point2f> )), &ca, SLOT(SetCalibrationPoints(vector<Point2f> )));

    th.start();
    w.show();

    QObject::connect(&w, &MainWindow::Quit, [&](){
        ca.Stop();
        th.exit();
    });

    QObject::connect(&w, SIGNAL(Quit()), &a, SLOT(quit()));

    return a.exec();
}   // main
