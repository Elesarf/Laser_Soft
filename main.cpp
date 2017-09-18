#include "mainwindow.h"
#include "contour_analysing.h"
#include "my_tcpsocket.h"
#include <QApplication>
#include <QThread>


Q_DECLARE_METATYPE(cv::Point)
Q_DECLARE_METATYPE(vector<vector<cv::Point> > )

int main(int argc, char * argv[])
{
    QApplication a(argc, argv);

    qRegisterMetaType<cv::Point>("cv::Point");
    qRegisterMetaType<vector<vector<cv::Point> > >("vector<vector<cv::Point>>");

    MainWindow w;
    contour_analysing ca;
    QThread th;

    ca.moveToThread(&th);
    th.start();

    QObject::connect(&ca, SIGNAL(Take_Frame(Mat const&,int)), &w, SLOT(imgShow(Mat const&,int)));
    QObject::connect(&ca, SIGNAL(talking(const QString&)), &w, SLOT(talking(const QString&)));
    QObject::connect(&ca, SIGNAL(Take_Contour(vector<vector<cv::Point> >,double,double)), &w, SLOT(tr_Take_Contour(vector<vector<cv::Point> >,double,double)));
    QObject::connect(&w, SIGNAL(NextFrame()), &ca, SLOT(NextFrame()));
    QObject::connect(&w, SIGNAL(SetCoeff(int,int,int)), &ca, SLOT(UpdateCoeff(int,int,int)));
    QObject::connect(&w, SIGNAL(SetSendFlag()), &ca, SLOT(SetSendFlag()));


    w.show();
    QObject::connect(&w, SIGNAL(Quit()), &a, SLOT(quit()));

    return a.exec();
}
