#include "contour_analysing.h"

#include <QDebug>
#include <QTime>
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#define SIZE_MIN_MARKER	80
#define SIZE_MAX_MARKER	240
#define TABLE_HEIGHT	620
#define TABLE_WIDTH		940
#define FRAME_WIDTH		2048
#define FRAME_HEIGHT	1526
// #define FRAME_WIDTH		3280
// #define FRAME_HEIGHT	2464

contour_analysing::contour_analysing(QObject * parent) : QObject(parent)
{
    min_con			= 300;
    max_con			= 500;
    wb				= 10;
    _coeff.x		= 0;
    _coeff.y		= 0;
    _mouseCoord.x	= 0;
    _mouseCoord.y	= 0;
    _mouseCoordList_hide.clear();

    timer = new  QTimer;

    time = new QTime;

    string filename = "/home/laser/qt_proj/Laser_Pi_Test_Cam/build-unidistortion-Desktop_Qt_5_7_1_GCC_64bit-Debug/cam.yml";
    FileStorage fs(filename, FileStorage::READ);
    fs["camera_matrix"] >> _cam_matrix;
    fs["distortion_coefficients"] >> _dist_coeff;

    QObject::connect(timer, &QTimer::timeout, [this](){
        Go();
        emit talking(QString("min %1, max %2, wb %3 ").arg(min_con).arg(max_con).arg(wb) );
    });
    timer->start(100);
}

void contour_analysing::Go()
{
    Mat rview, map1, map2;

    initUndistortRectifyMap(_cam_matrix, _dist_coeff, Mat(), getOptimalNewCameraMatrix(_cam_matrix, _dist_coeff, Size2i(2048, 1526), 1, Size2d(2048, 1526), 0), Size2d(2048,
      1526), CV_16SC2, map1, map2);

    if ( _flag ) {
        _frame = imread("/home/laser/raspberry_fs/pi/shr/1.jpg");
        //        remap(_frame, rview, map1, map2, INTER_LINEAR);
        //        rview.copyTo(_frame);
        _flag = false;
    }
    if ( !_frame.empty() ) {
        Contour_Analys(_frame);
    }
    timer->start(100);
}

void contour_analysing::UpdateCoeff(int min_con, int max_con, int wb)
{
    if ( min_con > 0 )
        this->min_con = min_con;
    if ( max_con > 0 )
        this->max_con = max_con;
    if ( wb  > 0 )
        this->wb = wb;
}

void contour_analysing::Contour_Analys(Mat fr)
{
    //    qDebug() << "Debug: CA";
    Mat frame;

    fr.copyTo(frame);
    emit Take_Frame(frame, 1);
    cv::Mat HSV, threshold;
    cv::cvtColor(frame, HSV, CV_BGR2HSV);
    inRange(HSV, Scalar(22, 115, 170), Scalar(34, 255, 255), threshold);

    vector<vector<Point> > contours1;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours1, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    vector<vector<Point> > contours;
    for ( unsigned i = 0; i < contours1.size(); ++i ) {
        if ( ( contours1[i].size() > static_cast<unsigned>(SIZE_MIN_MARKER) ) && ( contours1[i].size() < static_cast<unsigned>(SIZE_MAX_MARKER) ) ) {
            contours.push_back(contours1[i]);
        }
    }

    for ( unsigned i = 0; i < contours.size(); ++i ) {
        //        qDebug() << " -> " <<   contours[i].size();
        if ( contours[i].size() < static_cast<unsigned>(SIZE_MIN_MARKER) ) {
            contours.erase(contours.begin() + i);
        }
    }

    Mat mask = Mat::zeros(threshold.rows, threshold.cols, CV_8UC1);
    drawContours(mask, contours1, -1, Scalar(255), CV_FILLED, 1, hierarchy, 100, Point(0, 0) );

    Mat tmp;

    resize(mask, tmp, Size(1024, 768), 1, 1, CV_INTER_AREA);

    cvtColor(tmp, tmp, CV_GRAY2BGR);

    emit Take_Frame(tmp, 4);

    if ( contours.size() >= 4 ) {
        _p4.x	= contours[0][0].x;
        _p4.y	= contours[0][0].y;
        for ( unsigned i = 1; i < contours[0].size(); ++i ) {
            if ( contours[0][i].x < _p4.x ) {
                _p4.x = contours[0][i].x;
            }
            if ( contours[0][i].y < _p4.y ) {
                _p4.y = contours[0][i].y;
            }
        }

        _p3.x	= contours[1][0].x;
        _p3.y	= contours[1][0].y;
        for ( unsigned i = 1; i < contours[1].size(); ++i ) {
            if ( contours[1][i].x > _p3.x ) {
                _p3.x = contours[1][i].x;
            }
            if ( contours[1][i].y < _p3.y ) {
                _p3.y = contours[1][i].y;
            }
        }

        _p2.x	= contours[3][0].x;
        _p2.y	= contours[3][0].y;
        for ( unsigned i = 0; i < contours[3].size(); ++i ) {
            if ( contours[3][i].x > _p2.x ) {
                _p2.x = contours[3][i].x;
            }
            if ( _p2.y < contours[3][i].y ) {
                _p2.y = contours[3][i].y;
            }
        }

        _p1.x	= contours[2][0].x;
        _p1.y	= contours[2][0].y;
        for ( unsigned i = 1; i < contours[2].size(); ++i ) {
            if ( contours[2][i].x < _p1.x ) {
                _p1.x = contours[2][i].x;
            }
            if ( contours[2][i].y > _p1.y ) {
                _p1.y = contours[2][i].y;
            }
        }
    }

    //    Point pt = _p1;
    //    _p1 = _p2;
    //    _p2 = pt;
    //    pt	= _p3;
    //    _p3 = _p4;
    //    _p4 = pt;

    vector<Point2f> outputCorners(4);
    outputCorners[0]	= _p1;
    outputCorners[1]	= _p2;
    outputCorners[2]	= _p3;
    outputCorners[3]	= _p4;
    Rect br = boundingRect(outputCorners);


    vector<Point2f> inputCorners(4);
    inputCorners[0]		= br.tl();
    inputCorners[1].x	= br.br().x;
    inputCorners[1].y	= br.tl().y;
    inputCorners[2].x	= br.br().x;
    inputCorners[2].y	= br.br().y;
    inputCorners[3].x	= br.tl().x;
    inputCorners[3].y	= br.br().y;

    Mat M = getPerspectiveTransform(outputCorners, inputCorners);
    Mat out;

    warpPerspective(frame, out, M, Size(FRAME_WIDTH, FRAME_HEIGHT) );
    rectangle(frame, br, Scalar(255), 3, 2);

    for ( int i = 0; i < FRAME_WIDTH; i += 200 ) {
        putText(frame, QString::number(i).toStdString(), Point(i, 10), 1, 1, Scalar(0, 0, 125), 1, 1);
    }

    for ( int i = 0; i < FRAME_HEIGHT; i += 200 ) {
        putText(frame, QString::number(i).toStdString(), Point(10, i), 1, 1, Scalar(0, 0, 125), 1, 1);
    }

    putText(frame, QString(".1 %1:%2").arg(_p1.x).arg(_p1.y).toStdString(), _p1, 1, 2, Scalar(0, 0, 125), 2, 1);
    putText(frame, QString(".2 %1:%2").arg(_p2.x).arg(_p2.y).toStdString(), _p2, 1, 2, Scalar(0, 0, 125), 2, 1);
    putText(frame, QString(".3 %1:%2").arg(_p3.x).arg(_p3.y).toStdString(), _p3, 1, 2, Scalar(0, 0, 125), 2, 1);
    putText(frame, QString(".4 %1:%2").arg(_p4.x).arg(_p4.y).toStdString(), _p4, 1, 2, Scalar(0, 0, 125), 2, 1);
    out = out(br);
    Mat tmp1;
    flip(out, tmp1, 1);
    emit Take_Frame(frame, 1);
    //    imshow("dsdsds", frame);
    Coordinator(tmp1);
}   // contour_analysing::Contour_Analys

void myMouseCallback(int event, int x, int y, int flags, void * param);

void contour_analysing::Coordinator(Mat inImg)
{
    //    qDebug() << "Debug: Coord";
    emit Take_Frame(inImg, 3);

    cv::Mat HSV, threshold;
    GaussianBlur(inImg, inImg, cv::Size(3, 3), 2, 2);
    cv::cvtColor(inImg, HSV, CV_BGR2GRAY);
    adaptiveThreshold(HSV, threshold, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 5, 1);
    vector<vector<Point> > contours1;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours1, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    vector<vector<Point> > contours;

    for ( unsigned i = 0; i < contours1.size(); ++i ) {
        if ( ( contours1[i].size() > static_cast<unsigned>(min_con) ) && ( contours1[i].size() < static_cast<unsigned>(max_con) ) ) {
            contours.push_back(contours1[i]);
        }
    }

    Mat mask = Mat::zeros(threshold.rows, static_cast<int>( threshold.rows * (TABLE_WIDTH / static_cast<double>(TABLE_HEIGHT) ) ), CV_8UC1);
    _coeff.x	= mask.cols / static_cast<double>(TABLE_WIDTH) + 160 / 940; // TODO: fuck this magic
    _coeff.y	= mask.rows / static_cast<double>( TABLE_HEIGHT);

    vector<vector<Point> > contours_to_send;
    for ( auto &l:_mouseCoordList_hide ) {
        if ( l.x != 0 || l.y != 0 ) {
            Point p;
            p.x = l.x;
            p.y = l.y;
            for ( unsigned i = 0; i < contours.size(); ++i ) {
                Rect cr	= boundingRect(contours[i]);
                int x, y;
                x	= l.x;
                y	= l.y;
                for ( unsigned j = 0; j < contours[i].size(); ++j ) {
                    if ( x > cr.tl().x && x < cr.br().x ) {
                        if ( y > cr.tl().y && y < cr.br().y ) {
                            if ( !l.show ) {
                                contours.erase(contours.begin() + i);
                                break;
                            } else {
                                contours_to_send.push_back(contours[i]);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    drawContours(mask, contours, -1, Scalar(70), 1);
    drawContours(mask, contours_to_send, -1, Scalar(255), 1);

    for ( unsigned j = 0; j < contours.size(); ++j ) {
        Rect cr	= boundingRect(contours[j]);
        Point szMax(contours[j][0].x, contours[j][0].y), sz1_Min(contours[j][0].x, contours[j][0].y);
        for ( unsigned i = 0; i < contours[j].size(); ++i ) {
            if ( szMax.x < contours[j][i].x )
                szMax.x = contours[j][i].x;
            if ( sz1_Min.x > contours[j][i].x )
                sz1_Min.x = contours[j][i].x;

            if ( szMax.y < contours[j][i].y )
                szMax.y = contours[j][i].y;
            if ( sz1_Min.y > contours[j][i].y )
                sz1_Min.y = contours[j][i].y;
        }

        int lenght	= std::abs(sz1_Min.x - szMax.x);
        int height	= std::abs(sz1_Min.y - szMax.y);

        rectangle(mask, cr, Scalar(125), 1, 0);

        putText(mask, QString(".").toStdString(), Point(sz1_Min.x, sz1_Min.y), 1, 6, Scalar(255), 1, 1);

        putText(mask, QString("height = %1").arg(height / _coeff.x).toStdString(), Point(szMax.x - 100, szMax.y + 20), 1, 1, Scalar(255), 1, 1);
        putText(mask, QString("lenght = %1").arg(lenght / _coeff.x).toStdString(), Point(szMax.x - 100, szMax.y + 40), 1, 1, Scalar(255), 1, 1);

        putText(mask, QString("%1:%2").arg(mask.cols / _coeff.x - sz1_Min.x / _coeff.x).arg(sz1_Min.y / _coeff.y).toStdString(), sz1_Min, 1, 1, Scalar(
              255), 1, 1);

        putText(mask, QString("counter_size = %1").arg(contours[j].size() ).toStdString(), Point(szMax.x - 100, szMax.y + 60), 1, 1, Scalar(255), 1, 1);

        line(mask, Point(abs( (szMax.x + sz1_Min.x) / 2 )  - 20, szMax.y), Point(abs( (szMax.x + sz1_Min.x) / 2 ) + 20, szMax.y), Scalar(255) );
        line(mask, Point(abs( (sz1_Min.x + szMax.x) / 2 )  - 20, sz1_Min.y), Point(abs( (szMax.x + sz1_Min.x) / 2 ) + 20, sz1_Min.y), Scalar(255) );
    }

    cvtColor(mask, mask, CV_GRAY2BGR);
    emit Take_Frame(mask, 2);

    imshow("ds", mask);
    setMouseCallback("ds", myMouseCallback, &_mouseCoordList_hide);
    _mouseCoordList_hide.size();

    talking(QString("coeff = %1 %2").arg(_coeff.x, 3, 'A', 10, QChar('0') ).arg(_coeff.y, 3, 'A', 10, QChar('0') ) );
    if ( _flagToSend ) {
        emit Take_Contour(contours_to_send, _coeff.x, _coeff.y);
        _flagToSend = false;
    }
}   // contour_analysing::Coordinator

void contour_analysing::NextFrame()
{
    _mouseCoordList_hide.clear();
    _flag = true;
}

void myMouseCallback(int event, int x, int y, int flags, void * param)
{
    QList<contour_analysing::MouseCoord> * mclh = static_cast<QList<contour_analysing::MouseCoord> *>(param); \
    contour_analysing::MouseCoord mc;
    switch ( event ) {
        case CV_EVENT_MOUSEMOVE:
            break;

        case CV_EVENT_LBUTTONDOWN:
            mc.x	= x;
            mc.y	= y;
            mc.show = (flags == 40) ? false : true;
            mclh->append(mc);
            //            qDebug() << "Mouse press" << x << " : " << y;
            break;

        case CV_EVENT_LBUTTONUP:
            break;
        case CV_EVENT_RBUTTONDOWN:
            break;
    }
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
