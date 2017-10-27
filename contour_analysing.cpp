#include "contour_analysing.h"

#include <iostream>
#include <QDebug>
#include <QTime>
#include <math.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#define SIZE_MIN_MARKER	160
#define SIZE_MAX_MARKER	440
#define TABLE_HEIGHT	400
#define TABLE_WIDTH		400
// #define TABLE_HEIGHT	619
// #define TABLE_WIDTH		910
// #define TABLE_HEIGHT	620
// #define TABLE_WIDTH		940
// #define FRAME_WIDTH		2048 // standart Pi camera
// #define FRAME_HEIGHT	1526
#define FRAME_WIDTH		2592    // usb 3.0 camera module
#define FRAME_HEIGHT	1944

contour_analysing::contour_analysing(QObject * parent) : QObject(parent)
{
    min_con			= 300;
    max_con			= 500;
    wb				= 10;
    _scale			= 0;
    _coeff.x		= 0;
    _coeff.y		= 0;
    _mouseCoord.x	= 0;
    _mouseCoord.y	= 0;
    _mouseCoordList_hide.clear();


    timer = new  QTimer(this);
    outputCorners_ = new vector<Point2f>(4);

    QObject::connect(timer, &QTimer::timeout, [this](){
        Go();
        emit talking(QString("min %1, max %2, wb %3 ").arg(min_con).arg(max_con).arg(wb) );
    });
    timer->start(600);

    namedWindow("contours");
    resizeWindow("contours", 1, 1);
    moveWindow("contours", 765, 10);
}

contour_analysing::~contour_analysing()
{
    this->timer->stop();
    this->timer->deleteLater();
    delete outputCorners_;
}

void contour_analysing::Stop()
{
    timer->stop();
}

void contour_analysing::Go()
{
    if ( _flag ) {
        string filename = "/home/laser/qt_proj/Laser_Pi_Test_Cam/build-unidistortion-Desktop_Qt_5_7_1_GCC_64bit-Debug/cam.yml";
        FileStorage fs(filename, FileStorage::READ);
        fs["camera_matrix"] >> _cam_matrix;
        fs["distortion_coefficients"] >> _dist_coeff;

        _frame = imread("/home/laser/raspberry_fs/pi/shr/1.jpg");
        Mat rview, map1, map2;

        initUndistortRectifyMap(_cam_matrix, _dist_coeff, Mat(), getOptimalNewCameraMatrix(_cam_matrix, _dist_coeff, Size2i(2592, 1944), 1, Size2d(2592, 1944), 0), Size2d(2592,
          1944), CV_16SC2, map1, map2);
        remap(_frame, rview, map1, map2, INTER_LINEAR);
        rview.copyTo(_frame);
        //        imshow("sd", rview);
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

void contour_analysing::SetCalibrationPoints(uint point, bool axis, bool incdec)
{
    if ( point < 5 ) {
        if ( axis ) {       // x
            if ( incdec ) { // inc
                outputCorners_->at(point - 1).x++;
            } else {
                outputCorners_->at(point - 1).x--;
            }
        } else {
            if ( incdec ) { // inc
                outputCorners_->at(point - 1).y++;
            } else {
                outputCorners_->at(point - 1).y--;
            }
        }
        emit GetCalibrationPoints(*outputCorners_);
    }
    qDebug()	<< "1:" << outputCorners_->at(0).x << ":" << outputCorners_->at(0).y
                << ":2" << outputCorners_->at(1).x << ":" << outputCorners_->at(1).y
                << ":3" << outputCorners_->at(2).x << ":" << outputCorners_->at(2).y
                << ":4" << outputCorners_->at(3).x << ":" << outputCorners_->at(3).y;
}

void contour_analysing::SetCalibrationPoints(vector<Point2f> ov)
{
    ov.swap(*outputCorners_);
}

void contour_analysing::Contour_Analys(Mat fr)
{
    Mat frame;

    fr.copyTo(frame);
    //    if ( calibrateFlag_ )
    emit Take_Frame(frame, 1);
    cv::Mat HSV, threshold;
    cv::cvtColor(frame, HSV, CV_BGR2HSV);
    inRange(HSV, Scalar(20, 100, 170), Scalar(34, 255, 255), threshold);
    //    inRange(HSV, Scalar(165, 109, 146), Scalar(189, 195, 243), threshold);

    vector<vector<Point> > contours1;
    vector<Vec4i> hierarchy;
    findContours(threshold, contours1, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    vector<vector<Point> > contours;
    for ( unsigned i = 0; i < contours1.size(); ++i ) {
        if ( ( contours1.at(i).size() > static_cast<unsigned>(SIZE_MIN_MARKER) ) && ( contours1.at(i).size() < static_cast<unsigned>(SIZE_MAX_MARKER) ) ) {
            contours.push_back(contours1.at(i) );
        }
    }

    for ( unsigned i = 0; i < contours.size(); ++i ) {
        if ( contours.at(i).size() < static_cast<unsigned>(SIZE_MIN_MARKER) ) {
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
        if ( contours.at(2).at(0).x < contours.at(3).at(0).x )
            contours.at(2).swap(contours.at(3) );
        if ( contours.at(0).at(0).x < contours.at(1).at(0).x )
            contours.at(1).swap(contours.at(0) );
        _p4.x	= contours.at(0).at(0).x;
        _p4.y	= contours.at(0).at(0).y;
        for ( unsigned i = 1; i < contours.at(0).size(); ++i ) {
            if ( contours.at(0).at(i).x < _p4.x ) {
                _p4.x = contours.at(0).at(i).x;
            }
            if ( contours.at(0).at(i).y < _p4.y ) {
                _p4.y = contours.at(0).at(i).y;
            }
        }

        _p3.x	= contours.at(1).at(0).x;
        _p3.y	= contours.at(1).at(0).y;
        for ( unsigned i = 1; i < contours.at(1).size(); ++i ) {
            if ( contours.at(1).at(i).x > _p3.x ) {
                _p3.x = contours.at(1).at(i).x;
            }
            if ( contours.at(1).at(i).y < _p3.y ) {
                _p3.y = contours.at(1).at(i).y;
            }
        }

        _p2.x	= contours.at(3).at(0).x;
        _p2.y	= contours.at(3).at(0).y;
        for ( unsigned i = 0; i < contours.at(3).size(); ++i ) {
            if ( contours.at(3).at(i).x > _p2.x ) {
                _p2.x = contours.at(3).at(i).x;
            }
            if ( _p2.y < contours.at(3).at(i).y ) {
                _p2.y = contours.at(3).at(i).y;
            }
        }

        _p1.x	= contours.at(2).at(0).x;
        _p1.y	= contours.at(2).at(0).y;
        for ( unsigned i = 1; i < contours.at(2).size(); ++i ) {
            if ( contours.at(2).at(i).x < _p1.x ) {
                _p1.x = contours.at(2).at(i).x;
            }
            if ( contours.at(2).at(i).y > _p1.y ) {
                _p1.y = contours.at(2).at(i).y;
            }
        }
    }


    if ( calibrateFlag_ ) {
        try{
            outputCorners_->at(0)	= _p1;
            outputCorners_->at(1)	= _p2;
            outputCorners_->at(2)	= _p3;
            outputCorners_->at(3)	= _p4;
        } catch ( const exception &e ) {
            qDebug() << "Error calibrating " << e.what();
        }
    }

    Rect br = boundingRect(*outputCorners_);

    vector<Point2f> inputCorners(4);
    inputCorners.at(0)		= br.tl();
    inputCorners.at(1).x	= br.br().x;
    inputCorners.at(1).y	= br.tl().y;
    inputCorners.at(2).x	= br.br().x;
    inputCorners.at(2).y	= br.br().y;
    inputCorners.at(3).x	= br.tl().x;
    inputCorners.at(3).y	= br.br().y;

    Mat M = getPerspectiveTransform(*outputCorners_, inputCorners);
    Mat out;
    frame.copyTo(out);
    warpPerspective(frame, out, M, Size(FRAME_WIDTH, FRAME_HEIGHT) );
    rectangle(frame, br, Scalar(255), 3, 2);

    for ( int i = 0; i < FRAME_WIDTH; i += 200 ) {
        putText(frame, QString::number(i).toStdString(), Point(i, 10), 1, 1, Scalar(0, 0, 125), 1, 1);
    }

    for ( int i = 0; i < FRAME_HEIGHT; i += 200 ) {
        putText(frame, QString::number(i).toStdString(), Point(10, i), 1, 1, Scalar(0, 0, 125), 1, 1);
    }
    if ( calibrateFlag_ ) {
        putText(frame, QString(".1 %1:%2").arg(_p1.x).arg(_p1.y).toStdString(), _p1, 1, 3, Scalar(0, 0, 125), 2, 1);
        putText(frame, QString(".2 %1:%2").arg(_p2.x).arg(_p2.y).toStdString(), _p2, 1, 3, Scalar(0, 0, 125), 2, 1);
        putText(frame, QString(".3 %1:%2").arg(_p3.x).arg(_p3.y).toStdString(), _p3, 1, 3, Scalar(0, 0, 125), 2, 1);
        putText(frame, QString(".4 %1:%2").arg(_p4.x).arg(_p4.y).toStdString(), _p4, 1, 3, Scalar(0, 0, 125), 2, 1);

        line(frame, _p1, _p2, Scalar(0, 125, 0), 5, 1, 0);
        line(frame, _p2, _p3, Scalar(0, 125, 0), 5, 1, 0);
        line(frame, _p3, _p4, Scalar(0, 125, 0), 5, 1, 0);
        line(frame, _p4, _p1, Scalar(0, 125, 0), 5, 1, 0);
    }
    out = out(br);
    Mat tmp1;
    flip(out, tmp1, 1);
    //    out.copyTo(tmp1);
    if ( calibrateFlag_ )
        emit Take_Frame(frame, 1);
    Coordinator(tmp1);
}   // contour_analysing::Contour_Analys

void myMouseCallback(int event, int x, int y, int flags, void * param);

void contour_analysing::Coordinator(Mat inImg)
{
    cv::Mat HSV, threshold_img;
    cv::cvtColor(inImg, HSV, CV_BGR2GRAY);
    medianBlur(HSV, HSV, 7);
    threshold(HSV, threshold_img, _thVal, 255, THRESH_BINARY_INV);
    vector<vector<Point> > contours1;
    vector<Vec4i> hierarchy1;

    findContours(threshold_img, contours1, hierarchy1, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_KCOS, Point(0, 0) );
    cvtColor(threshold_img, threshold_img, CV_GRAY2BGR);
    emit Take_Frame(threshold_img, 3);
    vector<vector<Point> > contours;

    for ( unsigned i = 0; i < contours1.size(); ++i ) {
        if ( ( contours1.at(i).size() > static_cast<unsigned>(min_con) ) && ( contours1.at(i).size() < static_cast<unsigned>(max_con) ) ) {
            contours.push_back(contours1.at(i) );
        }
    }

    Mat mask = Mat::zeros(threshold_img.rows, threshold_img.cols, CV_8UC1);
    _coeff.x	= mask.cols / static_cast<double>(TABLE_WIDTH);
    _coeff.y	= mask.rows / static_cast<double>( TABLE_HEIGHT);


    vector<vector<Point> > contours_to_send;
    for ( auto &l:_mouseCoordList_hide ) {
        if ( l.x != 0 || l.y != 0 ) {
            Point p;
            p.x = l.x;
            p.y = l.y;
            for ( unsigned i = 0; i < contours.size(); ++i ) {
                Rect cr	= boundingRect(contours.at(i) );
                int x, y;
                x	= l.x;
                y	= l.y;
                for ( unsigned j = 0; j < contours.at(i).size(); ++j ) {
                    if ( x > cr.tl().x && x < cr.br().x ) {
                        if ( y > cr.tl().y && y < cr.br().y ) {
                            if ( !l.show ) {
                                contours.erase(contours.begin() + i);
                                break;
                            } else {
                                contours_to_send.push_back(contours.at(i) );
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    Mat tes;
    inImg.copyTo(tes);
    tes *= 0.9;
    vector<vector<Point> > contours_to_send_1(contours_to_send.size() );
    if ( _coeff.x > 1.5 ) {
        try {
            for ( unsigned i = 0; i < contours_to_send.size(); ++i ) {
                for ( unsigned j = 0; j < contours_to_send.at(i).size(); ++j ) {
                    //                    if ( j % wb == 0 ) {
                    contours_to_send_1.at(i).push_back(contours_to_send.at(i).at(j) );
                    //                    }
                }
            }
        } catch ( const std::exception &e ) {
            qDebug() << "Error decimation " << e.what();
        }

        try {
            for ( unsigned i = 0; i < contours_to_send_1.size(); ++i ) {
                for ( unsigned j = 0; j < (contours_to_send_1.at(i).size() / 2); ++j ) {
                    contours_to_send_1.at(i).at(j).x *= 1.0 - static_cast<double>(_scale) / 1000.0;
                    contours_to_send_1.at(i).at(contours_to_send_1.at(i).size() - j - 1).x *= 1.0 + static_cast<double>(_scale) / 1000.0;
                }
                unsigned long sh = contours_to_send_1.at(i).size() / 3 + contours_to_send_1.at(i).size() / 2;
                for ( unsigned long j = 0; j < (contours_to_send_1.at(i).size() / 2); ++j ) {
                    contours_to_send_1.at(i).at(sh - j).y *= 1.0 + static_cast<double>(_scale) / 1000.0;
                    if ( (j + sh) < contours_to_send_1.at(i).size() ) {
                        contours_to_send_1.at(i).at(sh + j).y *= 1.0 - static_cast<double>(_scale) / 1000.0;
                    } else {
                        contours_to_send_1.at(i).at(sh + j - contours_to_send_1.at(i).size() ).y *= 1.0 - static_cast<double>(_scale) / 1000.0;
                    }
                }
            }
        } catch ( const std::exception &e ) {
            qDebug() << "Error scalling " << e.what();
        }

        drawContours(tes, contours, -1, Scalar(255, 255, 0), 1);
        drawContours(tes, contours_to_send_1, -1, Scalar(0, 0, 255), 2);
    }

    for ( unsigned j = 0; j < contours.size(); ++j ) {
        Rect cr	= boundingRect(contours.at(j) );
        Point szMax(contours.at(j)[0].x, contours.at(j)[0].y), sz1_Min(contours.at(j)[0].x, contours.at(j)[0].y);
        for ( unsigned i = 0; i < contours.at(j).size(); ++i ) {
            if ( szMax.x < contours.at(j).at(i).x )
                szMax.x = contours.at(j).at(i).x;
            if ( sz1_Min.x > contours.at(j).at(i).x )
                sz1_Min.x = contours.at(j).at(i).x;

            if ( szMax.y < contours.at(j).at(i).y )
                szMax.y = contours.at(j).at(i).y;
            if ( sz1_Min.y > contours.at(j).at(i).y )
                sz1_Min.y = contours.at(j).at(i).y;
        }

        int lenght	= std::abs(sz1_Min.x - szMax.x);
        int height	= std::abs(sz1_Min.y - szMax.y);

        rectangle(tes, cr, Scalar(255, 125), 1, 0);

        putText(tes, QString("h: %1").arg(height / _coeff.x).toStdString(), Point(sz1_Min.x, sz1_Min.y + 20), 1, 1, Scalar(0, 255, 255), 1, 1);
        putText(tes, QString("l: %1").arg(lenght / _coeff.x).toStdString(), Point(sz1_Min.x, sz1_Min.y + 40), 1, 1, Scalar(0, 255, 255), 1, 1);

        putText(tes, QString("%1:%2").arg(static_cast<int>(tes.cols / _coeff.x - sz1_Min.x / _coeff.x) ).arg(
              static_cast<int>(sz1_Min.y / _coeff.y) ).toStdString(), sz1_Min, 1, 1, Scalar(0, 255, 255), 1, 1);

        putText(tes, QString("cs:  %1").arg(contours.at(j).size() ).toStdString(), Point(sz1_Min.x, sz1_Min.y + 60), 1, 1, Scalar(255, 255), 1, 1);

        //        for ( int i = 0; i < tes.cols; i += 40 * _coeff.x ) {
        //            line(tes, Point(i, 0), Point(i, tes.cols), Scalar(0, 255, 255), 2);
        //            line(tes, Point(i, 0), Point(i, tes.cols), Scalar(0, 255, 255), 2);
        //        }

        //        for ( int i = 0; i < tes.rows; i += 40 * _coeff.y ) {
        //            line(tes, Point(0, i), Point(tes.rows, i), Scalar(0, 255, 255), 2);
        //            line(tes, Point(0, i), Point(tes.rows, i), Scalar(0, 255, 255), 2);
        //        }
    }

    cvtColor(mask, mask, CV_GRAY2BGR);

    emit Take_Frame(mask, 2);
    Mat show;
    resize(tes, show, Size2d(tes.cols / 1.4, tes.rows / 1.4), 1, 1, cv::INTER_LINEAR);
    imshow("contours", show);
    setMouseCallback("contours", myMouseCallback, &_mouseCoordList_hide);
    _mouseCoordList_hide.size();

    emit talking(QString("coeff = %1 %2").arg(_coeff.x, 3, 'A', 10, QChar('0') ).arg(_coeff.y, 3, 'A', 10, QChar('0') ) );

    if ( _flagToSend ) {
        qDebug() << "mouse coord size" << _mouseCoordList_hide.size();
        emit Take_Contour(contours_to_send_1, _coeff.x, _coeff.y);
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
    QList<contour_analysing::MouseCoord> * mclh = static_cast<QList<contour_analysing::MouseCoord> *>(param);
    contour_analysing::MouseCoord mc;
    switch ( event ) {
        case CV_EVENT_MOUSEMOVE:
            break;

        case CV_EVENT_LBUTTONDOWN: {
            mc.x	= x * 1.4;
            mc.y	= y * 1.4;
            mc.show = (flags == 40) ? false : true;
            mclh->append(mc);
            break;
        }

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
