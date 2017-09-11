#ifndef CONTOUR_ANALYSING_H
#define CONTOUR_ANALYSING_H

#include <QObject>
#include <QTimer>

#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

class QTime;

class contour_analysing : public QObject
{
    Q_OBJECT
public:
    explicit contour_analysing(QObject * parent = NULL);

signals:
    void Take_Contour(vector<vector<cv::Point> >, double, double);
    void Take_Frame(Mat const &, int);
    void talking(const QString &);

public slots:

    void Contour_Analys(Mat);
    void Coordinator(Mat);
    void NextFrame();
    void Go();
    void UpdateCoeff(int, int, int);
    void SetSendFlag(){ _flagToSend = true; }

public:

    struct Coeff {
        double	x	= 0;
        double	y	= 0;
    };

private:

    Mat _frame;
    Mat _cam_matrix, _dist_coeff;
    Point _p1, _p2, _p3, _p4;
    Point proi1, proi2;
    Coeff _coeff;

    QTimer * timer;

    QTime * time;

    int min_con, max_con, wb;
    bool _flag = true;
    bool _flagToSend = false;
};

#endif  // CONTOUR_ANALYSING_H
