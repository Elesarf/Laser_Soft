#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QThread>
#include <QTimer>

#include "vector"

Q_DECLARE_METATYPE(cv::Point)
Q_DECLARE_METATYPE(vector<vector<cv::Point> > )

MainWindow::MainWindow(QWidget * parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<cv::Point>("cv::Point");
    qRegisterMetaType<vector<vector<cv::Point> > >("vector<vector<cv::Point>>");

    ui->setupUi(this);
    th			= new QThread();
    _contour	= new contour_analysing();

    _contour->moveToThread(th);
    th->start();

    connect(_contour, SIGNAL(Take_Frame(Mat const&,int)), this, SLOT(imgShow(Mat const&,int)));
    connect(this, SIGNAL(NextFrame()), _contour, SLOT(NextFrame()));
    connect(this, SIGNAL(SetCoeff(int,int,int)), _contour, SLOT(UpdateCoeff(int,int,int)));

    QObject::connect(_contour, &contour_analysing::talking, [ = ](QString s){
        ui->textBrowser->append(s);
    });

    scene	= new QGraphicsScene;
    itm		= new QGraphicsPixmapItem;
    scene->addItem(itm);
    ui->graphicsView->setScene(scene);

    _timer = new QTimer;
    _timer->start(100);
    connect(_timer, &QTimer::timeout, [](){
        qDebug() << "Tick";
    });
}

MainWindow::~MainWindow()
{
    if ( _socketState )
        delete _socket;
    delete _contour;
    delete ui;
}

void MainWindow::on_pushButton_4_clicked()
{
    if ( !_socketState ) {
        _socket = new My_TCPSocket();

        QObject::connect(_socket, &My_TCPSocket::talking, [ = ](QString s){
            ui->textBrowser_2->append(s);
        });

        _socket->Connect(ui->lineEdit->text().toUShort() );
        _socketState = _socket->GetState();
        connect(_contour, SIGNAL(Take_Contour(vector<vector<cv::Point> >,double,double)), _socket, SLOT(Send_Vector(vector<vector<cv::Point> >,double,double)));
        // _socket->moveToThread(th);
        ui->pushButton_4->setText("close socket");
    } else {
        delete _socket;
        _socketState = false;
        ui->pushButton_4->setText("open socket");
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    emit Quit();
}

void MainWindow::imgShow(const Mat &frame, int i)
{
    Mat tmp;

    qDebug() << "window on thread " << QObject::thread();
    frame.copyTo(tmp);
    cvtColor(tmp, tmp, CV_BGR2RGB);
    dst = new QImage(static_cast<uchar *>(tmp.data), tmp.cols, tmp.rows, static_cast<int>(tmp.step), QImage::Format_RGB888);
    dst->bits();

    if ( ui->radioButton->isChecked() && (i == 1) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    } else if ( ui->radioButton_2->isChecked() && (i == 2) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    } else if ( ui->radioButton_3->isChecked() && (i == 3) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    } else if ( ui->radioButton_4->isChecked() && (i == 4) ) {
        itm->setPixmap(QPixmap::fromImage(*dst) );
    }
    ui->graphicsView->fitInView(itm, Qt::AspectRatioMode::IgnoreAspectRatio);
    ui->graphicsView->update();

    delete  dst;
}   // MainWindow::imgShow

void MainWindow::on_pushButton_3_clicked()
{
    emit NextFrame();
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    emit SetCoeff(value);

    ui->label_3->setText(QString::number(value) );
}

void MainWindow::on_verticalSlider_2_valueChanged(int value)
{
    emit SetCoeff(-1, value);

    ui->label_4->setText(QString::number(value) );
}

void MainWindow::on_verticalSlider_3_valueChanged(int value)
{
    emit SetCoeff(-1, -1, value);

    ui->label_5->setText(QString::number(value) );
}

void MainWindow::on_pushButton_clicked()
{
    _contour->SetSendFlag();
}

void MainWindow::on_pushButton_5_clicked()
{
    if ( _socketState )
        _socket->SetShift(ui->lineEdit_3->text().toInt(), ui->lineEdit_4->text().toInt() );
}

void MainWindow::on_lineEdit_2_editingFinished()
{
    if ( _socketState )
        _socket->SetSize(ui->lineEdit_2->text().toInt() );
}

void MainWindow::on_pushButton_6_clicked()
{
    if ( _socketState ) {
        _socket->Write_X42();
        _socket->Reset();
    }
}

void MainWindow::on_pushButton_7_clicked()
{
    if ( _socketState )
        _socket->Write_X00();
}

void MainWindow::on_pushButton_8_clicked()
{
    if ( _socketState )
        _socket->SetSpeed(ui->lineEdit_5->text().toInt() );
}

void MainWindow::on_pushButton_9_clicked()
{
    if ( _socketState ) {
        if ( _socket->Get_Laser_State() )
            _socket->Write_X04(true);
        ui->pushButton_9->setText("piu_off");
    } else {
        _socket->Write_X04(false);
        ui->pushButton_9->setText("piu_on");
    }
}

void MainWindow::on_pushButton_10_clicked()
{
    if ( _socketState )
        _socket->SetPower(ui->lineEdit_6->text().toInt() );
}

void MainWindow::on_pushButton_15_clicked()
{
    double x, y;

    x	= ui->lineEdit_7->text().toDouble();
    y	= ui->lineEdit_8->text().toDouble();
    if ( _socketState ) {
        _socket->Write_X01(x, y, 0, 0);
        _socket->Write_X00();
    }
}

void MainWindow::on_pushButton_16_clicked()
{
    if ( _socketState )
        _socket->Write_X03();
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
//
//
//
//
//
//
//
//
