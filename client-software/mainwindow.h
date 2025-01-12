#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <QTcpSocket>

class QCustomPlot;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    typedef enum {
        triggerFalling,
        triggerRising
    } TriggerMode;

    struct Data {
        QVector<double> time;
        QVector<double> pres_1;
        QVector<double> temp_1;
        QVector<double> pres_2;
        QVector<double> temp_2;
    };

    struct Deviation {
        QVector<double> time;
        QVector<double> deviation_1;
        QVector<double> deviation_2;
    };

public slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onErrorOccurred(QAbstractSocket::SocketError socketError);

private slots:
    void on_options_clicked();

    void on_saveCancel_clicked();

    void on_trialNumber_valueChanged(int value);

private:
    Ui::MainWindow *ui;

    float maxPressure;

    QTcpSocket *socket;
    QString hostName;
    QString buf;

    TriggerMode triggerMode;
    float triggerPressure;

    Data liveData;
    Data savedData;
    Deviation deviation;

    bool triggered;
    float triggerTime;

    QString logFolder;
    int trialNumber;

    void configurePlot(QCustomPlot *plot);
    void updateTrigger();
    void updatePlots();
    void updateInterface();
    void saveData();
    void writeData();

    void setTrialNumber(int val);
};
#endif // MAINWINDOW_H
