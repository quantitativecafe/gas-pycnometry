#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QSettings>
#include <QBluetoothDeviceDiscoveryAgent>
#include <QLowEnergyController>
#include <QLowEnergyService>
#include <QLowEnergyCharacteristic>

#include "optionsdialog.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    maxPressure(103.42f),
    discoveryAgent(new QBluetoothDeviceDiscoveryAgent(this)),
    bleController(nullptr),
    bleService(nullptr),
    dataCharacteristic(),
    triggerMode(triggerFalling),
    triggerPressure(80),
    triggered(false),
    logFolder("trials"),
    trialNumber(1)
{
    ui->setupUi(this);

    // Get persistent settings
    QSettings settings("QuantitativeCafe", "GasPyc-2");
    triggerMode = (TriggerMode) settings.value("triggerMode", triggerMode).toInt();
    triggerPressure = settings.value("triggerPressure", triggerPressure).toFloat();
    logFolder = settings.value("logFolder", logFolder).toString();
    trialNumber = settings.value("trialNumber", trialNumber).toInt();

    // Configure plots
    configurePlot(ui->livePlot);
    configurePlot(ui->savedPlot);

    // Configure saved plot
    ui->savedPlot->setBackground(QBrush(QColor(0, 0, 0, 0)));
    ui->savedPlot->axisRect()->setBackground(QBrush(QColor(255, 255, 255, 255)));

    // Configure BLE
    connect(discoveryAgent, &QBluetoothDeviceDiscoveryAgent::deviceDiscovered, this, &MainWindow::onDeviceDiscovered);
    connect(discoveryAgent, &QBluetoothDeviceDiscoveryAgent::errorOccurred, this, &MainWindow::onDeviceDiscoveryError);

    ui->status->append("Scanning for BLE devices...");
    discoveryAgent->start();

    // Update filename
    setTrialNumber(trialNumber);
}

MainWindow::~MainWindow()
{
    if (bleController) {
        bleController->disconnectFromDevice();
        delete bleController;
    }
    delete discoveryAgent;
    delete ui;
}

// Calculates the linear regression of the points in xs and ys.
// Returns a pair (slope, intercept) representing the line y = slope * x + intercept.
static std::pair<double, double> linearRegression(const QVector<double> &xs, const QVector<double> &ys)
{
    // Validate input
    if (xs.size() != ys.size())
    {
        throw std::invalid_argument("xs and ys must have the same size");
    }
    if (xs.size() < 2)
    {
        throw std::invalid_argument("xs and ys must have at least 2 elements");
    }

    // Calculate the mean of x and y
    double x_mean = std::accumulate(xs.begin(), xs.end(), 0.0f) / xs.size();
    double y_mean = std::accumulate(ys.begin(), ys.end(), 0.0f) / ys.size();

    // Calculate the sum of the squares of the differences between x and the mean of x
    double x_diff_squared_sum = std::inner_product(xs.begin(), xs.end(), xs.begin(), 0.0f, std::plus<>(),
                                                  [&](double x, double y) { Q_UNUSED(y); return (x - x_mean) * (x - x_mean); });

    // Calculate the sum of the product of the differences between x and the mean of x and the differences between y and the mean of y
    double xy_diff_sum = std::inner_product(xs.begin(), xs.end(), ys.begin(), 0.0f, std::plus<>(),
                                           [&](double x, double y) { return (x - x_mean) * (y - y_mean); });

    // Calculate the slope of the line
    double slope = xy_diff_sum / x_diff_squared_sum;

    // Calculate the intercept of the line
    double intercept = y_mean - slope * x_mean;

    return {slope, intercept};
}

static void getFit(double *deltaPressure, double *deviation, const QVector<double> &times, const QVector<double> &pressures)
{
    std::pair fit = linearRegression(times, pressures);

    double sumsq = 0;
    for (int i = 0; i < times.length(); ++i) {
        double time = times[i];
        double pres = pressures[i];
        double fit_pres = fit.first * time + fit.second;
        double diff = pres - fit_pres;
        sumsq += diff * diff;
    }

    *deltaPressure = fit.first;
    *deviation = sqrt(sumsq / times.length());
}

void MainWindow::onDeviceDiscovered(const QBluetoothDeviceInfo &device)
{
    if (device.name().contains("ESP32_Pycnometer")) { // Replace with your BLE device name
        ui->status->append("Found BLE device: " + device.name());
        discoveryAgent->stop();

        bleController = QLowEnergyController::createCentral(device, this);
        connect(bleController, &QLowEnergyController::connected, this, &MainWindow::onConnected);
        connect(bleController, &QLowEnergyController::errorOccurred, this, &MainWindow::onErrorOccurred);
        connect(bleController, &QLowEnergyController::disconnected, this, &MainWindow::onDisconnected);
        connect(bleController, &QLowEnergyController::serviceDiscovered, this, &MainWindow::onServiceDiscovered);
        connect(bleController, &QLowEnergyController::discoveryFinished, this, &MainWindow::onServiceDiscoveryFinished);

        ui->status->append("Connecting to BLE device...");
        bleController->connectToDevice();
    }
}

void MainWindow::onDeviceDiscoveryError(QBluetoothDeviceDiscoveryAgent::Error error)
{
    Q_UNUSED(error);
    ui->status->append("BLE device discovery error: " + discoveryAgent->errorString());
}

void MainWindow::onConnected()
{
    ui->status->append("Connected to BLE device");
    bleController->discoverServices();
}

void MainWindow::onDisconnected()
{
    ui->status->append("Disconnected from BLE device");
    bleService = nullptr;
    dataCharacteristic = QLowEnergyCharacteristic();
}

void MainWindow::onServiceDiscovered(const QBluetoothUuid &serviceUuid)
{
    if (serviceUuid == QBluetoothUuid(QString(SERVICE_UUID))) {
        ui->status->append("Found service: " + serviceUuid.toString());
        bleService = bleController->createServiceObject(serviceUuid, this);
        if (bleService) {
            connect(bleService, &QLowEnergyService::stateChanged, this, &MainWindow::onServiceStateChanged);
            connect(bleService, &QLowEnergyService::characteristicChanged, this, &MainWindow::onCharacteristicChanged);
            bleService->discoverDetails();
        }
    }
}

void MainWindow::onServiceDiscoveryFinished()
{
    ui->status->append("Service discovery finished");
}

void MainWindow::onServiceStateChanged(QLowEnergyService::ServiceState newState)
{
    if (newState == QLowEnergyService::RemoteServiceDiscovered) {
        dataCharacteristic = bleService->characteristic(QBluetoothUuid(QString(CHARACTERISTIC_UUID)));
        if (dataCharacteristic.isValid()) {
            ui->status->append("Found data characteristic");
            bleService->writeDescriptor(dataCharacteristic.descriptor(QBluetoothUuid(QString("00002902-0000-1000-8000-00805f9b34fb"))),
                                        QByteArray::fromHex("0100")); // Enable notifications
        }
    }
}

void MainWindow::onCharacteristicChanged(const QLowEnergyCharacteristic &characteristic, const QByteArray &newValue)
{
    if (characteristic.uuid() == QBluetoothUuid(QString(CHARACTERISTIC_UUID))) {
        QString data = QString::fromUtf8(newValue);
        processData(data);
    }
}

void MainWindow::processData(const QString &data)
{
    QStringList cols = data.split(",");

    // Skip if the wrong number of columns
    if (cols.count() != 5) return;

    // Get values
    double x = cols[0].toDouble();
    double y1 = cols[1].toDouble();
    double z1 = cols[2].toDouble();
    double y2 = cols[3].toDouble();
    double z2 = cols[4].toDouble();

    // Add to samples
    liveData.time.push_back(x);
    liveData.pres_1.push_back(y1);
    liveData.temp_1.push_back(z1);
    liveData.pres_2.push_back(y2);
    liveData.temp_2.push_back(z2);

    // Set current data
    ui->currentTime_1->setText(QString::number(x, 'f', 1));
    ui->currentTime_2->setText(QString::number(x, 'f', 1));
    ui->currentPressure_1->setText(QString::number(y1, 'f', 3));
    ui->currentPressure_2->setText(QString::number(y2, 'f', 3));
    ui->currentTemperature_1->setText(QString::number(z1, 'f', 2));
    ui->currentTemperature_2->setText(QString::number(z2, 'f', 2));

    if (liveData.time.empty()) {
        ui->deltaPressure_1->clear();
        ui->deviation_1->clear();
        ui->deltaPressure_2->clear();
        ui->deviation_2->clear();
    } else {
        int i;

        const int n = liveData.time.length();
        const double tLast = liveData.time[n - 1];

        for (i = n - 1; i >= 0; --i) {
            if (liveData.time[i] < tLast - 30) break;
        }

        if (i < 0) {
            ui->deltaPressure_1->clear();
            ui->deviation_1->clear();
            ui->deltaPressure_2->clear();
            ui->deviation_2->clear();
        } else {
            double deltaPressure_1, deviation_1;
            double deltaPressure_2, deviation_2;

            // Calculate deviation
            getFit(&deltaPressure_1, &deviation_1, liveData.time.mid(i), liveData.pres_1.mid(i));
            getFit(&deltaPressure_2, &deviation_2, liveData.time.mid(i), liveData.pres_2.mid(i));

            // Update interface
            ui->deltaPressure_1->setText(QString::number(deltaPressure_1, 'f', 6));
            ui->deviation_1->setText(QString::number(deviation_1, 'f', 6));
            ui->deltaPressure_2->setText(QString::number(deltaPressure_2, 'f', 6));
            ui->deviation_2->setText(QString::number(deviation_2, 'f', 6));

            // Add to samples
            deviation.time.push_back(x);
            deviation.deviation_1.push_back(deviation_1);
            deviation.deviation_2.push_back(deviation_2);
        }
    }

    int n = liveData.time.length();
    if (n >= 2) {
        // Get current and previous midpoint
        double t0 = liveData.time[n - 1] - 30;
        double t0_prev = liveData.time[n - 2] - 30;

        // Check if the trigger crossed the midpoint
        if (triggered && (triggerTime > t0_prev) && (triggerTime < t0)) {
            // Save to disk
            saveData();

            // Clear trigger
            triggered = false;

            // Update interface
            updateInterface();
        }
    }

    // Update plots and trigger logic
    updatePlots();
    updateTrigger();
}

void MainWindow::onErrorOccurred(QLowEnergyController::Error error)
{
    Q_UNUSED(error);
    ui->status->append("BLE error: " + bleController->errorString());
}

void MainWindow::configurePlot(QCustomPlot *plot)
{
    plot->setBackground(QBrush(QColor(0, 0, 0, 0)));
    plot->axisRect()->setBackground(QBrush(QColor(255, 255, 255, 255)));

    plot->addGraph();
    plot->graph(0)->setPen(QPen(QColor(40, 110, 255)));

    plot->addGraph();
    plot->graph(1)->setPen(QPen(QColor(255, 110, 40)));

    plot->addGraph();
    plot->graph(2)->setPen(QPen(QColor(0, 0, 0), 0, Qt::DashLine));

    plot->addGraph(plot->xAxis, plot->yAxis2);
    plot->graph(3)->setPen(QPen(QColor(40, 110, 255), 0, Qt::DashLine));

    plot->addGraph(plot->xAxis, plot->yAxis2);
    plot->graph(4)->setPen(QPen(QColor(255, 110, 40), 0, Qt::DashLine));

    plot->xAxis->setLabel("Time (s)");
    plot->yAxis->setLabel("Pressure (kPa)");

    plot->xAxis->setRange(0, 60, Qt::AlignRight);
    plot->yAxis->setRange(0, maxPressure);

    plot->yAxis2->setVisible(true);
    plot->yAxis2->setLabel("");
    plot->yAxis2->setTickLabels(false);
    plot->yAxis2->setTicks(true);
    plot->yAxis2->setRange(0, 0.03);
}

void MainWindow::updateTrigger()
{
    // Check number of samples
    int n = liveData.time.length();
    if (n < 2) return;

    // Get last two samples
    double x1 = liveData.time[n - 2];
    double y1 = liveData.pres_1[n - 2];
    double x2 = liveData.time[n - 1];
    double y2 = liveData.pres_1[n - 1];

    // Check trigger condition
    float a = (triggerPressure - y1) / (y2 - y1);
    if ((0 < a) && (a <= 1)) {
        if (((triggerMode == triggerFalling) && (y2 < y1)) ||
                ((triggerMode == triggerRising) && (y2 > y1))) {
            // Save trigger time
            triggerTime = x1 + a * (x2 - x1);

            // Set trigger
            triggered = true;

            // Update interface
            updateInterface();
        }
    }
}

void MainWindow::updatePlots()
{
    // Set live data
    ui->livePlot->graph(0)->setData(liveData.time, liveData.pres_1);
    ui->livePlot->graph(1)->setData(liveData.time, liveData.pres_2);

    // Set deviation
    ui->livePlot->graph(3)->setData(deviation.time, deviation.deviation_1);
    ui->livePlot->graph(4)->setData(deviation.time, deviation.deviation_2);

    // Rescale
    if (!liveData.time.empty()) {
        ui->livePlot->xAxis->setRange(liveData.time.back(), 60, Qt::AlignRight);
    } else {
        ui->livePlot->xAxis->setRange(0, 60, Qt::AlignRight);
    }

    // Add trigger line
    QVector<double> xs, ys;
    if (triggered) {
        xs.append(triggerTime);
        ys.append(0);
        xs.append(triggerTime);
        ys.append(maxPressure);
    }
    ui->livePlot->graph(2)->setData(xs, ys);

    // Set saved data
    ui->savedPlot->graph(0)->setData(savedData.time, savedData.pres_1);
    ui->savedPlot->graph(1)->setData(savedData.time, savedData.pres_2);

    // Rescale
    if (!savedData.time.empty()) {
        ui->savedPlot->xAxis->setRange(savedData.time.back(), 60, Qt::AlignRight);
    } else {
        ui->savedPlot->xAxis->setRange(0, 60, Qt::AlignRight);
    }

    // Replot
    ui->livePlot->replot();
    ui->savedPlot->replot();
}

void MainWindow::saveData()
{
    const int n = liveData.time.length();
    const double tLast = liveData.time[n - 1];

    int start;
    for (start = n - 1; start > 0; --start) {
        if (liveData.time[start] < tLast - 60) break;
    }

    // Copy last 60 seconds to saved data
    savedData.time.clear();
    for (int i = start; i < liveData.time.length(); ++i) {
        savedData.time.append(liveData.time[i] - triggerTime);
    }
    savedData.pres_1 = liveData.pres_1.mid(start);
    savedData.temp_1 = liveData.temp_1.mid(start);
    savedData.pres_2 = liveData.pres_2.mid(start);
    savedData.temp_2 = liveData.temp_2.mid(start);

    // Write data to disk
    writeData();
}

void MainWindow::writeData()
{
    QFile file(ui->fileName->text());

    if (file.open(QFile::WriteOnly | QFile::NewOnly)) {
        QTextStream stream(&file);

        stream << "time,pres_1,temp_1,pres_2,temp_2" << Qt::endl;

        int n = savedData.time.length();
        for (int i = 0; i < n; ++i) {
            double x = savedData.time[i];
            double y1 = savedData.pres_1[i];
            double z1 = savedData.temp_1[i];
            double y2 = savedData.pres_2[i];
            double z2 = savedData.temp_2[i];

            stream << QString::number(x, 'f', 1) << "," <<
                      QString::number(y1, 'f', 3) << "," << QString::number(z1, 'f', 2) << "," <<
                      QString::number(y2, 'f', 3) << "," << QString::number(z2, 'f', 2) << Qt::endl;
        }

        file.close();

        // Increment trial number
        setTrialNumber(trialNumber + 1);
    } else {
        QMessageBox::critical(this, tr("Error"), file.errorString());
    }
}

void MainWindow::on_options_clicked()
{
    OptionsDialog dialog(this);

    // Copy current settings to dialog
    dialog.setTriggerMode(triggerMode);
    dialog.setTriggerPressure(triggerPressure);
    dialog.setLogFolder(logFolder);

    if (dialog.exec()) {
        // Update current settings
        triggerMode = dialog.triggerMode();
        triggerPressure = dialog.triggerPressure();
        logFolder = dialog.logFolder();

        // Update persistent settings
        QSettings settings("QuantitativeCafe", "GasPyc-2");
        settings.setValue("triggerMode", triggerMode);
        settings.setValue("triggerPressure", triggerPressure);
        settings.setValue("logFolder", logFolder);

        // Update filename
        setTrialNumber(trialNumber);
    }
}

void MainWindow::updateInterface()
{
    if (triggered) {
        ui->saveCancel->setText("Cancel");
        ui->saveCancel->setEnabled(true);
    } else {
        ui->saveCancel->setText("Save");
        ui->saveCancel->setEnabled(!savedData.time.isEmpty());
    }

    ui->trialNumber->setEnabled(!triggered);
    ui->fileName->setEnabled(!triggered);
}

void MainWindow::setTrialNumber(int val)
{
    // Update local value
    trialNumber = val;

    // Update UI
    ui->trialNumber->setValue(trialNumber);
    ui->fileName->setText(logFolder + "/trial-" + QString::number(trialNumber) + ".csv");

    // Update persistent settings
    QSettings settings("QuantitativeCafe", "GasPyc-2");
    settings.setValue("trialNumber", trialNumber);
}

void MainWindow::on_saveCancel_clicked()
{
    if (triggered) {
        // Clear trigger
        triggered = false;

        // Update interface
        updateInterface();

        // Update plots
        updatePlots();
    } else {
        // Write data to disk
        writeData();
    }

}

void MainWindow::on_trialNumber_valueChanged(int value)
{
    setTrialNumber(value);
}
