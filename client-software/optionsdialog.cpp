#include "optionsdialog.h"
#include "ui_optionsdialog.h"

#include <QFileDialog>

OptionsDialog::OptionsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OptionsDialog)
{
    ui->setupUi(this);
}

OptionsDialog::~OptionsDialog()
{
    delete ui;
}

void OptionsDialog::setHostName(const QString &name)
{
    ui->hostName->setText(name);
}

QString OptionsDialog::hostName(void) const
{
    return ui->hostName->text();
}

void OptionsDialog::setTriggerMode(MainWindow::TriggerMode mode)
{
    ui->triggerMode->setCurrentIndex((int) mode);
}

MainWindow::TriggerMode OptionsDialog::triggerMode() const
{
    return (MainWindow::TriggerMode) ui->triggerMode->currentIndex();
}

void OptionsDialog::setTriggerPressure(float pressure)
{
    ui->triggerPressure->setText(QString::number(pressure));
}

float OptionsDialog::triggerPressure() const
{
    return ui->triggerPressure->text().toFloat();
}

void OptionsDialog::setLogFolder(const QString &folder)
{
    ui->logFolder->setText(folder);
}

QString OptionsDialog::logFolder(void) const
{
    return ui->logFolder->text();
}

void OptionsDialog::on_chooseLogFolder_clicked()
{
    // Pick a folder
    QString dir = QFileDialog::getExistingDirectory(this, tr("Log Folder"), ui->logFolder->text());

    // If the user didn't click cancel
    if (!dir.isEmpty()) {
        ui->logFolder->setText(dir);
    }
}

