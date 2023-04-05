#include "rccpwindow.h"
#include "./ui_rccpwindow.h"

RCCPWindow::RCCPWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::RCCPWindow)
{
    ui->setupUi(this);
}

RCCPWindow::~RCCPWindow()
{
    delete ui;
}

