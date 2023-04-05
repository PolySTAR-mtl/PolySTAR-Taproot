#ifndef RCCPWINDOW_H
#define RCCPWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class RCCPWindow; }
QT_END_NAMESPACE

class RCCPWindow : public QMainWindow
{
    Q_OBJECT

public:
    RCCPWindow(QWidget *parent = nullptr);
    ~RCCPWindow();

private:
    Ui::RCCPWindow *ui;
};
#endif // RCCPWINDOW_H
