#include "mainwindow.h"

#include <QApplication>
#include <QFont>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QFont font = app.font();
    font.setFamilies({"Noto Sans CJK KR", "Noto Sans KR", "DejaVu Sans"});
    app.setFont(font);

    MainWindow window;
    window.show();
    return app.exec();
}
