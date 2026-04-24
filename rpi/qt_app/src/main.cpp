#include "mainwindow.h"

#include <QApplication>
#include <QFile>
#include "shm_reader.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // QSS 다크 테마 파일 로드 및 적용
    QFile styleFile(":/style.qss");

    if (styleFile.open(QFile::ReadOnly | QFile::Text)) {
        
        QString styleSheet = QLatin1String(styleFile.readAll());
        a.setStyleSheet(styleSheet);
        styleFile.close();
    }

    qRegisterMetaType<OdomData>("OdomData");
    qRegisterMetaType<MetaData>("MetaData");
    qRegisterMetaType<LidarData>("LidarData");

    MainWindow w;
    w.show();

    return QCoreApplication::exec();

}
