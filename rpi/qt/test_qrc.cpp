#include <QFile>
#include <QDebug>
#include <QCoreApplication>

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);
    QFile styleFile(":/style.qss");
    if (styleFile.exists()) {
        qDebug() << "Style file exists!";
    } else {
        qDebug() << "Style file DOES NOT exist!";
    }
    return 0;
}
