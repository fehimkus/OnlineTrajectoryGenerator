#include "OnlineWindow.h"

#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    OnlineWindow w;
    w.show();

    return app.exec();
}
