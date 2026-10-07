#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFileInfo>

#include "src/observer.h"
#include "src/models/camera.h"
#include "src/utils/motionControl.h"
#include "src/utils/mathUtils.h"

class M2Sim
{
public:
    explicit M2Sim(QQmlApplicationEngine &engine)
    {
        qmlRegisterType<Observer>("M2", 1, 0, "Observer");
        qmlRegisterType<Camera>("M2", 1, 0, "Camera");
        qmlRegisterType<MotionControl>("M2", 1, 0, "MotionControl");
        qmlRegisterType<MathUtils>("M2", 1, 0, "MathUtils");

        // Resolve repository resources from the executable location so launching
        // build/bin/m2-Sim from any working directory uses the same QML tree.
        const QDir projectDir(QDir::cleanPath(
            QDir(QCoreApplication::applicationDirPath()).filePath("../..")));
        const QUrl mainQmlUrl = QUrl::fromLocalFile(
            projectDir.filePath("src/qml/Main.qml"));

        QObject::connect(
            &engine,
            &QQmlApplicationEngine::objectCreated,
            &engine,
            // objUrl is the URL Qt resolved (absolute, file:///...), while
            // mainQmlUrl is relative, so comparing the two never matched and
            // the exit below never ran: a failed load left the process alive
            // with no window and no exit code. This engine loads exactly one
            // component, so a null obj is enough to know the load failed.
            [](QObject *obj, const QUrl &objUrl) {
                if (!obj) {
                    qCritical() << "Failed to load QML:" << objUrl;
                    QCoreApplication::exit(-1);
                }
            },
            Qt::QueuedConnection
        );

        engine.load(mainQmlUrl);
    }
};

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    // 設定ファイルは起動の引数で選ぶ。環境変数にしないのは、動いている sim がどのファイルを読んだかを
    // コマンド行 (ps) で見えるようにし、開いたままの端末や子のプロセスに選択が残らないようにするため。
    QCommandLineParser parser;
    parser.addHelpOption();
    const QCommandLineOption configOption(
        "config",
        "Settings file to read and to save to from the settings panel (default: "
            + Observer::defaultConfigFilePath() + "). A relative path is taken from the current directory.",
        "file");
    parser.addOption(configOption);
    parser.process(app);
    if (parser.isSet(configOption)) {
        const QFileInfo file(parser.value(configOption));
        if (!file.isFile()) {
            qCritical("[config] 設定ファイルが無い: %s", qPrintable(file.absoluteFilePath()));
            return 2;
        }
        Observer::setConfigFilePath(file.absoluteFilePath());
    }
    qInfo("[config] %s", qPrintable(Observer::configFilePath()));

    QQmlApplicationEngine engine;
    M2Sim sim(engine);

    return app.exec();
}
