#ifndef APPLICATION_H
#define APPLICATION_H

// 基础核心模块，提供应用程序的基础功能
#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QDir>
#include <QTextStream>

// GUI模块，提供图形界面相关的功能
#include <QGuiApplication> // 用于Qt5的应用程序框架
#include <QIcon>
#include <QPixmap>
#include <QScreen>

// Widgets模块，提供标准控件和布局管理器
#include <QWidget>
#include <QDockWidget>
#include <QMainWindow>
#include <QDialog>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QCheckBox>
#include <QRadioButton>
#include <QSlider>
#include <QProgressBar>
#include <QListWidget>
#include <QTreeWidget>
#include <QStackedWidget>
#include <QToolBar>
#include <QStatusBar>
#include <QMenu>
#include <QMenuBar>
#include <QComboBox>
#include <QGroupBox>
#include <QFrame>

// Layout模块，用于控件布局
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QStackedLayout>

// // 网络模块，提供网络编程功能
// #include <QNetworkAccessManager>
// #include <QNetworkRequest>
// #include <QNetworkReply>

// // SQL模块，提供数据库编程接口
// #include <QSqlDatabase>
// #include <QSqlQuery>
// #include <QSqlTableModel>
// #include <QSqlError>

// // XML模块，提供XML数据解析和生成
// #include <QXmlStreamReader>
// #include <QXmlStreamWriter>

// // 多线程模块，提供并发和多线程编程支持
// #include <QtConcurrent>

// // 测试模块，提供单元测试功能
// #include <QtTest>

// // 打印支持模块，提供打印功能
// #include <QPrinter>
// #include <QPrintDialog>

// // Web引擎模块，提供Web内容集成
// #include <QWebEngineView>

// // Quick模块，提供Qt Quick内容集成
// #include <QQuickWidget>

// // 多媒体模块，提供音频和视频功能
// #include <QMediaPlayer>
// #include <QMediaPlaylist>

// 其他可能需要的模块...

#endif // APPLICATION_H