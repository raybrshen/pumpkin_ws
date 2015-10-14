/********************************************************************************
** Form generated from reading UI file 'pumpkinqt.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PUMPKINQT_H
#define UI_PUMPKINQT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTreeView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PumpkinQTDesign
{
public:
    QAction *actionSSC;
    QAction *actionAbout;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QTreeView *folderTree;
    QListView *fileView;
    QTabWidget *mainBlock;
    QWidget *playbackTab;
    QGridLayout *gridLayout_2;
    QLabel *playbackFileName;
    QProgressBar *playbackProgress;
    QPushButton *playButton;
    QPushButton *stopPlayButton;
    QWidget *recordTab;
    QGridLayout *gridLayout_3;
    QLabel *minuteLabel;
    QSpinBox *minuteSpin;
    QLineEdit *recordFileName;
    QLabel *secondLabel;
    QSpinBox *secondSpin;
    QPushButton *recButton;
    QPushButton *stopRecButton;
    QMenuBar *menubar;
    QMenu *menuCommand;
    QMenu *menuHelp;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *PumpkinQTDesign)
    {
        if (PumpkinQTDesign->objectName().isEmpty())
            PumpkinQTDesign->setObjectName(QString::fromUtf8("PumpkinQTDesign"));
        PumpkinQTDesign->resize(604, 467);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/task-8x.png"), QSize(), QIcon::Normal, QIcon::Off);
        PumpkinQTDesign->setWindowIcon(icon);
        actionSSC = new QAction(PumpkinQTDesign);
        actionSSC->setObjectName(QString::fromUtf8("actionSSC"));
        actionAbout = new QAction(PumpkinQTDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        centralwidget = new QWidget(PumpkinQTDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        folderTree = new QTreeView(centralwidget);
        folderTree->setObjectName(QString::fromUtf8("folderTree"));
        folderTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
        folderTree->setSelectionBehavior(QAbstractItemView::SelectRows);

        gridLayout->addWidget(folderTree, 0, 0, 1, 1);

        fileView = new QListView(centralwidget);
        fileView->setObjectName(QString::fromUtf8("fileView"));
        fileView->setEditTriggers(QAbstractItemView::NoEditTriggers);
        fileView->setSelectionBehavior(QAbstractItemView::SelectRows);

        gridLayout->addWidget(fileView, 0, 1, 1, 1);

        mainBlock = new QTabWidget(centralwidget);
        mainBlock->setObjectName(QString::fromUtf8("mainBlock"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mainBlock->sizePolicy().hasHeightForWidth());
        mainBlock->setSizePolicy(sizePolicy);
        playbackTab = new QWidget();
        playbackTab->setObjectName(QString::fromUtf8("playbackTab"));
        playbackTab->setMinimumSize(QSize(457, 0));
        gridLayout_2 = new QGridLayout(playbackTab);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(10, 20, 10, 20);
        playbackFileName = new QLabel(playbackTab);
        playbackFileName->setObjectName(QString::fromUtf8("playbackFileName"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(playbackFileName->sizePolicy().hasHeightForWidth());
        playbackFileName->setSizePolicy(sizePolicy1);
        playbackFileName->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(playbackFileName, 0, 0, 1, 1);

        playbackProgress = new QProgressBar(playbackTab);
        playbackProgress->setObjectName(QString::fromUtf8("playbackProgress"));
        playbackProgress->setValue(0);

        gridLayout_2->addWidget(playbackProgress, 1, 0, 1, 1);

        playButton = new QPushButton(playbackTab);
        playButton->setObjectName(QString::fromUtf8("playButton"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/images/media-play-4x.png"), QSize(), QIcon::Normal, QIcon::Off);
        playButton->setIcon(icon1);

        gridLayout_2->addWidget(playButton, 0, 1, 2, 1);

        stopPlayButton = new QPushButton(playbackTab);
        stopPlayButton->setObjectName(QString::fromUtf8("stopPlayButton"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/images/media-stop-4x.png"), QSize(), QIcon::Normal, QIcon::Off);
        stopPlayButton->setIcon(icon2);

        gridLayout_2->addWidget(stopPlayButton, 0, 2, 2, 1);

        mainBlock->addTab(playbackTab, QString());
        recordTab = new QWidget();
        recordTab->setObjectName(QString::fromUtf8("recordTab"));
        gridLayout_3 = new QGridLayout(recordTab);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(10, 20, 10, 20);
        minuteLabel = new QLabel(recordTab);
        minuteLabel->setObjectName(QString::fromUtf8("minuteLabel"));
        minuteLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(minuteLabel, 1, 0, 1, 1);

        minuteSpin = new QSpinBox(recordTab);
        minuteSpin->setObjectName(QString::fromUtf8("minuteSpin"));
        minuteSpin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        minuteSpin->setMaximum(60);

        gridLayout_3->addWidget(minuteSpin, 1, 1, 1, 1);

        recordFileName = new QLineEdit(recordTab);
        recordFileName->setObjectName(QString::fromUtf8("recordFileName"));

        gridLayout_3->addWidget(recordFileName, 0, 0, 1, 4);

        secondLabel = new QLabel(recordTab);
        secondLabel->setObjectName(QString::fromUtf8("secondLabel"));

        gridLayout_3->addWidget(secondLabel, 1, 2, 1, 1);

        secondSpin = new QSpinBox(recordTab);
        secondSpin->setObjectName(QString::fromUtf8("secondSpin"));
        secondSpin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        secondSpin->setMaximum(59);

        gridLayout_3->addWidget(secondSpin, 1, 3, 1, 1);

        recButton = new QPushButton(recordTab);
        recButton->setObjectName(QString::fromUtf8("recButton"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/images/media-record-4x.png"), QSize(), QIcon::Normal, QIcon::Off);
        recButton->setIcon(icon3);

        gridLayout_3->addWidget(recButton, 0, 5, 2, 1);

        stopRecButton = new QPushButton(recordTab);
        stopRecButton->setObjectName(QString::fromUtf8("stopRecButton"));
        stopRecButton->setIcon(icon2);

        gridLayout_3->addWidget(stopRecButton, 0, 6, 2, 1);

        mainBlock->addTab(recordTab, QString());

        gridLayout->addWidget(mainBlock, 1, 0, 1, 2);

        PumpkinQTDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(PumpkinQTDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 604, 25));
        menuCommand = new QMenu(menubar);
        menuCommand->setObjectName(QString::fromUtf8("menuCommand"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        PumpkinQTDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(PumpkinQTDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        statusbar->setSizeGripEnabled(false);
        PumpkinQTDesign->setStatusBar(statusbar);

        menubar->addAction(menuCommand->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuCommand->addAction(actionSSC);
        menuHelp->addAction(actionAbout);

        retranslateUi(PumpkinQTDesign);

        mainBlock->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(PumpkinQTDesign);
    } // setupUi

    void retranslateUi(QMainWindow *PumpkinQTDesign)
    {
        PumpkinQTDesign->setWindowTitle(QApplication::translate("PumpkinQTDesign", "Pumpkin Playback And Record", 0, QApplication::UnicodeUTF8));
        actionSSC->setText(QApplication::translate("PumpkinQTDesign", "Send commands to SSC", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("PumpkinQTDesign", "About", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        folderTree->setStatusTip(QApplication::translate("PumpkinQTDesign", "Select a folder", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
#ifndef QT_NO_STATUSTIP
        fileView->setStatusTip(QApplication::translate("PumpkinQTDesign", "Select a files", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        playbackFileName->setText(QApplication::translate("PumpkinQTDesign", "Playback file", 0, QApplication::UnicodeUTF8));
        playButton->setText(QApplication::translate("PumpkinQTDesign", "Play", 0, QApplication::UnicodeUTF8));
        stopPlayButton->setText(QApplication::translate("PumpkinQTDesign", "Stop", 0, QApplication::UnicodeUTF8));
        mainBlock->setTabText(mainBlock->indexOf(playbackTab), QApplication::translate("PumpkinQTDesign", "Playback", 0, QApplication::UnicodeUTF8));
        minuteLabel->setText(QApplication::translate("PumpkinQTDesign", "Minutes: ", 0, QApplication::UnicodeUTF8));
        recordFileName->setPlaceholderText(QApplication::translate("PumpkinQTDesign", "Record file", 0, QApplication::UnicodeUTF8));
        secondLabel->setText(QApplication::translate("PumpkinQTDesign", "Seconds: ", 0, QApplication::UnicodeUTF8));
        recButton->setText(QApplication::translate("PumpkinQTDesign", "Record", 0, QApplication::UnicodeUTF8));
        stopRecButton->setText(QApplication::translate("PumpkinQTDesign", "Stop", 0, QApplication::UnicodeUTF8));
        mainBlock->setTabText(mainBlock->indexOf(recordTab), QApplication::translate("PumpkinQTDesign", "Record", 0, QApplication::UnicodeUTF8));
        menuCommand->setTitle(QApplication::translate("PumpkinQTDesign", "Command", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("PumpkinQTDesign", "Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PumpkinQTDesign: public Ui_PumpkinQTDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PUMPKINQT_H
