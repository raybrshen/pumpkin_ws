/********************************************************************************
** Form generated from reading UI file 'loadconfig.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOADCONFIG_H
#define UI_LOADCONFIG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_LoadConfigLayout
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *titleLabel;
    QComboBox *comboBox;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *LoadConfigLayout)
    {
        if (LoadConfigLayout->objectName().isEmpty())
            LoadConfigLayout->setObjectName(QString::fromUtf8("LoadConfigLayout"));
        LoadConfigLayout->resize(261, 126);
        verticalLayout = new QVBoxLayout(LoadConfigLayout);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        titleLabel = new QLabel(LoadConfigLayout);
        titleLabel->setObjectName(QString::fromUtf8("titleLabel"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(titleLabel->sizePolicy().hasHeightForWidth());
        titleLabel->setSizePolicy(sizePolicy);
        QFont font;
        font.setFamily(QString::fromUtf8("Liberation Sans"));
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        titleLabel->setFont(font);
        titleLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(titleLabel);

        comboBox = new QComboBox(LoadConfigLayout);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        verticalLayout->addWidget(comboBox);

        buttonBox = new QDialogButtonBox(LoadConfigLayout);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        buttonBox->setCenterButtons(true);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(LoadConfigLayout);
        QObject::connect(buttonBox, SIGNAL(accepted()), LoadConfigLayout, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), LoadConfigLayout, SLOT(reject()));

        QMetaObject::connectSlotsByName(LoadConfigLayout);
    } // setupUi

    void retranslateUi(QDialog *LoadConfigLayout)
    {
        LoadConfigLayout->setWindowTitle(QApplication::translate("LoadConfigLayout", "Dialog", 0, QApplication::UnicodeUTF8));
        titleLabel->setText(QApplication::translate("LoadConfigLayout", "Select an configuration file", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LoadConfigLayout: public Ui_LoadConfigLayout {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOADCONFIG_H
