#ifndef LOADCONFIG_H
#define LOADCONFIG_H

#include <QDialog>
#include "ui_loadconfig.h"

namespace pumpkin_qt {

using namespace Qt;

class LoadConfig : public QDialog
{
	Q_OBJECT

public:
	explicit LoadConfig(QWidget *parent = 0);
	virtual ~LoadConfig();

public Q_SLOTS:
	void load(const QString &base_path, const std::vector<std::string> &msg);
	void setConfigurationFile();

private:
	Ui::LoadConfigLayout _ui;
	QString _base_path;
};

}

#endif // LOADCONFIG_H
