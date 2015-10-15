#include "../include/pumpkin_qt/loadconfig.hpp"
#include <ros/ros.h>

#include <QComboBox>

namespace pumpkin_qt {

using namespace Qt;

LoadConfig::LoadConfig(QWidget *parent) :
	QDialog(parent)
{
	_ui.setupUi(this);
	this->setResult(QDialog::DialogCode::Rejected);

	QObject::connect(_ui.loadButton, SIGNAL(clicked()), this, SLOT(setConfigurationFile()));
}

LoadConfig::~LoadConfig()
{
}

void LoadConfig::load(const QString &base_path, const std::vector<std::string> &msg)
{
	_base_path = base_path;
	QStringList config_list;
	for(auto it = msg.begin(); it != msg.end(); ++it) {
		config_list.append(QString::fromStdString(*it));
	}
	_ui.comboBox->addItems(config_list);
}

void LoadConfig::setConfigurationFile()
{
	QString value = _ui.comboBox->currentText();
	value = _base_path + "/" + value;
	ros::param::set("/pumpkin/_config_file", value.toStdString());
	this->accept();
}

}
