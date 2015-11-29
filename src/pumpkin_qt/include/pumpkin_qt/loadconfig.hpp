#ifndef LOADCONFIG_H
#define LOADCONFIG_H

#include <QDialog>
#include "ui_loadconfig.h"

namespace pumpkin_qt {

using namespace Qt;

/*!
 * \brief The LoadConfig class is a simple dialog shown to load a config, by the first use of the Pumpkin.
 */
class LoadConfig : public QDialog
{
	Q_OBJECT

public:
	/*!
	 * \brief Constructs the `LoadConfig` dialog, load UI and connect signals.
	 * \param parent	The parent window. It should be the main window.
	 */
	explicit LoadConfig(QWidget *parent = 0);
	/*!
	 * \brief Default destructor.
	 */
	virtual ~LoadConfig();

public Q_SLOTS:
	/*!
	 * \brief Load the possible configuration files.
	 *
	 * This *slot* should be connected to the QNode.
	 *
	 * \param base_path	The path where the configuration files are.
	 * \param msg		The list of configuration files.
	 */
	void load(const QString &base_path, const std::vector<std::string> &msg);
	/*!
	 * \brief Set the selected configuration file in the ROS Parameter Server.
	 */
	void setConfigurationFile();

private:
	Ui::LoadConfigLayout _ui;	//!< The layout.
	QString _base_path;			//!< The path where the configuration files are.
};

}

#endif // LOADCONFIG_H
