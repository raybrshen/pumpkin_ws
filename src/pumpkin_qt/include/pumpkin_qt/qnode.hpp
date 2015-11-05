/**
 * @file /include/pumpkin_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef pumpkin_qt_QNODE_HPP_
#define pumpkin_qt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include "pumpkin_messages/Files.h"
#include "file_type.h"
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pumpkin_qt {

/*****************************************************************************
** Class
*****************************************************************************/

/*!
 * \brief The QNode class holds a thread that runs some of the ROS system,
 *  as the main ROS loop, and the file server.
 */
class QNode : public QThread {
    Q_OBJECT
public:
	/*!
	 * \brief Constructor. Init ROS.
	 * \param argc		Number of command-line arguments.
	 * \param argv		Array of command-line arguments.
	 * \param parent	The parent QObject (as this class is a QThread).
	 */
	QNode(int argc, char** argv, QObject *parent = nullptr);
	/*!
	 * \brief Destructor. Finishes ROS.
	 */
	virtual ~QNode();
	/*!
	 * \brief	Configure all the stuff the ROS need. And start `file_server` client.
	 * \return	False if there is no ROS Master running. True otherwise.
	 */
	bool init();

	/*!
	 * \brief The run thread method.
	 */
    void run();

Q_SIGNALS:
	/*!
	 * \brief Send the list of playback files and respective folder, as a result of `file_lister` call.
	 * \param base_path The base path of all listed file and folders.
	 * \param file_list The serialized tree of folder and files.
	 */
	void filesReady(const QString &base_path, const std::vector<pumpkin_messages::FileList> &file_list);
	/*!
	 * \brief Send the list of config files, and the path of them, as a result of `file_lister` call.
	 * \param base_path	The path of the config files.
	 * \param msg		The list of config files (without path).
	 */
	void configFilesReady(const QString &base_path, const std::vector<std::string> &msg);
	/*!
	 * \brief Send a message to the status bar.
	 * \param msg		The message string.
	 * \param timeout	The time that the message is shown.
	 */
	void sendStatusMessage(const QString &msg, int timeout);
	/*!
	 * \brief Send a signal indicating that it was requested to shutdown (i.e. pressing *Ctrl-C*).
	 */
    void rosShutdown();

public Q_SLOTS:
	/*!
	 * \brief Call the `file_lister` service to list playback files.
	 */
    void callFiles();
	/*!
	 * \brief Call the `file_lister` service to list config files.
	 */
	void callConfigFiles();

private:
	int init_argc;						//!< The number of command_line arguments.
	char** init_argv;					//!< The array of command_line arguments.
	ros::ServiceClient _file_client;	//!< The `file_lister` service client.
};

}  // namespace pumpkin_qt

#endif /* pumpkin_qt_QNODE_HPP_ */
