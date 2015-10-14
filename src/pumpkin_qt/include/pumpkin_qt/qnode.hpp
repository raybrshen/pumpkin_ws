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

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv, QObject *parent = nullptr);
	virtual ~QNode();
	bool init();

    void run();

Q_SIGNALS:
	void filesReady(const QString &base_path, const std::vector<pumpkin_messages::FileList> &file_list);
	void configFilesReady(const QString &base_path, const std::vector<std::string> &msg);
	void sendStatusMessage(const QString &msg, int timeout);
    void rosShutdown();

public Q_SLOTS:
    void callFiles();
	void callConfigFiles();

private:
	int init_argc;
	char** init_argv;
    ros::ServiceClient _file_client;
};

}  // namespace pumpkin_qt

#endif /* pumpkin_qt_QNODE_HPP_ */
