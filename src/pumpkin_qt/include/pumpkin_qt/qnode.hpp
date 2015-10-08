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
#include "pumpkin_messages/PlaybackAction.h"
#include "file_type.h"
#include "pumpkin_messages/RecordAction.h"
#include <actionlib/client/simple_action_client.h>
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
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();

    void run();

Q_SIGNALS:
    void filesReady(QString base_path, std::vector<pumpkin_messages::FileList> file_list);
    void playbackFeedback(double percentage);
    void playbackFinished(int state);
    void recordMinuteFeedback(int minutes);
    void recordSecondFeedback(int seconds);
    void recordFinished(int state);
    void rosShutdown();

public Q_SLOTS:
    void callFiles();
    void playbackFile(QString file_name);
    void recordFile(QString file_name);

private:
	int init_argc;
	char** init_argv;
    ros::ServiceClient _file_client;
    actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction> _playback_client;
    actionlib::SimpleActionClient<pumpkin_messages::RecordAction> _record_client;

    void playbackGoalCallback();
};

}  // namespace pumpkin_qt

#endif /* pumpkin_qt_QNODE_HPP_ */
