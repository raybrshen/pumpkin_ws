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
    QNode(int argc, char** argv, QString *filename);
	virtual ~QNode();
	bool init();

    void run();

Q_SIGNALS:
    void filesReady(QString base_path, std::vector<pumpkin_messages::FileList> file_list);
    void playbackPercentage(int percentage);
    void playbackFinished(int state);
    void recordMinuteFeedback(int minutes);
    void recordSecondFeedback(int seconds);
    void recordFinished(int state);
    void lockTab(bool lock);
    void rosShutdown();

public Q_SLOTS:
    void callFiles();
    void playbackFile();
    void playbackStop();
    void recordFile();
    void recordStop();

private:
	int init_argc;
	char** init_argv;
    ros::ServiceClient _file_client;
    QString *_filename_ref;
    actionlib::SimpleActionClient<pumpkin_messages::PlaybackAction> *_playback_client;
    actionlib::SimpleActionClient<pumpkin_messages::RecordAction> *_record_client;

    void playbackDoneCallback(const actionlib::SimpleClientGoalState &goal, const pumpkin_messages::PlaybackResultConstPtr &result);
    void playbackActiveCallback();
    void playbackFeedbackCallback(const pumpkin_messages::PlaybackFeedbackConstPtr &feedback);
};

}  // namespace pumpkin_qt

#endif /* pumpkin_qt_QNODE_HPP_ */
