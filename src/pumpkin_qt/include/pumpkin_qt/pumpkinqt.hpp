#ifndef PUMPKINQT_H
#define PUMPKINQT_H

#include "ui_pumpkinqt.h"
#include "qnode.hpp"
#include "playbackactionclient.hpp"
#include "recordactionclient.hpp"
#include "loadconfig.hpp"
#include "sscmovecommand.hpp"
#include "filesdialog.hpp"
#include "rviz.hpp"

#include <QMainWindow>
#include <QStringListModel>
#include <vector>
#include <QList>
#include <QPair>
#include "pumpkin_messages/Files.h"
#include "pumpkin_messages/PlaybackAction.h"
#include "pumpkin_messages/RecordAction.h"

namespace pumpkin_qt {

/*!
 * \brief The FolderItem is a class to manipulate the folder tree structure for this project.
 *
 * This is just a tree structure, containing an integer that represents the folder ID.
 * (this is used for relate the file list).
 * And a string that is the folder name.
 *
 * For organizing, it has a pointer to the parent, and a pointer list to their children.
 */
class FolderItem {
private:
	FolderItem *_parent;			//< The pointer to the parent
	QList<FolderItem *> _children;	//< The list of pointers to the children
	int _id;						//< The ID of the folder
	QString _folder;				//< The name of the folder

	/*!
	 * \brief Append a child folder to this folder.
	 *
	 * This method maintains the consistency of the tree.
	 *
	 * \param child	The child folder.
	 */
    void appendChild(FolderItem *child) {_children.append(child); }
public:
	/*!
	 * \brief Constructor that create a new folder, child of respective parent.
	 * \param name		The name of the new folder
	 * \param index		The ID of the folder
	 * \param parent	The parent folder, if any.
	 */
    explicit FolderItem(const QString &name, int index, FolderItem *parent = 0);
	/*!
	 *	\brief Destructor. It delete the folder, and, recursively, all the descendants.
	 */
    ~FolderItem();
	/*!
	 * \brief Gets the n-th child.
	 * \param row	The index of the child.
	 * \return A pointer to that child.
	 */
    inline FolderItem *child(int row) {return _children.value(row);}
	/*!
	 * \brief Gets the number of children folders.
	 * \return The number of children/
	 */
    inline int childCount() const {return _children.size();}
	/*!
	 * \brief Gets que amount of info that a folder carry.
	 * \return 2. (The ID and the folder name)
	 */
    inline int columnCount() const {return 2;}
	/*!
	 * \brief Gets some info about the folder.
	 * \param column	The column (0 for ID, 1 for name)
	 * \return The data.
	 */
    QVariant data(int column) const;
	/*!
	 * \brief Gets the index of this folder related to this parent.
	 * \return The index, or 0, in case of this folder be the root.
	 */
    int row() const;
	/*!
	 * \brief Gets the parent folder.
	 * \return A pointer to the parent folder.
	 */
    inline FolderItem * parent() {return _parent;}
};

/*!
 * \brief FolderListType if the type to identify a list of folders.
 *
 * Each folder has the name, and related parent ID.
 */
typedef QList<QPair<int, QString> > FolderListType;

/*!
 * \brief The FolderModel is the implement of the abstract model to use on the Tree View
 */
class FolderModel : public QAbstractItemModel {
    Q_OBJECT
private:
	/*!
	 * \brief The root folder.
	 *
	 * This is a virtual folder. It is now shown, it is just do handle the other real folders.
	 */
    FolderItem *_root;
public:
	/*!
	 * \brief Create the folder model with the list of folders.
	 *
	 * \param folders	The list of folders
	 * \param parent	The parent QObject.
	 *
	 * \sa FolderListType
	 */
	explicit FolderModel(const FolderListType & folders, QObject *parent = nullptr);
	/*!
	 * \brief Destructor. Delete de model, and the folders it holds.
	 */
	virtual ~FolderModel();
	/*!
	 * \brief Gets some info from an folder. This is intended to be used in the Tree View.
	 * \param index		The related folder
	 * \param role		The type of info. Actualy it only gets the related folder data.
	 * \return The required info.
	 */
    QVariant data(const QModelIndex &index, int role) const;
	/*!
	 * \brief Get the flags that identify wich type of model is this.
	 * This is intended to be used by the Tree View.
	 * \param index	The element.
	 * \return The element flags. Actualy just say the item is editable and enabled.
	 */
    Qt::ItemFlags flags(const QModelIndex &index) const;
	/*!
	 * \brief Gets some header data, like title and other stuff.
	 * Actually just print a __Folder__ header.
	 * \param section		The header section
	 * \param orientation	The Orientation. Only Horizontal is accepted here.
	 * \param role			The type of data.
	 * \return				The header.
	 */
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
	/*!
	 * \brief Gets the element index in the Model. This is like a reference to work with it.
	 * \param row		The index of this folder as child of another.
	 * \param column	The column info (0 for ID, 1 for name)
	 * \param parent	The parent element, if any.
	 * \return A reference to the folder in the Model and View.
	 */
    QModelIndex index(int row, int column, const QModelIndex &parent) const;
	/*!
	 * \brief Get the parent of this element in the Model.
	 * \param index	The element.
	 * \return The parent, as a Model reference.
	 */
    QModelIndex parent(const QModelIndex &index) const;
	/*!
	 * \brief Tell the number of children a folder has.
	 * \param parent The element.
	 * \return The numbe of children.
	 */
    int rowCount(const QModelIndex &parent) const;
	/*!
	 * \brief The number of columns.
	 * \param parent The element.
	 * \return 2.
	 */
    int columnCount(const QModelIndex &parent) const;
};

/*!
 * \brief The PumpkinQT class is the implementation of the main window of this node.
 */
class PumpkinQT : public QMainWindow
{
    Q_OBJECT
public:
	/*!
	 * \brief Creates the window, with related parent Widget, and passing the command line arguments to QNode.
	 * \param argc		The number of command-line arguments.
	 * \param argv		The array of command-line arguments.
	 * \param parent	The parent widget
	 */
    explicit PumpkinQT(int argc, char *argv[], QWidget *parent = 0);
	/*!
	 * \brief Destroy the window, and the children widgets.
	 */
	virtual ~PumpkinQT();

protected:
	/*!
	 * \brief This is an overriten event. This is most for ellipsing the big strings, like the playback name.
	 * \param event The event.
	 */
    void resizeEvent(QResizeEvent *event);

Q_SIGNALS:
	void changePlaybackFilename(const QString & str);
	void changeRecordFilename(const QString & str);
	void selectFolder(const QString &folder);
	void selectFile(const QString &file);
	void setSceneFilenames(const std::vector<std::string>& filenames);

public Q_SLOTS:
	/*!
	 * \brief Fill the folder tree and create the file list models.
	 * \param base_path	The base path of the files.
	 * \param file_list	The list of the files received from the `file_server`.
	 */
    void fillTable(const QString& base_path, const std::vector<pumpkin_messages::FileList>& file_list);
	/*!
	 * \brief When selected the folder, link the respective file list model to the File View.
	 * \param index	The selected folder.
	 */
    void folderSelected(const QModelIndex &index);
	/*!
	 * \brief When the file is selected in the list, update the `_filename` and the playback/record widgets.
	 * \param index The selected file.
	 */
    void fileSelected(const QModelIndex &index);
	/*!
	 * \brief Run the playback with the specific `_filename` movement.
	 */
	void runPlayback();
	/*!
	 * \brief Record a movement into the `_filename` file.
	 */
	void runRecord();
	/*!
	 * \brief Run scene with the listed movements in the scene list widget.
	 */
	void runScene();
	/*!
	 * \brief Add the file `_filename` to the scene list.
	 */
	void addSceneFile();
	/*!
	 * \brief Remove the selected file in the scene list from it.
	 */
	void removeSceneFile();
	/*!
	 * \brief Change the `_filename`.
	 * \param filename The new file name.
	 */
	void changeFilename(const QString& filename);
	/*!
	 * \brief Called when a playback/record/scene action is finished.
	 * \param state The result state of the action.
	 */
	void actionFinished(int state);
	/*!
	 * \brief Just show the SSC Move Dialog.
	 */
	void showSSCMoveDialog();
	/*!
	 * \brief Show an cute & simple About Dialog.
	 */
	void showAboutDialog();
	/*!
	 * \brief Updates the widgets about the current scene state
	 * \param step			The current movement (or planned translation).
	 * \param total			The total of movements.
	 * \param percentage	The current percentage of atual movement (or planned translation).
	 */
	void updateSceneFeedback(int step, int total, int percentage);

private:
	Ui::PumpkinQTDesign _ui;	//< The Design (auto-generated code)
	QNode _node;				//< The QNode
	QString _base_path, _relative_path, _filename;	//< The paths and filename
	QList<QStringListModel *> _model_list;			//< The list of file list models
	FolderModel *_folder_model;						//< The Folder model
	PlaybackActionClient _playback;					//< The playback action client
	RecordActionClient _record;						//< The record action client
	LoadConfig *_config_dialog;						//< The dialog that loads the config file
	SSCMoveCommand *_move_dialog;					//< The dialog to sent direct commands to SSC
	FilesDialog *_files_dialog;						//< The dialog to handle files
	RViz *_robot_model;								//< The Robot 3D Model widget
};

}
#endif // PUMPKINQT_H
