#ifndef FILESDIALOG_HPP
#define FILESDIALOG_HPP

#include <QDialog>

namespace Ui {
class FilesDialog;
}

class FilesDialog : public QDialog
{
	Q_OBJECT

public:
	explicit FilesDialog(QWidget *parent = 0);
	~FilesDialog();

private:
	Ui::FilesDialog *ui;
};

#endif // FILESDIALOG_HPP
