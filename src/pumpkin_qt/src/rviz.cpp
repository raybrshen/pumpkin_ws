#include "rviz.hpp"
#include "rviz/yaml_config_reader.h"
#include "rviz/config.h"
#include "rviz/view_manager.h"

#include <QFile>
#include <QTextStream>
#include <QVBoxLayout>

namespace pumpkin_qt {

RViz::RViz(QWidget *parent) : _render(new rviz::RenderPanel)
{
	QFile gridFile(":/rviz/Grid"), robotFile(":/rviz/Pumpkin"), viewsFile(":/rviz/Views");
	//QVBoxLayout *layout = new QVBoxLayout;
	//layout->addWidget(_render);
	//parent->setLayout(layout);
	parent->layout()->addWidget(_render);

	rviz::YamlConfigReader reader;
	rviz::Config gridConfig, robotConfig, viewsConfig;

	reader.readString(gridConfig, QTextStream(&gridFile).readAll());
	reader.readString(robotConfig, QTextStream(&robotFile).readAll());
	reader.readString(viewsConfig, QTextStream(&viewsFile).readAll());

	gridFile.close();
	robotFile.close();
	viewsFile.close();

	//Initialize rviz models
	_manager = new rviz::VisualizationManager(_render);
	_render->initialize(_manager->getSceneManager(), _manager);
	_manager->initialize();
	_manager->startUpdate();

	_manager->setFixedFrame("map");

	//_render->setStatusTip("Hello Pumpkin.");
	//_render->setToolTip("Hello Pumpkin.");

	//Create the display objects
	_grid = _manager->createDisplay("rviz/Grid", "Ground grid", true);
	_robot = _manager->createDisplay("rviz/RobotModel", "Pumpkin Model", true);
	ROS_ASSERT(_grid != nullptr && _robot != nullptr);

	_grid->load(gridConfig);
	_robot->load(robotConfig);

	_manager->getViewManager()->load(viewsConfig);

	_render->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

RViz::~RViz()
{
	delete _manager;
	if (_grid)
		delete _grid;
	if (_robot)
		delete _robot;
	if (_render)
		delete _render;
}

} //END NAMESPACE
