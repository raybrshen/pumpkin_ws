#ifndef RVIZ_HPP
#define RVIZ_HPP

#include <ros/ros.h>
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/render_panel.h"

#include <QWidget>

namespace pumpkin_qt {

class RViz
{
public:
	explicit RViz(QWidget *parent = 0);
	virtual ~RViz();

private:
	rviz::VisualizationManager *_manager;
	rviz::Display *_grid, *_robot;
	rviz::RenderPanel *_render;

};

}	//END NAMESPACE

#endif // RVIZ_HPP
