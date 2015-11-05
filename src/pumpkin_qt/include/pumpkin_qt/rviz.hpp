#ifndef RVIZ_HPP
#define RVIZ_HPP

#include <ros/ros.h>
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/render_panel.h"

#include <QWidget>

namespace pumpkin_qt {

/*!
 * \brief The RViz class is the widget to handle and display the 3D robot model in a virtual world.
 */
class RViz
{
public:
	/*!
	 * \brief Constructs the widget. Also load, and displays the robot model.
	 * \param parent	The parent widget. It should be some container in the main window.
	 */
	explicit RViz(QWidget *parent = 0);
	/*!
	 * \brief Destroy the views and the child widgets.
	 */
	virtual ~RViz();

private:
	rviz::VisualizationManager *_manager;	//!< The manager to render and display the RViz data
	rviz::Display *_grid;					//!< Render a grid to be a virtual ground.
	rviz::Display *_robot;					//!< Render the 3D Pumpkin Model
	rviz::RenderPanel *_render;				//!< The widget where the rendered data is showed

};

}	//END NAMESPACE

#endif // RVIZ_HPP
