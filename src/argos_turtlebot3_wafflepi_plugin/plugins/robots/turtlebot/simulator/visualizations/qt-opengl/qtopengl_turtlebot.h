/**
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 */

#ifndef QTOPENGL_TURTLEBOT_H
#define QTOPENGL_TURTLEBOT_H

#include <string>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos
{
class CQTOpenGLTurtleBot;
class CTurtleBotEntity;
} // namespace argos

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

using namespace std;

namespace argos
{

class Point3f
{

  public:
    Point3f(Real x, Real y, Real z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Real x;
    Real y;
    Real z;
};

class CQTOpenGLTurtleBot
{

  public:
    CQTOpenGLTurtleBot();

    virtual ~CQTOpenGLTurtleBot();

    virtual void Draw(CTurtleBotEntity &c_entity);

  protected:
    /** Sets a white plastic material */
    void SetWhitePlasticMaterial();
    /** Sets a black tire material */
    void SetBlackTireMaterial();
    /** Set gray robot material */
    void SetGrayMaterial();
    /** Sets gray robot material */
    void SetDarkGrayMaterial();
    /** Sets a circuit board material */
    void SetCircuitBoardMaterial();
    /** Sets a colored LED material */
    void SetLEDMaterial(GLfloat f_red, GLfloat f_green, GLfloat f_blue);

    /** Renders the wheels */
    void RenderWheel();
    /** Renders the lid on top of the base */
    void RenderLid();
    /** Makes a waffle plate */
    void RenderWafflePlate();
    /** Renders the laser */
    void RenderLaser();

  private:
    /** Start of the display list index */
    GLuint m_unLists;

    /** Turtlebot wheel */
    GLuint m_unWheelList;

    /** Turtlebot waffle plate */
    GLuint m_unWafflePlateList;

    /** Turtlebot laser scanner */
    GLuint m_unLaserList;

    /** Number of vertices to display the round parts
          (wheels, chassis, etc.) */
    GLuint m_unVertices;

    Point3f applyPointTransform(Point3f origin, string transformType, float width, float length, float height);

    void drawVerticalCylinder(Point3f origin, Real radius, Real height);

    void drawRectangle(Point3f origin, Real width, Real length, Real height);

    void drawFace(Point3f a, Point3f b, Point3f c, Point3f d);
};

} // namespace argos

#endif
