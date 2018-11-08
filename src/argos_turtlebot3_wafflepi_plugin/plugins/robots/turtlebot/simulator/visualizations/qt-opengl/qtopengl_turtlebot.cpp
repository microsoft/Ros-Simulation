/**
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Yi Gu - <guyi@microsoft.com>
 * @author Xinyi Joffre - <xiou@microsoft.com>
 */

#include "qtopengl_turtlebot.h"
#include <turtlebot/simulator/entities/turtlebot_entity.h>
#include <turtlebot/simulator/turtlebot_measures.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/gripper_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos
{

/****************************************/
/****************************************/

/* All measures are in meters */

/* Wheel measures */
static const Real WHEEL_WIDTH = TURTLEBOT_WHEEL_WIDTH; //approximately
static const Real HALF_WHEEL_WIDTH = WHEEL_WIDTH * 0.5f;
static const Real WHEEL_RADIUS = TURTLEBOT_WHEEL_RADIUS;
static const Real WHEEL_DIAMETER = WHEEL_RADIUS * 2.0f;
static const Real INTERWHEEL_DISTANCE = TURTLEBOT_WHEEL_DISTANCE;
static const Real HALF_INTERWHEEL_DISTANCE = INTERWHEEL_DISTANCE * 0.5f;

static const Real AXLE_TO_FRONT = TURTLEBOT_AXLE_TO_FRONT;
static const Real AXLE_TO_REAR = TURTLEBOT_AXLE_TO_REAR;

static const Real WAFFLE_PLATE_THICKNESS = 0.010f; //approximately

/****************************************/
/****************************************/

CQTOpenGLTurtleBot::CQTOpenGLTurtleBot() : m_unVertices(40)
{

    /* Reserve the needed display lists */
    m_unLists = glGenLists(3);

    /* Assign indices for better referencing (later) */
    m_unWheelList = m_unLists;
    m_unWafflePlateList = m_unLists + 1;
    m_unLaserList = m_unLists + 2;

    /* Create the wheel display list */
    glNewList(m_unWheelList, GL_COMPILE);
    RenderWheel();
    glEndList();

    /* Create the base module display list */
    glNewList(m_unWafflePlateList, GL_COMPILE);
    RenderWafflePlate();
    glEndList();

    glNewList(m_unLaserList, GL_COMPILE);
    RenderLaser();
    glEndList();
}

/****************************************/
/****************************************/

CQTOpenGLTurtleBot::~CQTOpenGLTurtleBot()
{
    glDeleteLists(m_unLists, 3);
}

/****************************************/
/****************************************/

void CQTOpenGLTurtleBot::Draw(CTurtleBotEntity &c_entity)
{

    /* Place the wheels */
    glPushMatrix();
    glTranslatef((TURTLEBOT_BASE_LENGTH/2 - AXLE_TO_FRONT), HALF_INTERWHEEL_DISTANCE, 0.0f);
    glCallList(m_unWheelList);
    glPopMatrix();

    glPushMatrix();
    glTranslatef((TURTLEBOT_BASE_LENGTH/2 - AXLE_TO_FRONT), -HALF_INTERWHEEL_DISTANCE, 0.0f);
    glCallList(m_unWheelList);
    glPopMatrix();

    /* Place the bottom waffle plate */
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, TURTLEBOT_BASE_ELEVATION);
    glCallList(m_unWafflePlateList);
    glPopMatrix();

    /* Place the middle waffle plate */
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, (TURTLEBOT_BASE_ELEVATION + (TURTLEBOT_BASE_TOP - WAFFLE_PLATE_THICKNESS)) / 2);
    glCallList(m_unWafflePlateList);
    glPopMatrix();

    /* Place the top waffle plate */
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, TURTLEBOT_BASE_TOP - WAFFLE_PLATE_THICKNESS);
    glCallList(m_unWafflePlateList);
    glPopMatrix();

    /* Place laser scanner */
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, TURTLEBOT_LIDAR_ELEVATION);
    glCallList(m_unLaserList);
    glPopMatrix();
}

/****************************************/
/****************************************/
void CQTOpenGLTurtleBot::SetWhitePlasticMaterial()
{
    const GLfloat pfColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    const GLfloat pfSpecular[] = {0.9f, 0.9f, 0.9f, 1.0f};
    const GLfloat pfShininess[] = {100.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTurtleBot::SetBlackTireMaterial()
{
    const GLfloat pfColor[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfShininess[] = {0.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

void CQTOpenGLTurtleBot::SetDarkGrayMaterial()
{
    const GLfloat pfColor[] = {0.05f, 0.05f, 0.05f, 1.0f};
    const GLfloat pfSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfShininess[] = {0.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

void CQTOpenGLTurtleBot::SetGrayMaterial()
{
    const GLfloat pfColor[] = {0.5f, 0.5f, 0.5f, 1.0f};
    const GLfloat pfSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfShininess[] = {0.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTurtleBot::SetCircuitBoardMaterial()
{
    const GLfloat pfColor[] = {0.0f, 0.0f, 1.0f, 1.0f};
    const GLfloat pfSpecular[] = {0.5f, 0.5f, 1.0f, 1.0f};
    const GLfloat pfShininess[] = {10.0f};
    const GLfloat pfEmission[] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/

void CQTOpenGLTurtleBot::SetLEDMaterial(GLfloat f_red, GLfloat f_green, GLfloat f_blue)
{
    const GLfloat pfColor[] = {f_red, f_green, f_blue, 1.0f};
    const GLfloat pfSpecular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat pfShininess[] = {0.0f};
    const GLfloat pfEmission[] = {f_red, f_green, f_blue, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
}

/****************************************/
/****************************************/
void CQTOpenGLTurtleBot::RenderWheel()
{
    /* Set material */
    SetDarkGrayMaterial();

    /* Right side */
    CVector2 cVertex(WHEEL_RADIUS, 0.0f);
    CRadians cAngle(CRadians::TWO_PI / m_unVertices);
    CVector3 cNormal(-1.0f, -1.0f, 0.0f);
    cNormal.Normalize();
    glBegin(GL_POLYGON);
    for (GLuint i = 0; i <= m_unVertices; i++)
    {
        glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
        glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
        cVertex.Rotate(cAngle);
        cNormal.RotateY(cAngle);
    }
    glEnd();
    /* Left side */
    cVertex.Set(WHEEL_RADIUS, 0.0f);
    cAngle = -cAngle;
    cNormal.Set(-1.0f, 1.0f, 0.0f);
    cNormal.Normalize();
    glBegin(GL_POLYGON);
    for (GLuint i = 0; i <= m_unVertices; i++)
    {
        glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
        glVertex3f(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
        cVertex.Rotate(cAngle);
        cNormal.RotateY(cAngle);
    }
    glEnd();

    /* Tire */
    SetBlackTireMaterial();
    cNormal.Set(1.0f, 0.0f, 0.0f);
    cVertex.Set(WHEEL_RADIUS, 0.0f);
    cAngle = -cAngle;
    glBegin(GL_QUAD_STRIP);
    for (GLuint i = 0; i <= m_unVertices; i++)
    {
        glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
        glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
        glVertex3f(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
        cVertex.Rotate(cAngle);
        cNormal.RotateY(cAngle);
    }
    glEnd();
}

void CQTOpenGLTurtleBot::RenderWafflePlate()
{
    /* Set material */
    SetDarkGrayMaterial();

    /* BEGIN BASE RECTANGLE */
    Point3f origin = Point3f(0.0f, 0.0f, 0.0f);

    drawRectangle(origin, TURTLEBOT_BASE_WIDTH, TURTLEBOT_BASE_LENGTH, WAFFLE_PLATE_THICKNESS);
}

void CQTOpenGLTurtleBot::RenderLaser()
{
    /* Set material */
    SetGrayMaterial();

    Point3f origin = Point3f(0.0f, 0.0f, 0.0f);

    drawVerticalCylinder(origin, TURTLEBOT_LIDAR_RADIUS, TURTLEBOT_LIDAR_HEIGHT); 
}

void CQTOpenGLTurtleBot::drawVerticalCylinder(Point3f origin, Real radius, Real height)
{
    /* Upper cylinder circle */
    CVector2 cVertex(0.0f, radius);
    CRadians cAngle(CRadians::TWO_PI / m_unVertices);
    glBegin(GL_POLYGON);
    for (GLuint i = 0; i <= m_unVertices; i++)
    {
        glVertex3f(cVertex.GetX(),cVertex.GetY(),height);
        cVertex.Rotate(cAngle);
    }
    glEnd();

    /* Lower cylinder circle */
    cVertex.Set(0.0f, radius);
    cAngle = -cAngle;
    glBegin(GL_POLYGON);
    for (GLuint i = 0; i <= m_unVertices; i++)
    {
        glVertex3f(cVertex.GetX(),cVertex.GetY(),0.0);
        cVertex.Rotate(cAngle);
    }
    glEnd();

    /* Cylinder body */
    cVertex.Set(0.0f, radius);
    glBegin(GL_QUAD_STRIP);
    for (GLuint i = 0; i <= m_unVertices; i++)
    {
        glVertex3f(cVertex.GetX(), cVertex.GetY(), 0.0);
        glVertex3f(cVertex.GetX(), cVertex.GetY(), height);
        cVertex.Rotate(cAngle);
    }
    glEnd();
}

void CQTOpenGLTurtleBot::drawRectangle(Point3f origin, Real width, Real length, Real height)
{
    /* BEGIN RECTANGLE */
    Point3f p0 = applyPointTransform(origin, "++-", width, length, height);
    Point3f p1 = applyPointTransform(origin, "+--", width, length, height);
    Point3f p2 = applyPointTransform(origin, "---", width, length, height);
    Point3f p3 = applyPointTransform(origin, "-+-", width, length, height);
    Point3f p4 = applyPointTransform(origin, "+++", width, length, height);
    Point3f p5 = applyPointTransform(origin, "+-+", width, length, height);
    Point3f p6 = applyPointTransform(origin, "--+", width, length, height);
    Point3f p7 = applyPointTransform(origin, "-++", width, length, height);

    //near wall
    glBegin(GL_QUADS);
    drawFace(p3, p2, p6, p7);

    //near wall II
    drawFace(p7, p6, p2, p3);

    // far wall
    drawFace(p0, p1, p5, p4);

    // far wall II
    drawFace(p4, p5, p1, p0);

    // right wall
    drawFace(p2, p1, p5, p6);

    // right wall II
    drawFace(p6, p5, p1, p2);

    // left wall
    drawFace(p3, p0, p4, p7);

    // left wall II
    drawFace(p7, p4, p0, p3);

    // bottom
    drawFace(p0, p3, p2, p1);

    // bottom II
    drawFace(p1, p2, p3, p0);

    // top
    drawFace(p4, p7, p6, p5);

    // top II
    drawFace(p5, p6, p7, p4);
    glEnd();
}

void CQTOpenGLTurtleBot::drawFace(Point3f a, Point3f b, Point3f c, Point3f d)
{
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
    glVertex3f(c.x, c.y, c.z);
    glVertex3f(d.x, d.y, d.z);
}

Point3f CQTOpenGLTurtleBot::applyPointTransform(Point3f origin, string transformType, float width, float length, float height)
{
    Real halfX = length / 2;
    Real halfY = width / 2;
    Real halfZ = height / 2;

    if (transformType == "++-")
    {
        return Point3f(origin.x + halfX, origin.y + halfY, origin.z - halfZ);
    }
    if (transformType == "+--")
    {
        return Point3f(origin.x + halfX, origin.y - halfY, origin.z - halfZ);
    }
    if (transformType == "---")
    {
        return Point3f(origin.x - halfX, origin.y - halfY, origin.z - halfZ);
    }
    if (transformType == "-+-")
    {
        return Point3f(origin.x - halfX, origin.y + halfY, origin.z - halfZ);
    }
    if (transformType == "+++")
    {
        return Point3f(origin.x + halfX, origin.y + halfY, origin.z + halfZ);
    }
    if (transformType == "+-+")
    {
        return Point3f(origin.x + halfX, origin.y - halfY, origin.z + halfZ);
    }
    if (transformType == "--+")
    {
        return Point3f(origin.x - halfX, origin.y - halfY, origin.z + halfZ);
    }
    if (transformType == "-++")
    {
        return Point3f(origin.x - halfX, origin.y + halfY, origin.z + halfZ);
    }
}

/****************************************/
/****************************************/

class CQTOpenGLOperationDrawTurtleBotNormal : public CQTOpenGLOperationDrawNormal
{
  public:
    void ApplyTo(CQTOpenGLWidget &c_visualization,
                 CTurtleBotEntity &c_entity)
    {
        static CQTOpenGLTurtleBot m_cModel;
        c_visualization.DrawRays(c_entity.GetControllableEntity());
        c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
        m_cModel.Draw(c_entity);
    }
};

class CQTOpenGLOperationDrawTurtleBotSelected : public CQTOpenGLOperationDrawSelected
{
  public:
    void ApplyTo(CQTOpenGLWidget &c_visualization,
                 CTurtleBotEntity &c_entity)
    {
        c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
    }
};

REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawTurtleBotNormal, CTurtleBotEntity);

REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawTurtleBotSelected, CTurtleBotEntity);

/****************************************/
/****************************************/

} // namespace argos
