#include <iostream>

#include "rbdl_dynamics/Logging.h"
#include "rbdl_dynamics/Model.h"
#include "rbdl_dynamics/Kinematics.h"

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class CalcVelocitiesTest : public testing::Test
{
public:
    CalcVelocitiesTest()
    {

    }

    const double TEST_PREC = 1.0e-14;
};

struct ModelVelocitiesFixture
{
    ModelVelocitiesFixture()
    {
        ClearLogOutput();
        model = new Model;

        body_a = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        Joint joint_a(SpatialVector(0., 0., 1., 0., 0., 0.));

        body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

        body_b = Body(1., Vector3d(0., 1., 0.), Vector3d(1., 1., 1.));
        Joint joint_b(SpatialVector(0., 1., 0., 0., 0., 0.));

        body_b_id = model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

        body_c = Body(1., Vector3d(1., 0., 0.), Vector3d(1., 1., 1.));
        Joint joint_c(SpatialVector(1., 0., 0., 0., 0., 0.));

        body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

        Q = VectorNd::Constant((size_t) model->dof_count, 0.);
        QDot = VectorNd::Constant((size_t) model->dof_count, 0.);

        point_position = Vector3d::Zero(3);
        point_velocity = Vector3d::Zero(3);

        ref_body_id = 0;

        ClearLogOutput();
    }

    ~ModelVelocitiesFixture()
    {
        delete model;
    }

    Model *model;

    unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
    Body body_a, body_b, body_c;
    Joint joint_a, joint_b, joint_c;

    VectorNd Q;
    VectorNd QDot;

    Vector3d point_position, point_velocity;
};

TEST_F(CalcVelocitiesTest, TestCalcPointSimple)
{
    ModelVelocitiesFixture m;
    m.ref_body_id = 1;
    m.QDot[0] = 1.;
    m.point_position = Vector3d(1., 0., 0.);
    m.point_velocity = CalcPointVelocity(*m.model, m.Q, m.QDot, m.ref_body_id, m.point_position);

    EXPECT_NEAR(0., m.point_velocity[0], TEST_PREC);
    EXPECT_NEAR(1., m.point_velocity[1], TEST_PREC);
    EXPECT_NEAR(0., m.point_velocity[2], TEST_PREC);

//  LOG << "Point velocity = " << m.point_velocity << endl;
    //	cout << LogOutput.str() << endl;
}

TEST_F(CalcVelocitiesTest, TestCalcPointRotatedBaseSimple)
{
    // rotated first joint
    ModelVelocitiesFixture m;

    m.ref_body_id = 1;
    m.Q[0] = M_PI * 0.5;
    m.QDot[0] = 1.;
    m.point_position = Vector3d(1., 0., 0.);
    m.point_velocity = CalcPointVelocity(*m.model, m.Q, m.QDot, m.ref_body_id, m.point_position);

    EXPECT_NEAR(-1., m.point_velocity[0], TEST_PREC);
    EXPECT_NEAR(0., m.point_velocity[1], TEST_PREC);
    EXPECT_NEAR(0., m.point_velocity[2], TEST_PREC);

    //	cout << LogOutput.str() << endl;
}

TEST_F(CalcVelocitiesTest, TestCalcPointRotatingBodyB)
{
    // rotating second joint, point at third body

    ModelVelocitiesFixture m;

    m.ref_body_id = 3;
    m.QDot[1] = 1.;
    m.point_position = Vector3d(1., 0., 0.);
    m.point_velocity = CalcPointVelocity(*m.model, m.Q, m.QDot, m.ref_body_id, m.point_position);

    //	cout << LogOutput.str() << endl;

    EXPECT_NEAR(0., m.point_velocity[0], TEST_PREC);
    EXPECT_NEAR(0., m.point_velocity[1], TEST_PREC);
    EXPECT_NEAR(-1., m.point_velocity[2], TEST_PREC);
}

TEST_F(CalcVelocitiesTest, TestCalcPointRotatingBaseXAxis)
{
    // also rotate the first joint and take a point that is
    // on the X direction

    ModelVelocitiesFixture m;

    m.ref_body_id = 3;
    m.QDot[0] = 1.;
    m.QDot[1] = 1.;
    m.point_position = Vector3d(1., -1., 0.);
    m.point_velocity = CalcPointVelocity(*m.model, m.Q, m.QDot, m.ref_body_id, m.point_position);

    //	cout << LogOutput.str() << endl;

    EXPECT_NEAR(0., m.point_velocity[0], TEST_PREC);
    EXPECT_NEAR(2., m.point_velocity[1], TEST_PREC);
    EXPECT_NEAR(-1., m.point_velocity[2], TEST_PREC);
}

TEST_F(CalcVelocitiesTest, TestCalcPointRotatedBaseXAxis)
{
    // perform the previous test with the first joint rotated by pi/2
    // upwards

    ModelVelocitiesFixture m;

    ClearLogOutput();

    m.ref_body_id = 3;
    m.point_position = Vector3d(1., -1., 0.);

    m.Q[0] = M_PI * 0.5;
    m.QDot[0] = 1.;
    m.QDot[1] = 1.;
    m.point_velocity = CalcPointVelocity(*m.model, m.Q, m.QDot, m.ref_body_id, m.point_position);

    //	cout << LogOutput.str() << endl;

    EXPECT_NEAR(-2., m.point_velocity[0], TEST_PREC);
    EXPECT_NEAR(0., m.point_velocity[1], TEST_PREC);
    EXPECT_NEAR(-1., m.point_velocity[2], TEST_PREC);
}

TEST_F(CalcVelocitiesTest, TestCalcPointBodyOrigin) {
  // Checks whether the computation is also correct for points at the origin
  // of a body

    ModelVelocitiesFixture m;

  m.ref_body_id = m.body_b_id;
  m.point_position = Vector3d (0., 0., 0.);

  m.Q[0] = 0.;
  m.QDot[0] = 1.;

  m.point_velocity = CalcPointVelocity(*m.model, m.Q, m.QDot, m.ref_body_id, m.point_position);

  // cout << LogOutput.str() << endl;

  EXPECT_NEAR( 0., m.point_velocity[0], TEST_PREC);
  EXPECT_NEAR( 1., m.point_velocity[1], TEST_PREC);
  EXPECT_NEAR( 0., m.point_velocity[2], TEST_PREC);
}

TEST_F( CalcVelocitiesTest,FixedJointCalcPointVelocity ) {
  // the standard modeling using a null body
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

  SpatialTransform transform = Xtrans (Vector3d (1., 0., 0.));
  unsigned int fixed_body_id = model.AppendBody (transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

  VectorNd Q = VectorNd::Zero (model.dof_count);
  VectorNd QDot = VectorNd::Zero (model.dof_count);

  QDot[0] = 1.;

  ClearLogOutput();
  Vector3d point0_velocity = CalcPointVelocity (model, Q, QDot, fixed_body_id, Vector3d (0., 0., 0.));
  // cout << LogOutput.str() << endl;
  Vector3d point1_velocity = CalcPointVelocity (model, Q, QDot, fixed_body_id, Vector3d (1., 0., 0.));

  EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(Vector3d (0., 1., 0.).data(), point0_velocity.data(), 3, TEST_PREC));
  EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(Vector3d (0., 2., 0.).data(), point1_velocity.data(), 3, TEST_PREC));
}

TEST_F(CalcVelocitiesTest,FixedJointCalcPointVelocityRotated ) {
  // the standard modeling using a null body
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

  SpatialTransform transform = Xtrans (Vector3d (1., 0., 0.));
  unsigned int fixed_body_id = model.AppendBody (transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

  VectorNd Q = VectorNd::Zero (model.dof_count);
  VectorNd QDot = VectorNd::Zero (model.dof_count);

  Q[0] = M_PI * 0.5;
  QDot[0] = 1.;

  ClearLogOutput();
  Vector3d point0_velocity = CalcPointVelocity (model, Q, QDot, fixed_body_id, Vector3d (0., 0., 0.));
  // cout << LogOutput.str() << endl;
  Vector3d point1_velocity = CalcPointVelocity (model, Q, QDot, fixed_body_id, Vector3d (1., 0., 0.));

  EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(Vector3d (-1., 0., 0.).data(), point0_velocity.data(), 3, TEST_PREC));
  EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(Vector3d (-2., 0., 0.).data(), point1_velocity.data(), 3, TEST_PREC));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}