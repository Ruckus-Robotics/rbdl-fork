#include <iostream>

#include <gtest/gtest.h>

#include "rbdl_dynamics/Logging.h"
#include "rbdl_dynamics/Model.h"
#include "rbdl_dynamics/Kinematics.h"

#include "UnitTestUtils.hpp"

#include "Fixtures.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class CalcAccelerationsTests : public testing::Test
{
public:
    CalcAccelerationsTests()
    {

    }

    void SetUp()
    {

    }

    void TearDown()
    {

    }

    const double TEST_PREC = 1.0e-14;
};

TEST_F(CalcAccelerationsTests, TestCalcPointSimple)
{
    FixedBase3DoF f;

    f.QDDot[0] = 1.;
    f.ref_body_id = f.body_a_id;
    f.point_position = Vector3d(1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    // cout << LogOutput.str() << endl;

    EXPECT_NEAR(0., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(1., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);

    // LOG << "Point accel = " << point_acceleration << endl;
}

TEST_F(CalcAccelerationsTests, TestCalcPointSimpleRotated)
{
    FixedBase3DoF f;

    f.Q[0] = 0.5 * M_PI;

    f.ref_body_id = f.body_a_id;
    f.QDDot[0] = 1.;
    f.point_position = Vector3d(1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    //	cout << LogOutput.str() << endl;

    EXPECT_NEAR(-1., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);

    // LOG << "Point accel = " << point_acceleration << endl;
}

TEST_F(CalcAccelerationsTests, TestCalcPointRotation)
{
    FixedBase3DoF f;

    f.ref_body_id = 1;
    f.QDot[0] = 1.;
    f.point_position = Vector3d(1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    //	cout << LogOutput.str() << endl;

    EXPECT_NEAR(-1., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);

    ClearLogOutput();

    // if we are on the other side we should have the opposite value
    f.point_position = Vector3d(-1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    //	cout << LogOutput.str() << endl;

    EXPECT_NEAR(1., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);
}

TEST_F(CalcAccelerationsTests, TestCalcPointRotatedBaseSimple)
{
    // rotated first joint

    FixedBase3DoF f;
    f.ref_body_id = 1;
    f.Q[0] = M_PI * 0.5;
    f.QDot[0] = 1.;
    f.point_position = Vector3d(1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    EXPECT_NEAR(0., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(-1., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);

    f.point_position = Vector3d(-1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    EXPECT_NEAR(0., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(1., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);
    //	cout << LogOutput.str() << endl;
}

TEST_F(CalcAccelerationsTests, TestCalcPointRotatingBodyB)
{
    // rotating second joint, point at third body

    FixedBase3DoF f;

    f.ref_body_id = 3;
    f.QDot[1] = 1.;
    f.point_position = Vector3d(1., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    // cout << LogOutput.str() << endl;

    EXPECT_NEAR(-1., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);

    // move it a bit further up (acceleration should stay the same)
    f.point_position = Vector3d(1., 1., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    // cout << LogOutput.str() << endl;

    EXPECT_NEAR(-1., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);
}

TEST_F(CalcAccelerationsTests, TestCalcPointBodyOrigin)
{
    // rotating second joint, point at third body

    FixedBase3DoF f;

    f.QDot[0] = 1.;

    f.ref_body_id = f.body_b_id;
    f.point_position = Vector3d(0., 0., 0.);
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.ref_body_id, f.point_position);

    // cout << LogOutput.str() << endl;

    EXPECT_NEAR(-1., f.point_acceleration[0], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[1], TEST_PREC);
    EXPECT_NEAR(0., f.point_acceleration[2], TEST_PREC);
}

TEST_F(CalcAccelerationsTests, TestAccelerationLinearFuncOfQddot)
{
    // rotating second joint, point at third body

    FixedBase3DoF f;

    f.QDot[0] = 1.1;
    f.QDot[1] = 1.3;
    f.QDot[2] = 1.5;

    f.ref_body_id = f.body_c_id;
    f.point_position = Vector3d(1., 1., 1.);

    VectorNd qddot_1 = VectorNd::Zero(f.model->dof_count);
    VectorNd qddot_2 = VectorNd::Zero(f.model->dof_count);
    VectorNd qddot_0 = VectorNd::Zero(f.model->dof_count);

    qddot_1[0] = 0.1;
    qddot_1[1] = 0.2;
    qddot_1[2] = 0.3;

    qddot_2[0] = 0.32;
    qddot_2[1] = -0.1;
    qddot_2[2] = 0.53;

    Vector3d acc_1 = CalcPointAcceleration(*f.model, f.Q, f.QDot, qddot_1, f.ref_body_id, f.point_position);
    Vector3d acc_2 = CalcPointAcceleration(*f.model, f.Q, f.QDot, qddot_2, f.ref_body_id, f.point_position);

    MatrixNd G = MatrixNd::Zero(3, f.model->dof_count);
    CalcPointJacobian(*f.model, f.Q, f.ref_body_id, f.point_position, G, true);

    VectorNd net_acc = G * (qddot_1 - qddot_2);

    Vector3d acc_new = acc_1 - acc_2;

    EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(net_acc.data(), acc_new.data(), 3, TEST_PREC));
}

TEST_F (CalcAccelerationsTests, TestAccelerationFloatingBaseWithUpdateKinematics)
{
    FloatingBase12DoF f;
    ForwardDynamics(*f.model, f.Q, f.QDot, f.Tau, f.QDDot);

    ClearLogOutput();
    Vector3d accel = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.child_2_rot_x_id, Vector3d(0., 0., 0.), true);

    EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(Vector3d(0., -9.81, 0.).data(), accel.data(), 3, TEST_PREC));
}

TEST_F (CalcAccelerationsTests, TestAccelerationFloatingBaseWithoutUpdateKinematics)
{
    FloatingBase12DoF f;
    ForwardDynamics(*f.model, f.Q, f.QDot, f.Tau, f.QDDot);

    //ClearLogOutput();
    Vector3d accel = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.child_2_rot_x_id, Vector3d(0., 0., 0.), false);

    EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(Vector3d(0., 0., 0.).data(), accel.data(), 3, TEST_PREC));
    //	cout << LogOutput.str() << endl;
    //	cout << accel.transpose() << endl;
}

TEST_F(CalcAccelerationsTests, TestCalcPointRotationFixedJoint)
{
    FixedBase3DoF f;
    Body fixed_body(1., Vector3d(1., 0.4, 0.4), Vector3d(1., 1., 1.));
    unsigned int fixed_body_id = f.model->AddBody(f.body_c_id, Xtrans(Vector3d(1., -1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");

    f.QDot[0] = 1.;
    f.point_position = Vector3d(0., 0., 0.);
    Vector3d point_acceleration_reference = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, f.body_c_id, Vector3d(1., -1., 0.));

    ClearLogOutput();
    f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, fixed_body_id, f.point_position);
    //	cout << LogOutput.str() << endl;

    EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose(point_acceleration_reference.data(),
                                                         f.point_acceleration.data(),
                                                         3,
                                                         TEST_PREC));
}

TEST_F(CalcAccelerationsTests, TestCalcPointRotationFixedJointRotatedTransform)
{
    FixedBase3DoF f;
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  SpatialTransform fixed_transform = Xtrans (Vector3d (1., -1., 0.)) * Xrotz(M_PI * 0.5);
  unsigned int fixed_body_id = f.model->AddBody (f.body_c_id, fixed_transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

  f.QDot[0] = 1.;
  f.point_position = Vector3d (0., 0., 0.);
  ClearLogOutput();
  Vector3d point_acceleration_reference = CalcPointAcceleration (*f.model, f.Q, f.QDot, f.QDDot, f.body_c_id, Vector3d (1., 1., 0.));
  // cout << LogOutput.str() << endl;

  // cout << "Point position = " << CalcBodyToBaseCoordinates (*model, Q, fixed_body_id, Vector3d (0., 0., 0.)).transpose() << endl;
  // cout << "Point position_ref = " << CalcBodyToBaseCoordinates (*model, Q, body_c_id, Vector3d (1., 1., 0.)).transpose() << endl;

  ClearLogOutput();
  f.point_acceleration = CalcPointAcceleration(*f.model, f.Q, f.QDot, f.QDDot, fixed_body_id, f.point_position);
  // cout << LogOutput.str() << endl;

  EXPECT_TRUE(unit_test_utils::checkArraysEpsilonClose (point_acceleration_reference.data(),
      f.point_acceleration.data(),
      3,
      TEST_PREC));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}