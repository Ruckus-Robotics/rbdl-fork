#include <gtest/gtest.h>
#include <eigen3/Eigen/Eigen>
#include "frl/geometry/RigidBodyTransform.hpp"
#include "GeometryUtilitiesTestHelper.hpp"

using namespace frl;
using namespace geometry;

class RigidBodyTransformTest : public ::testing::Test
{
protected:

    virtual void SetUp()
    {
        std::srand(time(NULL));
    }

    virtual void TearDown()
    {

    }

    int nTests = 1000;
};

TEST_F(RigidBodyTransformTest, testSetRotationAndZeroTranslationWithAxisAngle)
{
    Eigen::Vector3d vector;
    vector << 1, 2, 3;

    for (int i = 0; i < nTests; i++)
    {
        Eigen::AngleAxis<double> axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle<double>();

        RigidBodyTransform<double> transform(axisAngle, vector);

        Eigen::AngleAxis<double> axisAngleToCheck;

        transform.getRotation(axisAngleToCheck);

        bool success = GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual<double>(axisAngle, axisAngleToCheck, 1e-5);
        if(!success)
        {
            std::cout << axisAngle.axis()(0) << "," << axisAngle.axis()(1) << "," << axisAngle.axis()(2) << "," << axisAngle.angle() << std::endl;
            std::cout << axisAngleToCheck.axis()(0) << "," << axisAngleToCheck.axis()(1) << "," << axisAngleToCheck.axis()(2) << "," << axisAngleToCheck.angle() << std::endl;
            ASSERT_TRUE(false);
        }
    }
}

TEST_F(RigidBodyTransformTest, testCreateTransformWithAxisAngle4dAndRandomVector3d)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Vector3d vector;

        vector = GeometryUtilitiesTestHelper::createRandomVector3<double>();
        Eigen::AngleAxis<double> axisAngle = GeometryUtilitiesTestHelper::createRandomAxisAngle<double>();

        RigidBodyTransform<double> transform(axisAngle, vector);

        Eigen::AngleAxis<double> axisAngleToCheck;

        transform.getRotation(axisAngleToCheck);

        ASSERT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual<double>(axisAngle, axisAngleToCheck, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testNormalize)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix<double,4,4> matrix = GeometryUtilitiesTestHelper::createRandomMatrix4<double>();

        RigidBodyTransform<double> transform(matrix);

        transform.normalize();

        Eigen::Matrix<double,4,4> matrixToCheck;
        transform.get(matrixToCheck);

        if (!GeometryUtilitiesTestHelper::checkOrthogonality<double>(matrixToCheck))
        {
            std::cout << matrixToCheck << std::endl;
            std::cout << matrix << std::endl;
            ASSERT_TRUE(false);
        }
    }
}

TEST_F(RigidBodyTransformTest, testUseQuaternions_1)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Quaternion<double> quat1 = GeometryUtilitiesTestHelper::createRandomQuaternion<double>();

        RigidBodyTransform<double> transform;
        transform.setRotationAndZeroTranslation(quat1);

        Eigen::Quaternion<double> quatToCheck;
        transform.getRotation(quatToCheck);

        bool success = GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual<double>(quat1, quatToCheck, 1e-5);
        if (!success)
        {
            ASSERT_TRUE(false);
        }
    }
}

TEST_F(RigidBodyTransformTest, testCreateTransformWithQuaternionAndVector3)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix<double,3,1> vector = GeometryUtilitiesTestHelper::createRandomVector3<double>();
        Eigen::Quaternion<double> quat1 = GeometryUtilitiesTestHelper::createRandomQuaternion<double>();

        RigidBodyTransform<double> transform(quat1, vector);

        Eigen::Quaternion<double> quatCheck;
        Eigen::Matrix<double,3,1> vecCheck;

        transform.getRotation(quatCheck);
        transform.getTranslation(vecCheck);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual<double>(quat1, quatCheck, 1e-5));
        EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3sEpsilonEqual<double>(vector, vecCheck, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testCreateTransformWithAxisAngleAndVector3)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix<double,3,1> vector = GeometryUtilitiesTestHelper::createRandomVector3<double>();
        Eigen::AngleAxis<double> a1 = GeometryUtilitiesTestHelper::createRandomAxisAngle<double>();

        RigidBodyTransform<double> transform(a1, vector);

        Eigen::AngleAxis<double> axisAngleCheck;
        Eigen::Matrix<double,3,1> vecCheck;

        transform.getRotation(axisAngleCheck);
        transform.getTranslation(vecCheck);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual<double>(a1, axisAngleCheck, 1e-5));
        EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3sEpsilonEqual<double>(vector, vecCheck, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testSetWithEuler)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix<double,3,1> rpy;

        rpy(0) = 2.0 * M_PI * rand() / RAND_MAX - M_PI;
        rpy(1) = 2.0 * (M_PI / 2.0 - 0.01) * rand() / RAND_MAX - (M_PI / 2.0 - 0.01);
        rpy(2) = 2.0 * M_PI * rand() / RAND_MAX - M_PI;

        RigidBodyTransform<double> transform;
        transform.setEulerXYZ(rpy);

        Eigen::Matrix<double,3,1> rpyCheck;
        transform.getEulerXYZ(rpyCheck);

        bool xYes = fabs(rpy(0) - rpyCheck(0)) < 1e-5;
        bool yYes = fabs(rpy(1) - rpyCheck(1)) < 1e-5;
        bool zYes = fabs(rpy(2) - rpyCheck(2)) < 1e-5;
        if(!xYes || !yYes || !zYes)
        {
            std::cout << rpy(0) - rpyCheck(0) << std::endl;
            std::cout << rpy(1) - rpyCheck(1) << std::endl;
            std::cout << rpy(2) - rpyCheck(2) << std::endl;

            ASSERT_TRUE(false);
        }
    }
}

TEST_F(RigidBodyTransformTest, testZeroTranslation)
{
    Eigen::Matrix4d m1;
    Eigen::Vector3d translation;

    for (int i = 0; i < nTests; i++)
    {
        m1 = GeometryUtilitiesTestHelper::createRandomMatrix4<double>();
        RigidBodyTransform<double> transform(m1);

        transform.zeroTranslation();

        transform.getTranslation(translation);

        ASSERT_TRUE(translation(0) == 0.0);
        ASSERT_TRUE(translation(1) == 0.0);
        ASSERT_TRUE(translation(2) == 0.0);
    }
}

TEST_F(RigidBodyTransformTest, testSetRotationWithRotationMatrix)
{
    Eigen::Matrix3d rot1;
    Eigen::Matrix3d rot2;


    for (int i = 0; i < nTests; i++)
    {
        rot1 = GeometryUtilitiesTestHelper::createRandomRotationMatrix<double>();

        RigidBodyTransform<double> transform(rot1);

        transform.getRotation(rot2);

        bool tmp = GeometryUtilitiesTestHelper::areMatrix3EpsilonEqual<double>(rot1, rot2, 10);

        EXPECT_TRUE(tmp);
    }
}

TEST_F(RigidBodyTransformTest, testSetRotationWithQuaternion)
{
    Eigen::Quaternion<double> q1;
    Eigen::Quaternion<double> q2;

    for (int i = 0; i < nTests; i++)
    {
        q1 = GeometryUtilitiesTestHelper::createRandomQuaternion<double>();

        RigidBodyTransform<double> transform(q1);

        transform.getRotation(q2);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areQuaternionsEpsilonEqual<double>(q1, q2, 1e-4));
    }
}

TEST_F(RigidBodyTransformTest, testSetRotationWithAxisAngle)
{
    Eigen::AngleAxis<double> a1;
    Eigen::AngleAxis<double> a2;

    for (int i = 0; i < nTests; i++)
    {
        a1 = GeometryUtilitiesTestHelper::createRandomAxisAngle<double>();

        RigidBodyTransform<double> transform(a1);

        transform.getRotation(a2);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areAxisAngleEpsilonEqual<double>(a1, a2, 1e-4));
    }
}

TEST_F(RigidBodyTransformTest, testApplyRotationX)
{
    Eigen::Matrix3d rotX;

    for (int i = 0; i < nTests; i++)
    {
        rotX(0, 0) = 1;
        rotX(0, 1) = 0;
        rotX(0, 2) = 0;
        rotX(1, 0) = 0;
        rotX(2, 0) = 0;

        double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
        rotX(1, 1) = cos(angle);
        rotX(2, 2) = cos(angle);
        rotX(1, 2) = -sin(angle);
        rotX(2, 1) = sin(angle);

        RigidBodyTransform<double> transform;

        transform.applyRotationX(angle);

        Eigen::Matrix3d rot;

        transform.getRotation(rot);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix3EpsilonEqual<double>(rotX, rot, 1e-5));
    }

}

TEST_F(RigidBodyTransformTest, testApplyRotationY)
{
    Eigen::Matrix3d rotY;

    for (int i = 0; i < nTests; i++)
    {
        rotY(2, 1) = 0;
        rotY(0, 1) = 0;
        rotY(1, 0) = 0;
        rotY(1, 1) = 1;
        rotY(1, 2) = 0;

        double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
        rotY(0, 0) = cos(angle);
        rotY(2, 2) = cos(angle);
        rotY(2, 0) = -sin(angle);
        rotY(0, 2) = sin(angle);

        RigidBodyTransform<double> transform;

        transform.applyRotationY(angle);

        Eigen::Matrix3d rot;

        transform.getRotation(rot);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix3EpsilonEqual<double>(rotY, rot, 1e-5));
    }

}

TEST_F(RigidBodyTransformTest, testApplyRotationZ)
{
    Eigen::Matrix3d rotZ;

    for (int i = 0; i < nTests; i++)
    {
        rotZ(0, 2) = 0;
        rotZ(1, 2) = 0;
        rotZ(2, 0) = 0;
        rotZ(2, 1) = 0;
        rotZ(2, 2) = 1;

        double angle = (2 * M_PI * rand() / RAND_MAX - M_PI);
        rotZ(0, 0) = cos(angle);
        rotZ(1, 1) = cos(angle);
        rotZ(0, 1) = -sin(angle);
        rotZ(1, 0) = sin(angle);

        RigidBodyTransform<double> transform;

        transform.applyRotationZ(angle);

        Eigen::Matrix3d rot;

        transform.getRotation(rot);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix3EpsilonEqual<double>(rotZ, rot, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testMultiply1)
{
    Eigen::Matrix4d mat1;
    Eigen::Matrix4d mat2;
    Eigen::Matrix4d mat3;

    RigidBodyTransform<double> transform1;
    RigidBodyTransform<double> transform2;
    RigidBodyTransform<double> transform3;


    for (int i = 0; i < nTests; i++)
    {
        mat1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        mat2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();

        transform1.set(mat1);
        transform2.set(mat2);

        transform3.multiply(transform1, transform2);
        mat3 = mat1 * mat2;

        Eigen::Matrix4d mat4;

        transform3.get(mat4);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual<double>(mat4, mat3, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testMultiply2)
{
    Eigen::Matrix4d mat1;
    Eigen::Matrix4d mat2;
    Eigen::Matrix4d mat3;

    RigidBodyTransform<double> transform1;
    RigidBodyTransform<double> transform2;


    for (int i = 0; i < nTests; i++)
    {
        mat1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        mat2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();

        transform1.set(mat1);
        transform2.set(mat2);

        transform1.multiply(transform2);

        mat3 = mat1 * mat2;

        Eigen::Matrix4d mat4;

        transform1.get(mat4);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual<double>(mat4, mat3, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testMultiply3)
{
    Eigen::Matrix4d mat1;
    Eigen::Matrix4d mat2;
    Eigen::Matrix4d mat3;

    RigidBodyTransform<double> transform1;
    RigidBodyTransform<double> transform2;
    RigidBodyTransform<double> transform3;


    for (int i = 0; i < nTests; i++)
    {
        mat1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        mat2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();

        transform1.set(mat1);
        transform2.set(mat2);

        transform3 = transform1 * transform2;

        mat3 = mat1 * mat2;

        Eigen::Matrix4d mat4;

        transform3.get(mat4);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual<double>(mat4, mat3, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testIsRotationMatrixEpsilonIdentity)
{
    RigidBodyTransform<double> transform1;

    EXPECT_TRUE(transform1.isRotationMatrixEpsilonIdentity(1e-10));

    Eigen::Matrix3d matrix;

    transform1.setRotation(0.2,-0.5,0.9,0.8);

    EXPECT_FALSE(transform1.isRotationMatrixEpsilonIdentity(1e-5));

    transform1.setRotation(1.-5,1e-5,1e-5,1);

    EXPECT_FALSE(transform1.isRotationMatrixEpsilonIdentity(1e-6));
}

TEST_F(RigidBodyTransformTest, testOverloadTimesEquals)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        Eigen::Matrix4d m2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        RigidBodyTransform<double> t1(m1);
        RigidBodyTransform<double> t2(m2);
        RigidBodyTransform<double> t3;
        RigidBodyTransform<double> t4 = t1;

        t3.multiply(t1, t2);

        t4 *= t2;

        t4.get(m1);
        t3.get(m2);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual<double>(m1, m2, 1e-5));

    }
}

TEST_F(RigidBodyTransformTest, testInvert1)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        Eigen::Matrix4d m2 = m1;

        RigidBodyTransform<double> t1(m1);

        t1.invert();

        t1.get(m1);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual<double>(m1, m2.inverse(), 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testInvert2)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        Eigen::Matrix4d m2;

        RigidBodyTransform<double> t1(m1);
        RigidBodyTransform<double> t2;

        t2.invert(t1);

        t1.invert();

        t1.get(m1);
        t2.get(m2);

        EXPECT_TRUE(GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual<double>(m1, m2, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testGetTranslationDifference)
{
    for (int i = 0; i < nTests; i++)
    {
        Eigen::Matrix4d m1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        Eigen::Matrix4d m2 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
        Eigen::Vector3d v1;
        Eigen::Vector3d v2;

        RigidBodyTransform<double> t1(m1);
        RigidBodyTransform<double> t2(m2);

        t1.getTranslation(v1);
        t2.getTranslation(v2);

        Eigen::Matrix<double,3,1> vec = RigidBodyTransform<double>::getTranslationDifference(t1, t2);
        EXPECT_TRUE(GeometryUtilitiesTestHelper::areVector3sEpsilonEqual<double>(vec, v2 - v1, 1e-5));
    }
}

TEST_F(RigidBodyTransformTest, testTransformVector3)
{
    Eigen::Vector3d v1(1, 2, 3);

    RigidBodyTransform<double> transform;
    transform.setEulerXYZ(0.0, 0.0, -M_PI / 2);

    transform.transform(v1);
    EXPECT_TRUE(v1(0) == 2);
    EXPECT_TRUE(v1(1) - (-1) < 1e-15);
    EXPECT_TRUE(v1(2) == 3);
}

TEST_F(RigidBodyTransformTest, testTransformVector3_2)
{
    Eigen::Vector3d v1(1, 2, 3),v2;


    RigidBodyTransform<double> transform;
    transform.setEulerXYZ(0.0, 0.0, -M_PI / 2);

    transform.transform(v1,v2);

    EXPECT_TRUE(v2(0) == 2);
    EXPECT_TRUE(v2(1) - (-1) < 1e-15);
    EXPECT_TRUE(v2(2) - 3 < 1e-15);
}

TEST_F(RigidBodyTransformTest, testTransformVector4)
{
    Eigen::Vector4d v1(1, 2, 3,1);

    RigidBodyTransform<double> transform;
    transform.setEulerXYZ(0.0, 0.0, -M_PI / 2);
    transform.setTranslation(-1,-2,-3);

    transform.transform(v1);

    EXPECT_TRUE(v1(0) == 1);
    EXPECT_TRUE(v1(1) - (-3) < 1e-15);
    EXPECT_TRUE(v1(2) == 0);
}

TEST_F(RigidBodyTransformTest, testTransformVector4_2)
{
    Eigen::Vector4d v1(1, 2, 3,1),v2;

    RigidBodyTransform<double> transform;
    transform.setEulerXYZ(0.0, 0.0, -M_PI / 2);
    transform.setTranslation(-1,-2,-3);

    transform.transform(v1,v2);

    EXPECT_TRUE(v2(0) == 1);
    EXPECT_TRUE(v2(1) - (-3) < 1e-15);
    EXPECT_TRUE(v2(2) == 0);
}

TEST_F(RigidBodyTransformTest, testTransformPoints1)
{
    Point3d p1(3, 2, 4);
    Point3d p2;

    RigidBodyTransform<double> t1;
    t1.setEulerXYZ(-M_PI / 2, 0.0, 0.0);

    t1.transform(p1, p2);

    EXPECT_TRUE(p2.getX() - 3 < 1e-15);
    EXPECT_TRUE(p2.getY() - 4 < 1e-15);
    EXPECT_TRUE(p2.getZ() - (-2) < 1e-15);
}

TEST_F(RigidBodyTransformTest, testTransformPoints2)
{
    Point3d p1(3, 2, 4);
    Point3d p2;

    RigidBodyTransform<double> t1;
    t1.setEulerXYZ(-M_PI / 2, 0.0, 0.0);

    t1.transform(p1);

    EXPECT_TRUE(p1.getX() - 3 < 1e-15);
    EXPECT_TRUE(p1.getY() - 4 < 1e-15);
    EXPECT_TRUE(p1.getZ() - (-2) < 1e-15);
}

TEST_F(RigidBodyTransformTest, testEqualsOperator)
{
    RigidBodyTransform3d t1 = GeometryUtilitiesTestHelper::createRandomTransformationMatrix<double>();
    RigidBodyTransform3f t2;

    t2=t1;

    Eigen::Matrix4d m1,m2;

    t1.get(m1);
    t2.get(m2);

    GeometryUtilitiesTestHelper::areMatrix4EpsilonEqual(m1,m2,1e-5);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS();
}