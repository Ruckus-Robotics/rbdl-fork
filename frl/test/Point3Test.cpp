
#include "frl/geometry/Point3.hpp"
#include "GeometryUtilitiesTestHelper.hpp"
#include <gtest/gtest.h>

using namespace frl;
using namespace geometry;

class Point3Test : public ::testing::Test
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

TEST_F(Point3Test, testDistanceSquared)
{
    for (int i = 0; i < nTests; i++)
    {
        double x = GeometryUtilitiesTestHelper::getRandomDouble();
        double y = GeometryUtilitiesTestHelper::getRandomDouble();
        double z = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point(x, y, z);

        double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point2(x2, y2, z2);

        double distanceSquared = pow((x2 - x), 2) + pow((y2 - y), 2) + pow((z2 - z), 2);

        EXPECT_TRUE(distanceSquared - point.distanceSquared(point2) < 1e-8);
    }

    for (int i = 0; i < nTests; i++)
    {
        float x = GeometryUtilitiesTestHelper::getRandomFloat();
        float y = GeometryUtilitiesTestHelper::getRandomFloat();
        float z = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point(x, y, z);

        float x2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float y2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float z2 = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point2(x2, y2, z2);

        float dx,dy,dz;
        dx=x-x2;
        dy=y-y2;
        dz=z-z2;

        float distanceSquared = dx*dx+dy*dy+dz*dz;

        bool res = (distanceSquared - point.distanceSquared(point2)) < 1e-8;
        EXPECT_TRUE(res);

        if(!res)
        {
            std::cout << std::setprecision(15) << x << "," << y << "," << z << "," << x2 << "," << y2 << "," << z2 << std::endl;
            std::cout << std::setprecision(15) << point.x << "," << point.y << "," << point.z << "," << point2.x << "," << point2.y << "," << point2.z << std::endl;
            std::cout << std::setprecision(15) << distanceSquared << "," << point.distanceSquared(point2) << std::endl;
        }
    }
}

TEST_F(Point3Test, testDistance)
{
    for (int i = 0; i < nTests; i++)
    {
        double x = GeometryUtilitiesTestHelper::getRandomDouble();
        double y = GeometryUtilitiesTestHelper::getRandomDouble();
        double z = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point(x, y, z);

        double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point2(x2, y2, z2);

        double distance = sqrt(pow((x2 - x), 2) + pow((y2 - y), 2) + pow((z2 - z), 2));

        EXPECT_TRUE(distance - point.distance(point2) < 1e-8);
    }

    for (int i = 0; i < nTests; i++)
    {
        float x = GeometryUtilitiesTestHelper::getRandomFloat();
        float y = GeometryUtilitiesTestHelper::getRandomFloat();
        float z = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point(x, y, z);

        float x2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float y2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float z2 = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point2(x2, y2, z2);

        float dx,dy,dz;
        dx=x-x2;
        dy=y-y2;
        dz=z-z2;

        float distance = sqrt(dx*dx+dy*dy+dz*dz);

        bool res = (distance - point.distance(point2) < 1e-8);
        EXPECT_TRUE(res);

        if(!res)
        {
            std::cout << std::setprecision(15) << x << "," << y << "," << z << "," << x2 << "," << y2 << "," << z2 << std::endl;
            std::cout << std::setprecision(15) << point.x << "," << point.y << "," << point.z << "," << point2.x << "," << point2.y << "," << point2.z << std::endl;
            std::cout << std::setprecision(15) << distance << "," << point.distance(point2) << std::endl;
        }
    }
}

TEST_F(Point3Test, testDistanceL1)
{
    for (int i = 0; i < nTests; i++)
    {
        double x = GeometryUtilitiesTestHelper::getRandomDouble();
        double y = GeometryUtilitiesTestHelper::getRandomDouble();
        double z = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point(x, y, z);

        double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point2(x2, y2, z2);

        double distanceL1 = fabs(x2 - x) + fabs(y2 - y) + fabs(z2 - z);

        EXPECT_TRUE(distanceL1 - point.distanceL1(point2) < 1e-8);
    }

    for (int i = 0; i < nTests; i++)
    {
        float x = GeometryUtilitiesTestHelper::getRandomFloat();
        float y = GeometryUtilitiesTestHelper::getRandomFloat();
        float z = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point(x, y, z);

        float x2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float y2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float z2 = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point2(x2, y2, z2);

        float distanceL1 = fabs(x2 - x) + fabs(y2 - y) + fabs(z2 - z);

        EXPECT_TRUE(distanceL1 - point.distanceL1(point2) < 1e-8);
    }
}

TEST_F(Point3Test, testDistanceLInf)
{
    for (int i = 0; i < nTests; i++)
    {
        double x = GeometryUtilitiesTestHelper::getRandomDouble();
        double y = GeometryUtilitiesTestHelper::getRandomDouble();
        double z = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point(x, y, z);

        double x2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double y2 = GeometryUtilitiesTestHelper::getRandomDouble();
        double z2 = GeometryUtilitiesTestHelper::getRandomDouble();

        Point3d point2(x2, y2, z2);

        double distanceLInf = std::max(fabs(x2 - x), fabs(y2 - y));
        distanceLInf = std::max(distanceLInf, fabs(z2 - z));

        EXPECT_TRUE(distanceLInf - point.distanceLinf(point2) < 1e-8);
    }

    for (int i = 0; i < nTests; i++)
    {
        float x = GeometryUtilitiesTestHelper::getRandomFloat();
        float y = GeometryUtilitiesTestHelper::getRandomFloat();
        float z = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point(x, y, z);

        float x2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float y2 = GeometryUtilitiesTestHelper::getRandomFloat();
        float z2 = GeometryUtilitiesTestHelper::getRandomFloat();

        Point3f point2(x2, y2, z2);

        float distanceLInf = std::max(fabs(x2 - x), fabs(y2 - y));
        distanceLInf = std::max(distanceLInf, (float)fabs(z2 - z));

        EXPECT_TRUE(distanceLInf - point.distanceLinf(point2) < 1e-8);
    }
}

TEST_F(Point3Test, testAdd1)
{
    for (int i = 0; i < 1000; i++)
    {
        double array1[3] = {GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble()};
        double array2[3] = {GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble()};

        Point3d point1(array1);
        Point3d point2(array2);

        Point3d point3(array1[0] + array2[0], array1[1] + array2[1], array1[2] + array2[2]);

        point1 += point2;

        EXPECT_TRUE(point1.epsilonEquals(point3, 1e-12));
    }

    for (int i = 0; i < 1000; i++)
    {
        float array1[3] = {GeometryUtilitiesTestHelper::getRandomFloat(), GeometryUtilitiesTestHelper::getRandomFloat(), GeometryUtilitiesTestHelper::getRandomFloat()};
        float array2[3] = {GeometryUtilitiesTestHelper::getRandomFloat(), GeometryUtilitiesTestHelper::getRandomFloat(), GeometryUtilitiesTestHelper::getRandomFloat()};

        Point3f point1(array1);
        Point3f point2(array2);

        Point3f point3(array1[0] + array2[0], array1[1] + array2[1], array1[2] + array2[2]);

        point1 += point2;

        EXPECT_TRUE(point1.epsilonEquals(point3, 1e-12));
    }
}

TEST_F(Point3Test, testAdd2)
{
    Point3d point1,point2;
    Point3f point3,point4;

    for (int i = 0; i < 1000; i++)
    {
        point1.set(GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble(), GeometryUtilitiesTestHelper::getRandomDouble());

        std::vector<double> vector(3);
        vector[0] = GeometryUtilitiesTestHelper::getRandomDouble();
        vector[1] = GeometryUtilitiesTestHelper::getRandomDouble();
        vector[2] = GeometryUtilitiesTestHelper::getRandomDouble();

        point2.set(vector);

        Point3d tmpPoint;
        tmpPoint.set(vector[0] + point1.getX(), vector[1] + point1.getY(), vector[2] + point1.getZ());

        point2 = point2 + point1;

        EXPECT_TRUE(point2.epsilonEquals(tmpPoint, 1e-12));
    }

    for (int i = 0; i < 1000; i++)
    {
        point3.set(GeometryUtilitiesTestHelper::getRandomFloat(), GeometryUtilitiesTestHelper::getRandomFloat(), GeometryUtilitiesTestHelper::getRandomFloat());

        std::vector<float> vector(3);
        vector[0] = GeometryUtilitiesTestHelper::getRandomFloat();
        vector[1] = GeometryUtilitiesTestHelper::getRandomFloat();
        vector[2] = GeometryUtilitiesTestHelper::getRandomFloat();

        point4.set(vector);

        Point3f tmpPoint;
        tmpPoint.set(vector[0] + point3.getX(), vector[1] + point3.getY(), vector[2] + point3.getZ());

        point4 = point4 + point3;

        EXPECT_TRUE(point4.epsilonEquals(tmpPoint, 1e-12));
    }
}

TEST_F(Point3Test, testSubtract1)
{
    for (int i = 0; i < 1000; i++)
    {
        Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3<double>();
        Point3d point2 = GeometryUtilitiesTestHelper::getRandomPoint3<double>();
        Point3d point3(point1.getX() - point2.getX(), point1.getY() - point2.getY(), point1.getZ() - point2.getZ());
        point1 -= point2;

        EXPECT_TRUE(point3.epsilonEquals(point1, 1e-12));
    }

    for (int i = 0; i < 1000; i++)
    {
        Point3f point1 = GeometryUtilitiesTestHelper::getRandomPoint3<float>();
        Point3f point2 = GeometryUtilitiesTestHelper::getRandomPoint3<float>();
        Point3f point3(point1.getX() - point2.getX(), point1.getY() - point2.getY(), point1.getZ() - point2.getZ());
        point1 -= point2;

        EXPECT_TRUE(point3.epsilonEquals(point1, 1e-12));
    }
}

TEST_F(Point3Test, testScale1)
{
    for (int i = 0; i < 1000; i++)
    {
        Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3<double>();
        Point3d point2 = point1;
        double scale = rand() % 100 - 50;

        point1 *= scale;

        EXPECT_TRUE(point1.getX() == point2.getX() * scale);
        EXPECT_TRUE(point1.getY() == point2.getY() * scale);
        EXPECT_TRUE(point1.getZ() == point2.getZ() * scale);
    }
}

TEST_F(Point3Test, testAbsoluteValue1)
{
    for (int i = 0; i < 1000; i++)
    {
        Point3d point1 = GeometryUtilitiesTestHelper::getRandomPoint3<double>();
        Point3d point2 = point1;
        point1.absoluteValue();

        EXPECT_TRUE(point1.getX() == fabs(point2.getX()));
        EXPECT_TRUE(point1.getY() == fabs(point2.getY()));
        EXPECT_TRUE(point1.getZ() == fabs(point2.getZ()));
    }

    for (int i = 0; i < 1000; i++)
    {
        Point3f point1 = GeometryUtilitiesTestHelper::getRandomPoint3<float>();
        Point3f point2 = point1;
        point1.absoluteValue();

        EXPECT_TRUE(point1.getX() == fabs(point2.getX()));
        EXPECT_TRUE(point1.getY() == fabs(point2.getY()));
        EXPECT_TRUE(point1.getZ() == fabs(point2.getZ()));
    }
}

TEST_F(Point3Test, testClampMinMax1)
{
    {
        Point3d point1(100, 200, 300);
        Point3d point2 = point1;
        point2.clampMinMax(-100, -50);

        EXPECT_TRUE(point2.getX() == -50);
        EXPECT_TRUE(point2.getY() == -50);
        EXPECT_TRUE(point2.getZ() == -50);
    }

    {
        Point3f point1(100, 200, 300);
        Point3f point2 = point1;
        point2.clampMinMax(-100, -50);

        EXPECT_TRUE(point2.getX() == -50);
        EXPECT_TRUE(point2.getY() == -50);
        EXPECT_TRUE(point2.getZ() == -50);
    }
}

TEST_F(Point3Test, testClampMinMax2)
{
    {
        Point3d point1(100, 200, 300);
        Point3d point2 = point1;
        point2.clampMinMax(-100, 200);

        EXPECT_TRUE(point2.getX() == 100);
        EXPECT_TRUE(point2.getY() == 200);
        EXPECT_TRUE(point2.getZ() == 200);
    }

    {
        Point3f point1(100, 200, 300);
        Point3f point2 = point1;
        point2.clampMinMax(-100, 200);

        EXPECT_TRUE(point2.getX() == 100);
        EXPECT_TRUE(point2.getY() == 200);
        EXPECT_TRUE(point2.getZ() == 200);
    }
}

TEST_F(Point3Test, testClampMinMax3)
{
    {
        Point3d point1(100, 200, 300);
        Point3d point2 = point1;
        point2.clampMinMax(201, 220);

        EXPECT_TRUE(point2.getX() == 201);
        EXPECT_TRUE(point2.getY() == 201);
        EXPECT_TRUE(point2.getZ() == 220);
    }

    {
        Point3f point1(100, 200, 300);
        Point3f point2 = point1;
        point2.clampMinMax(201, 220);

        EXPECT_TRUE(point2.getX() == 201);
        EXPECT_TRUE(point2.getY() == 201);
        EXPECT_TRUE(point2.getZ() == 220);
    }
}

TEST_F(Point3Test, testClampMin)
{
    {
        Point3d point1(100, 200, 300);
        Point3d point2 = point1;
        point2.clampMin(201);

        EXPECT_TRUE(point2.getX() == 201);
        EXPECT_TRUE(point2.getY() == 201);
        EXPECT_TRUE(point2.getZ() == 300);
    }

    {
        Point3f point1(100, 200, 300);
        Point3f point2 = point1;
        point2.clampMin(201);

        EXPECT_TRUE(point2.getX() == 201);
        EXPECT_TRUE(point2.getY() == 201);
        EXPECT_TRUE(point2.getZ() == 300);
    }
}

TEST_F(Point3Test, testClampMax)
{
    {
        Point3d point1(100, 200, 300);
        Point3d point2 = point1;
        point2.clampMax(201);

        EXPECT_TRUE(point2.getX() == 100);
        EXPECT_TRUE(point2.getY() == 200);
        EXPECT_TRUE(point2.getZ() == 201);
    }

    {
        Point3f point1(100, 200, 300);
        Point3f point2 = point1;
        point2.clampMax(201);

        EXPECT_TRUE(point2.getX() == 100);
        EXPECT_TRUE(point2.getY() == 200);
        EXPECT_TRUE(point2.getZ() == 201);
    }
}

TEST_F(Point3Test, testEqualsEquals)
{
    {
        double x = GeometryUtilitiesTestHelper::getRandomDouble();
        double y = GeometryUtilitiesTestHelper::getRandomDouble();
        double z = GeometryUtilitiesTestHelper::getRandomDouble();
        Point3d point1(x, y, z);
        Point3d point2 = point1;

        EXPECT_TRUE(point2==point1);
        EXPECT_FALSE(point2!=point1);
    }

    {
        float x = GeometryUtilitiesTestHelper::getRandomFloat();
        float y = GeometryUtilitiesTestHelper::getRandomFloat();
        float z = GeometryUtilitiesTestHelper::getRandomFloat();
        Point3f point1(x, y, z);
        Point3f point2 = point1;

        EXPECT_TRUE(point2==point1);
        EXPECT_FALSE(point2!=point1);
    }
}

TEST_F(Point3Test, testTemplateTypeInference)
{
    double x = GeometryUtilitiesTestHelper::getRandomDouble();
    double y = GeometryUtilitiesTestHelper::getRandomDouble();
    double z = GeometryUtilitiesTestHelper::getRandomDouble();
    std::vector<float> v(3);
    v[0] = x;
    v[1] = y;
    v[2] = z;
    Point3d point1(v);
    Point3f point2(x,y,z);

    EXPECT_NEAR(point1.getX(),point2.getX(),1e-5);
    EXPECT_NEAR(point1.getY(),point2.getY(),1e-5);
    EXPECT_NEAR(point1.getZ(),point2.getZ(),1e-5);

    Point3f point3 = GeometryUtilitiesTestHelper::getRandomPoint3<float>();

    point1.set(point3);

    EXPECT_NEAR(point3.getX(),point1.getX(),1e-5);
    EXPECT_NEAR(point3.getY(),point1.getY(),1e-5);
    EXPECT_NEAR(point3.getZ(),point1.getZ(),1e-5);

    Point3d point4(point3);

    EXPECT_NEAR(point3.getX(),point4.getX(),1e-5);
    EXPECT_NEAR(point3.getY(),point4.getY(),1e-5);
    EXPECT_NEAR(point3.getZ(),point4.getZ(),1e-5);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS();
}