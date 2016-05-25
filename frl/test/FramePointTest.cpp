#include <gtest/gtest.h>
#include "frl/frames/ReferenceFrame.hpp"
#include "frl/frames/FramePoint.hpp"
#include "ReferenceFrameTestHelper.hpp"

using namespace frl;
using namespace frames;

class FramePointTest : public ::testing::Test
{
protected:

    virtual void SetUp()
    {
        std::srand( time(NULL) );
    }

    virtual void TearDown()
    {
    }

    std::unique_ptr<ReferenceFrame> root1 = ReferenceFrame::createARootFrame("root1");

    int nTests = 1000;

private:

};

TEST_F(FramePointTest, testChangeFrame)
{
    geometry::RigidBodyTransform<double> transform1;

    transform1.setIdentity();
    Eigen::Vector3d rpy(M_PI/2, 0, 0);
    Eigen::Vector3d translation(5.0, 0.0, 0.0);
    transform1.setEulerXYZ(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameA(new RandomUnchangingFrame("A", root1.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << 0, M_PI/2, 0.0;
    translation << 5.0, 0, 0;
    transform1.setEulerXYZ(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameB(new RandomUnchangingFrame("B", frameA.get(), transform1));

    transform1.setIdentity();
    rpy.setZero();
    rpy << 0, 0, M_PI/2;
    translation << 5.0, 0, 0;
    transform1.setEulerXYZ(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameC(new RandomUnchangingFrame("C", frameB.get(), transform1));

    double x = 4.0;
    double y = 7.0;
    double z = 2.0;

    FramePoint<double> framePoint("FramePoint", frameC.get(), x, y, z);

    EXPECT_TRUE(fabs(framePoint.getX() - x) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() - y) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - z) < 1e-8);

    framePoint.changeFrame(frameB.get());

    EXPECT_TRUE(fabs(framePoint.getX() + 2) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() - 4) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - 2) < 1e-8);

    framePoint.changeFrame(frameA.get());

    EXPECT_TRUE(fabs(framePoint.getX() - 7) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() - 4) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - 2) < 1e-8);

    framePoint.changeFrame(root1.get());

    EXPECT_TRUE(fabs(framePoint.getX() - 12) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getY() + 2) < 1e-8);
    EXPECT_TRUE(fabs(framePoint.getZ() - 4) < 1e-8);
}

TEST_F(FramePointTest, testDistance)
{
    FramePoint<double> framePoint1("FramePoint1", root1.get(), 1, 2, 3);
    FramePoint<double> framePoint2("FramePoint2", root1.get(), -1, -2, -3);

    geometry::RigidBodyTransform<double> transform1;

    transform1.setIdentity();
    Eigen::Vector3d rpy(M_PI/2, 0, 0);
    Eigen::Vector3d translation(5.0, 0.0, 0.0);
    transform1.setEulerXYZ(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameA(new RandomUnchangingFrame("A", root1.get(), transform1));

    FramePoint<double> framePoint3("FramePoint3",frameA.get(),1,2,3);

    EXPECT_TRUE(framePoint1.distance(framePoint2)==sqrt(36+16+4));

    try
    {
        framePoint3.distance(framePoint2);
        EXPECT_TRUE(false);
    }
    catch( ... )
    {
        EXPECT_TRUE(true);
    }
}

TEST_F(FramePointTest, testDistance2)
{
    std::shared_ptr<FramePoint<double>> framePoint1(new frl::frames::FramePoint<double>("FramePoint1", root1.get(), 1, 2, 3));
    std::shared_ptr<FramePoint<double>> framePoint2(new frl::frames::FramePoint<double>("FramePoint2", root1.get(), -1, -2, -3));

    geometry::RigidBodyTransform<double> transform1;

    transform1.setIdentity();
    Eigen::Vector3d rpy(M_PI/2, 0, 0);
    Eigen::Vector3d translation(5.0, 0.0, 0.0);
    transform1.setEulerXYZ(rpy);
    transform1.setTranslation(translation);

    std::shared_ptr<ReferenceFrame> frameA(new RandomUnchangingFrame("A", root1.get(), transform1));

    std::shared_ptr<FramePoint<double>> framePoint3(new frl::frames::FramePoint<double>("FramePoint3",frameA.get(),1,2,3));

    EXPECT_TRUE(framePoint1->distance(*framePoint2)==sqrt(36+16+4));

    try
    {
        framePoint3->distance(*framePoint2);
        EXPECT_TRUE(false);
    }
    catch( ... )
    {
        EXPECT_TRUE(true);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS();
}

