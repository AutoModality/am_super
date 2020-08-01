#include <gtest/gtest.h>
#include <super_lib/am_life_cycle_mediator.h>

using namespace am;
using namespace std;

TEST(LifeCycleMediator, getAndSetStatus_OK)
{
    LifeCycleStatus expected_status = LifeCycleStatus::OK;
    AMLifeCycleMediator mediator;
    AMLifeCycleMediator::LifeCycleInfo info;

    bool success = mediator.setStatus(expected_status, info);
    LifeCycleStatus actual = mediator.getStatus(info);
    EXPECT_EQ(actual, expected_status);
    EXPECT_TRUE(success);
}

TEST(LifeCycleMediator, getAndSetStatus_ERROR)
{
    LifeCycleStatus expected_status = LifeCycleStatus::ERROR;
    AMLifeCycleMediator mediator;
    AMLifeCycleMediator::LifeCycleInfo info;

    bool success = mediator.setStatus(expected_status, info);
    LifeCycleStatus actual = mediator.getStatus(info);
    EXPECT_EQ(actual, expected_status);
    EXPECT_TRUE(success);
}