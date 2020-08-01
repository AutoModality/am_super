#include <gtest/gtest.h>
#include <super_lib/am_life_cycle_mediator.h>

using namespace am;
using namespace std;



void ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus expected_status,bool expected_success)
{
    AMLifeCycleMediator mediator;
    AMLifeCycleMediator::LifeCycleInfo info;
    bool success = mediator.setStatus(expected_status, info);
    LifeCycleStatus actual = mediator.getStatus(info);
    EXPECT_EQ(actual, expected_status);
    EXPECT_EQ(success,expected_success);
}

TEST(LifeCycleMediator, getAndSetStatus_OK)
{
    ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus::OK,true);
}

TEST(LifeCycleMediator, getAndSetStatus_WARN)
{
    ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus::WARN,true);
}

TEST(LifeCycleMediator, getAndSetStatus_ERROR)
{
    ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus::ERROR,true);
   
}