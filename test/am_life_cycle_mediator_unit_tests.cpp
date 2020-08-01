#include <gtest/gtest.h>
#include <super_lib/am_life_cycle_mediator.h>

using namespace am;
using namespace std;

TEST(LifeCycleMediator, getAndSetStatus)
{
    LifeCycleStatus expected_status = LifeCycleStatus::ERROR;
    AMLifeCycleMediator mediator;
    AMLifeCycleMediator::LifeCycleInfo info;

    mediator.setStatus(expected_status, info);
    LifeCycleStatus actual = mediator.getStatus(info);
    EXPECT_EQ(actual, expected_status);
}