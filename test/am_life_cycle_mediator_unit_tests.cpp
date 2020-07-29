TEST()
{
    LifeCycleStatus expected_status=LifeCycleStatus::ERROR;
    LifeCycleInfo info;
    mediator.setStatus(status,info);
    LifeCycleInfo actual = mediator.getStatus(info);
    EXPECT_EQ(actual, expected_state);
}