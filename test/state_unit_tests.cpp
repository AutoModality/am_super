#include <gtest/gtest.h>  // googletest header file
#include <am_super/state_mediator.h>
#include <am_super/super_state.h>

#include <string>
using std::string;
using namespace am;

TEST(StateMediator, allowsTransition_OffToBootingIsAllowed)
{
  StateMediator mediator;
  bool allowed = mediator.allowsTransition(SuperState::OFF,SuperState::BOOTING);
  EXPECT_TRUE(allowed);
}
