

#ifndef AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_
#define AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_

using namespace std;
#include <brain_box_msgs/LifeCycleState.h>
#include <super_lib/am_life_cycle_types.h>

namespace am
{

class SuperNodeMediator
{
public:
  SuperNodeMediator();

  /**Standardizes the node name which sometimes starts with `/`.
   * @param node_name orginal name with characgters
   * @return node name stripped of characters.
   */
  std::string nodeNameStripped(std::string node_name);

private:
};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_ */
